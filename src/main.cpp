#include <Arduino.h>
#include "dac_output.h"
#include "phase_capture.h"
#include "dpll_controller.h"
#include "com.h"
#include "eeprom_24lc16b.h"
#include "eeprom_config.h"
#include "dds_ad9833.h"
#include <HardwareSerial.h>
extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

#define HEARTBEAT_LED      PC13
#define LOCK_INDICATOR_LED PC10 // active high. Lock indicator when state changes to Locked.
void heartbeat();
void processSerialDebugCommand();
void processDPLL();
void debugPrint();
void sendMonitorStream();
void commandProccessor();
void setupSerialDebug();
// All loop-config state lives in dpll_controller (accessible via dpll:: getters/setters).
// Default values are set there; use opcode or UART command to change at runtime.
HardwareSerial DebugPort(PA10, PA9); // RX, TX
void setup()
{
  setupSerialDebug();
  com::begin();
  pinMode(HEARTBEAT_LED, OUTPUT);
  digitalWrite(HEARTBEAT_LED, HIGH);
  pinMode(LOCK_INDICATOR_LED, OUTPUT);
  digitalWrite(LOCK_INDICATOR_LED, LOW);
  // Startup self-test: blink lock indicator LED 3 times (active HIGH)
  for (int i = 0; i < 3; i++) {
    digitalWrite(LOCK_INDICATOR_LED, HIGH);
    delay(80);
    digitalWrite(LOCK_INDICATOR_LED, LOW);
    delay(80);
  }

  // 1. Initialize 24LC16B I2C EEPROM on PB6 (SCL) and PB7 (SDA)
  if (eeprom::begin(PB6, PB7, 400000))
  {
    DebugPort.println(F("EEPROM 24LC16B: Initialized and detected on I2C (PB6/PB7)"));
  }
  else
  {
    DebugPort.println(F("EEPROM 24LC16B: WARNING - Device not responding on I2C (check wiring/pullups)"));
  }

  // 2. Load configuration from EEPROM first, or establish default parameters
  if (eeprom_config::loadSavedConfig())
  {
    DebugPort.println(F("EEPROM: Saved DPLL configuration loaded and applied."));
  }
  else
  {
    // Apply baseline default DPLL & DDS parameters
    dpll::begin(1.65f, 0.000002f, 0.0002f, 0.0f);
    dpll::setTargetPhase(600.0f);
    dpll::setOutputLimits(0.0f, 3.3f);
    dpll::setMaxSlew(30.0f);
    dpll::setDdsCenterFreq(28000.0f);
    dds::setMcoClock(dds::MCO_CLOCK_HSE_8MHZ);
    DebugPort.println(F("EEPROM: Using default DPLL configuration."));
  }

  // 3. Initialize hardware peripherals according to the active configuration:

  // DAC control output: PA4, configured with active center voltage (or 1.65 V neutral in DDS mode)
  dac::begin();
  if (dpll::getGeneratorSource() == dpll::GENERATOR_SOURCE_DDS)
  {
    dac::setVoltage(1.65f); // park DAC at neutral mid-scale
  }
  else
  {
    dac::setVoltage(dpll::getCenterVoltage());
  }

  // Initialize AD9833 DDS Generator:
  // Starts directly with configured MCO clock preset and active center frequency (no intermediate jumping)
  // PA8 = MCLK (MCO1 hardware clock), PC4 = FSYNC / SS, PA5 = SCK, PA7 = MOSI
  dds::begin((double)dpll::getDdsCenterFreq(), SQUARE_WAVE, dds::getMcoClock());
  DebugPort.printf("AD9833 DDS: Initialized Square Wave (MCLK: %s, %u Hz, Res: %.6f Hz, Freq: %.2f Hz)\n",
                   dds::getMcoClockName(), dds::getMclkFreq(), dds::getResolution(), dds::getFrequency());

  // Initialize 32-bit Input Capture on TIM2:
  // PA0 = Reference input (Rising/Falling/Both edge trigger from configuration)
  // PA1 = ZCD feedback input (Rising/Falling/Both edge trigger from configuration)
  // zcdTimeoutMs is set to 30 ms to avoid false signal-loss dropouts during sonication.
  phase_capture::begin(PA0, PA1, phase_capture::getRefEdge(), phase_capture::getZcdEdge(), 30);
}

void loop()
{
  processDPLL();
  com::receive_command();
  commandProccessor();
  processSerialDebugCommand();
  sendMonitorStream();
  com::FlushTxQueue(); // flush any pending packets to USB
  //debugPrint();
  heartbeat();
}

void setupSerialDebug()
{
  DebugPort.begin(115200);
  DebugPort.println(F("DPLL Ultrasonic Frequency Tracking"));
  DebugPort.println(F("Type 'help' for commands."));
}
void commandProccessor()
{
  static ComPacket RxPacket;
  // Drain the entire RX queue per loop cycle so a burst of opcodes (e.g. the
  // host's stream-enable + 13-opcode GET refresh) is fully processed in one
  // pass without blocking the control loop for long.
  if (com::getAvailableRxPackets(&RxPacket) != ILEGAL_OPCODE)
  {
    const uint16_t opcode = RxPacket.header.opcode;
    const uint8_t *pl = RxPacket.payload;
    const uint16_t plen = RxPacket.header.payloadLength; // includes checksum byte

    // Debug: print every incoming opcode so we can see what the host sends.
    // Useful when debugging "no data" issues over the debug port (serial monitor).
    DebugPort.printf("[RX] opcode=0x%04X len=%u\n", opcode, plen);

    // Payload helpers (little-endian, host sends the same layout).
    auto payloadFloat = [&]() -> float {
      float f;
      memcpy(&f, pl, sizeof(f));
      return f;
    };
    auto payloadU32 = [&]() -> uint32_t {
      return (uint32_t)pl[0] | ((uint32_t)pl[1] << 8) |
             ((uint32_t)pl[2] << 16) | ((uint32_t)pl[3] << 24);
    };
    auto sendFloat = [&](uint16_t resp, float v) {
      com::sendPacket(resp, 0, (const uint8_t *)&v, sizeof(v));
    };
    auto sendU32 = [&](uint16_t resp, uint32_t v) {
      com::sendPacket(resp, 0, (const uint8_t *)&v, sizeof(v));
    };
    auto sendBool = [&](uint16_t resp, bool v) {
      uint8_t b = v ? 1 : 0;
      com::sendPacket(resp, 0, &b, 1);
    };

    switch (opcode)
    {
      case OPCODE_GET_VERSION:
      {
        // Null-terminated version string — the DAQ parser reads until '\0'.
        const char *version = "DPLL v2.0.0";
        com::sendPacket(OPCODE_GET_VERSION, 0, (const uint8_t *)version, strlen(version) + 1);
        break;
      }
      case OPCODE_SET_ALLOW_SEND_STREAM:
        if (plen >= 1)
        {
          com::SetAllowSendStream(pl[0] != 0);
          sendBool(OPCODE_GET_ALLOW_SEND_STREAM, com::GetAllowSendStream());
        }
        break;
      case OPCODE_GET_ALLOW_SEND_STREAM:
        sendBool(OPCODE_GET_ALLOW_SEND_STREAM, com::GetAllowSendStream());
        break;

      // --- PID gains (float LE) ---
      case OPCODE_SET_KP:
        if (plen >= 4) dpll::setGains(payloadFloat(), dpll::getKi());
        break;
      case OPCODE_GET_KP:
        sendFloat(OPCODE_GET_KP, dpll::getKp());
        break;
      case OPCODE_SET_KI:
        if (plen >= 4) dpll::setGains(dpll::getKp(), payloadFloat());
        break;
      case OPCODE_GET_KI:
        sendFloat(OPCODE_GET_KI, dpll::getKi());
        break;
      case OPCODE_SET_KD:
        if (plen >= 4) dpll::setKd(payloadFloat());
        break;
      case OPCODE_GET_KD:
        sendFloat(OPCODE_GET_KD, dpll::getKd());
        break;

      // --- Center / target / slew (float LE) ---
      case OPCODE_SET_CENTER_VOLTAGE:
        if (plen >= 4)
        {
          float v = payloadFloat();
          if (v >= 0.0f && v <= 3.3f)
            dpll::begin(v, dpll::getKp(), dpll::getKi(), dpll::getKd());
        }
        break;
      case OPCODE_GET_CENTER_VOLTAGE:
        sendFloat(OPCODE_GET_CENTER_VOLTAGE, dpll::getCenterVoltage());
        break;
      case OPCODE_SET_TARGET_PHASE:
        if (plen >= 4) dpll::setTargetPhase(payloadFloat());
        break;
      case OPCODE_GET_TARGET_PHASE:
        sendFloat(OPCODE_GET_TARGET_PHASE, dpll::getTargetPhase());
        break;
      case OPCODE_SET_MAX_SLEW:
        if (plen >= 4)
        {
          float v = payloadFloat();
          if (v > 0.0f) dpll::setMaxSlew(v);
        }
        break;
      case OPCODE_GET_MAX_SLEW:
        sendFloat(OPCODE_GET_MAX_SLEW, dpll::getMaxSlew());
        break;

      // --- Output limits (two floats LE) ---
      case OPCODE_SET_OUTPUT_LIMITS:
        if (plen >= 8)
        {
          float minV = payloadFloat();
          float maxV;
          memcpy(&maxV, pl + 4, sizeof(maxV));
          dpll::setOutputLimits(minV, maxV);
        }
        break;
      case OPCODE_GET_OUTPUT_LIMITS:
      {
        // Firmware only exposes a fixed 0..3.3 V clamp via the controller.
        float limits[2] = {0.0f, 3.3f};
        com::sendPacket(OPCODE_GET_OUTPUT_LIMITS, 0, (const uint8_t *)limits, sizeof(limits));
        break;
      }

      // --- Loop enable / reset / shutdown / voltage ---
      case OPCODE_SET_ENABLE_LOOP:
        if (plen >= 1)
        {
          bool on = pl[0] != 0;
          dpll::setManualMode(!on);
          if (on) dpll::enable(true);
        }
        break;
      case OPCODE_GET_LOOP_ENABLE:
        sendBool(OPCODE_GET_LOOP_ENABLE, dpll::isEnabled() && !dpll::getManualMode());
        break;
      case OPCODE_RESET_LOOP:
        dpll::setManualMode(false);
        dpll::reset();
        dpll::enable(true);
        break;
      case OPCODE_SHUTDOWN_LOOP:
        dpll::shutdown();
        break;
      case OPCODE_SET_VOLTAGE:
        if (plen >= 4)
        {
          float v = payloadFloat();
          if (v >= 0.0f && v <= 3.3f)
          {
            dpll::setManualMode(true);
            dpll::manualSet(v);
            DebugPort.printf("[DPLL] Manual DAC voltage set: %.3f V (loop disengaged)\n", v);
          }
        }
        break;
      case OPCODE_GET_VOLTAGE:
        sendFloat(OPCODE_GET_VOLTAGE, dpll::lastVoltage());
        break;

      // --- Loop / acquisition config ---
      case OPCODE_SET_LOOP_PERIOD:
        if (plen >= 4)
        {
          uint32_t v = payloadU32();
          if (v >= 1 && v <= 1000) dpll::setLoopPeriodMs(v);
        }
        break;
      case OPCODE_GET_LOOP_PERIOD:
        sendU32(OPCODE_GET_LOOP_PERIOD, dpll::getLoopPeriodMs());
        break;
      case OPCODE_SET_LOCK_THRESHOLD:
        if (plen >= 4) dpll::setLockThresholdNs(payloadFloat());
        break;
      case OPCODE_GET_LOCK_THRESHOLD:
        sendFloat(OPCODE_GET_LOCK_THRESHOLD, dpll::getLockThresholdNs());
        break;
      case OPCODE_SET_MANUAL_MODE:
        if (plen >= 1)
        {
          bool on = pl[0] != 0;
          dpll::setManualMode(on);
          if (on) {
            dpll::enable(false);
            DebugPort.println("[DPLL] Manual test mode engaged (closed loop disabled)");
          } else {
            dpll::enable(true);
            DebugPort.println("[DPLL] Closed loop tracking resumed");
          }
        }
        break;
      case OPCODE_GET_MANUAL_MODE:
        sendBool(OPCODE_GET_MANUAL_MODE, dpll::getManualMode());
        break;
      case OPCODE_SET_LOCK_HOLD_CYCLES:
        if (plen >= 4) dpll::setLockHoldCycles(payloadU32());
        break;
      case OPCODE_GET_LOCK_HOLD_CYCLES:
        sendU32(OPCODE_GET_LOCK_HOLD_CYCLES, dpll::getLockHoldCycles());
        break;
      case OPCODE_SET_LOCK_MEMORY_TIMEOUT:
        if (plen >= 4) dpll::setLockMemoryTimeoutMs(payloadU32());
        break;
      case OPCODE_GET_LOCK_MEMORY_TIMEOUT:
        sendU32(OPCODE_GET_LOCK_MEMORY_TIMEOUT, dpll::getLockMemoryTimeoutMs());
        break;
      case OPCODE_SET_SIGNAL_LOSS_BEHAVIOR:
        if (plen >= 1)
        {
          uint8_t b = pl[0];
          if (b <= 2) dpll::setSignalLossBehavior((dpll::SignalLossBehavior)b);
        }
        break;
      case OPCODE_GET_SIGNAL_LOSS_BEHAVIOR:
      {
        uint8_t b = (uint8_t)dpll::getSignalLossBehavior();
        com::sendPacket(OPCODE_GET_SIGNAL_LOSS_BEHAVIOR, 0, &b, 1);
        break;
      }
      case OPCODE_SET_STREAM_PERIOD:
        if (plen >= 4)
        {
          uint32_t v = payloadU32();
          if (v >= 1 && v <= 65535) dpll::setStreamPeriodMs(v);
        }
        break;
      case OPCODE_GET_STREAM_PERIOD:
        sendU32(OPCODE_GET_STREAM_PERIOD, dpll::getStreamPeriodMs());
        break;
      case OPCODE_GET_EEPROM_STATUS:
      {
        uint8_t st = eeprom::isConnected() ? 1 : 0;
        com::sendPacket(OPCODE_GET_EEPROM_STATUS, 0, &st, 1);
        break;
      }
      case OPCODE_SAVE_CONFIG_EEPROM:
      {
        bool ok = eeprom_config::saveCurrentConfig();
        uint8_t st = ok ? 1 : 0;
        com::sendPacket(OPCODE_SAVE_CONFIG_EEPROM, 0, &st, 1);
        DebugPort.printf("[EEPROM] Save configuration: %s\n", ok ? "OK" : "FAILED");
        break;
      }
      case OPCODE_LOAD_CONFIG_EEPROM:
      {
        bool ok = eeprom_config::loadSavedConfig();
        uint8_t st = ok ? 1 : 0;
        com::sendPacket(OPCODE_LOAD_CONFIG_EEPROM, 0, &st, 1);
        DebugPort.printf("[EEPROM] Load configuration: %s\n", ok ? "OK" : "FAILED");
        break;
      }
      case OPCODE_RESET_CONFIG_EEPROM:
      {
        bool ok = eeprom_config::resetConfig();
        uint8_t st = ok ? 1 : 0;
        com::sendPacket(OPCODE_RESET_CONFIG_EEPROM, 0, &st, 1);
        DebugPort.printf("[EEPROM] Reset configuration: %s\n", ok ? "OK" : "FAILED");
        break;
      }
      case OPCODE_SET_GENERATOR_SOURCE:
        if (plen >= 1)
        {
          uint8_t src = pl[0];
          if (src <= 1)
          {
            dpll::setGeneratorSource((dpll::GeneratorSource)src);
            DebugPort.printf("[DPLL] Generator source set to: %s\n", src == 1 ? "DDS (AD9833)" : "DAC (CD4046)");
          }
        }
        break;
      case OPCODE_GET_GENERATOR_SOURCE:
      {
        uint8_t src = (uint8_t)dpll::getGeneratorSource();
        com::sendPacket(OPCODE_GET_GENERATOR_SOURCE, 0, &src, 1);
        break;
      }
      case OPCODE_SET_DDS_FREQ_BOUNDS:
        if (plen >= 8)
        {
          float minHz, maxHz;
          memcpy(&minHz, pl, 4);
          memcpy(&maxHz, pl + 4, 4);
          dpll::setDdsFreqLimits(minHz, maxHz);
          DebugPort.printf("[DPLL] DDS bounds set: %.1f - %.1f Hz\n", minHz, maxHz);
        }
        break;
      case OPCODE_GET_DDS_FREQ_BOUNDS:
      {
        float bounds[2] = { dpll::getDdsMinFreq(), dpll::getDdsMaxFreq() };
        com::sendPacket(OPCODE_GET_DDS_FREQ_BOUNDS, 0, (const uint8_t *)bounds, sizeof(bounds));
        break;
      }
      case OPCODE_SET_DDS_CENTER_FREQ:
        if (plen >= 4)
        {
          dpll::setDdsCenterFreq(payloadFloat());
          DebugPort.printf("[DPLL] DDS center freq set: %.1f Hz\n", dpll::getDdsCenterFreq());
        }
        break;
      case OPCODE_GET_DDS_CENTER_FREQ:
        sendFloat(OPCODE_GET_DDS_CENTER_FREQ, dpll::getDdsCenterFreq());
        break;
      case OPCODE_SET_DDS_MANUAL_FREQ:
        if (plen >= 4)
        {
          float f = payloadFloat();
          if (f >= 1.0f && f <= 500000.0f)
          {
            dpll::setManualMode(true);
            dpll::manualSetFreq(f);
            DebugPort.printf("[DPLL] Manual DDS frequency set: %.2f Hz\n", f);
          }
        }
        break;
      case OPCODE_GET_DDS_MANUAL_FREQ:
        sendFloat(OPCODE_GET_DDS_MANUAL_FREQ, dpll::lastFrequency());
        break;
      case OPCODE_SET_DDS_MCO_CLOCK:
        if (plen >= 1)
        {
          uint8_t clkSel = pl[0];
          if (dds::setMcoClock((dds::McoClockSelection)clkSel))
          {
            DebugPort.printf("[DDS] MCO Clock set to: %s (%u Hz, Res: %.6f Hz)\n",
                             dds::getMcoClockName(), dds::getMclkFreq(), dds::getResolution());
          }
        }
        break;
      case OPCODE_GET_DDS_MCO_CLOCK:
      {
        uint8_t clkSel = (uint8_t)dds::getMcoClock();
        com::sendPacket(OPCODE_GET_DDS_MCO_CLOCK, 0, &clkSel, 1);
        break;
      }
      case OPCODE_SET_CAPTURE_EDGE:
        if (plen >= 2)
        {
          uint8_t refE = pl[0];
          uint8_t zcdE = pl[1];
          if (refE <= 2) phase_capture::setRefEdge((phase_capture::EdgePolarity)refE);
          if (zcdE <= 2) phase_capture::setZcdEdge((phase_capture::EdgePolarity)zcdE);
          DebugPort.printf("[CAPTURE] Edge polarity set: REF=%s, ZCD=%s\n",
                           refE == 1 ? "FALLING" : (refE == 2 ? "BOTH" : "RISING"),
                           zcdE == 1 ? "FALLING" : (zcdE == 2 ? "BOTH" : "RISING"));
        }
        break;
      case OPCODE_GET_CAPTURE_EDGE:
      {
        uint8_t edges[2] = {
          (uint8_t)phase_capture::getRefEdge(),
          (uint8_t)phase_capture::getZcdEdge()
        };
        com::sendPacket(OPCODE_GET_CAPTURE_EDGE, 0, edges, sizeof(edges));
        break;
      }
      case OPCODE_SET_TRACKING_DIR:
        if (plen >= 1)
        {
          uint8_t dir = pl[0];
          if (dir <= 1)
          {
            dpll::setTrackingDirection((dpll::TrackingDirection)dir);
            DebugPort.printf("[DPLL] Tracking direction set to: %s\n",
                             dir == 1 ? "INVERTED (Parallel fp)" : "NORMAL (Series fs)");
          }
        }
        break;
      case OPCODE_GET_TRACKING_DIR:
      {
        uint8_t dir = (uint8_t)dpll::getTrackingDirection();
        com::sendPacket(OPCODE_GET_TRACKING_DIR, 0, &dir, 1);
        break;
      }
    }
  }
}
void debugPrint()
{
  // --- Status print (500 ms) ---
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 500)
  {
    lastPrint = millis();

    phase_capture::CaptureData data = phase_capture::getData();
    const char *manual = dpll::getManualMode() ? " [MANUAL]" : "";
    if (data.valid)
    {
      float absPhase = fabsf(data.phaseDiffNs - dpll::getTargetPhase());
      const char *state = (absPhase <= dpll::getLockThresholdNs()) ? "LOCK" : "TRACK";
      DebugPort.printf("%s%s | Freq: %.2f Hz | Phase: %.1f ns | Period: %.2f us | DAC: %.2f V\n",
                    state, manual,
                    data.frequencyHz,
                    data.phaseDiffNs,
                    phase_capture::ticksToUs(data.periodTicks),
                    dac::lastRaw() * (3.3f / 4095.0f));
    }
    else if (data.refValid && !data.zcdPresent)
    {
      DebugPort.printf("WAIT ZCD%s | REF Freq: %.2f Hz | Power Enable OFF / ZCD missing | DAC: %.2f V\n",
                    manual,
                    data.frequencyHz,
                    dac::lastRaw() * (3.3f / 4095.0f));
    }
    else
    {
      DebugPort.printf("NO REF SIGNAL%s | Waiting for generator input on PA0... | DAC: %.2f V\n",
                    manual,
                    dac::lastRaw() * (3.3f / 4095.0f));
    }
  }
}
// State machine states for DPLL controller
enum DpllState : uint8_t {
  STATE_NO_SIGNAL = 0,
  STATE_ACQUIRING,
  STATE_TRACKING,
  STATE_LOCKED
};
static DpllState s_dpllState = STATE_NO_SIGNAL;
static phase_capture::CaptureData s_latestCaptureData{};

// Fixed-rate monitor stream — runs independently of the settling window.
// Rate is set by dpll::getStreamPeriodMs() (default 100 ms = 10 Hz).
// Sends a packet for ALL states: LOCK, TRACK, WAIT_ZCD, NO_REF.
void sendMonitorStream()
{
  if (!com::GetAllowSendStream()) return;

  static uint32_t lastStream = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastStream < dpll::getStreamPeriodMs()) return;
  lastStream = nowMs;

  phase_capture::CaptureData data = s_latestCaptureData;
  if (data.periodTicks == 0U) {
    data = phase_capture::getData();
  }
  float currentDacV = dac::lastRaw() * (3.3f / 4095.0f);

  const bool isSignalActive = (s_dpllState != STATE_NO_SIGNAL);
  const bool isPhaseValid   = isSignalActive && data.valid;

  // Phase value reported while ZCD/REF is absent.
  // Updated ONLY while signal is actively being tracked or locked.
  static float s_holdPhaseNs = 0.0f;
  if (isPhaseValid) {
    s_holdPhaseNs = data.phaseDiffNs;
  }

  dpllStatusData status;
  // LockStatus: 0=NO_REF, 1=WAIT_ZCD, 2=TRACK, 3=LOCK, 4=MANUAL/OPEN_LOOP
  if (dpll::getManualMode()) {
    status.LockStatus = 4;
  } else if (!data.refValid) {
    status.LockStatus = 0;
  } else if (!isSignalActive || !data.zcdPresent) {
    status.LockStatus = 1;
  } else if (s_dpllState == STATE_LOCKED) {
    status.LockStatus = 3;
  } else {
    status.LockStatus = 2;
  }
  status.ReferenceFrequencyHz = data.frequencyHz;
  status.PhaseError_ns        = isPhaseValid ? data.phaseDiffNs : s_holdPhaseNs;
  status.PhaseStale           = isPhaseValid ? 0 : 1;
  status.DACVoltage_V         = currentDacV;

  static uint32_t s_lastEepromCheck = 0;
  static uint8_t s_eepromStatus = 0;
  if (nowMs - s_lastEepromCheck >= 1000U || s_lastEepromCheck == 0)
  {
    s_lastEepromCheck = nowMs;
    s_eepromStatus = eeprom::isConnected() ? 1 : 0;
  }
  status.EepromStatus = s_eepromStatus;
  uint8_t ddsSt = (uint8_t)dds::getStatus(data.refValid ? data.frequencyHz : 0.0f);
  if (dpll::getGeneratorSource() == dpll::GENERATOR_SOURCE_DDS)
  {
    ddsSt |= 0x80; // Bit 7 set indicates DDS is active generator source
  }
  if (dpll::getTrackingDirection() == dpll::TRACKING_DIR_INVERTED)
  {
    ddsSt |= 0x40; // Bit 6 set indicates tracking direction is INVERTED (diagnostic)
  }
  status.DdsStatus = ddsSt;

  com::sendPacket(OPCODE_STREAM_DPLL_STATUS, 0, (uint8_t *)&status, sizeof(status));
}
// ---------------------------------------------------------------------------
// Lock history. The last windows of every sonication pulse are garbage (ZCD
// dies, phase swings by microseconds). Tracking the "locked centre" with the
// latest sample therefore stored a corrupted DAC value that was reused as the
// restart point of the next pulse. We keep a short history of trusted LOCKED
// outputs and, on signal loss, commit a value that is at least
// kLockHistGuardMs older than the newest sample.
// ---------------------------------------------------------------------------
namespace {
constexpr uint8_t  kLockHistSize    = 32;
constexpr uint32_t kLockHistGuardMs = 20;
struct LockSample { uint32_t ms; float v; };
LockSample s_lockHist[kLockHistSize];
uint8_t    s_lockHistCount = 0;
uint8_t    s_lockHistHead  = 0; // next write index

void lockHistClear() { s_lockHistCount = 0; s_lockHistHead = 0; }

void lockHistPush(uint32_t ms, float v)
{
  s_lockHist[s_lockHistHead] = { ms, v };
  s_lockHistHead = (uint8_t)((s_lockHistHead + 1) % kLockHistSize);
  if (s_lockHistCount < kLockHistSize) s_lockHistCount++;
}

// Pick newest entry that is >= guard older than the newest one; fall back to
// the oldest stored entry. Requires at least 3 samples to avoid single-cycle glitches.
bool lockHistPick(float &v)
{
  if (s_lockHistCount < 3) return false;
  const uint8_t newestIdx = (uint8_t)((s_lockHistHead + kLockHistSize - 1) % kLockHistSize);
  const uint32_t newestMs = s_lockHist[newestIdx].ms;
  uint8_t idx = newestIdx;
  for (uint8_t i = 0; i < s_lockHistCount; i++) {
    idx = (uint8_t)((newestIdx + kLockHistSize - i) % kLockHistSize);
    if ((uint32_t)(newestMs - s_lockHist[idx].ms) >= kLockHistGuardMs) {
      v = s_lockHist[idx].v;
      return true;
    }
  }
  v = s_lockHist[idx].v; // oldest entry
  return true;
}
} // namespace

void processDPLL()
{
  // --- DPLL State Machine & Control Loop ---
  // Sequence per cycle:
  //   1. Wait loopPeriodMs for system & transducer settling.
  //   2. Read fresh phase data from phase_capture.
  //   3. Handle state transitions: NO_SIGNAL -> ACQUIRING -> TRACKING -> LOCKED.
  //   4. Execute PI controller update and apply DAC voltage.
  //   5. Record timestamp for next settling window.

  static uint32_t lastDacUpdate = 0;
  static bool firstRun = true;
  static uint32_t lockHoldCount = 0;
  static uint32_t unlockHoldCount = 0;
  static uint32_t railClampCount = 0;
  static uint32_t trackCycles = 0;
  static uint32_t s_lastLockMs = 0;
  // Number of windows to ignore right after a pulse starts. Allows acoustic
  // ring-up under viscous / swab loading to settle before active PI steering.
  static uint32_t settleCycles = 0;
  static uint8_t  glitchSkipCount = 0;
  // Lock-independent protection / park bookkeeping.
  static float    s_lastGoodPhase = 0.0f; // last window phase accepted into the PI
  static uint8_t  s_goodCount = 0;        // accepted PI updates in this pulse
  static float    s_parkV = 0.0f;         // clean end-of-pulse locked output
  static bool     s_haveParkV = false;
  static bool     s_lockedThisPulse = false;
  static uint8_t  lockMissCount = 0;
  // Lock needs a few in-window cycles only; large configured hold values
  // (EEPROM default 10 = 200 ms) never completed within a 450 ms pulse.
  constexpr uint32_t kMaxEffectiveHold = 5;

  // Restart / park value: clean steady locked centre, else clean end-of-pulse
  // value, else the supplied fallback. All candidate values are bounded by a
  // strict sanity check against calibrated center to prevent false-lock traps.
  auto restartValue = [&](float fallbackV) -> float {
    float candidate = fallbackV;
    if (dpll::haveLockedCenter()) {
      candidate = dpll::getLockedCenterV();
    } else if (s_haveParkV) {
      candidate = s_parkV;
    }

    if (dpll::getGeneratorSource() == dpll::GENERATOR_SOURCE_DDS) {
      const float centerF = dpll::getDdsCenterFreq();
      if (candidate <= 0.0f || fabsf(candidate - centerF) > 1500.0f) {
        return centerF;
      }
    } else {
      const float centerV = dpll::getCenterVoltage();
      if (candidate < 1.50f || fabsf(candidate - centerV) > 0.22f) {
        return centerV;
      }
    }
    return candidate;
  };

  uint32_t nowMs = millis();

  if (firstRun) {
    phase_capture::clearWindowData();
    lastDacUpdate = nowMs;
    firstRun = false;
    return;
  }

  // Wait until settling window expires
  if (nowMs - lastDacUpdate < dpll::getLoopPeriodMs()) {
    return;
  }

  // Use the actual elapsed time rather than assuming the scheduler hit the
  // configured period exactly. This keeps I/D scaling correct under jitter.
  float dt = static_cast<float>(nowMs - lastDacUpdate) * 0.001f;
  phase_capture::CaptureData data = phase_capture::getData();
  s_latestCaptureData = data;
  phase_capture::PhaseWindowData window = phase_capture::consumeWindowData();
  const float kLockThr = dpll::getLockThresholdNs();

  // Manual override: loop disengaged
  if (dpll::getManualMode()) {
    digitalWrite(LOCK_INDICATOR_LED, LOW);
    phase_capture::clearWindowData();
    lastDacUpdate = millis();
    return;
  }

  // Check signal presence
  if (!data.valid) {
    const bool wasActive = (s_dpllState != STATE_NO_SIGNAL);
    s_dpllState = STATE_NO_SIGNAL;
    glitchSkipCount = 0;
    if (wasActive) {
      // Commit a clean locked centre (excludes the pulse-end transient).
      float cleanV;
      if (lockHistPick(cleanV)) {
        s_parkV = cleanV;
        s_haveParkV = true;
        if (s_lockedThisPulse) {
          dpll::setLockedCenter(cleanV);
        }
      }
      lockHistClear();
    }
    s_goodCount = 0;
    lockMissCount = 0;
    s_lockedThisPulse = false;
    if (nowMs - s_lastLockMs >= 500U) {
      digitalWrite(LOCK_INDICATOR_LED, LOW);
    }
    lockHoldCount = 0;
    unlockHoldCount = 0;
    railClampCount = 0;
    trackCycles = 0;

    switch (dpll::getSignalLossBehavior())
    {
      case dpll::SIGNAL_LOSS_CENTER:
        dpll::reset();       // DAC -> centerVoltage, clear integral
        dpll::enable(false);
        break;
      case dpll::SIGNAL_LOSS_ZERO:
        dpll::shutdown();    // DAC -> 0 V
        break;
      case dpll::SIGNAL_LOSS_FREEZE:
      default:
        if (s_haveStartV || dpll::haveLockedCenter() || s_haveParkV) {
          // Park at the cold-start point so the next pulse begins near lock.
          const float parkV = restartValue(0.0f);
          if (dpll::getGeneratorSource() == dpll::GENERATOR_SOURCE_DDS) {
            dpll::restartAtFrequency(parkV);
          } else {
            dpll::restartAtVoltage(parkV);
          }
        }
        dpll::enable(false); // Hold last locked voltage / frequency
        break;
    }
    dpll::tickSignalAbsent(nowMs);
    phase_capture::clearWindowData();
    phase_capture::resetFilterState(); // Prepare filter for immediate acceptance on next pulse
    lastDacUpdate = millis();
    return;
  }

  // Signal is valid
  dpll::tickSignalAbsent(0); // Signal present

  // State Transition & Re-acquisition logic
  if (s_dpllState == STATE_NO_SIGNAL) {
    if (dpll::getSignalLossBehavior() == dpll::SIGNAL_LOSS_CENTER) {
      dpll::reset();
      DebugPort.printf("[DPLL] Re-acquire: reset to center\n");
    } else {
      // FREEZE / Resume: start exactly from the locked center point
      if (dpll::getGeneratorSource() == dpll::GENERATOR_SOURCE_DDS) {
        float resumeF = restartValue(dpll::lastFrequency());
        dpll::restartAtFrequency(resumeF);
        DebugPort.printf("[DPLL] Re-acquire: resume at Freq: %.1f Hz\n", resumeF);
      } else {
        float resumeV = restartValue(dpll::lastVoltage());
        dpll::restartAtVoltage(resumeV);
        DebugPort.printf("[DPLL] Re-acquire: resume at %.3f V (Freq: %.1f Hz)\n", resumeV, data.frequencyHz);
      }
    }
    settleCycles = 0;
    s_firstLockPending = true;
    glitchSkipCount = 0;
    s_goodCount = 0;
    lockMissCount = 0;
    s_lockedThisPulse = false;
    lockHistClear();
    s_dpllState = STATE_TRACKING;
    lockHoldCount = 0;
    unlockHoldCount = 0;
    railClampCount = 0;
    trackCycles = 0;
    phase_capture::resetFilterState();
  }

  // Ignore the start-up transient windows of each pulse (no PID update).
  if (settleCycles < 2) {
    settleCycles++;
    phase_capture::clearWindowData();
    lastDacUpdate = millis();
    return;
  }

  // Window confidence filtering:
  // Bypass window rejection at the start of a pulse (trackCycles < 2) so the DPLL
  // immediately begins tracking without wasting 40-60 ms of a 450 ms sonication pulse.
  bool pulseStart = (trackCycles < 2);
  float filteredPhaseNs = window.phaseDiffNs;
  bool usedFallback = false;

  if (!window.trustworthy) {
    // The strict filter rejected this window. Previously this froze the loop
    // for the whole sonication pulse (the frequency never moved). Instead fall
    // back to the best available estimate, bounded to a sane range, so the
    // PID keeps tracking. Lock is never declared from a fallback window.
    const float halfPeriodNs = (data.frequencyHz > 1.0f) ? (0.5e9f / data.frequencyHz) : 15000.0f;
    float fb = window.phaseComputed ? window.phaseDiffNs : data.phaseDiffNs;
    const bool usable = !window.overflow && (window.sampleCount >= 2) &&
                        fabsf(fb) < (0.8f * halfPeriodNs);
    static uint32_t lastFilterLogMs = 0;
    if (nowMs - lastFilterLogMs >= 500U) {
      lastFilterLogMs = nowMs;
      DebugPort.printf("[DPLL FILTER FALLBACK] N=%u accepted=%u outlier=%.1f%% noise=%.2f deg slope=%.2f deg/ms%s%s phase=%.0f ns %s\n",
                       window.sampleCount,
                       window.acceptedCount,
                       window.outlierRatio * 100.0f,
                       window.noiseMadDeg,
                       window.slopeDegPerMs,
                       window.largeShift ? " LARGE_SHIFT" : "",
                       window.overflow ? " OVERFLOW" : "",
                       fb,
                       usable ? "USED" : "SKIPPED");
    }
    if (!usable) {
      if (s_dpllState == STATE_LOCKED) {
        unlockHoldCount++;
        if (unlockHoldCount >= 3) {
          s_dpllState = STATE_TRACKING;
          if (nowMs - s_lastLockMs >= 500U) {
            digitalWrite(LOCK_INDICATOR_LED, LOW);
          }
          lockHoldCount = 0;
        }
      }
      // A skipped window neither counts towards nor resets the lock hold.
      phase_capture::clearWindowData();
      lastDacUpdate = millis();
      return;
    }
    filteredPhaseNs = fb;
    usedFallback = true;
  }
  (void)pulseStart;

  // Pulse-end glitch guard: shortly after lock, a phase jump of several
  // microseconds is never real (locked sigma ~ tens of ns); it is the dying
  // ZCD at the end of a sonication pulse. Do not let it move the DAC.
  // A persistent step (>= 4 cycles) is accepted as real.
  {
    // Independent of lock state: compare with the last accepted window
    // (or the target while no history exists in this pulse).
    const bool haveGood = (s_goodCount >= 2);
    const float kJumpThr = fmaxf(1.5f * kLockThr, 1000.0f);
    bool glitch = false;
    if (haveGood) {
      glitch = fabsf(filteredPhaseNs - s_lastGoodPhase) > kJumpThr;
    } else {
      // Pulse start: an untrusted window far from target is the transducer
      // start-up transient, not a usable error.
      glitch = usedFallback &&
               fabsf(filteredPhaseNs - dpll::getTargetPhase()) > fmaxf(4.0f * kLockThr, 2500.0f);
    }
    if (glitch && glitchSkipCount < 4) {
      glitchSkipCount++;
      phase_capture::clearWindowData();
      lastDacUpdate = millis();
      return;
    }
    glitchSkipCount = 0;
  }

  // Run PI controller update (Continuous Incremental Tracker)
  dpll::enable(true);
  float controlOutput = dpll::update(filteredPhaseNs, dt);
  trackCycles++;
  s_lastGoodPhase = filteredPhaseNs;
  if (s_goodCount < 255) s_goodCount++;
  lockHistPush(nowMs, controlOutput); // clean-output history, any state

  // Check if controller is pinned to rails:
  // In DAC mode: check 0.05 V - 3.25 V
  // In DDS mode: check within 5 Hz of [minFreq, maxFreq]
  bool isRailed = false;
  if (dpll::getGeneratorSource() == dpll::GENERATOR_SOURCE_DDS) {
    float currF = controlOutput;
    float minF  = dpll::getDdsMinFreq();
    float maxF  = dpll::getDdsMaxFreq();
    if (currF <= (minF + 5.0f) || currF >= (maxF - 5.0f)) {
      isRailed = true;
    }
  } else {
    float currV = dac::lastRaw() * (3.3f / 4095.0f);
    if (currV <= 0.05f || currV >= 3.25f) {
      isRailed = true;
    }
  }

  if (isRailed) {
    railClampCount++;
  } else {
    railClampCount = 0;
  }

  // Rail Clamping: dpll::update() already clamps within limits with anti-windup.
  // Do NOT reset to center mid-way — allow continuous smooth tracking!
  static uint32_t lastRailWarnMs = 0;
  if (isRailed) {
    if (nowMs - lastRailWarnMs >= 1000U) {
      lastRailWarnMs = nowMs;
      DebugPort.printf("[DPLL RAIL] Pinned to %s rail limit!\n",
                       dpll::getGeneratorSource() == dpll::GENERATOR_SOURCE_DDS ? "DDS Frequency" : "DAC Voltage");
    }
  }

  // Check Lock Condition with hysteresis / debounce
  const float phaseErr = fabsf(filteredPhaseNs - dpll::getTargetPhase());
  const bool withinLockWindow = (phaseErr <= kLockThr);

  if (s_dpllState == STATE_LOCKED) {
    // Already LOCKED: require 3 consecutive cycles outside threshold before dropping lock.
    // Isolated single-cycle noise glitches or fallback windows do NOT cause lock loss or LED flicker.
    // Hysteresis: once locked, stay locked until error exceeds 2x lock threshold.
    const bool withinKeepWindow = (phaseErr <= kLockThr * 2.0f);
    if (!withinKeepWindow && (!usedFallback || phaseErr > (kLockThr * 3.0f))) {
      unlockHoldCount++;
      if (unlockHoldCount >= 3) {
        s_dpllState = STATE_TRACKING;
        lockHoldCount = 0;
        unlockHoldCount = 0;
        DebugPort.printf("[DPLL UNLOCKED] Phase Error = %.1f ns (target=%.1f, thr=%.1f)\n",
                         filteredPhaseNs, dpll::getTargetPhase(), kLockThr);
      }
    } else {
      unlockHoldCount = 0;
    }
  } else {
    // Currently TRACKING: require lockHoldCycles consecutive cycles within lock threshold
    unlockHoldCount = 0;
    if (withinLockWindow) {
      lockMissCount = 0;
      lockHoldCount++;
      const uint32_t holdNeed = dpll::getLockHoldCycles() < kMaxEffectiveHold
                              ? dpll::getLockHoldCycles() : kMaxEffectiveHold;
      if (lockHoldCount >= holdNeed) {
        s_dpllState = STATE_LOCKED;
        s_lockedThisPulse = true;
        if (dpll::getGeneratorSource() == dpll::GENERATOR_SOURCE_DDS) {
          DebugPort.printf("[DPLL LOCKED] Freq = %.1f Hz | Phase = %.1f ns\n",
                           controlOutput, filteredPhaseNs);
        } else {
          DebugPort.printf("[DPLL LOCKED] DAC = %.3f V | Freq = %.1f Hz | Phase = %.1f ns\n",
                           controlOutput, data.frequencyHz, filteredPhaseNs);
        }
        dpll::setLockedCenter(controlOutput);
        if (s_firstLockPending) {
          s_pulseStartV = controlOutput;
          s_haveStartV = true;
          s_firstLockPending = false;
        }
      }
    } else {
      // Tolerate one isolated out-of-window cycle.
      if (++lockMissCount >= 2) {
        lockHoldCount = 0;
        lockMissCount = 0;
      }
    }
  }

  {
    static uint32_t lastDiagMs = 0;
    if (nowMs - lastDiagMs >= 500U) {
      lastDiagMs = nowMs;
      DebugPort.printf("[DPLL DIAG] win=%.0f ns tgt=%.0f thr=%.0f hold=%u/%u(cfg %u) %s%s\n",
                       filteredPhaseNs, dpll::getTargetPhase(), kLockThr,
                       (unsigned)lockHoldCount, (unsigned)kMaxEffectiveHold,
                       (unsigned)dpll::getLockHoldCycles(),
                       s_dpllState == STATE_LOCKED ? "LOCKED" : "TRACK",
                       usedFallback ? " fallback" : "");
    }
  }

  // Update hardware lock indicator LED (active high with 500 ms pulse-stretcher)
  if (s_dpllState == STATE_LOCKED) {
    s_lastLockMs = nowMs;
    digitalWrite(LOCK_INDICATOR_LED, HIGH);
  } else if (nowMs - s_lastLockMs >= 500U) {
    digitalWrite(LOCK_INDICATOR_LED, LOW);
  }

  phase_capture::clearWindowData();
  lastDacUpdate = millis();
}
// Parse and execute one full command line (already trimmed).
void handleDebugCommand(const String &cmd)
{
  int sp = cmd.indexOf(' ');
  String name = (sp < 0) ? cmd : cmd.substring(0, sp);
  String arg = (sp < 0) ? "" : cmd.substring(sp + 1);
  name.trim();
  arg.trim();

  if (name == "help" || name == "?")
  {
    DebugPort.println(F("Commands:"));
    DebugPort.println(F("  dac <volt>    : manual DAC voltage (0.0-3.3), disables loop"));
    DebugPort.println(F("  kp <val>      : set proportional gain (V/ns)"));
    DebugPort.println(F("  ki <val>      : set integral gain (V/ns/s)"));
    DebugPort.println(F("  kd <val>      : set derivative gain (V/ns/s)"));
    DebugPort.println(F("  center <volt> : set center voltage"));
    DebugPort.println(F("  target <ns>   : set lock delay (ns)"));
    DebugPort.println(F("  slew <V/s>    : set max DAC slew rate (V/s)"));
    DebugPort.println(F("  loop <ms>     : set control loop period (ms)"));
    DebugPort.println(F("  gain          : show current gains"));
    DebugPort.println(F("  reset         : clear integrator, restart from center"));
    DebugPort.println(F("  run           : re-enable control loop"));
    DebugPort.println(F("  loss <0|1|2>  : signal-loss DAC behaviour: 0=freeze 1=center 2=zero"));
    DebugPort.println(F("  timeout <ms>  : lock memory expiry (ms, 0=never). Default 5000"));
    DebugPort.println(F("  eeprom        : status, test, save, load, reset, read <addr>, write <addr> <val>, dump <addr> [len]"));
    DebugPort.println(F("  dds           : status, freq <Hz>, clock [idx|preset], on, off, wave <sine|triangle|square>"));
    DebugPort.println(F("  mco [preset]  : select DDS MCLK clock preset (resolution & stability: 8mhz, 4mhz, 2mhz, 1.6mhz, 16mhz)"));
    DebugPort.println(F("  source <dac|dds> : select generator source (CD4046 VCO vs AD9833 DDS)"));
    DebugPort.println(F("  ddsfreq <hz>  : set DDS center frequency"));
    DebugPort.println(F("  ddsrange <min> <max> : set DDS tracking limits (Hz)"));
    DebugPort.println(F("  edge [ref 0..2] [zcd 0..2] : set capture edge polarity (0=rising, 1=falling, 2=both)"));
  }
  else if (name == "dac")
  {
    if (arg.length() == 0)
    {
      DebugPort.printf("DAC now: %.3f V\n", dac::lastRaw() * (3.3f / 4095.0f));
      return;
    }
    float v = arg.toFloat();
    if (v >= 0.0f && v <= 3.3f)
    {
      dpll::setManualMode(true);
      dpll::manualSet(v);
      DebugPort.printf("DAC set to %.2f V (raw %u) - loop disabled\n", v, dac::lastRaw());
    }
    else
    {
      DebugPort.println("ERR: Voltage out of range (0.0 - 3.3 V)");
    }
  }
  else if (name == "kp")
  {
    float v = arg.toFloat();
    dpll::setGains(v, dpll::getKi());
    DebugPort.printf("Kp = %.5f V/ns\n", v);
  }
  else if (name == "ki")
  {
    float v = arg.toFloat();
    dpll::setGains(dpll::getKp(), v);
    DebugPort.printf("Ki = %.5f V/ns/s\n", v);
  }
  else if (name == "kd")
  {
    float v = arg.toFloat();
    dpll::setKd(v);
    DebugPort.printf("Kd = %.5f V/ns/s\n", v);
  }
  else if (name == "center")
  {
    float v = arg.toFloat();
    if (v >= 0.0f && v <= 3.3f)
    {
      dpll::begin(v, dpll::getKp(), dpll::getKi(), dpll::getKd());
      DebugPort.printf("Center = %.2f V\n", v);
    }
    else
    {
      DebugPort.println("ERR: out of range (0.0 - 3.3 V)");
    }
  }
  else if (name == "target")
  {
    float v = arg.toFloat();
    dpll::setTargetPhase(v);
    DebugPort.printf("Target phase = %.1f ns\n", v);
  }
  else if (name == "gain")
  {
    DebugPort.printf("Kp=%.6f V/ns | Ki=%.6f V/ns/s | Kd=%.6f V/ns/s | center=%.2f V | target=%.1f ns | dir=%s | slew=%.1f V/s | manual=%s | loop=%u ms | thr=%.0f ns | hold=%u | timeout=%u ms | stream=%u ms | lockedV=%.3f V%s | loss=%u\n",
                  dpll::getKp(), dpll::getKi(), dpll::getKd(), dpll::getCenterVoltage(),
                  dpll::getTargetPhase(),
                  dpll::getTrackingDirection() == dpll::TRACKING_DIR_INVERTED ? "INVERTED" : "NORMAL",
                  dpll::getMaxSlew(), dpll::getManualMode() ? "yes" : "no",
                  dpll::getLoopPeriodMs(), dpll::getLockThresholdNs(),
                  dpll::getLockHoldCycles(), dpll::getLockMemoryTimeoutMs(),
                  dpll::getStreamPeriodMs(),
                  dpll::getLockedCenterV(),
                  dpll::haveLockedCenter() ? "" : " (default)",
                  (uint8_t)dpll::getSignalLossBehavior());
  }
  else if (name == "slew")
  {
    float v = arg.toFloat();
    if (v > 0.0f)
    {
      dpll::setMaxSlew(v);
      DebugPort.printf("Max slew = %.1f V/s\n", v);
    }
    else
    {
      DebugPort.println("ERR: slew must be > 0 (V/s)");
    }
  }
  else if (name == "loop")
  {
    int v = arg.toInt();
    if (v >= 1 && v <= 1000)
    {
      dpll::setLoopPeriodMs((uint32_t)v);
      DebugPort.printf("Loop period = %u ms\n", dpll::getLoopPeriodMs());
    }
    else
    {
      DebugPort.println("ERR: loop period must be 1-1000 ms");
    }
  }
  else if (name == "reset")
  {
    dpll::setManualMode(false);
    dpll::reset();
    dpll::enable(true);
    DebugPort.println("Controller reset to center voltage, loop enabled");
  }
  else if (name == "run")
  {
    dpll::setManualMode(false);
    dpll::enable(true);
    DebugPort.println("Control loop enabled (closed-loop DPLL tracking active)");
  }
  else if (name == "stop" || name == "manual")
  {
    dpll::setManualMode(true);
    dpll::enable(false);
    DebugPort.println("Control loop disabled (open-loop fixed test mode active)");
  }
  else if (name == "fixed" || name == "test")
  {
    float f = arg.toFloat();
    if (f >= 1.0f && f <= 500000.0f)
    {
      dpll::manualSetFreq(f);
      DebugPort.printf("Fixed test frequency set to %.2f Hz (loop disabled)\n", f);
    }
    else
    {
      DebugPort.println("ERR: Frequency must be 1.0 - 500000.0 Hz");
    }
  }
  else if (name == "loss")
  {
    int v = arg.toInt();
    if (v >= 0 && v <= 2)
    {
      dpll::setSignalLossBehavior((dpll::SignalLossBehavior)v);
      const char* names[] = {"freeze", "center", "zero"};
      DebugPort.printf("Signal-loss behaviour = %s\n", names[v]);
    }
    else
    {
      DebugPort.println("ERR: loss must be 0=freeze 1=center 2=zero");
    }
  }
  else if (name == "timeout")
  {
    if (arg.length() == 0)
    {
      DebugPort.printf("Lock memory timeout = %u ms%s\n",
                       dpll::getLockMemoryTimeoutMs(),
                       dpll::getLockMemoryTimeoutMs() == 0 ? " (never expire)" : "");
    }
    else
    {
      uint32_t v = (uint32_t)arg.toInt();
      dpll::setLockMemoryTimeoutMs(v);
      DebugPort.printf("Lock memory timeout = %u ms%s\n", v,
                       v == 0 ? " (never expire)" : "");
    }
  }
  else if (name == "led")
  {
    if (arg == "on" || arg == "1")
    {
      digitalWrite(LOCK_INDICATOR_LED, HIGH);
      DebugPort.println(F("LOCK LED (PC10) set to HIGH (ON)"));
    }
    else if (arg == "off" || arg == "0")
    {
      digitalWrite(LOCK_INDICATOR_LED, LOW);
      DebugPort.println(F("LOCK LED (PC10) set to LOW (OFF)"));
    }
    else
    {
      DebugPort.println(F("Usage: led on | led off"));
    }
  }
  else if (name == "eeprom")
  {
    int subSp = arg.indexOf(' ');
    String subCmd = (subSp < 0) ? arg : arg.substring(0, subSp);
    String subArg = (subSp < 0) ? "" : arg.substring(subSp + 1);
    subCmd.trim();
    subArg.trim();

    if (subCmd == "status" || subCmd == "")
    {
      bool connected = eeprom::isConnected();
      DebugPort.printf("EEPROM 24LC16B: %s (Capacity: %u bytes, Page: %u bytes, SCL=PB6, SDA=PB7)\n",
                       connected ? "Connected (ACK OK)" : "Disconnected / No ACK",
                       eeprom::CAPACITY_BYTES, eeprom::PAGE_SIZE);
      bool hasCfg = eeprom_config::hasValidConfig();
      DebugPort.printf("Config Status : %s\n", hasCfg ? "VALID (Saved in EEPROM)" : "DEFAULT (No valid saved config)");
      DebugPort.printf("MCO Clock Set : %s (%u Hz, Res: %.6f Hz)\n",
                       dds::getMcoClockName(), dds::getMclkFreq(), dds::getResolution());
    }
    else if (subCmd == "test")
    {
      DebugPort.println(F("Running EEPROM self-test (non-destructive)..."));
      if (eeprom::selfTest())
      {
        DebugPort.println(F("EEPROM self-test PASSED!"));
      }
      else
      {
        DebugPort.println(F("EEPROM self-test FAILED! Check wiring, pull-ups, or address."));
      }
    }
    else if (subCmd == "read")
    {
      if (subArg.length() == 0)
      {
        DebugPort.println(F("Usage: eeprom read <addr (0-2047)>"));
      }
      else
      {
        uint16_t addr = (uint16_t)subArg.toInt();
        if (addr >= eeprom::CAPACITY_BYTES)
        {
          DebugPort.println(F("ERR: Address out of range (0-2047)"));
        }
        else
        {
          bool ok = false;
          uint8_t val = eeprom::readByte(addr, &ok);
          if (ok)
          {
            DebugPort.printf("EEPROM[0x%04X (%u)] = 0x%02X (%u)\n", addr, addr, val, val);
          }
          else
          {
            DebugPort.println(F("ERR: I2C read failed"));
          }
        }
      }
    }
    else if (subCmd == "write")
    {
      int sp2 = subArg.indexOf(' ');
      if (sp2 < 0)
      {
        DebugPort.println(F("Usage: eeprom write <addr (0-2047)> <val (0-255)>"));
      }
      else
      {
        String sAddr = subArg.substring(0, sp2);
        String sVal = subArg.substring(sp2 + 1);
        sAddr.trim();
        sVal.trim();
        uint16_t addr = (uint16_t)sAddr.toInt();
        uint8_t val = (uint8_t)sVal.toInt();
        if (addr >= eeprom::CAPACITY_BYTES)
        {
          DebugPort.println(F("ERR: Address out of range (0-2047)"));
        }
        else
        {
          if (eeprom::writeByte(addr, val))
          {
            DebugPort.printf("EEPROM[0x%04X] written 0x%02X (%u)\n", addr, val, val);
          }
          else
          {
            DebugPort.println(F("ERR: I2C write failed"));
          }
        }
      }
    }
    else if (subCmd == "dump")
    {
      uint16_t addr = 0;
      uint16_t len = 64;
      int sp2 = subArg.indexOf(' ');
      if (subArg.length() > 0)
      {
        if (sp2 >= 0)
        {
          addr = (uint16_t)subArg.substring(0, sp2).toInt();
          len = (uint16_t)subArg.substring(sp2 + 1).toInt();
        }
        else
        {
          addr = (uint16_t)subArg.toInt();
        }
      }
      if (addr >= eeprom::CAPACITY_BYTES)
      {
        addr = 0;
      }
      if (len == 0 || (uint32_t)addr + len > eeprom::CAPACITY_BYTES)
      {
        len = eeprom::CAPACITY_BYTES - addr;
      }
      if (len > 256)
      {
        len = 256;
      }

      uint8_t buf[16];
      DebugPort.printf("--- EEPROM Dump: Addr 0x%04X, Len %u ---\n", addr, len);
      for (uint16_t i = 0; i < len; i += 16)
      {
        uint16_t curAddr = addr + i;
        uint16_t chunk = ((len - i) < 16) ? (len - i) : 16;
        if (!eeprom::readBuffer(curAddr, buf, chunk))
        {
          DebugPort.println(F("ERR: Read error during dump"));
          break;
        }
        DebugPort.printf("0x%04X: ", curAddr);
        for (uint16_t b = 0; b < 16; ++b)
        {
          if (b < chunk)
          {
            DebugPort.printf("%02X ", buf[b]);
          }
          else
          {
            DebugPort.print("   ");
          }
        }
        DebugPort.print(" |");
        for (uint16_t b = 0; b < chunk; ++b)
        {
          char c = (buf[b] >= 32 && buf[b] <= 126) ? (char)buf[b] : '.';
          DebugPort.print(c);
        }
        DebugPort.println("|");
      }
    }
    else if (subCmd == "save")
    {
      if (eeprom_config::saveCurrentConfig())
      {
        DebugPort.println(F("EEPROM: Configuration successfully saved to 24LC16B!"));
      }
      else
      {
        DebugPort.println(F("ERR: Failed to save configuration to EEPROM"));
      }
    }
    else if (subCmd == "load")
    {
      if (eeprom_config::loadSavedConfig())
      {
        DebugPort.println(F("EEPROM: Configuration successfully loaded and applied!"));
      }
      else
      {
        DebugPort.println(F("ERR: Failed to load configuration from EEPROM (invalid CRC or not found)"));
      }
    }
    else if (subCmd == "reset" || subCmd == "erase")
    {
      if (eeprom_config::resetConfig())
      {
        DebugPort.println(F("EEPROM: Configuration reset to factory defaults and memory erased."));
      }
      else
      {
        DebugPort.println(F("ERR: Failed to reset EEPROM configuration"));
      }
    }
    else
    {
      DebugPort.println(F("ERR: Unknown eeprom subcommand. Use: status, test, save, load, reset, read, write, dump"));
    }
  }
  else if (name == "dds")
  {
    int subSp = arg.indexOf(' ');
    String subCmd = (subSp < 0) ? arg : arg.substring(0, subSp);
    String subArg = (subSp < 0) ? "" : arg.substring(subSp + 1);
    subCmd.trim();
    subArg.trim();

    if (subCmd == "status" || subCmd == "")
    {
      DebugPort.println(F("--- AD9833 DDS Status ---"));
      dds::DdsStatus st = dds::getStatus();
      const char* stStr = (st == dds::DDS_STATUS_SIGNAL_OK) ? "SIGNAL VERIFIED ON PA0" :
                          (st == dds::DDS_STATUS_READY) ? "READY / OK (Clock & SPI Running)" : "ERROR / NOT READY";
      DebugPort.printf("Hardware  : %s\n", stStr);
      DebugPort.printf("Output    : %s\n", dds::isEnabled() ? "ENABLED (ACTIVE)" : "DISABLED (MUTED)");
      WaveformType wf = dds::getWaveform();
      const char* wfStr = (wf == SQUARE_WAVE) ? "SQUARE WAVE" : (wf == SINE_WAVE) ? "SINE WAVE" : "TRIANGLE WAVE";
      DebugPort.printf("Waveform  : %s\n", wfStr);
      DebugPort.printf("Frequency : %.4f Hz\n", dds::getFrequency());
      DebugPort.printf("MCLK Clock: %s (%u Hz)\n", dds::getMcoClockName(), dds::getMclkFreq());
      DebugPort.printf("Resolution: %.6f Hz (Step size)\n", dds::getResolution());
      DebugPort.println(F("Pins      : FSYNC=PC4, SCK=PA5, MOSI=PA7, MCLK=PA8 (MCO1)"));
    }
    else if (subCmd == "clock" || subCmd == "mco")
    {
      if (subArg.length() == 0)
      {
        DebugPort.println(F("--- AD9833 DDS MCO Clock Presets ---"));
        DebugPort.println(F("Idx | Clock Source   | MCLK Freq    | Resolution   | Stability / Characteristics"));
        DebugPort.println(F("----+----------------+--------------+--------------+---------------------------------------------"));
        for (uint8_t i = 0; i < dds::MCO_CLOCK_COUNT; ++i)
        {
          const dds::McoClockInfo &inf = dds::getMcoClockInfo((dds::McoClockSelection)i);
          bool isAct = (dds::getMcoClock() == (dds::McoClockSelection)i);
          DebugPort.printf(" %u  | %-14s | %8u Hz | %.6f Hz | %-13s %s\n",
                           i, inf.name, inf.freqHz, inf.resolutionHz, inf.stability,
                           isAct ? "[*ACTIVE*]" : "");
        }
        DebugPort.printf("Active MCO Clock : %s (%u Hz, Res: %.6f Hz)\n",
                         dds::getMcoClockName(), dds::getMclkFreq(), dds::getResolution());
        DebugPort.println(F("Usage: dds clock <0-6> or dds clock <8mhz|4mhz|2mhz|1.6mhz|16mhz>"));
      }
      else
      {
        dds::McoClockSelection sel = dds::parseMcoClock(subArg.c_str());
        if (sel < dds::MCO_CLOCK_COUNT)
        {
          dds::setMcoClock(sel);
          const dds::McoClockInfo &inf = dds::getMcoClockInfo(sel);
          DebugPort.printf("DDS MCO Clock set to: %s (%u Hz)\n", inf.name, inf.freqHz);
          DebugPort.printf("Frequency Resolution: %.6f Hz (Step size)\n", dds::getResolution());
          DebugPort.printf("Clock Characteristics: %s\n", inf.description);
          DebugPort.println(F("Hint: Use 'eeprom save' to preserve this clock setting permanently across reboots."));
        }
        else
        {
          DebugPort.println(F("ERR: Invalid clock preset. Type 'dds clock' to list all available presets."));
        }
      }
    }
    else if (subCmd == "freq")
    {
      if (subArg.length() == 0)
      {
        DebugPort.printf("DDS Frequency: %.4f Hz\n", dds::getFrequency());
      }
      else
      {
        double f = strtod(subArg.c_str(), nullptr);
        if (f >= 1.0 && f <= 500000.0)
        {
          dpll::setManualMode(true);
          dpll::manualSetFreq((float)f);
          DebugPort.printf("DDS Frequency set to %.4f Hz (manual mode)\n", dds::getFrequency());
        }
        else
        {
          DebugPort.println(F("ERR: Frequency out of range (1.0 to 500000.0 Hz)"));
        }
      }
    }
    else if (subCmd == "on")
    {
      dds::enableOutput(true);
      DebugPort.println(F("DDS output ENABLED"));
    }
    else if (subCmd == "off")
    {
      dds::enableOutput(false);
      DebugPort.println(F("DDS output DISABLED (MUTED)"));
    }
    else if (subCmd == "wave")
    {
      if (subArg == "sine")
      {
        dds::setWaveform(SINE_WAVE);
        DebugPort.println(F("DDS Waveform: SINE"));
      }
      else if (subArg == "triangle")
      {
        dds::setWaveform(TRIANGLE_WAVE);
        DebugPort.println(F("DDS Waveform: TRIANGLE"));
      }
      else if (subArg == "square")
      {
        dds::setWaveform(SQUARE_WAVE);
        DebugPort.println(F("DDS Waveform: SQUARE"));
      }
      else
      {
        DebugPort.println(F("ERR: Waveform must be sine, triangle, or square"));
      }
    }
    else
    {
      DebugPort.println(F("ERR: Unknown dds subcommand. Use: status, freq <Hz>, clock [idx|preset], on, off, wave <sine|triangle|square>"));
    }
  }
  else if (name == "mco" || name == "ddsclock")
  {
    if (arg.length() == 0)
    {
      DebugPort.println(F("--- AD9833 DDS MCO Clock Presets ---"));
      DebugPort.println(F("Idx | Clock Source   | MCLK Freq    | Resolution   | Stability / Characteristics"));
      DebugPort.println(F("----+----------------+--------------+--------------+---------------------------------------------"));
      for (uint8_t i = 0; i < dds::MCO_CLOCK_COUNT; ++i)
      {
        const dds::McoClockInfo &inf = dds::getMcoClockInfo((dds::McoClockSelection)i);
        bool isAct = (dds::getMcoClock() == (dds::McoClockSelection)i);
        DebugPort.printf(" %u  | %-14s | %8u Hz | %.6f Hz | %-13s %s\n",
                         i, inf.name, inf.freqHz, inf.resolutionHz, inf.stability,
                         isAct ? "[*ACTIVE*]" : "");
      }
      DebugPort.printf("Active MCO Clock : %s (%u Hz, Res: %.6f Hz)\n",
                       dds::getMcoClockName(), dds::getMclkFreq(), dds::getResolution());
      DebugPort.println(F("Usage: mco <0-6> or mco <8mhz|4mhz|2mhz|1.6mhz|16mhz>"));
    }
    else
    {
      dds::McoClockSelection sel = dds::parseMcoClock(arg.c_str());
      if (sel < dds::MCO_CLOCK_COUNT)
      {
        dds::setMcoClock(sel);
        const dds::McoClockInfo &inf = dds::getMcoClockInfo(sel);
        DebugPort.printf("DDS MCO Clock set to: %s (%u Hz)\n", inf.name, inf.freqHz);
        DebugPort.printf("Frequency Resolution: %.6f Hz (Step size)\n", dds::getResolution());
        DebugPort.printf("Clock Characteristics: %s\n", inf.description);
        DebugPort.println(F("Hint: Use 'eeprom save' to preserve this clock setting permanently across reboots."));
      }
      else
      {
        DebugPort.println(F("ERR: Invalid clock preset. Type 'mco' to list all available presets."));
      }
    }
  }
  else if (name == "source")
  {
    if (arg.length() == 0)
    {
      DebugPort.printf("Generator Source: %s\n",
                       dpll::getGeneratorSource() == dpll::GENERATOR_SOURCE_DDS ? "DDS (AD9833)" : "DAC (CD4046 VCO)");
    }
    else if (arg == "dds" || arg == "1")
    {
      dpll::setGeneratorSource(dpll::GENERATOR_SOURCE_DDS);
      DebugPort.println(F("Generator Source set to DDS (AD9833). Ensure physical resistor jumper is set to DDS!"));
    }
    else if (arg == "dac" || arg == "0")
    {
      dpll::setGeneratorSource(dpll::GENERATOR_SOURCE_DAC);
      DebugPort.println(F("Generator Source set to DAC (CD4046 VCO)."));
    }
    else
    {
      DebugPort.println(F("ERR: Invalid source. Use 'dac' or 'dds'."));
    }
  }
  else if (name == "ddsfreq")
  {
    if (arg.length() == 0)
    {
      DebugPort.printf("DDS Center Frequency: %.2f Hz\n", dpll::getDdsCenterFreq());
    }
    else
    {
      float f = arg.toFloat();
      dpll::setDdsCenterFreq(f);
      DebugPort.printf("DDS Center Frequency set to: %.2f Hz\n", dpll::getDdsCenterFreq());
    }
  }
  else if (name == "ddsrange")
  {
    if (arg.length() == 0)
    {
      DebugPort.printf("DDS Frequency Limits: %.1f - %.1f Hz\n", dpll::getDdsMinFreq(), dpll::getDdsMaxFreq());
    }
    else
    {
      int sp2 = arg.indexOf(' ');
      if (sp2 > 0)
      {
        float minF = arg.substring(0, sp2).toFloat();
        float maxF = arg.substring(sp2 + 1).toFloat();
        dpll::setDdsFreqLimits(minF, maxF);
        DebugPort.printf("DDS Frequency Limits set to: %.1f - %.1f Hz\n", dpll::getDdsMinFreq(), dpll::getDdsMaxFreq());
      }
      else
      {
        DebugPort.println(F("ERR: Usage: ddsrange <min_hz> <max_hz>"));
      }
    }
  }
  else if (name == "edge")
  {
    if (arg.length() == 0)
    {
      const char* edgeNames[] = {"RISING", "FALLING", "BOTH"};
      DebugPort.printf("Capture edges: REF (PA0) = %s (%u), ZCD (PA1) = %s (%u)\n",
                       edgeNames[(uint8_t)phase_capture::getRefEdge()], (uint8_t)phase_capture::getRefEdge(),
                       edgeNames[(uint8_t)phase_capture::getZcdEdge()], (uint8_t)phase_capture::getZcdEdge());
    }
    else
    {
      int sp2 = arg.indexOf(' ');
      int p1 = (sp2 < 0) ? arg.toInt() : arg.substring(0, sp2).toInt();
      int p2 = (sp2 < 0) ? p1 : arg.substring(sp2 + 1).toInt();
      if (p1 >= 0 && p1 <= 2 && p2 >= 0 && p2 <= 2)
      {
        phase_capture::setRefEdge((phase_capture::EdgePolarity)p1);
        phase_capture::setZcdEdge((phase_capture::EdgePolarity)p2);
        const char* edgeNames[] = {"RISING", "FALLING", "BOTH"};
        DebugPort.printf("Capture edges set: REF=%s, ZCD=%s\n", edgeNames[p1], edgeNames[p2]);
      }
      else
      {
        DebugPort.println(F("ERR: edge <ref 0..2> [zcd 0..2] (0=Rising, 1=Falling, 2=Both)"));
      }
    }
  }
  else if (name == "dir" || name == "direction")
  {
    if (arg.length() == 0)
    {
      DebugPort.printf("Tracking Direction: %s (%u)\n",
                       dpll::getTrackingDirection() == dpll::TRACKING_DIR_INVERTED ? "INVERTED (Parallel fp)" : "NORMAL (Series fs)",
                       (uint8_t)dpll::getTrackingDirection());
    }
    else if (arg == "0" || arg == "normal" || arg == "fs")
    {
      dpll::setTrackingDirection(dpll::TRACKING_DIR_NORMAL);
      DebugPort.println(F("Tracking Direction set to NORMAL (Series fs resonance)."));
    }
    else if (arg == "1" || arg == "inverted" || arg == "fp" || arg == "inv")
    {
      dpll::setTrackingDirection(dpll::TRACKING_DIR_INVERTED);
      DebugPort.println(F("Tracking Direction set to INVERTED (Parallel fp resonance / Inverting)."));
    }
    else
    {
      DebugPort.println(F("ERR: Usage: dir <0=normal|1=inverted>"));
    }
  }
  else
  {
    DebugPort.println("ERR: Unknown command. Type \"help\".");
  }
}

// UART command parser (non-blocking: one char per call).
void processSerialDebugCommand()
{
  static String cmd = "";

  if (!DebugPort.available())
  {
    return; // Nothing to read, return immediately
  }

  char c = (char)DebugPort.read();
  if (c == '\n')
  {
    cmd.trim();
    if (cmd.length() > 0)
    {
      handleDebugCommand(cmd);
    }
    cmd = "";
  }
  else if (c != '\r')
  {
    cmd += c;
  }
}

void heartbeat()
{
  static uint32_t lastToggleTime = 0;
  uint32_t currentTime = millis();

  if (currentTime - lastToggleTime >= 500)
  {
    lastToggleTime = currentTime;
    digitalToggle(HEARTBEAT_LED);
  }
}
