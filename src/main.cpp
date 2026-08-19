#include <Arduino.h>
#include "dac_output.h"
#include "phase_capture.h"
#include "dpll_controller.h"
#include "com.h"
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

#define HEARTBEAT_LED PA_6
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
  pinMode(digitalPinToPinName(HEARTBEAT_LED), OUTPUT);
  digitalWriteFast(digitalPinToPinName(HEARTBEAT_LED), HIGH);

  // DAC control output: PA4, mid-scale (1.65 V) on start.
  dac::begin();
  dac::setVoltage(1.65f);

  // Initialize 32-bit Input Capture on TIM2:
  // PA0 = Reference input (Rising edge trigger)
  // PA1 = ZCD feedback input (Rising edge trigger)
  phase_capture::begin(PA0, PA1, phase_capture::CAPTURE_RISING, phase_capture::CAPTURE_RISING);

  // Initialize DPLL PI controller.
  // center 1.65 V. POSITIVE gains (polarity now corrected).
  // Series resonance: below resonance -> current LEADS -> phase negative;
  //                    above resonance -> current LAGS -> phase positive.
  //   error = target - phase ; voltage = center + Kp*error
  //   - phase positive (above res) -> error negative -> voltage DOWN -> freq down
  //   - phase negative (below res) -> error positive -> voltage UP  -> freq up
  // Gains are V per nanosecond (Kp) and V/ns per second (Ki).
  // Start small; the phase slope near resonance is very steep for high-Q.
  dpll::begin(1.65f, 0.000002f, 0.0002f, 0.0f);
  dpll::setTargetPhase(0.0f);
  dpll::setOutputLimits(0.0f, 3.3f);
  dpll::setMaxSlew(30.0f);
}

void loop()
{
  processDPLL();
  com::receive_command();
  commandProccessor();
  processSerialDebugCommand();
  sendMonitorStream();
  com::FlushTxQueue(); // flush any pending packets to USB
  debugPrint();
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
  // Drain entire RX queue per loop cycle — prevents stall on burst opcodes.
  if (com::getAvailableRxPackets(&RxPacket) != ILEGAL_OPCODE)
  {
    if (RxPacket.header.opcode == OPCODE_SET_ALLOW_SEND_STREAM)
    {
      if (RxPacket.header.payloadLength >= 1)
      {
        bool allow = RxPacket.payload[0] != 0;
        com::SetAllowSendStream(allow);
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
      float absPhase = data.phaseDiffNs < 0.0f ? -data.phaseDiffNs : data.phaseDiffNs;
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

  phase_capture::CaptureData data = phase_capture::getData();
  float currentDacV = dac::lastRaw() * (3.3f / 4095.0f);
  const float kLockThr = dpll::getLockThresholdNs();
  bool isLocked = data.valid &&
                  (data.phaseDiffNs >= -kLockThr) &&
                  (data.phaseDiffNs <=  kLockThr);

  // Hold last valid phase measurement — used when ZCD is absent (DAC frozen, VCO still running).
  static float s_lastValidPhaseNs = 0.0f;
  if (data.zcdPresent) {
    s_lastValidPhaseNs = data.phaseDiffNs;
  }

  dpllStatusData status;
  // LockStatus: 0=NO_REF, 1=WAIT_ZCD, 2=TRACK, 3=LOCK
  if (!data.refValid)        status.LockStatus = 0;
  else if (!data.zcdPresent) status.LockStatus = 1;
  else if (!isLocked)        status.LockStatus = 2;
  else                       status.LockStatus = 3;
  status.ReferenceFrequencyHz = data.frequencyHz;
  // When ZCD absent: send last valid phase (DAC frozen → VCO at same freq → phase estimate still valid).
  // PhaseStale=1 tells host this is a held value, not a fresh measurement.
  status.PhaseError_ns        = s_lastValidPhaseNs;
  status.PhaseStale           = data.zcdPresent ? 0 : 1;
  status.DACVoltage_V         = currentDacV;
  status._pad[0] = status._pad[1] = 0;
  com::sendPacket(OPCODE_STREAM_DPLL_STATUS, 0, (uint8_t *)&status, sizeof(status));
}
void processDPLL()
{
  // --- DPLL control loop ---
  // Sequence per cycle:
  //   1. DAC was updated at lastDacUpdate.
  //   2. Wait g_loopPeriodMs ms for transducer to settle.
  //   3. Read phase (getData) AFTER settling is complete.
  //   4. Update DAC -> record lastDacUpdate, start next settling window.
  static uint32_t lastDacUpdate = 0;
  static bool wasValid = false;
  static bool firstRun = true;
  static uint32_t lockHoldCount = 0;  // consecutive LOCK cycles counter

  uint32_t nowMs = millis();

  // On first run, start the settling timer immediately without updating DAC.
  if (firstRun) {
    lastDacUpdate = nowMs;
    firstRun = false;
    return;
  }

  // Wait until settling window expires before reading phase.
  if (nowMs - lastDacUpdate < dpll::getLoopPeriodMs()) {
    return;
  }

  // Settling done — read phase NOW (transducer has settled after last DAC step).
  phase_capture::CaptureData data = phase_capture::getData();

  // --- Lock-point memory ---
  // Track consecutive LOCK cycles. Once stable for kLockHoldCycles,
  // save current DAC voltage as the new center for next re-acquire.
  float currentDacV = dac::lastRaw() * (3.3f / 4095.0f);
  const float kLockThr = dpll::getLockThresholdNs();
  bool isLocked = data.valid &&
                  (data.phaseDiffNs >= -kLockThr) &&
                  (data.phaseDiffNs <=  kLockThr);
  if (isLocked) {
    lockHoldCount++;
    if (lockHoldCount >= dpll::getLockHoldCycles()) {
      // Lock confirmed stable — update center voltage memory.
      if (!dpll::haveLockedCenter() || (currentDacV != dpll::getLockedCenterV())) {
        dpll::setLockedCenter(currentDacV);
        DebugPort.printf("[LOCK SAVED] center = %.3f V\n", currentDacV);
      }
    }
  } else {
    lockHoldCount = 0;
  }

  // Update DAC — record timestamp immediately after update to start next settling window.
  if (dpll::getManualMode())
  {
    // Manual: keep DAC as user set it; do not run the loop.
  }
  else if (data.valid)
  {
    if (!wasValid)
    {
      // Re-acquire: choose start center based on signal-loss behaviour.
      // FREEZE/ZERO : start from locked center (known good operating point).
      // CENTER      : DAC already at centerVoltage after reset() — start from there
      //               to avoid a sudden DAC step back to lockedCenterV.
      float center;
      if (dpll::getSignalLossBehavior() == dpll::SIGNAL_LOSS_CENTER)
      {
        center = dpll::getCenterVoltage();
      }
      else
      {
        center = dpll::getLockedCenterV();
      }
      dpll::begin(center, dpll::getKp(), dpll::getKi(), dpll::getKd());
      dpll::tickSignalAbsent(0); // reset absent timer now that signal is back
      DebugPort.printf("[RE-ACQUIRE] start center = %.3f V%s\n", center,
                       dpll::haveLockedCenter() ? "" : " (from nominal center, lock memory expired)");
    }
    dpll::enable(true);
    dpll::update(data.phaseDiffNs, dpll::getLoopPeriodMs() * 0.001f);
  }
  else
  {
    // Signal missing — behaviour set by setSignalLossBehavior().
    switch (dpll::getSignalLossBehavior())
    {
      case dpll::SIGNAL_LOSS_CENTER:
        dpll::reset();       // DAC → centerVoltage, clear integrator
        dpll::enable(false);
        break;
      case dpll::SIGNAL_LOSS_ZERO:
        dpll::shutdown();    // DAC → 0 V, clear integrator
        break;
      case dpll::SIGNAL_LOSS_FREEZE:
      default:
        dpll::enable(false); // DAC frozen at last position
        break;
    }
    dpll::tickSignalAbsent(nowMs); // track how long signal has been absent
  }
  // Record the exact moment DAC was updated — settling window starts NOW.
  lastDacUpdate = millis();
  wasValid = data.valid;
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
    DebugPort.printf("Kp=%.6f V/ns | Ki=%.6f V/ns/s | Kd=%.6f V/ns/s | center=%.2f V | target=%.1f ns | slew=%.1f V/s | manual=%s | loop=%u ms | thr=%.0f ns | lockedV=%.3f V%s | loss=%u\n",
                  dpll::getKp(), dpll::getKi(), dpll::getKd(), dpll::getCenterVoltage(),
                  dpll::getTargetPhase(), dpll::getMaxSlew(), dpll::getManualMode() ? "yes" : "no",
                  dpll::getLoopPeriodMs(), dpll::getLockThresholdNs(), dpll::getLockedCenterV(),
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
    DebugPort.println("Control loop enabled");
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
    digitalToggleFast(digitalPinToPinName(HEARTBEAT_LED));
  }
}
