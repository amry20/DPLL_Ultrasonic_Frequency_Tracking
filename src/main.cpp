#include <Arduino.h>
#include "dac_output.h"
#include "phase_capture.h"
#include "dpll_controller.h"

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
void processSerialCommand();

// DPLL loop update period (seconds)
constexpr float kLoopDt = 0.01f; // 10 ms control interval
// Phase error (absolute nanoseconds) below which the loop is considered "LOCK".
constexpr float kLockThresholdNs = 500.0f;

// Manual mode: when true, the DPLL loop is disengaged and the DAC is set by hand.
static bool g_manualMode = false;

void setup() {
  Serial.setTx(PA9);
  Serial.setRx(PA10);
  Serial.begin(115200);
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
  // center 1.65 V. NEGATIVE gains: with "phase = ZCD - REF" convention,
  // positive delay (ZCD lag = VCO slow) must INCREASE voltage to speed the VCO.
  //   error = target - phase  ->  positive delay gives negative error
  //   voltage = center + Kp*error  ->  negative Kp => voltage RISES for lag.
  // Gains are V per nanosecond (Kp) and V/ns per second (Ki).
  // Start small; tune up gradually. Typical order for a ~33 kHz VCO with
  // ~2.5 kHz/V sensitivity: Kp ~ 2e-6 .. 1e-5 V/ns.
  dpll::begin(1.65f, -0.000002f, -0.0002f);
  dpll::setTargetPhase(0.0f);
  dpll::setOutputLimits(0.0f, 3.3f);
  dpll::setMaxSlew(30.0f);

  Serial.println(F("DPLL Ultrasonic Frequency Tracking"));
  Serial.println(F("Type \"help\" for commands."));
}

void loop() {
  heartbeat();
  processSerialCommand();

  // --- DPLL control loop (fixed 10 ms rate, non-blocking) ---
  static uint32_t lastControl = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastControl >= (uint32_t)(kLoopDt * 1000.0f)) {
    lastControl = nowMs;

    phase_capture::CaptureData data = phase_capture::getData();

    static bool wasValid = false;
    if (g_manualMode) {
      // Manual control: keep DAC as the user set it; do not run the loop.
    } else if (data.valid) {
      if (!wasValid) {
        // Re-acquire: restart from center voltage for a clean lock.
        dpll::reset();
      }
      dpll::enable(true);
      dpll::update(data.phaseDiffNs, kLoopDt);
    } else {
      // Power off / signal missing -> drive DAC to 0 V.
      dpll::shutdown();
    }
    wasValid = data.valid;
  }

  // --- Status print (500 ms) ---
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    phase_capture::CaptureData data = phase_capture::getData();
    const char* manual = g_manualMode ? " [MANUAL]" : "";
    if (data.valid) {
      float absPhase = data.phaseDiffNs < 0.0f ? -data.phaseDiffNs : data.phaseDiffNs;
      const char* state = (absPhase <= kLockThresholdNs) ? "LOCK" : "TRACK";
      Serial.printf("%s%s | Freq: %.2f Hz | Phase: %.1f ns | Period: %.2f us | DAC: %.2f V\n",
                    state, manual,
                    data.frequencyHz,
                    data.phaseDiffNs,
                    phase_capture::ticksToUs(data.periodTicks),
                    dac::lastRaw() * (3.3f / 4095.0f));
    } else if (data.refValid && !data.zcdPresent) {
      Serial.printf("WAIT ZCD%s | REF Freq: %.2f Hz | Power Enable OFF / ZCD missing | DAC: %.2f V\n",
                    manual,
                    data.frequencyHz,
                    dac::lastRaw() * (3.3f / 4095.0f));
    } else {
      Serial.printf("NO REF SIGNAL%s | Waiting for generator input on PA0... | DAC: %.2f V\n",
                    manual,
                    dac::lastRaw() * (3.3f / 4095.0f));
    }
  }
}

// Parse and execute one full command line (already trimmed).
void handleCommand(const String& cmd) {
  int sp = cmd.indexOf(' ');
  String name = (sp < 0) ? cmd : cmd.substring(0, sp);
  String arg  = (sp < 0) ? ""  : cmd.substring(sp + 1);
  name.trim();
  arg.trim();

  if (name == "help" || name == "?") {
    Serial.println(F("Commands:"));
    Serial.println(F("  dac <volt>    : manual DAC voltage (0.0-3.3), disables loop"));
    Serial.println(F("  kp <val>      : set proportional gain (V/ns)"));
    Serial.println(F("  ki <val>      : set integral gain (V/ns/s)"));
    Serial.println(F("  center <volt> : set center voltage"));
    Serial.println(F("  target <ns>   : set lock delay (ns)"));
    Serial.println(F("  slew <V/s>    : set max DAC slew rate (V/s)"));
    Serial.println(F("  gain          : show current gains"));
    Serial.println(F("  reset         : clear integrator, restart from center"));
    Serial.println(F("  run           : re-enable control loop"));
  }
  else if (name == "dac") {
    if (arg.length() == 0) {
      Serial.printf("DAC now: %.3f V\n", dac::lastRaw() * (3.3f / 4095.0f));
      return;
    }
    float v = arg.toFloat();
    if (v >= 0.0f && v <= 3.3f) {
      g_manualMode = true;
      dpll::manualSet(v);
      Serial.printf("DAC set to %.2f V (raw %u) - loop disabled\n", v, dac::lastRaw());
    } else {
      Serial.println("ERR: Voltage out of range (0.0 - 3.3 V)");
    }
  }
  else if (name == "kp") {
    float v = arg.toFloat();
    dpll::setGains(v, dpll::getKi());
    Serial.printf("Kp = %.5f V/ns\n", v);
  }
  else if (name == "ki") {
    float v = arg.toFloat();
    dpll::setGains(dpll::getKp(), v);
    Serial.printf("Ki = %.5f V/ns/s\n", v);
  }
  else if (name == "center") {
    float v = arg.toFloat();
    if (v >= 0.0f && v <= 3.3f) {
      dpll::begin(v, dpll::getKp(), dpll::getKi());
      Serial.printf("Center = %.2f V\n", v);
    } else {
      Serial.println("ERR: out of range (0.0 - 3.3 V)");
    }
  }
  else if (name == "target") {
    float v = arg.toFloat();
    dpll::setTargetPhase(v);
    Serial.printf("Target phase = %.1f ns\n", v);
  }
  else if (name == "gain") {
    Serial.printf("Kp=%.6f V/ns | Ki=%.6f V/ns/s | center=%.2f V | target=%.1f ns | slew=%.1f V/s | manual=%s\n",
                  dpll::getKp(), dpll::getKi(), dpll::getCenterVoltage(),
                  dpll::getTargetPhase(), dpll::getMaxSlew(), g_manualMode ? "yes" : "no");
  }
  else if (name == "slew") {
    float v = arg.toFloat();
    if (v > 0.0f) {
      dpll::setMaxSlew(v);
      Serial.printf("Max slew = %.1f V/s\n", v);
    } else {
      Serial.println("ERR: slew must be > 0 (V/s)");
    }
  }
  else if (name == "reset") {
    g_manualMode = false;
    dpll::reset();
    dpll::enable(true);
    Serial.println("Controller reset to center voltage, loop enabled");
  }
  else if (name == "run") {
    g_manualMode = false;
    dpll::enable(true);
    Serial.println("Control loop enabled");
  }
  else {
    Serial.println("ERR: Unknown command. Type \"help\".");
  }
}

// UART command parser (non-blocking: one char per call).
void processSerialCommand() {
  static String cmd = "";

  if (!Serial.available()) {
    return; // Nothing to read, return immediately
  }

  char c = (char)Serial.read();
  if (c == '\n') {
    cmd.trim();
    if (cmd.length() > 0) {
      handleCommand(cmd);
    }
    cmd = "";
  } else if (c != '\r') {
    cmd += c;
  }
}


void heartbeat() {
  static uint32_t lastToggleTime = 0;
  uint32_t currentTime = millis();

  if (currentTime - lastToggleTime >= 500) {
    lastToggleTime = currentTime;
    digitalToggleFast(digitalPinToPinName(HEARTBEAT_LED));
  }
}
