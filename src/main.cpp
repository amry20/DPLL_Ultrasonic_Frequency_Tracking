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
  // center 1.65 V, Kp = 0.01 V/deg, Ki = 0.5 V/deg/s (tune as needed).
  dpll::begin(1.65f, 0.01f, 0.5f);
  dpll::setTargetPhase(0.0f);
  dpll::setOutputLimits(0.0f, 3.3f);

  Serial.println(F("DPLL Ultrasonic Frequency Tracking"));
  Serial.println(F("Commands: \"dac <volt>\" to set DAC voltage (0.0 - 3.3), e.g. \"dac 1.2\""));
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
    if (data.valid) {
      float outVolt = dpll::update(data.phaseDiffDeg, kLoopDt);
      (void)outVolt;
    } else if (!data.zcdPresent) {
      // ZCD missing -> hold output (controller keeps last voltage)
      dpll::enable(false);
    } else {
      dpll::enable(true);
    }
  }

  // --- Status print (500 ms) ---
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    phase_capture::CaptureData data = phase_capture::getData();
    if (data.valid) {
      Serial.printf("LOCK | Freq: %.2f Hz | Phase: %.2f deg | Period: %.2f us | DAC: %.2f V\n",
                    data.frequencyHz,
                    data.phaseDiffDeg,
                    phase_capture::ticksToUs(data.periodTicks),
                    dac::lastRaw() * (3.3f / 4095.0f));
    } else if (data.refValid && !data.zcdPresent) {
      Serial.printf("WAIT ZCD | REF Freq: %.2f Hz | Power Enable OFF / ZCD missing | DAC: %.2f V\n",
                    data.frequencyHz,
                    dac::lastRaw() * (3.3f / 4095.0f));
    } else {
      Serial.printf("NO REF SIGNAL | Waiting for generator input on PA0... | DAC: %.2f V\n",
                    dac::lastRaw() * (3.3f / 4095.0f));
    }
  }
}

// UART command parser: "dac <voltage>" sets DAC output voltage (0.0 - 3.3 V)
// Non-blocking: processes at most ONE character per call, so other code keeps running.
void processSerialCommand() {
  static String cmd = "";

  if (!Serial.available()) {
    return; // Nothing to read, return immediately
  }

  char c = (char)Serial.read();
  if (c == '\n') {
    cmd.trim();
    if (cmd.length() > 0) {
      if (cmd.startsWith("dac")) {
        float volt = cmd.substring(4).toFloat();
        if (volt >= 0.0f && volt <= 3.3f) {
          dac::setVoltage(volt);
          Serial.printf("DAC set to %.2f V (raw %u)\n", volt, dac::lastRaw());
        } else {
          Serial.println("ERR: Voltage out of range (0.0 - 3.3 V)");
        }
      } else {
        Serial.println("ERR: Unknown command. Usage: \"dac <volt>\"");
      }
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
