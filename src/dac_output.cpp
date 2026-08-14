/*
 * dac_output.cpp
 *
 * See dac_output.h for the public interface.
 *
 * Note on HAL_DAC_MspInit: the stm32duino core (SrcWrapper/analog.cpp) already
 * provides a strong definition of HAL_DAC_MspInit(), which enables the DAC
 * clock and (only when driven through analogWrite) configures the GPIO via the
 * core's g_current_pin. Because g_current_pin is private to the core, we cannot
 * rely on it here, so this module enables the clock and configures PA4 as
 * analog manually, then calls HAL_DAC_Init() (which invokes the core MspInit —
 * harmless). Do NOT define our own HAL_DAC_MspInit: it would collide at link
 * time.
 */

#include "dac_output.h"

#include <Arduino.h>

namespace dac {

namespace {

DAC_HandleTypeDef hdac{};
uint16_t          s_lastRaw = 0U;
bool              s_started = false;

} // namespace

void begin()
{
  if (s_started) {
    return;
  }

  // 1) Clocks: DAC peripheral + GPIOA (PA4). Idempotent.
  __HAL_RCC_DAC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // 2) PA4 must be in analog mode for DAC_OUT1.
  GPIO_InitTypeDef gpio{};
  gpio.Pin  = GPIO_PIN_4;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  // 3) Init the DAC instance. HAL_DAC_Init() calls the core's HAL_DAC_MspInit()
  //    (enables the clock again; GPIO handling there is a no-op since the core
  //    never set g_current_pin for us).
  hdac.Instance = DAC1;
  if (HAL_DAC_Init(&hdac) != HAL_OK) {
    return; // DAC unavailable.
  }

  // 4) Channel 1: free-running conversion, output buffer enabled.
  DAC_ChannelConfTypeDef cfg{};
  cfg.DAC_Trigger      = DAC_TRIGGER_NONE;
  cfg.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &cfg, DAC_CHANNEL_1) != HAL_OK) {
    return;
  }

  // 5) Enable the channel and start at mid-scale.
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  setRaw(kMaxValue / 2U);

  s_started = true;
}

void setRaw(uint16_t value)
{
  if (value > kMaxValue) {
    value = kMaxValue;
  }
  s_lastRaw = value;
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value);
}

void setVoltage(float volts)
{
  if (volts < 0.0f) {
    volts = 0.0f;
  }
  if (volts > kVref) {
    volts = kVref;
  }
  setRaw(static_cast<uint16_t>((volts / kVref) * static_cast<float>(kMaxValue) + 0.5f));
}

uint16_t lastRaw()
{
  return s_lastRaw;
}

} // namespace dac
