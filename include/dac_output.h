/*
 * dac_output.h
 *
 * DAC1 output control for the STM32F407.
 *
 * The DPLL adjusts the resonance frequency of the ultrasonic transducer by
 * driving a control voltage from the DAC. On this MCU:
 *   - DAC1 channel 1 -> PA4  (used here)
 *   - DAC1 channel 2 -> PA5  (available if a second output is ever needed)
 *
 * The DAC is 12-bit, free-running (no trigger), with its output buffer enabled.
 */

#pragma once

#include <stdint.h>

namespace dac {

// DAC resolution (STM32F4 DAC is 12-bit).
constexpr uint16_t kMaxValue = 4095U;

// Reference voltage used by setVoltage(). DAC VREF+ is typically VDDA = 3.3 V.
constexpr float kVref = 3.3f;

// Initialize DAC1 channel 1 (PA4) and start it at mid-scale.
// Call once from setup().
void begin();

// Set the raw 12-bit DAC value [0 .. 4095]. Values are clamped.
void setRaw(uint16_t value);

// Set the output voltage [0.0 .. kVref] volts. Values are clamped.
void setVoltage(float volts);

// Return the last raw value written to the DAC.
uint16_t lastRaw();

} // namespace dac
