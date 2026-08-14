/*
 * dpll_controller.h
 *
 * Digital Phase-Locked Loop (DPLL) PI controller.
 *
 * Drives the VCO control voltage (via the DAC on PA4) so that the phase
 * difference between the reference signal and the ZCD feedback converges to a
 * target value (usually 0 degrees = resonance).
 *
 * The controller is a standard PI loop:
 *
 *   error        = targetPhase - measuredPhase   (degrees, wrapped to [-180,180])
 *   integral    += error * dt
 *   voltage      = centerVoltage + (Kp * error + Ki * integral)
 *
 * The output is clamped to [minVoltage, maxVoltage] with anti-windup on the
 * integrator. All timing is derived from the caller-supplied dt so the loop
 * is rate-independent.
 */

#pragma once

#include <stdint.h>

namespace dpll {

// Initialize the controller.
// centerVoltage : VCO voltage that produces the nominal/center frequency.
// kp            : proportional gain (V per degree of phase error).
// ki            : integral gain (V per degree per second).
void begin(float centerVoltage, float kp, float ki);

// Set the phase the loop should lock onto, in degrees (default 0).
void setTargetPhase(float targetDeg);

// Update the PI gains at runtime.
void setGains(float kp, float ki);

// Clamp the DAC output to [minV, maxV] volts (default 0.0 .. 3.3).
void setOutputLimits(float minV, float maxV);

// Enable or disable the control loop. When disabled, output holds last value.
void enable(bool on);
bool isEnabled();

// Clear the integrator and return the output to centerVoltage.
void reset();

// Run one control step.
// phaseErrorDeg : measured phase difference in degrees (from phase_capture).
// dtSeconds     : time since the previous update call, in seconds.
// Returns the resulting output voltage in volts (already applied to the DAC).
float update(float phaseErrorDeg, float dtSeconds);

// Return the last output voltage in volts.
float lastVoltage();

} // namespace dpll
