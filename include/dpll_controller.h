/*
 * dpll_controller.h
 *
 * Digital Phase-Locked Loop (DPLL) PI controller.
 *
 * Drives the VCO control voltage (via the DAC on PA4) so that the phase
 * difference between the reference signal and the ZCD feedback converges to a
 * target value (usually 0 degrees = resonance).
 *
 * The controller is a PID loop:
 *
 *   error        = targetDelay - measuredDelay   (nanoseconds)
 *   integral    += error * dt
 *   derivative   = d(error)/dt
 *   voltage      = centerVoltage + (Kp*error + Ki*integral + Kd*derivative)
 *
 * Kd (derivative) defaults to 0, reducing the loop to PI, which is the
 * recommended configuration for a noisy phase signal. Enable it only with a
 * small value if faster transient damping is needed.
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
// kp            : proportional gain (V per nanosecond of phase error).
// ki            : integral gain (V per nanosecond per second).
void begin(float centerVoltage, float kp, float ki);

// Set the phase delay the loop should lock onto, in nanoseconds (default 0).
void setTargetPhase(float targetNs);

// Update the PI gains at runtime.
void setGains(float kp, float ki);

// Set the derivative gain (V per ns/s). Default 0 (derivative disabled).
void setKd(float kd);

// Clamp the DAC output to [minV, maxV] volts (default 0.0 .. 3.3).
void setOutputLimits(float minV, float maxV);

// Limit the maximum DAC change rate (V/s). Prevents violent slamming when
// gains are misconfigured. Default 30 V/s (0.3 V per 10 ms step).
void setMaxSlew(float voltsPerSecond);

// Enable or disable the control loop. When disabled, output holds last value.
void enable(bool on);
bool isEnabled();

// Clear the integrator and return the output to centerVoltage.
void reset();

// Run one control step.
// phaseErrorNs : measured phase difference in nanoseconds (from phase_capture).
// dtSeconds    : time since the previous update call, in seconds.
// Returns the resulting output voltage in volts (already applied to the DAC).
float update(float phaseErrorNs, float dtSeconds);

// Return the last output voltage in volts.
float lastVoltage();

// Disable the loop and force the DAC to minVoltage (0 V). Use on power-off.
void shutdown();

// Disable the loop and force the DAC to a fixed voltage (manual control).
void manualSet(float voltage);

// Current configuration getters (for UART status/tuning).
float getKp();
float getKi();
float getKd();
float getCenterVoltage();
float getTargetPhase();
float getMaxSlew();

} // namespace dpll
