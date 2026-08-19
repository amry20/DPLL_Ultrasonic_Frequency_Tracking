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

namespace dpll
{

    // Initialize the controller.
    // centerVoltage : VCO voltage that produces the nominal/center frequency.
    // kp            : proportional gain (V per nanosecond of phase error).
    // ki            : integral gain (V per nanosecond per second).
    // kd            : derivative gain (V per nanosecond per second).
    void begin(float centerVoltage, float kp, float ki, float kd = 0.0f);

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

    // --- Loop / acquisition config (all configurable via opcode) ---

    // Control loop period in milliseconds (1–1000). Default 20.
    uint32_t getLoopPeriodMs();
    void     setLoopPeriodMs(uint32_t ms);

    // Phase error magnitude (ns) below which the loop is considered locked. Default 500.
    float    getLockThresholdNs();
    void     setLockThresholdNs(float ns);

    // Manual mode: when true, DPLL loop is disengaged, DAC held by user.
    bool     getManualMode();
    void     setManualMode(bool on);

    // Lock-point memory: DAC voltage saved when lock is stable.
    float    getLockedCenterV();
    bool     haveLockedCenter();
    void     setLockedCenter(float v);   // sets voltage + marks valid
    void     clearLockedCenter();        // resets to 1.65 V, marks invalid

    // Consecutive LOCK cycles required before committing center voltage. Default 10.
    uint32_t getLockHoldCycles();
    void     setLockHoldCycles(uint32_t n);

    // Lock memory timeout: if signal is absent longer than this (ms), the saved
    // lockedCenterV is discarded and re-acquire will start from centerVoltage.
    // Set to 0 to disable (lock memory never expires — risk of false lock on sample change).
    // Default: 5000 ms (5 s).
    uint32_t getLockMemoryTimeoutMs();
    void     setLockMemoryTimeoutMs(uint32_t ms);

    // Called by main loop every cycle when signal is absent.
    // Internally tracks elapsed absent time and clears locked center on timeout.
    void     tickSignalAbsent(uint32_t nowMs);

    // Monitor stream rate in milliseconds (1–65535). Default 100 ms = 10 Hz.
    // sendMonitorStream() uses this to decouple stream rate from settling window.
    uint32_t getStreamPeriodMs();
    void     setStreamPeriodMs(uint32_t ms);

    // Behaviour when the input signal is lost (REF or ZCD absent).
    enum SignalLossBehavior : uint8_t {
        SIGNAL_LOSS_FREEZE = 0,  // Hold DAC at last position (default — best for transient dropouts)
        SIGNAL_LOSS_CENTER = 1,  // Drive DAC back to centerVoltage, clear integrator
        SIGNAL_LOSS_ZERO   = 2,  // Drive DAC to 0 V, clear integrator (safe shutdown)
    };
    SignalLossBehavior getSignalLossBehavior();
    void               setSignalLossBehavior(SignalLossBehavior b);

} // namespace dpll
