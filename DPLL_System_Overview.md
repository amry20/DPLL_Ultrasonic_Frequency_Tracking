# Digital Phase-Locked Loop (DPLL) System Overview

## 1. System Abstract & Architecture

An embedded Digital Phase-Locked Loop (DPLL) was implemented on an STM32F407VET6 microcontroller operating at 168 MHz. The system maintains the resonance frequency of an ultrasonic transducer under dynamic electrical and mechanical loading conditions. The closed-loop controller continuously tracks the zero-phase difference between the reference driving signal and the zero-crossing detector (ZCD) current feedback signal.

The complete signal chain consists of a phase capture stage (dual-channel TIM2 input capture on PA0 for the reference and PA1 for the ZCD feedback), a PID controller with anti-windup clamping and slew-rate limiting, a 12-bit DAC1 output (PA4), and the analog hardware plant comprising a CD4046 VCO, gate driver, and ultrasonic transducer load.

---

## 2. Key Working Principles

### 2.1 Closed-Loop Signal Flow

The DPLL operates as a negative-feedback control system that steers the transducer driving frequency until the current and the driving voltage are in phase (resonance). The loop runs at a fixed rate (default 20 ms) and follows this cycle every step:

1. **Capture the full window** — The 32-bit hardware timer `TIM2` (clocked at 84 MHz, ~11.9 ns resolution) timestamps every rising edge of the reference square wave on `PA0` and the zero-crossing-detector (ZCD) square wave on `PA1`. Each paired integer sample is placed in an ISR-safe double buffer throughout the 20 ms settling window.
2. **Wrap each sample** — Each raw time difference `(ZCD timestamp − REF timestamp)` is wrapped into `[−period/2, +period/2]`. Circular phase differences are used so a normal `+180°` to `−180°` crossing is not mistaken for a spike.
3. **Estimate window confidence** — Samples are reduced to chronological block medians, fitted with a robust Theil-Sen trend, and screened with median-absolute-deviation (MAD) gates. The result contains end-of-window phase, slope, noise, outlier ratio, overflow status, and a confidence flag.
4. **Gate the controller** — A low-confidence, overflowing, spiking, or abruptly shifted window holds the DAC and freezes the PID integral. A large clean inter-window shift must repeat in a second clean window before it is accepted.
5. **Calculate error** — For a trusted window, `error = φ_target − φ_filtered`, where `φ_target = 0 ns`.
6. **Compute and protect control voltage** — A PID law converts the filtered error into a voltage offset, followed by output clamping, integral anti-windup, and slew-rate limiting.
7. **Actuate** — The 12-bit DAC (`PA4`) applies the voltage to the CD4046 VCO, which shifts the frequency toward resonance.
8. **Repeat** — The capture bank is cleared immediately after the control decision so the next 20 ms window contains only samples taken after that decision.

### 2.2 The PID Control Law

The controller implements a discrete PID (defaults to PI because Kd = 0):

```
error        = φ_target − φ_measured              (nanoseconds)

proportional = Kp × error                          (volts)
integral     = integral + Ki × error × dt          (volts, accumulated)
derivative   = Kd × (error − error_previous) / dt  (volts, disabled by default)

V_raw        = V_center + proportional + integral + derivative
```

- **Proportional term** responds immediately to the current phase error.
- **Integral term** accumulates over time and removes any steady-state phase offset, guaranteeing the loop settles exactly on the target phase.
- **Derivative term** predicts the error trend and damps oscillations, but amplifies measurement noise. It is disabled (Kd = 0) for the noisy phase signal and enabled only with a small value when faster damping is required.

### 2.3 Output Protections (Safety Limits)

Before the voltage reaches the DAC, three safeguards are applied:

1. **Output clamping** — The voltage is limited to `[0.0 V, 3.3 V]` to protect the VCO input.
2. **Integral anti-windup** — Whenever the voltage saturates at a limit, the integrator is back-computed so it stops accumulating. This prevents the classic "windup" overshoot when the loop recovers from saturation.
3. **Slew-rate limiting** — The voltage change per step is capped at 30 V/s. This prevents violent voltage slams if the gains are misconfigured or the phase signal is momentarily noisy, protecting the gate driver and the transducer.

### 2.4 Lock Detection

The system reports its state based on the absolute phase error:

| State | Condition | Meaning |
|---|---|---|
| LOCK | abs(φ) ≤ 500 ns | Loop has converged onto resonance |
| TRACK | abs(φ) > 500 ns | Loop is actively correcting toward resonance |
| WAIT ZCD | ZCD signal missing | Power stage off / feedback absent |
| NO REF | Reference signal missing | Waiting for generator input on PA0 |

### 2.5 Control Loop Timing

The control loop runs at a fixed period (default **20 ms**, runtime-configurable via the `loop <ms>` command) rather than as fast as possible. This period is not arbitrary — it is chosen to match the physical settling time of the ultrasonic transducer.

A high-Q ultrasonic transducer does not change its phase instantaneously after a DAC step; it rings for a characteristic time constant:

```
settling time  τ  ≈  Q / (2·π·f0)
```

where `Q` is the mechanical quality factor and `f0` is the resonant frequency. If the loop updated faster than the transducer could settle, each new phase measurement would be corrupted by the still-decaying response of the previous step, causing instability.

The design rule applied is:

```
loop period  ≥  5 · τ
```

so that the transducer has effectively settled before the next correction. The default of 20 ms is a safe starting point for typical ultrasonic transducers, but it must be increased for higher-Q loads. The same `dt` value (in seconds) is fed into the integral and derivative terms, which keeps the controller rate-independent: re-scaling the loop period by itself does not change the effective integral/derivative strength, because those terms scale with `dt`.

As a safety measure, `dt` is sanitized inside the controller: values below 1 µs and above 1 s are clamped, so the first call or a corrupted period value cannot produce a huge integral windup spike.

### 2.6 Robust 20 ms Phase Window

At approximately 30–35 kHz, a 1 ms control interval contains roughly 30–35 ZCD cycles and a 20 ms interval contains roughly 600–700 cycles. The firmware processes all available cycles as a measurement window rather than using only the final timestamp. The ISR stores only integer timer values; floating-point filtering and sorting are performed outside the interrupt while the alternate capture bank continues collecting edges.

Block size adapts to the captured sample count, targeting approximately eight chronological blocks per window and clamping each block to 4–32 samples. The confidence policy requires at least 12 samples, at least three accepted blocks, at least 70% accepted samples, no buffer overflow, residual MAD-derived noise no greater than 12 degrees, and no estimated movement greater than 60 degrees within one window. A large inter-window shift must remain consistent for 20 ms of real time, independent of whether the loop runs at 1 ms or 20 ms. These are conservative starting values in `phase_capture.cpp` and should be validated against oscilloscope data from the real transducer and bubble-collapse operating conditions.

The filter cannot recover polarity information that has already been discarded by the external ZCD circuit. If the ZCD emits indistinguishable pulses for both half-cycles, an additional polarity-preserving signal or hardware measurement is required.

---

## 3. Tunable System Parameters

The DPLL controller exposes real-time tunable parameters via UART interface:

| Parameter | Symbol / Variable | Default Value | Description |
|---|---|---|---|
| Proportional Gain | Kp | Configurable | Adjusts system responsiveness to phase error |
| Integral Gain | Ki | Configurable | Eliminates steady-state phase error |
| Derivative Gain | Kd | 0.0 | Damps transient oscillations and overshoot |
| Target Phase | Δφ_set | 0.0 ns | Desired phase offset setpoint between REF and ZCD |
| Center Voltage | V_center | 1.65 V | Baseline DAC output voltage corresponding to center frequency |
| Max Slew Rate | Slew_max | 30.0 V/s | Maximum rate of change for DAC voltage transition |
| Loop Period | dt | 20 ms | Control loop update interval |
| Output Clamping | V_min, V_max | 0.0 V – 3.3 V | Absolute voltage bounds protecting the VCO input |

---

## 4. Baseline Configuration (Verified Locking)

The following values are the current factory/baseline configuration that has been verified to achieve and hold LOCK. They serve as the reference starting point for tuning:

| Parameter | Value | Unit | Notes |
|---|---|---|---|
| Proportional Gain (Kp) | 0.000002 | V/ns | = 2 × 10⁻⁶ V/ns |
| Integral Gain (Ki) | 0.0002 | V/ns/s | = 2 × 10⁻⁴ V/ns/s |
| Derivative Gain (Kd) | 0.0 | V/ns/s | disabled (PI mode) |
| Target Phase (φ_target) | 0.0 | ns | resonance setpoint |
| Center Voltage (V_center) | 1.65 | V | DAC mid-scale |
| Max Slew Rate | 30.0 | V/s | DAC ramp limit |
| Loop Period | 20 | ms | control update interval |
| Output Clamp | 0.0 – 3.3 | V | DAC absolute bounds |
| Lock Threshold | 500 | ns | abs(φ) ≤ 500 ns = LOCK |

These constants are initialized in `setup()` via the calls `dpll::begin(1.65f, 0.000002f, 0.0002f)`, `dpll::setTargetPhase(0.0f)`, `dpll::setOutputLimits(0.0f, 3.3f)`, and `dpll::setMaxSlew(30.0f)`, together with the default `g_loopPeriodMs = 20` and `kLockThresholdNs = 500.0f`.

---

## 5. UART Command Reference

| Command Format | Example | Action |
|---|---|---|
| `kp <val>` | `kp 0.001` | Set Proportional Gain |
| `ki <val>` | `ki 0.0001` | Set Integral Gain |
| `kd <val>` | `kd 0.00005` | Set Derivative Gain |
| `target <val>` | `target 0.0` | Set Target Phase Setpoint (ns) |
| `center <val>` | `center 1.65` | Set Center Voltage (V) |
| `slew <val>` | `slew 30.0` | Set Slew Rate Limit (V/s) |
| `loop <val>` | `loop 20.0` | Set Control Loop Period (ms) |
| `status` | `status` | Read current parameters and state |
