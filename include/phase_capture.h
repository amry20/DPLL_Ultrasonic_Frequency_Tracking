/*
 * phase_capture.h
 *
 * Dual-channel Input Capture using TIM2 (32-bit hardware timer).
 *
 * Channel 1 (PA0) -> Reference square wave (from waveform generator)
 * Channel 2 (PA1) -> Zero-Crossing Detector (ZCD) square wave
 *
 * Edge polarity for each channel can be selected independently:
 *   - CAPTURE_RISING   : capture on rising edge (default)
 *   - CAPTURE_FALLING  : capture on falling edge
 *   - CAPTURE_BOTH     : capture on both edges
 *
 * Prescaler is 1 (84 MHz clock tick = ~11.9 ns resolution).
 * Rollover is 0xFFFFFFFF (32-bit full scale ~51 seconds).
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace phase_capture {

enum EdgePolarity : uint8_t {
  CAPTURE_RISING  = 0,
  CAPTURE_FALLING = 1,
  CAPTURE_BOTH    = 2
};

struct CaptureData {
  uint32_t refTimestamp;    // TIM2 tick count on reference edge
  uint32_t zcdTimestamp;    // TIM2 tick count on ZCD edge
  uint32_t periodTicks;     // Reference signal period in TIM2 ticks
  int32_t  phaseDiffTicks;  // (ZCD - REF) wrapped to [-period/2 .. +period/2]
  float    phaseDiffNs;     // Phase difference as time delay ZCD vs REF (nanoseconds)
  float    frequencyHz;     // Measured reference frequency in Hz
  uint32_t captureCount;    // Incremented every reference edge
  uint32_t zcdCaptureCount; // Incremented every ZCD edge
  bool     refValid;        // True if reference signal active & period measured
  bool     zcdPresent;      // True if ZCD pulse received within timeout window
  bool     valid;           // True if both REF and ZCD are valid & current
};

// Robust result calculated from every paired REF/ZCD sample collected during
// one control window.  The ISR only records integer timer values; all floating
// point filtering is performed when consumeWindowData() is called from loop().
struct PhaseWindowData {
  float    phaseDiffNs;       // Robust phase estimate at the end of the window
  float    phaseDegrees;      // Same estimate, wrapped to [-180, +180] degrees
  float    slopeDegPerMs;     // Robust phase trend across the window
  float    noiseMadDeg;       // Robust residual noise estimate (1-sigma MAD)
  float    outlierRatio;      // Fraction of samples rejected [0, 1]
  uint16_t sampleCount;       // Paired REF/ZCD samples captured in this window
  uint16_t acceptedCount;     // Samples retained by the robust estimator
  bool     overflow;          // Sample bank filled before the window ended
  bool     largeShift;        // Large intra/inter-window phase movement detected
  bool     trustworthy;       // Safe to feed this estimate to the PID
  bool     phaseComputed;     // phaseDiffNs holds a usable (maybe untrusted) estimate
};

// Configurable setup function.
// refPin  : defaults to PA0 (TIM2_CH1)
// zcdPin  : defaults to PA1 (TIM2_CH2)
// refEdge : rising/falling/both
// zcdEdge : rising/falling/both
// zcdTimeoutMs: Timeout to consider ZCD signal missing (default 50 ms)
void begin(uint32_t refPin  = PA0,
           uint32_t zcdPin  = PA1,
           EdgePolarity refEdge = CAPTURE_RISING,
           EdgePolarity zcdEdge = CAPTURE_RISING,
           uint32_t zcdTimeoutMs = 50);

// Set ZCD timeout in milliseconds
void setZcdTimeout(uint32_t timeoutMs);

// Set reference signal timeout in milliseconds
// (used to detect that the generator input disappeared)
void setRefTimeout(uint32_t timeoutMs);

// Set/change edge polarity after begin() if needed
void setRefEdge(EdgePolarity edge);
void setZcdEdge(EdgePolarity edge);

// Get currently configured edge polarity
EdgePolarity getRefEdge();
EdgePolarity getZcdEdge();

// Get the latest snapshot of captured phase & period metrics
CaptureData getData();

// Atomically close the current capture window, start a fresh one, and process
// all samples from the closed window. Intended to be called once per PID period
// (20 ms by default). A low-confidence result must not be sent to the PID.
PhaseWindowData consumeWindowData();

// Discard samples captured while the previous window was being processed.
// Call immediately after applying (or deliberately holding) the DAC so the
// next 20 ms measurement window starts at the actual control decision time.
void clearWindowData();

// Reset robust filter state (anchor, shift candidates) so a new pulse immediately acquires
void resetFilterState();

// Helper: convert TIM2 ticks to microseconds
float ticksToUs(uint32_t ticks);

} // namespace phase_capture
