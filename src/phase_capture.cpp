/*
 * phase_capture.cpp
 *
 * Implementation of 32-bit dual input capture on TIM2 using Arduino HardwareTimer API.
 */

#include "phase_capture.h"
#include <HardwareTimer.h>
#include <algorithm>
#include <cmath>

namespace phase_capture {

namespace {

HardwareTimer* s_timer = nullptr;

uint32_t s_refPin = PA0;
uint32_t s_zcdPin = PA1;

EdgePolarity s_refEdge = CAPTURE_RISING;
EdgePolarity s_zcdEdge = CAPTURE_RISING;

// Volatile state written inside ISR
volatile uint32_t s_lastRefCapture = 0;
volatile uint32_t s_prevRefCapture = 0;
volatile uint32_t s_lastZcdCapture = 0;
volatile uint32_t s_periodTicks    = 0;
volatile uint32_t s_captureCount   = 0;
volatile uint32_t s_zcdCount       = 0;
volatile uint32_t s_lastZcdMs      = 0;
volatile uint32_t s_lastRefMs      = 0;
volatile bool     s_refValid       = false;
volatile int32_t  s_lastSynchronizedDiff = 0;
volatile bool     s_haveSynchronizedDiff = false;
volatile uint8_t  s_zcdStartupEdges      = 0;

uint32_t s_zcdTimeoutMs = 50;
uint32_t s_refTimeoutMs = 100;

// Maximum allowed age of the last ZCD edge before declaring ZCD absent.
// At ~33.5 kHz, 1 period is ~29.8 us. 6 periods = ~179 us (< 0.2 ms).
// If 6 consecutive cycles have no ZCD edge, the pulse has ended or signal is lost.
constexpr uint32_t kZcdMaxAgePeriods = 6;

static float    s_stableRefFreqHz = 0.0f;
static uint32_t s_stableRefMs     = 0;
constexpr float kPulseEndFreqJumpHz = 200.0f;
constexpr uint32_t kStableRefFreshMs = 200U;

constexpr float kTimerClockHz = 84000000.0f; // 84 MHz APB1 timer clock on STM32F407

// At 35 kHz a 20 ms window contains about 700 samples. 1024 leaves headroom
// without using dynamic allocation. If a longer runtime-configured loop period
// overflows the bank, that window is rejected rather than silently truncated.
constexpr size_t kMaxWindowSamples = 1024;
constexpr size_t kMaxBlockSize     = 32;
constexpr size_t kMinBlockSize     = 4;
constexpr size_t kTargetBlockCount = 8;
constexpr size_t kMaxBlocks        = 32;

// Robust-filter tuning. Values are deliberately conservative: a large change
// is held for confirmation instead of immediately moving the power stage.
constexpr float kMinGradientGateDeg    = 3.0f;
constexpr float kGradientMadMultiplier = 6.0f;
constexpr float kMinResidualGateDeg    = 5.0f;
constexpr float kResidualMadMultiplier = 4.0f;
constexpr float kMaxResidualNoiseDeg   = 12.0f;
constexpr float kMaxWindowShiftDeg     = 60.0f;
constexpr float kShiftConfirmBandDeg   = 15.0f;
constexpr float kMinAcceptedRatio      = 0.70f;
constexpr uint16_t kMinWindowSamples   = 12;
constexpr size_t kMinAcceptedBlocks    = 3;
constexpr float kMinWindowDurationMs   = 0.25f;
constexpr uint32_t kShiftConfirmMs     = 20;

struct PhaseSample {
  int32_t  phaseTicks;     // Already wrapped to [-period/2, +period/2]
  uint32_t periodTicks;
  uint32_t timestampTicks; // ZCD timestamp; subtraction is rollover-safe
};

// Double-buffered capture storage. The ISR writes only to the active bank.
// consumeWindowData() swaps banks with interrupts briefly disabled, then does
// all expensive work on the inactive bank while capture continues normally.
PhaseSample       s_sampleBanks[2][kMaxWindowSamples]{};
volatile uint16_t s_bankCount[2]    = {0, 0};
volatile bool     s_bankOverflow[2] = {false, false};
volatile uint8_t  s_activeBank      = 0;

// Static workspaces keep the MCU stack small and avoid heap allocation.
float s_wrappedDeg[kMaxWindowSamples]{};
float s_unwrappedDeg[kMaxWindowSamples]{};
float s_gradientDeg[kMaxWindowSamples]{};
bool  s_gradientOutlier[kMaxWindowSamples]{};
float s_sortScratch[kMaxWindowSamples]{};

float    s_blockTimeMs[kMaxBlocks]{};
float    s_blockPhaseDeg[kMaxBlocks]{};
uint16_t s_blockGoodSamples[kMaxBlocks]{};
bool     s_blockAccepted[kMaxBlocks]{};

bool  s_haveTrustedWindow = false;
float s_lastTrustedPhaseDeg = 0.0f;
bool  s_haveShiftCandidate = false;
float s_shiftCandidateDeg = 0.0f;
uint32_t s_shiftCandidateSinceMs = 0;

float wrapDegrees(float degrees)
{
  while (degrees > 180.0f) degrees -= 360.0f;
  while (degrees < -180.0f) degrees += 360.0f;
  return degrees;
}

float medianInPlace(float* values, size_t count)
{
  if (count == 0) return 0.0f;
  std::sort(values, values + count);
  if ((count & 1U) != 0U) return values[count / 2U];
  return 0.5f * (values[count / 2U - 1U] + values[count / 2U]);
}

float medianCopy(const float* values, size_t count)
{
  if (count == 0) return 0.0f;
  for (size_t i = 0; i < count; ++i) s_sortScratch[i] = values[i];
  return medianInPlace(s_sortScratch, count);
}

float robustSigmaFromMad(const float* values, size_t count, float center)
{
  if (count == 0) return 0.0f;
  for (size_t i = 0; i < count; ++i) {
    s_sortScratch[i] = fabsf(values[i] - center);
  }
  return 1.4826f * medianInPlace(s_sortScratch, count);
}

void storePhaseSampleFromIsr(uint32_t zcdCapture)
{
  if (!s_refValid || s_periodTicks == 0U) return;

  int32_t diff = static_cast<int32_t>(zcdCapture - s_lastRefCapture);
  const int32_t period = static_cast<int32_t>(s_periodTicks);
  const int32_t halfPeriod = period / 2;

  while (diff > halfPeriod) diff -= period;
  while (diff < -halfPeriod) diff += period;

  // Save atomically paired difference for instant snapshot reads in getData()
  s_lastSynchronizedDiff = diff;
  s_haveSynchronizedDiff = true;

  const uint8_t bank = s_activeBank;
  const uint16_t index = s_bankCount[bank];
  if (index >= kMaxWindowSamples) {
    s_bankOverflow[bank] = true;
    return;
  }

  s_sampleBanks[bank][index].phaseTicks = diff;
  s_sampleBanks[bank][index].periodTicks = s_periodTicks;
  s_sampleBanks[bank][index].timestampTicks = zcdCapture;
  s_bankCount[bank] = static_cast<uint16_t>(index + 1U);
}

TimerModes_t toTimerMode(EdgePolarity edge)
{
  switch (edge) {
    case CAPTURE_FALLING: return TIMER_INPUT_CAPTURE_FALLING;
    case CAPTURE_BOTH:    return TIMER_INPUT_CAPTURE_BOTHEDGE;
    case CAPTURE_RISING:
    default:              return TIMER_INPUT_CAPTURE_RISING;
  }
}

// ISR Callback for Channel 1 (Reference pin)
void onRefCapture()
{
  uint32_t cap = s_timer->getCaptureCompare(1);
  s_prevRefCapture = s_lastRefCapture;
  s_lastRefCapture = cap;
  s_lastRefMs      = millis();

  if (s_captureCount > 0) {
    s_periodTicks = s_lastRefCapture - s_prevRefCapture; // Handles 32-bit overflow automatically
    s_refValid = (s_periodTicks > 0);
  }
  s_captureCount++;
}

// ISR Callback for Channel 2 (ZCD pin)
void onZcdCapture()
{
  uint32_t cap = s_timer->getCaptureCompare(2);
  uint32_t nowMs = millis();

  // Fresh onset detection: if ZCD was absent longer than timeout, reset startup edge blanking
  if (s_lastZcdMs == 0 || (nowMs - s_lastZcdMs) > s_zcdTimeoutMs) {
    s_zcdStartupEdges = 0;
  }

  s_lastZcdCapture = cap;
  s_lastZcdMs      = nowMs;
  s_zcdCount++;

  // Discard the first 4 edges (~120 us) at onset to shield buffer from FET switching transients
  if (s_zcdStartupEdges < 4) {
    s_zcdStartupEdges++;
    return;
  }

  storePhaseSampleFromIsr(s_lastZcdCapture);
}

} // namespace

void begin(uint32_t refPin, uint32_t zcdPin, EdgePolarity refEdge, EdgePolarity zcdEdge, uint32_t zcdTimeoutMs)
{
  s_refPin       = refPin;
  s_zcdPin       = zcdPin;
  s_refEdge      = refEdge;
  s_zcdEdge      = zcdEdge;
  s_zcdTimeoutMs = zcdTimeoutMs;

  // Pin check: TIM2_CH1 is PA0, TIM2_CH2 is PA1
  TIM_TypeDef* instance = (TIM_TypeDef*)pinmap_peripheral(digitalPinToPinName(s_refPin), PinMap_TIM);
  if (instance != TIM2) {
    // Expected TIM2 for 32-bit resolution
    return;
  }

  s_timer = new HardwareTimer(TIM2);

  // Configure 32-bit timer: prescaler = 1 (full 84 MHz count speed)
  s_timer->setPrescaleFactor(1);
  s_timer->setOverflow(0xFFFFFFFF, TICK_FORMAT); // Full 32-bit range

  // Configure Channel 1 (Reference)
  s_timer->setMode(1, toTimerMode(s_refEdge), s_refPin);
  s_timer->attachInterrupt(1, onRefCapture);

  // Configure Channel 2 (ZCD)
  s_timer->setMode(2, toTimerMode(s_zcdEdge), s_zcdPin);
  s_timer->attachInterrupt(2, onZcdCapture);

  // Reset counters and resume timer
  s_captureCount = 0;
  s_zcdCount     = 0;
  s_refValid     = false;
  s_lastZcdMs    = 0;
  s_lastRefMs    = 0;
  s_activeBank   = 0;
  s_bankCount[0] = 0;
  s_bankCount[1] = 0;
  s_bankOverflow[0] = false;
  s_bankOverflow[1] = false;
  s_haveTrustedWindow = false;
  s_haveShiftCandidate = false;
  s_shiftCandidateSinceMs = 0;
  s_timer->resume();
}

void setZcdTimeout(uint32_t timeoutMs)
{
  s_zcdTimeoutMs = timeoutMs;
}

void setRefTimeout(uint32_t timeoutMs)
{
  s_refTimeoutMs = timeoutMs;
}


void setRefEdge(EdgePolarity edge)
{
  s_refEdge = edge;
  s_refValid = false;
  s_captureCount = 0;
  if (s_timer) {
    s_timer->setMode(1, toTimerMode(s_refEdge), s_refPin);
  }
}

void setZcdEdge(EdgePolarity edge)
{
  s_zcdEdge = edge;
  if (s_timer) {
    s_timer->setMode(2, toTimerMode(s_zcdEdge), s_zcdPin);
  }
}

EdgePolarity getRefEdge()
{
  return s_refEdge;
}

EdgePolarity getZcdEdge()
{
  return s_zcdEdge;
}

CaptureData getData()
{
  CaptureData out{};

  uint32_t lastZcdMs = 0;
  uint32_t lastRefMs = 0;
  uint32_t nowTicks  = 0;
  int32_t  syncDiff  = 0;
  bool     haveSync  = false;

  // Atomic snapshot of ISR variables
  noInterrupts();
  nowTicks            = s_timer ? s_timer->getCount() : 0;
  out.refTimestamp    = s_lastRefCapture;
  out.zcdTimestamp    = s_lastZcdCapture;
  out.periodTicks     = s_periodTicks;
  out.captureCount    = s_captureCount;
  out.zcdCaptureCount = s_zcdCount;
  out.refValid        = s_refValid;
  lastZcdMs           = s_lastZcdMs;
  lastRefMs           = s_lastRefMs;
  syncDiff            = s_lastSynchronizedDiff;
  haveSync            = s_haveSynchronizedDiff;
  interrupts();

  uint32_t now = millis();

  // Freshness checks: signal must be present AND recently active
  out.zcdPresent = (lastZcdMs > 0) && ((now - lastZcdMs) <= s_zcdTimeoutMs);
  if (out.zcdPresent && out.periodTicks != 0) {
    const uint32_t zcdAgeTicks = nowTicks - out.zcdTimestamp;
    if (zcdAgeTicks > kZcdMaxAgePeriods * out.periodTicks) {
      out.zcdPresent = false;
    }
  }
  out.refValid   = s_refValid && (lastRefMs > 0) && ((now - lastRefMs) <= s_refTimeoutMs);
  out.valid      = out.refValid && out.zcdPresent;

  if (!out.refValid || out.periodTicks == 0) {
    out.phaseDiffTicks = 0;
    out.phaseDiffNs    = 0.0f;
    out.frequencyHz    = 0.0f;
    return out;
  }

  // Frequency in Hz
  out.frequencyHz = kTimerClockHz / static_cast<float>(out.periodTicks);

  // Pulse-end detection: if REF frequency abruptly jumps while ZCD is present,
  // the generator has switched to its idle frequency (+500 Hz jump) at the end of a pulse.
  // Both the DPLL control loop and monitor stream must immediately treat this as signal loss.
  if (out.zcdPresent) {
    const bool stableFresh = (s_stableRefMs != 0U) && ((now - s_stableRefMs) <= kStableRefFreshMs);
    if (stableFresh && fabsf(out.frequencyHz - s_stableRefFreqHz) > kPulseEndFreqJumpHz) {
      out.zcdPresent = false;
      out.valid      = false;
      out.phaseDiffTicks = 0;
      out.phaseDiffNs    = 0.0f;
    } else {
      s_stableRefFreqHz = stableFresh ? (0.8f * s_stableRefFreqHz + 0.2f * out.frequencyHz)
                                      : out.frequencyHz;
      s_stableRefMs     = now;
    }
  }

  if (!out.zcdPresent) {
    // ZCD not present/too old -> phase diff invalid
    out.phaseDiffTicks = 0;
    out.phaseDiffNs    = 0.0f;
    out.valid          = false;
    return out;
  }

  // Use cycle-synchronized difference if available; fallback to raw timestamp delta
  int32_t diff = 0;
  if (haveSync) {
    diff = syncDiff;
  } else {
    diff = static_cast<int32_t>(out.zcdTimestamp - out.refTimestamp);
    int32_t halfPeriod = static_cast<int32_t>(out.periodTicks / 2);
    while (diff > halfPeriod) {
      diff -= static_cast<int32_t>(out.periodTicks);
    }
    while (diff < -halfPeriod) {
      diff += static_cast<int32_t>(out.periodTicks);
    }
  }

  out.phaseDiffTicks = diff;

  // 3) Convert phase diff to nanoseconds (time delay ZCD vs REF).
  //    Timer tick = 1/84 MHz ~= 11.9 ns.
  out.phaseDiffNs = static_cast<float>(diff) * (1e9f / kTimerClockHz);

  return out;
}

PhaseWindowData consumeWindowData()
{
  PhaseWindowData result{};

  // Close the active bank and immediately give the ISR a clean bank. This is
  // the only critical section; filtering below runs with interrupts enabled.
  noInterrupts();
  const uint8_t processingBank = s_activeBank;
  const uint8_t newActiveBank = static_cast<uint8_t>(processingBank ^ 1U);
  s_bankCount[newActiveBank] = 0;
  s_bankOverflow[newActiveBank] = false;
  s_activeBank = newActiveBank;
  const uint16_t sampleCount = s_bankCount[processingBank];
  const bool overflow = s_bankOverflow[processingBank];
  interrupts();

  result.sampleCount = sampleCount;
  result.overflow = overflow;
  if (sampleCount < 2U) {
    s_haveShiftCandidate = false;
    return result;
  }

  const PhaseSample* samples = s_sampleBanks[processingBank];

  // Convert each paired capture to phase angle. Using angle rather than raw
  // nanoseconds keeps the filter threshold independent of drive frequency.
  uint16_t phaseHistogram[72]{}; // 5-degree circular bins
  uint64_t periodSum = 0;
  for (uint16_t i = 0; i < sampleCount; ++i) {
    const uint32_t period = samples[i].periodTicks;
    if (period == 0U) continue;
    float phase = (static_cast<float>(samples[i].phaseTicks) * 360.0f) /
                  static_cast<float>(period);
    phase = wrapDegrees(phase);
    s_wrappedDeg[i] = phase;
    periodSum += period;

    int bin = static_cast<int>((phase + 180.0f) / 5.0f);
    if (bin < 0) bin = 0;
    if (bin > 71) bin = 71;
    phaseHistogram[bin]++;
  }

  // Select the densest circular neighbourhood as an unwrap anchor. This is
  // robust against an isolated burst being the first sample in the window.
  uint16_t bestDensity = 0;
  int bestBin = 0;
  for (int bin = 0; bin < 72; ++bin) {
    const int previous = (bin + 71) % 72;
    const int next = (bin + 1) % 72;
    const uint16_t density = static_cast<uint16_t>(phaseHistogram[previous] +
                                                   phaseHistogram[bin] +
                                                   phaseHistogram[next]);
    if (density > bestDensity) {
      bestDensity = density;
      bestBin = bin;
    }
  }
  const float anchorDeg = -180.0f + (static_cast<float>(bestBin) + 0.5f) * 5.0f;

  for (uint16_t i = 0; i < sampleCount; ++i) {
    s_unwrappedDeg[i] = anchorDeg + wrapDegrees(s_wrappedDeg[i] - anchorDeg);
    s_gradientOutlier[i] = false;
  }

  // Circular gradient prevents the normal +180/-180 boundary crossing from
  // looking like a 360-degree spike.
  for (uint16_t i = 1; i < sampleCount; ++i) {
    s_gradientDeg[i - 1U] = wrapDegrees(s_wrappedDeg[i] - s_wrappedDeg[i - 1U]);
  }
  const size_t gradientCount = static_cast<size_t>(sampleCount - 1U);
  const float gradientMedian = medianCopy(s_gradientDeg, gradientCount);
  const float gradientSigma = robustSigmaFromMad(s_gradientDeg, gradientCount,
                                                  gradientMedian);
  const float gradientGate = std::max(kMinGradientGateDeg,
                                      kGradientMadMultiplier * gradientSigma);
  for (uint16_t i = 1; i < sampleCount; ++i) {
    if (fabsf(s_gradientDeg[i - 1U] - gradientMedian) > gradientGate) {
      s_gradientOutlier[i] = true;
    }
  }

  // Reduce the window to medians of small chronological blocks. A burst may
  // occupy several consecutive samples; robust regression below rejects whole
  // displaced blocks instead of allowing a plateau to bias the PID input.
  size_t blockCount = 0;
  size_t blockSize = static_cast<size_t>(sampleCount) / kTargetBlockCount;
  blockSize = std::max(kMinBlockSize, std::min(kMaxBlockSize, blockSize));
  float blockValues[kMaxBlockSize]{};
  const uint32_t firstTimestamp = samples[0].timestampTicks;
  for (uint16_t start = 0; start < sampleCount; start += blockSize) {
    const uint16_t end = static_cast<uint16_t>(
        std::min<size_t>(static_cast<size_t>(sampleCount),
                         static_cast<size_t>(start) + blockSize));
    size_t goodCount = 0;
    float timeSumMs = 0.0f;
    for (uint16_t i = start; i < end; ++i) {
      if (s_gradientOutlier[i]) continue;
      blockValues[goodCount++] = s_unwrappedDeg[i];
      const uint32_t elapsedTicks = samples[i].timestampTicks - firstTimestamp;
      timeSumMs += static_cast<float>(elapsedTicks) / (kTimerClockHz * 0.001f);
    }

    const size_t blockLength = static_cast<size_t>(end - start);
    const size_t minimumGood = std::max<size_t>(2U, (blockLength + 1U) / 2U);
    if (goodCount < minimumGood || blockCount >= kMaxBlocks) continue;

    s_blockTimeMs[blockCount] = timeSumMs / static_cast<float>(goodCount);
    s_blockPhaseDeg[blockCount] = medianInPlace(blockValues, goodCount);
    s_blockGoodSamples[blockCount] = static_cast<uint16_t>(goodCount);
    s_blockAccepted[blockCount] = false;
    blockCount++;
  }

  if (blockCount < 2U) {
    s_haveShiftCandidate = false;
    return result;
  }

  // Theil-Sen gives a robust preliminary line using at most 496 slopes for 32
  // blocks, which is inexpensive compared with doing all 600x600 pairs.
  size_t slopeCount = 0;
  for (size_t a = 0; a < blockCount; ++a) {
    for (size_t b = a + 1U; b < blockCount; ++b) {
      const float dx = s_blockTimeMs[b] - s_blockTimeMs[a];
      if (dx > 1e-6f) {
        s_sortScratch[slopeCount++] =
            (s_blockPhaseDeg[b] - s_blockPhaseDeg[a]) / dx;
      }
    }
  }
  const float robustSlope = medianInPlace(s_sortScratch, slopeCount);
  for (size_t i = 0; i < blockCount; ++i) {
    s_sortScratch[i] = s_blockPhaseDeg[i] - robustSlope * s_blockTimeMs[i];
  }
  const float robustIntercept = medianInPlace(s_sortScratch, blockCount);

  for (size_t i = 0; i < blockCount; ++i) {
    s_sortScratch[i] = fabsf(s_blockPhaseDeg[i] -
        (robustIntercept + robustSlope * s_blockTimeMs[i]));
  }
  const float residualMad = medianInPlace(s_sortScratch, blockCount);
  const float residualSigma = 1.4826f * residualMad;
  const float residualGate = std::max(kMinResidualGateDeg,
                                      kResidualMadMultiplier * residualSigma);

  size_t acceptedBlocks = 0;
  uint16_t acceptedSamples = 0;
  float sumX = 0.0f;
  float sumY = 0.0f;
  for (size_t i = 0; i < blockCount; ++i) {
    const float residual = s_blockPhaseDeg[i] -
        (robustIntercept + robustSlope * s_blockTimeMs[i]);
    if (fabsf(residual) <= residualGate) {
      s_blockAccepted[i] = true;
      acceptedBlocks++;
      acceptedSamples = static_cast<uint16_t>(acceptedSamples + s_blockGoodSamples[i]);
      sumX += s_blockTimeMs[i];
      sumY += s_blockPhaseDeg[i];
    }
  }

  result.acceptedCount = acceptedSamples;
  result.outlierRatio = 1.0f -
      static_cast<float>(acceptedSamples) / static_cast<float>(sampleCount);
  result.noiseMadDeg = residualSigma;

  if (acceptedBlocks < 2U) {
    s_haveShiftCandidate = false;
    return result;
  }

  // Final least-squares line uses only blocks accepted by the robust pass.
  const float meanX = sumX / static_cast<float>(acceptedBlocks);
  const float meanY = sumY / static_cast<float>(acceptedBlocks);
  float covariance = 0.0f;
  float varianceX = 0.0f;
  for (size_t i = 0; i < blockCount; ++i) {
    if (!s_blockAccepted[i]) continue;
    const float dx = s_blockTimeMs[i] - meanX;
    covariance += dx * (s_blockPhaseDeg[i] - meanY);
    varianceX += dx * dx;
  }
  const float finalSlope = (varianceX > 1e-6f) ? covariance / varianceX : 0.0f;
  const float finalIntercept = meanY - finalSlope * meanX;
  const uint32_t durationTicks = samples[sampleCount - 1U].timestampTicks - firstTimestamp;
  const float durationMs = static_cast<float>(durationTicks) /
                           (kTimerClockHz * 0.001f);
  const float endpointUnwrappedDeg = finalIntercept + finalSlope * durationMs;
  const float endpointDeg = wrapDegrees(endpointUnwrappedDeg);
  const float windowShiftDeg = finalSlope * durationMs;

  result.phaseDegrees = endpointDeg;
  result.slopeDegPerMs = finalSlope;
  result.largeShift = fabsf(windowShiftDeg) > kMaxWindowShiftDeg;

  const float averagePeriodTicks = static_cast<float>(periodSum) /
                                   static_cast<float>(sampleCount);
  const float periodNs = averagePeriodTicks * (1.0e9f / kTimerClockHz);
  result.phaseDiffNs = (endpointDeg / 360.0f) * periodNs;
  result.phaseComputed = true;

  bool locallyTrustworthy = !overflow &&
      sampleCount >= kMinWindowSamples &&
      acceptedBlocks >= kMinAcceptedBlocks &&
      (1.0f - result.outlierRatio) >= kMinAcceptedRatio &&
      residualSigma <= kMaxResidualNoiseDeg &&
      durationMs >= kMinWindowDurationMs &&
      !result.largeShift;

  // Confirm a clean but very large inter-window jump for a fixed amount of
  // real time. At a 1 ms loop this requires many consistent windows; at 20 ms
  // it retains the previous two-window behaviour.
  if (locallyTrustworthy && s_haveTrustedWindow) {
    const float interWindowShift = fabsf(wrapDegrees(endpointDeg - s_lastTrustedPhaseDeg));
    if (interWindowShift > kMaxWindowShiftDeg) {
      result.largeShift = true;
      const uint32_t nowMs = millis();
      if (s_haveShiftCandidate &&
          fabsf(wrapDegrees(endpointDeg - s_shiftCandidateDeg)) <= kShiftConfirmBandDeg) {
        if ((nowMs - s_shiftCandidateSinceMs) >= kShiftConfirmMs) {
          s_lastTrustedPhaseDeg = endpointDeg;
          s_haveShiftCandidate = false;
        } else {
          locallyTrustworthy = false;
        }
      } else {
        s_shiftCandidateDeg = endpointDeg;
        s_haveShiftCandidate = true;
        s_shiftCandidateSinceMs = nowMs;
        locallyTrustworthy = false;
      }
    } else {
      s_lastTrustedPhaseDeg = endpointDeg;
      s_haveShiftCandidate = false;
    }
  } else if (locallyTrustworthy) {
    s_lastTrustedPhaseDeg = endpointDeg;
    s_haveTrustedWindow = true;
    s_haveShiftCandidate = false;
  } else {
    s_haveShiftCandidate = false;
  }

  result.trustworthy = locallyTrustworthy;
  return result;
}

void clearWindowData()
{
  noInterrupts();
  const uint8_t bank = s_activeBank;
  s_bankCount[bank] = 0;
  s_bankOverflow[bank] = false;
  interrupts();
}

void resetFilterState()
{
  s_haveTrustedWindow = false;
  s_haveShiftCandidate = false;
  s_shiftCandidateSinceMs = 0;
  s_lastTrustedPhaseDeg = 0.0f;
  s_haveSynchronizedDiff = false;
  s_zcdStartupEdges = 0;
}

float ticksToUs(uint32_t ticks)
{
  return (static_cast<float>(ticks) / kTimerClockHz) * 1e6f;
}

} // namespace phase_capture
