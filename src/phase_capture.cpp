/*
 * phase_capture.cpp
 *
 * Implementation of 32-bit dual input capture on TIM2 using Arduino HardwareTimer API.
 */

#include "phase_capture.h"
#include <HardwareTimer.h>

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

uint32_t s_zcdTimeoutMs = 50;
uint32_t s_refTimeoutMs = 100;

constexpr float kTimerClockHz = 84000000.0f; // 84 MHz APB1 timer clock on STM32F407

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
  s_lastZcdCapture = s_timer->getCaptureCompare(2);
  s_lastZcdMs      = millis();
  s_zcdCount++;
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

CaptureData getData()
{
  CaptureData out{};

  uint32_t lastZcdMs = 0;
  uint32_t lastRefMs = 0;

  // Atomic snapshot of ISR variables
  noInterrupts();
  out.refTimestamp    = s_lastRefCapture;
  out.zcdTimestamp    = s_lastZcdCapture;
  out.periodTicks     = s_periodTicks;
  out.captureCount    = s_captureCount;
  out.zcdCaptureCount = s_zcdCount;
  out.refValid        = s_refValid;
  lastZcdMs           = s_lastZcdMs;
  lastRefMs           = s_lastRefMs;
  interrupts();

  uint32_t now = millis();

  // Freshness checks: signal must be present AND recently active
  out.zcdPresent = (lastZcdMs > 0) && ((now - lastZcdMs) <= s_zcdTimeoutMs);
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

  if (!out.zcdPresent) {
    // ZCD not present/too old -> phase diff invalid
    out.phaseDiffTicks = 0;
    out.phaseDiffNs    = 0.0f;
    return out;
  }

  // 1) Raw time difference: ZCD timestamp relative to REF timestamp
  int32_t diff = static_cast<int32_t>(out.zcdTimestamp - out.refTimestamp);

  // 2) Wrap difference to range [-periodTicks/2 .. +periodTicks/2]
  int32_t halfPeriod = static_cast<int32_t>(out.periodTicks / 2);
  while (diff > halfPeriod) {
    diff -= static_cast<int32_t>(out.periodTicks);
  }
  while (diff < -halfPeriod) {
    diff += static_cast<int32_t>(out.periodTicks);
  }

  out.phaseDiffTicks = diff;

  // 3) Convert phase diff to nanoseconds (time delay ZCD vs REF).
  //    Timer tick = 1/84 MHz ~= 11.9 ns.
  out.phaseDiffNs = static_cast<float>(diff) * (1e9f / kTimerClockHz);

  return out;
}

float ticksToUs(uint32_t ticks)
{
  return (static_cast<float>(ticks) / kTimerClockHz) * 1e6f;
}

} // namespace phase_capture

