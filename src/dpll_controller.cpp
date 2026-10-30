/*
 * dpll_controller.cpp
 *
 * Implementation of the DPLL PI controller supporting dual generator sources:
 *   1. GENERATOR_SOURCE_DAC : Controls VCO (CD4046) voltage via DAC1 (PA4)
 *   2. GENERATOR_SOURCE_DDS : Modulates frequency directly on AD9833 DDS via SPI1
 */

#include "dpll_controller.h"
#include "dac_output.h"
#include "dds_ad9833.h"
#include "dma_dithering_sqw.h"

namespace dpll {

namespace {

GeneratorSource s_generatorSource = GENERATOR_SOURCE_PWM_DITHER;
TrackingDirection s_trackingDirection = TRACKING_DIR_NORMAL;

// DAC mode PID gains (V/ns, V/ns/s)
float s_dacKp = 0.000002f;
float s_dacKi = 0.0002f;
float s_dacKd = 0.0f;

// DDS mode PID gains (Hz/ns, Hz/ns/s)
// Tuned for stable tracking without high-frequency phase hunting:
// 0.01 Hz/ns corresponds to 10 Hz per 1000 ns (~11.8° at 32 kHz).
float s_ddsKp = 0.01f;
float s_ddsKi = 0.1f;
float s_ddsKd = 0.0f;

float s_centerVoltage = 1.65f;
float s_targetPhaseNs = 600.0f;

float s_lastError = 0.0f;
bool  s_haveLastError = false;

// DAC limits & output
float s_minVoltage = 0.0f;
float s_maxVoltage = 3.3f;
float s_outputVoltage = 1.65f;
float s_maxSlew = 30.0f; // V/s

// DDS frequency limits & output
float s_ddsCenterFreq = 28000.0f; // Hz
float s_ddsMinFreq    = 20000.0f; // Hz
float s_ddsMaxFreq    = 40000.0f; // Hz
float s_ddsMaxSlew    = 20000.0f; // Hz/s
float s_outputFreq    = 28000.0f; // Hz

float s_integral = 0.0f;
bool s_enabled = true;

// --- Loop / acquisition config (configurable via opcode) ---
uint32_t s_loopPeriodMs      = 20;
float    s_lockThresholdNs   = 500.0f;
bool     s_manualMode        = false;
float    s_lockedCenterV     = 1.65f;
bool     s_haveLockedCenter  = false;
uint32_t s_lockHoldCycles    = 2;
uint32_t s_streamPeriodMs    = 10;   // monitor stream rate (ms); 10 ms = 100 Hz fixed

SignalLossBehavior s_signalLossBehavior = SIGNAL_LOSS_FREEZE;

// Lock memory timeout
uint32_t s_lockMemoryTimeoutMs = 5000; // 0 = never expire
uint32_t s_signalAbsentSince   = 0;    // millis() when signal first went absent
bool     s_signalWasAbsent     = false;

} // namespace

bool isFrequencyGenerator()
{
  return (s_generatorSource == GENERATOR_SOURCE_DDS || s_generatorSource == GENERATOR_SOURCE_PWM_DITHER);
}

void begin(float centerVoltage, float kp, float ki, float kd)
{
  s_centerVoltage = centerVoltage;
  if (isFrequencyGenerator()) {
    s_ddsKp = kp;
    s_ddsKi = ki;
    s_ddsKd = kd;
  } else {
    s_dacKp = kp;
    s_dacKi = ki;
    s_dacKd = kd;
  }
  s_integral = 0.0f;
  s_outputVoltage = centerVoltage;
  s_outputFreq = s_ddsCenterFreq;
  s_haveLastError = false;
  s_enabled = true;

  if (s_generatorSource == GENERATOR_SOURCE_PWM_DITHER) {
    dac::setVoltage(1.65f); // park DAC
    dma_sqw::begin(s_outputFreq);
    dma_sqw::enableOutput(true);
  } else if (s_generatorSource == GENERATOR_SOURCE_DDS) {
    dac::setVoltage(1.65f); // park DAC
    dds::setFrequency((double)s_outputFreq);
    dds::enableOutput(true);
  } else {
    dac::setVoltage(s_outputVoltage);
    // Keep DDS alive at configured frequency for scope testing / dual monitoring
    dds::setFrequency((double)s_outputFreq);
    dds::enableOutput(true);
  }
}

GeneratorSource getGeneratorSource()
{
  return s_generatorSource;
}

void setGeneratorSource(GeneratorSource src)
{
  if (src == s_generatorSource) {
    return;
  }

  // Stop previous generator if needed
  if (s_generatorSource == GENERATOR_SOURCE_PWM_DITHER) {
    dma_sqw::stop();
  }

  s_generatorSource = src;
  s_integral = 0.0f;
  s_haveLastError = false;

  if (s_generatorSource == GENERATOR_SOURCE_PWM_DITHER) {
    dac::setVoltage(1.65f);
    dds::enableOutput(false); // Mute DDS
    s_outputFreq = s_ddsCenterFreq;
    dma_sqw::begin(s_outputFreq);
    dma_sqw::enableOutput(s_enabled);
  } else if (s_generatorSource == GENERATOR_SOURCE_DDS) {
    dac::setVoltage(1.65f);
    dds::setMcoClock(dds::getMcoClock()); // Reconfigure PA8 back to MCO1
    s_outputFreq = s_ddsCenterFreq;
    dds::setFrequency((double)s_outputFreq);
    dds::enableOutput(s_enabled);
  } else {
    // Restore DAC to center voltage
    s_outputVoltage = s_centerVoltage;
    dac::setVoltage(s_outputVoltage);
    // Keep DDS running at center frequency
    s_outputFreq = s_ddsCenterFreq;
    dds::setFrequency((double)s_outputFreq);
  }
}

TrackingDirection getTrackingDirection()
{
  return s_trackingDirection;
}

void setTrackingDirection(TrackingDirection dir)
{
  s_trackingDirection = dir;
}

float getDdsCenterFreq()
{
  return s_ddsCenterFreq;
}

void setDdsCenterFreq(float freqHz)
{
  if (freqHz >= 1000.0f && freqHz <= 500000.0f) {
    s_ddsCenterFreq = freqHz;
    s_outputFreq = s_ddsCenterFreq;
    if (s_generatorSource == GENERATOR_SOURCE_PWM_DITHER) {
      dma_sqw::setFrequency(s_outputFreq);
    } else {
      dds::setFrequency((double)s_outputFreq);
    }
  }
}

float getDdsMinFreq()
{
  return s_ddsMinFreq;
}

float getDdsMaxFreq()
{
  return s_ddsMaxFreq;
}

void setDdsFreqLimits(float minHz, float maxHz)
{
  if (minHz >= 1000.0f && maxHz > minHz && maxHz <= 500000.0f) {
    s_ddsMinFreq = minHz;
    s_ddsMaxFreq = maxHz;
  }
}

float getDdsMaxSlew()
{
  return s_ddsMaxSlew;
}

void setDdsMaxSlew(float hzPerSecond)
{
  if (hzPerSecond > 0.0f) {
    s_ddsMaxSlew = hzPerSecond;
  }
}

float lastFrequency()
{
  if (s_generatorSource == GENERATOR_SOURCE_PWM_DITHER) {
    return dma_sqw::getFrequency();
  } else if (s_generatorSource == GENERATOR_SOURCE_DDS) {
    return static_cast<float>(dds::getFrequency());
  }
  return s_outputFreq;
}

void setTargetPhase(float targetNs)
{
  s_targetPhaseNs = targetNs;
}

void setGains(float kp, float ki)
{
  if (isFrequencyGenerator()) {
    s_ddsKp = kp;
    s_ddsKi = ki;
  } else {
    s_dacKp = kp;
    s_dacKi = ki;
  }
}

void setKd(float kd)
{
  if (isFrequencyGenerator()) {
    s_ddsKd = kd;
  } else {
    s_dacKd = kd;
  }
}

void setOutputLimits(float minV, float maxV)
{
  s_minVoltage = minV;
  s_maxVoltage = maxV;
}

void setMaxSlew(float ratePerSecond)
{
  if (ratePerSecond <= 0.0f) {
    ratePerSecond = isFrequencyGenerator() ? 20000.0f : 30.0f;
  }
  if (isFrequencyGenerator()) {
    s_ddsMaxSlew = ratePerSecond;
  } else {
    s_maxSlew = ratePerSecond;
  }
}

void enable(bool on)
{
  if (on && !s_enabled) {
    if (isFrequencyGenerator()) {
      s_integral = s_outputFreq - s_ddsCenterFreq;
      if (s_generatorSource == GENERATOR_SOURCE_PWM_DITHER) {
        dma_sqw::enableOutput(true);
      } else {
        dds::enableOutput(true);
      }
    } else {
      s_integral = s_outputVoltage - s_centerVoltage;
    }
    s_haveLastError = false;
  }
  if (on) {
    s_manualMode = false;
  }
  s_enabled = on;
}

bool isEnabled()
{
  return s_enabled;
}

void reset()
{
  s_manualMode = false;
  s_outputVoltage = s_centerVoltage;
  s_outputFreq = s_ddsCenterFreq;
  s_haveLastError = false;

  if (s_generatorSource == GENERATOR_SOURCE_PWM_DITHER) {
    dma_sqw::setFrequency(s_outputFreq);
  } else if (s_generatorSource == GENERATOR_SOURCE_DDS) {
    dds::setFrequency((double)s_outputFreq);
  } else {
    dac::setVoltage(s_outputVoltage);
  }
}

void restartAtVoltage(float startingVoltage)
{
  if (s_manualMode) return; // Do not disturb manual test mode
  if (startingVoltage < s_minVoltage) startingVoltage = s_minVoltage;
  if (startingVoltage > s_maxVoltage) startingVoltage = s_maxVoltage;
  s_outputVoltage = startingVoltage;
  s_haveLastError = false;
  s_enabled = true;
  if (s_generatorSource == GENERATOR_SOURCE_DAC) {
    dac::setVoltage(s_outputVoltage);
  }
}

void restartAtFrequency(float startingFreq)
{
  if (s_manualMode) return; // Do not disturb manual test mode
  if (startingFreq < s_ddsMinFreq) startingFreq = s_ddsMinFreq;
  if (startingFreq > s_ddsMaxFreq) startingFreq = s_ddsMaxFreq;
  s_outputFreq = startingFreq;
  s_haveLastError = false;
  s_enabled = true;
  if (s_generatorSource == GENERATOR_SOURCE_PWM_DITHER) {
    dma_sqw::enableOutput(true);
    dma_sqw::setFrequency(s_outputFreq);
  } else if (s_generatorSource == GENERATOR_SOURCE_DDS) {
    dds::enableOutput(true);
    dds::setFrequency((double)s_outputFreq);
  }
}

float update(float phaseErrorNs, float dtSeconds)
{
  if (!s_enabled) {
    return isFrequencyGenerator() ? s_outputFreq : s_outputVoltage;
  }

  // Sanitize dt (avoid huge steps on first call / overflow)
  if (dtSeconds <= 0.0f) {
    dtSeconds = 1e-6f;
  }
  if (dtSeconds > 1.0f) {
    dtSeconds = 1.0f; // Cap at 1 s to avoid integral windup spikes
  }

  // Error in nanoseconds (target - measured)
  float error = s_targetPhaseNs - phaseErrorNs;
  if (s_trackingDirection == TRACKING_DIR_INVERTED) {
    error = -error;
  }

  // Integral Deadband: When phase error is very close to target (<= 30 ns), freeze
  // integral accumulation to prevent hunting/creep. Full integral action is active
  // whenever phase is outside 30 ns so it quickly pulls phase right to the target!
  float intError = error;
  if (fabsf(error) <= 30.0f) {
    intError = 0.0f;
  }

  if (isFrequencyGenerator())
  {
    // --- Direct Frequency Tracking (Continuous Incremental Form: f_new = f_curr ± Δf) ---
    float deltaProp = s_ddsKp * error;
    float deltaInt  = s_ddsKi * intError * dtSeconds;

    float derivative = 0.0f;
    if (s_haveLastError) {
      derivative = s_ddsKd * (error - s_lastError) / dtSeconds;
    }
    s_lastError = error;
    s_haveLastError = true;

    // Accumulate integral smoothly from current operating point
    s_outputFreq += deltaInt;
    float targetFreq = s_outputFreq + deltaProp + derivative;

    // Anti-stall guard (Frequency mode):
    // If operating below resonance frequency and measured phase is in capacitive dead zone,
    // prevent downward positive feedback runaway by steering frequency back toward center.
    if ((phaseErrorNs > (s_targetPhaseNs + 800.0f)) &&
        (s_outputFreq < (s_ddsCenterFreq - 300.0f))) {
      const float recoveryStep = s_ddsMaxSlew * 0.5f * dtSeconds;
      s_outputFreq += recoveryStep;
      if (s_outputFreq > s_ddsCenterFreq) {
        s_outputFreq = s_ddsCenterFreq;
      }
      targetFreq = s_outputFreq;
      s_haveLastError = false;
    }

    // Clamp output within frequency bounds [min, max] with anti-windup
    if (targetFreq > s_ddsMaxFreq) {
      targetFreq = s_ddsMaxFreq;
      s_outputFreq = s_ddsMaxFreq - deltaProp;
    } else if (targetFreq < s_ddsMinFreq) {
      targetFreq = s_ddsMinFreq;
      s_outputFreq = s_ddsMinFreq - deltaProp;
    }

    // Frequency slew-rate limiting (Hz/s)
    float maxStep = s_ddsMaxSlew * dtSeconds;
    float lastF = (s_generatorSource == GENERATOR_SOURCE_PWM_DITHER)
                    ? dma_sqw::getFrequency()
                    : static_cast<float>(dds::getFrequency());
    if (lastF <= 0.0f) lastF = s_outputFreq;
    if (targetFreq - lastF > maxStep) {
      targetFreq = lastF + maxStep;
    } else if (lastF - targetFreq > maxStep) {
      targetFreq = lastF - maxStep;
    }

    if (s_generatorSource == GENERATOR_SOURCE_PWM_DITHER) {
      dma_sqw::setFrequency(targetFreq);
    } else {
      dds::setFrequency((double)targetFreq);
    }
    return targetFreq;
  }
  else
  {
    // --- External VCO DAC Voltage Tracking (Continuous Incremental Form: V_new = V_curr ± ΔV) ---
    float deltaProp = s_dacKp * error;
    float deltaInt  = s_dacKi * intError * dtSeconds;

    float derivative = 0.0f;
    if (s_haveLastError) {
      derivative = s_dacKd * (error - s_lastError) / dtSeconds;
    }
    s_lastError = error;
    s_haveLastError = true;

    s_outputVoltage += deltaInt;
    float targetV = s_outputVoltage + deltaProp + derivative;

    // Anti-stall guard (DAC mode):
    // When DAC drops below resonance (< center - 0.12 V) and the transducer slips
    // into the capacitive dead zone (phase >> target, e.g. > target + 800 ns),
    // the system experiences phase slope reversal (dPhase/dDAC < 0).
    // The negative error would normally drive the DAC into the bottom rail.
    // Detect this condition and steer voltage back up towards center to re-acquire resonance.
    if ((phaseErrorNs > (s_targetPhaseNs + 800.0f)) &&
        (s_outputVoltage < (s_centerVoltage - 0.12f))) {
      const float recoveryStep = s_maxSlew * 0.5f * dtSeconds;
      s_outputVoltage += recoveryStep;
      if (s_outputVoltage > s_centerVoltage) {
        s_outputVoltage = s_centerVoltage;
      }
      targetV = s_outputVoltage;
      s_haveLastError = false;
    }

    // Clamp output with anti-windup
    if (targetV > s_maxVoltage) {
      targetV = s_maxVoltage;
      s_outputVoltage = s_maxVoltage - deltaProp;
    } else if (targetV < s_minVoltage) {
      targetV = s_minVoltage;
      s_outputVoltage = s_minVoltage - deltaProp;
    }

    // Voltage slew-rate limit
    float maxStep = s_maxSlew * dtSeconds;
    float lastV = dac::lastRaw() * (3.3f / 4095.0f);
    if (targetV - lastV > maxStep) {
      targetV = lastV + maxStep;
    } else if (lastV - targetV > maxStep) {
      targetV = lastV - maxStep;
    }

    dac::setVoltage(targetV);
    return targetV;
  }
}

float lastVoltage()
{
  return (s_generatorSource == GENERATOR_SOURCE_DAC) ? (dac::lastRaw() * (3.3f / 4095.0f)) : s_outputVoltage;
}

void shutdown()
{
  s_enabled = false;
  s_integral = 0.0f;
  if (s_generatorSource == GENERATOR_SOURCE_PWM_DITHER) {
    dma_sqw::enableOutput(false);
  } else if (s_generatorSource == GENERATOR_SOURCE_DDS) {
    dds::enableOutput(false);
  } else {
    s_outputVoltage = s_minVoltage;
    dac::setVoltage(s_outputVoltage);
  }
}

void manualSet(float voltage)
{
  s_manualMode = true;
  s_enabled = false;
  s_integral = 0.0f;
  s_generatorSource = GENERATOR_SOURCE_DAC;
  if (voltage < s_minVoltage) { voltage = s_minVoltage; }
  if (voltage > s_maxVoltage) { voltage = s_maxVoltage; }
  s_outputVoltage = voltage;
  dac::setVoltage(s_outputVoltage);
}

void manualSetFreq(float freqHz)
{
  s_manualMode = true;
  s_enabled = false;
  s_integral = 0.0f;
  if (!isFrequencyGenerator()) {
    s_generatorSource = GENERATOR_SOURCE_PWM_DITHER;
  }
  if (freqHz < s_ddsMinFreq) freqHz = s_ddsMinFreq;
  if (freqHz > s_ddsMaxFreq) freqHz = s_ddsMaxFreq;
  s_outputFreq = freqHz;
  dac::setVoltage(1.65f); // Park DAC safely at 1.65 V neutral
  if (s_generatorSource == GENERATOR_SOURCE_PWM_DITHER) {
    dma_sqw::setFrequency(s_outputFreq);
    dma_sqw::enableOutput(true);
  } else {
    dds::enableOutput(true);
    dds::setFrequency((double)s_outputFreq);
  }
}

float getKp()            { return isFrequencyGenerator() ? s_ddsKp : s_dacKp; }
float getKi()            { return isFrequencyGenerator() ? s_ddsKi : s_dacKi; }
float getKd()            { return isFrequencyGenerator() ? s_ddsKd : s_dacKd; }
float getCenterVoltage() { return s_centerVoltage; }
float getTargetPhase()   { return s_targetPhaseNs; }
float getMaxSlew()       { return isFrequencyGenerator() ? s_ddsMaxSlew : s_maxSlew; }

// --- Loop / acquisition config ---
uint32_t getLoopPeriodMs()          { return s_loopPeriodMs; }
void     setLoopPeriodMs(uint32_t v){ if (v >= 1 && v <= 1000) s_loopPeriodMs = v; }

float    getLockThresholdNs()       { return s_lockThresholdNs; }
void     setLockThresholdNs(float v){ if (v > 0.0f) s_lockThresholdNs = v; }

bool     getManualMode()            { return s_manualMode; }
void     setManualMode(bool v)      { s_manualMode = v; }

float    getLockedCenterV()         { return s_lockedCenterV; }
bool     haveLockedCenter()         { return s_haveLockedCenter; }
void     setLockedCenter(float v)   {
  s_lockedCenterV    = v;
  s_haveLockedCenter = true;
  s_signalWasAbsent  = false;
}
void     clearLockedCenter()        { s_haveLockedCenter = false; s_lockedCenterV = s_centerVoltage; }

uint32_t getLockHoldCycles()        { return s_lockHoldCycles; }
void     setLockHoldCycles(uint32_t v){ if (v >= 1) s_lockHoldCycles = v; }

uint32_t getStreamPeriodMs()        { return s_streamPeriodMs; }
void     setStreamPeriodMs(uint32_t v){ if (v >= 1) s_streamPeriodMs = v; }

SignalLossBehavior getSignalLossBehavior()                    { return s_signalLossBehavior; }
void               setSignalLossBehavior(SignalLossBehavior b) { s_signalLossBehavior = b; }

uint32_t getLockMemoryTimeoutMs()          { return s_lockMemoryTimeoutMs; }
void     setLockMemoryTimeoutMs(uint32_t ms) { s_lockMemoryTimeoutMs = ms; }

void tickSignalAbsent(uint32_t nowMs)
{
  if (nowMs == 0) {
    s_signalWasAbsent = false;
    return;
  }
  if (!s_signalWasAbsent) {
    s_signalAbsentSince = nowMs;
    s_signalWasAbsent   = true;
  }
  if (s_lockMemoryTimeoutMs > 0) {
    if ((nowMs - s_signalAbsentSince) >= s_lockMemoryTimeoutMs) {
      s_haveLockedCenter = false;
      s_lockedCenterV    = s_centerVoltage;
      if (s_signalLossBehavior == SIGNAL_LOSS_FREEZE) {
        reset();
      }
    }
  }

  // Handle signal loss behavior
  if (s_signalLossBehavior == SIGNAL_LOSS_CENTER) {
    reset();
  } else if (s_signalLossBehavior == SIGNAL_LOSS_ZERO) {
    shutdown();
  }
  // SIGNAL_LOSS_FREEZE holds last output
}

} // namespace dpll
