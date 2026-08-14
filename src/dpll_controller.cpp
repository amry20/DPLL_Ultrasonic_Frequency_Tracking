/*
 * dpll_controller.cpp
 *
 * Implementation of the DPLL PI controller (see header for theory of operation).
 */

#include "dpll_controller.h"
#include "dac_output.h"

namespace dpll {

namespace {

float s_kp = 0.0f;
float s_ki = 0.0f;

float s_centerVoltage = 1.65f;
float s_targetPhaseNs = 0.0f;

float s_minVoltage = 0.0f;
float s_maxVoltage = 3.3f;

float s_integral = 0.0f;
float s_outputVoltage = 1.65f;

bool s_enabled = true;

} // namespace

void begin(float centerVoltage, float kp, float ki)
{
  s_centerVoltage = centerVoltage;
  s_kp = kp;
  s_ki = ki;
  s_integral = 0.0f;
  s_outputVoltage = centerVoltage;
  s_enabled = true;
  dac::setVoltage(s_outputVoltage);
}

void setTargetPhase(float targetNs)
{
  s_targetPhaseNs = targetNs;
}

void setGains(float kp, float ki)
{
  s_kp = kp;
  s_ki = ki;
}

void setOutputLimits(float minV, float maxV)
{
  s_minVoltage = minV;
  s_maxVoltage = maxV;
}

void enable(bool on)
{
  s_enabled = on;
}

bool isEnabled()
{
  return s_enabled;
}

void reset()
{
  s_integral = 0.0f;
  s_outputVoltage = s_centerVoltage;
  dac::setVoltage(s_outputVoltage);
}

float update(float phaseErrorNs, float dtSeconds)
{
  if (!s_enabled) {
    return s_outputVoltage; // Hold last value
  }

  // Sanitize dt (avoid huge steps on first call / overflow)
  if (dtSeconds <= 0.0f) {
    dtSeconds = 1e-6f;
  }
  if (dtSeconds > 1.0f) {
    dtSeconds = 1.0f; // Cap at 1 s to avoid integral windup spikes
  }

  // Error in nanoseconds. The capture layer already wraps the raw (ZCD - REF)
  // time difference to [-period/2 .. +period/2], so no extra wrap is needed.
  float error = s_targetPhaseNs - phaseErrorNs;

  // Proportional term
  float proportional = s_kp * error;

  // Integral term (with anti-windup via output clamping below)
  s_integral += s_ki * error * dtSeconds;

  // Unclamped control voltage
  float voltage = s_centerVoltage + proportional + s_integral;

  // Clamp output
  if (voltage > s_maxVoltage) {
    voltage = s_maxVoltage;
    // Anti-windup: back off the integrator so it does not keep growing
    s_integral = s_maxVoltage - s_centerVoltage - proportional;
  } else if (voltage < s_minVoltage) {
    voltage = s_minVoltage;
    s_integral = s_minVoltage - s_centerVoltage - proportional;
  }

  s_outputVoltage = voltage;
  dac::setVoltage(s_outputVoltage);

  return s_outputVoltage;
}

float lastVoltage()
{
  return s_outputVoltage;
}

void shutdown()
{
  s_enabled = false;
  s_integral = 0.0f;
  s_outputVoltage = s_minVoltage;
  dac::setVoltage(s_outputVoltage);
}

void manualSet(float voltage)
{
  s_enabled = false;
  s_integral = 0.0f;
  if (voltage < s_minVoltage) { voltage = s_minVoltage; }
  if (voltage > s_maxVoltage) { voltage = s_maxVoltage; }
  s_outputVoltage = voltage;
  dac::setVoltage(s_outputVoltage);
}

float getKp()            { return s_kp; }
float getKi()            { return s_ki; }
float getCenterVoltage() { return s_centerVoltage; }
float getTargetPhase()   { return s_targetPhaseNs; }

} // namespace dpll
