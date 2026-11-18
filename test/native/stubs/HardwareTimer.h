#pragma once
#include "Arduino.h"
enum TimerModes_t { TIMER_INPUT_CAPTURE_FALLING, TIMER_INPUT_CAPTURE_BOTHEDGE, TIMER_INPUT_CAPTURE_RISING };
constexpr int TICK_FORMAT = 0;
class HardwareTimer {
public:
    uint32_t captures[3]{};
    uint32_t count = 0;
    bool running = false;
    explicit HardwareTimer(TIM_TypeDef*) {}
    void setInterruptPriority(uint32_t, uint32_t) {}
    void setPrescaleFactor(uint32_t) {}
    void setOverflow(uint32_t, int) {}
    void setMode(uint32_t, TimerModes_t, uint32_t) {}
    void attachInterrupt(uint32_t, void (*)()) {}
    void resume() { running = true; }
    void pause() { running = false; }
    uint32_t getCaptureCompare(uint32_t channel) { return captures[channel]; }
    uint32_t getCount() { return count; }
};
