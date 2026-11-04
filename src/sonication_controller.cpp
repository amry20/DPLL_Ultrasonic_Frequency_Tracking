/*
 * sonication_controller.cpp
 *
 * Hardware Timer (TIM4) driven Pulse Sonication Controller for STM32F407.
 * Dedicated control of PA5 (Push-Pull Sonication Gate Output).
 * Frequency generation (PA8) runs independently and is never altered or muted
 * by sonication sequence transitions.
 */

#include "sonication_controller.h"
#include <HardwareTimer.h>

namespace sonication {

namespace {

HardwareTimer* s_timer = nullptr;

volatile SonicationState s_state        = STATE_IDLE;
volatile uint32_t        s_onTimeMs     = 0;
volatile uint32_t        s_restTimeMs   = 0;
volatile uint32_t        s_targetCycles = 0;
volatile uint32_t        s_currentCycle = 0;
volatile uint32_t        s_remainingMs  = 0;
volatile uint32_t        s_elapsedMs    = 0;
volatile bool            s_stateChanged = false;

// Hardware Timer ISR callback (executed every 1 ms when timer is running)
void onTimerTick()
{
    if (s_state == STATE_IDLE) {
        return;
    }

    s_elapsedMs++;

    if (s_remainingMs > 0) {
        s_remainingMs--;
    }

    if (s_remainingMs == 0) {
        if (s_state == STATE_ON) {
            // ON phase completed -> turn off PA5 gate output
            GPIOA->BSRR = (1U << (5 + 16)); // PA5 LOW

            if (s_restTimeMs > 0) {
                s_state = STATE_REST;
                s_remainingMs = s_restTimeMs;
                s_stateChanged = true;
            } else {
                // Zero rest time -> advance cycle immediately
                s_currentCycle++;
                if (s_targetCycles > 0 && s_currentCycle > s_targetCycles) {
                    s_state = STATE_IDLE;
                    s_stateChanged = true;
                    if (s_timer) s_timer->pause();
                } else {
                    s_remainingMs = s_onTimeMs;
                    s_stateChanged = true;
                    GPIOA->BSRR = (1U << 5); // PA5 HIGH
                }
            }
        } else if (s_state == STATE_REST) {
            // REST phase completed -> advance cycle and start next ON pulse
            s_currentCycle++;
            if (s_targetCycles > 0 && s_currentCycle > s_targetCycles) {
                s_state = STATE_IDLE;
                s_stateChanged = true;
                if (s_timer) s_timer->pause();
            } else {
                s_state = STATE_ON;
                s_remainingMs = s_onTimeMs;
                s_stateChanged = true;
                GPIOA->BSRR = (1U << 5); // PA5 HIGH
            }
        }
    }
}

} // anonymous namespace

void begin()
{
    // Configure PA5 as digital output, default LOW
    pinMode(kSonicationPin, OUTPUT);
    digitalWriteFast(PA_5, LOW);

    // Initialize TIM4 as a 1 kHz (1 ms period) periodic tick timer
    if (!s_timer) {
        s_timer = new HardwareTimer(TIM4);
        s_timer->setOverflow(1000, HERTZ_FORMAT); // 1 kHz tick -> 1.0 ms
        s_timer->attachInterrupt(onTimerTick);
        s_timer->pause();
    }
}

bool start(uint32_t onTimeMs, uint32_t restTimeMs, uint32_t cycles)
{
    if (onTimeMs == 0) {
        return false;
    }

    noInterrupts();
    if (s_timer) {
        s_timer->pause();
    }
    s_onTimeMs     = onTimeMs;
    s_restTimeMs   = restTimeMs;
    s_targetCycles = cycles;
    s_currentCycle = 1;
    s_remainingMs  = onTimeMs;
    s_elapsedMs    = 0;
    s_state        = STATE_ON;
    s_stateChanged = true;

    // Immediately assert PA5 HIGH
    GPIOA->BSRR = (1U << 5); // PA5 HIGH

    if (s_timer) {
        s_timer->setCount(0);
        s_timer->resume();
    }
    interrupts();

    return true;
}

void stop()
{
    noInterrupts();
    if (s_timer) {
        s_timer->pause();
    }
    GPIOA->BSRR = (1U << (5 + 16)); // PA5 LOW

    bool changed = (s_state != STATE_IDLE);
    s_state        = STATE_IDLE;
    s_remainingMs  = 0;
    if (changed) {
        s_stateChanged = true;
    }
    interrupts();
}

Status getStatus()
{
    Status st;
    noInterrupts();
    st.state        = (uint8_t)s_state;
    st.currentCycle = (s_state == STATE_IDLE) ? 0 : s_currentCycle;
    st.targetCycles = s_targetCycles;
    st.remainingMs  = s_remainingMs;
    st.elapsedMs    = s_elapsedMs;
    interrupts();
    return st;
}

bool hasStateChanged()
{
    return s_stateChanged;
}

void clearStateChanged()
{
    s_stateChanged = false;
}

} // namespace sonication
