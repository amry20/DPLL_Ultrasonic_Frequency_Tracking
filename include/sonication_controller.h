#pragma once
#include <Arduino.h>
#include <stdint.h>

namespace sonication {

enum SonicationState : uint8_t {
    STATE_IDLE = 0,
    STATE_ON   = 1,
    STATE_REST = 2
};

struct Status {
    uint8_t  state;         // 0=IDLE, 1=ON, 2=REST
    uint32_t currentCycle;  // 1-based current cycle, or 0 if IDLE
    uint32_t targetCycles;  // Total target cycles (0 = continuous)
    uint32_t remainingMs;   // Remaining ms in current phase
    uint32_t elapsedMs;     // Total elapsed ms since sequence started
} __attribute__((packed));

// Sonication control pin (PA5, Push-Pull digital output)
constexpr uint32_t kSonicationPin = PA5;

// Initialize PA5 GPIO output (default LOW) and HardwareTimer (TIM4)
void begin();

// Start a pulse sonication sequence
// onTimeMs: duration of sonication ON in ms (PA5 HIGH)
// restTimeMs: duration of pause/rest in ms (PA5 LOW)
// cycles: number of repetitions (0 = continuous until stopped)
bool start(uint32_t onTimeMs, uint32_t restTimeMs, uint32_t cycles);

// Abort/stop sonication immediately (PA5 LOW, timer paused)
void stop();

// Query live status (atomic/thread-safe)
Status getStatus();

// Check if state changed since last call (useful for event-driven telemetry push)
bool hasStateChanged();

// Clear state changed flag
void clearStateChanged();

} // namespace sonication

