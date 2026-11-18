#pragma once
#include <stdint.h>
#include <cmath>

namespace dma_sqw {
constexpr uint32_t F_TIM_HZ = 168000000UL;
constexpr uint32_t N_BUF = 32;
constexpr float MIN_FREQUENCY_HZ = 1000.0f;
constexpr float MAX_FREQUENCY_HZ = 100000.0f;

inline bool isSupportedFrequency(float frequency) {
    return std::isfinite(frequency) && frequency >= MIN_FREQUENCY_HZ &&
           frequency <= MAX_FREQUENCY_HZ;
}

struct DitherPlan {
    uint16_t arr[N_BUF];
    uint32_t divider;
    uint32_t base;
    uint32_t totalTicks;
    float frequency;
};

inline bool makeDitherPlan(float frequency, DitherPlan& plan) {
    if (!isSupportedFrequency(frequency)) return false;
    plan.divider = static_cast<uint32_t>(std::ceil(
        static_cast<double>(F_TIM_HZ) / (static_cast<double>(frequency) * 65536.0)));
    const double ticks = static_cast<double>(F_TIM_HZ) * N_BUF /
                         (static_cast<double>(plan.divider) * frequency);
    plan.totalTicks = static_cast<uint32_t>(ticks + 0.5);
    plan.base = plan.totalTicks / N_BUF;
    const uint32_t remainder = plan.totalTicks % N_BUF;
    uint32_t accumulator = 0;
    for (uint32_t i = 0; i < N_BUF; ++i) {
        accumulator += remainder;
        const bool longer = accumulator >= N_BUF;
        if (longer) accumulator -= N_BUF;
        plan.arr[i] = static_cast<uint16_t>(plan.base - 1U + (longer ? 1U : 0U));
    }
    plan.frequency = static_cast<float>(static_cast<double>(F_TIM_HZ) * N_BUF /
        (static_cast<double>(plan.divider) * plan.totalTicks));
    return true;
}
} // namespace dma_sqw
