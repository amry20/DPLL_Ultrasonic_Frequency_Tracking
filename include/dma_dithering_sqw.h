/*
 * dma_dithering_sqw.h
 *
 * STM32F407 Fine-Resolution Square Wave Generator via Timer 1 & DMA Period Dithering.
 * Generates square wave on PA8 (TIM1_CH1) with sub-Hertz resolution (~0.2 - 0.3 Hz at 30-40 kHz)
 * with zero CPU load once running.
 *
 * Uses:
 *   - TIM1 running from 168 MHz APB2 timer clock (PSC = 0, PWM Mode 1 on CH1)
 *   - PA8 configured as Alternate Function AF1 (TIM1_CH1)
 *   - DMA2 Stream 5 Channel 6 (TIM1_UP request) in Circular Mode
 *   - 32-entry circular buffer with Bresenham-distributed ARR period dithering
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

namespace dma_sqw
{
    static constexpr uint32_t F_TIM_HZ = 168000000UL; // APB2 Timer Clock for TIM1 (168 MHz)
    static constexpr uint32_t N_BUF    = 32;          // Dither buffer length: avg step ~ f^2/(F_TIM*N_BUF)

    /**
     * @brief Initialize TIM1, DMA2 Stream 5 Channel 6, and PA8 for DMA dither square wave output.
     * @param initialFreqHz Initial square wave frequency in Hz (default 28000.0f)
     */
    void begin(float initialFreqHz = 28000.0f);

    /**
     * @brief Update the square wave frequency at runtime with sub-Hertz resolution.
     * Recalculates the Bresenham ARR buffer and adjusts CCR1 duty cycle.
     * @param freqHz Target frequency in Hz (e.g. 20000.0f - 50000.0f)
     */
    void setFrequency(float freqHz);

    /**
     * @brief Get currently configured frequency in Hz.
     */
    float getFrequency();

    /**
     * @brief Enable or disable square wave output on PA8.
     * When disabled, output is muted (TIM1 MOE / CC1E disabled, pin driven LOW).
     */
    void enableOutput(bool enable);

    /**
     * @brief Check whether output is currently enabled.
     */
    bool isOutputEnabled();

    /**
     * @brief Stop TIM1 and DMA2, release peripheral resources and mute output.
     */
    void stop();

    /**
     * @brief Calculate frequency step resolution in Hz at given frequency: f^2 / (F_TIM * N_BUF).
     */
    float getResolution(float freqHz);

    /**
     * @brief Check if generator has been initialized.
     */
    bool isInitialized();

} // namespace dma_sqw

