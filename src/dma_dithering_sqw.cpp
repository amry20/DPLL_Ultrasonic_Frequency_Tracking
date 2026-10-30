/*
 * dma_dithering_sqw.cpp
 *
 * STM32F407 Fine-Resolution Square Wave Generator via Timer 1 & DMA Period Dithering.
 * Generates square wave on PA8 (TIM1_CH1) with sub-Hertz resolution (~0.2 - 0.3 Hz at 30-40 kHz)
 * with zero CPU load once running.
 */

#include "dma_dithering_sqw.h"
#include "stm32f4xx.h"
#include <Arduino.h>

namespace dma_sqw
{
    // One ARR value per period in circular buffer (16-bit to match TIM1->ARR).
    static volatile uint16_t s_arr_buf[N_BUF];

    static float    s_activeFreq  = 28000.0f;
    static uint32_t s_activeBase  = 0;
    static bool     s_initialized = false;
    static bool     s_enabled     = false;

    float getResolution(float freqHz)
    {
        if (freqHz <= 0.0f) return 0.0f;
        // Step size ~ f^2 / (F_TIM * N_BUF)
        return (freqHz * freqHz) / ((float)F_TIM_HZ * (float)N_BUF);
    }

    void setFrequency(float freqHz)
    {
        // Clamp frequency to reasonable operational boundaries for ultrasonic transducers
        if (freqHz < 1000.0f)   freqHz = 1000.0f;
        if (freqHz > 100000.0f) freqHz = 100000.0f;

        s_activeFreq = freqHz;

        // Total timer ticks across N_BUF periods, rounded to nearest integer
        double totalTicks = ((double)F_TIM_HZ * (double)N_BUF) / (double)freqHz;
        uint64_t total = (uint64_t)(totalTicks + 0.5);
        uint32_t base  = (uint32_t)(total / N_BUF);
        uint32_t rem   = (uint32_t)(total % N_BUF);
        uint32_t acc   = 0;

        s_activeBase = base;

        // Bresenham-style spreading: the +1-tick periods are evenly interleaved,
        // which keeps short-term frequency deviation as small as possible (~6 ns max).
        // Since ARR = period - 1:
        //   - base ticks     -> ARR = base - 1
        //   - base + 1 ticks -> ARR = base
        for (uint32_t i = 0; i < N_BUF; ++i) {
            acc += rem;
            if (acc >= N_BUF) {
                acc -= N_BUF;
                s_arr_buf[i] = (uint16_t)base;       // base + 1 ticks
            } else {
                s_arr_buf[i] = (uint16_t)(base - 1); // base ticks
            }
        }

        // ~50% duty cycle (CCR1 preload takes effect at next update event)
        if (s_initialized) {
            TIM1->CCR1 = (uint16_t)(base / 2);
        }
    }

    float getFrequency()
    {
        return s_activeFreq;
    }

    void enableOutput(bool enable)
    {
        s_enabled = enable;
        if (!s_initialized) return;

        if (enable) {
            TIM1->CCER |= TIM_CCER_CC1E;
            TIM1->BDTR |= TIM_BDTR_MOE;
        } else {
            TIM1->CCER &= ~TIM_CCER_CC1E;
            TIM1->BDTR &= ~TIM_BDTR_MOE;
        }
    }

    bool isOutputEnabled()
    {
        return s_enabled && s_initialized && ((TIM1->BDTR & TIM_BDTR_MOE) != 0);
    }

    bool isInitialized()
    {
        return s_initialized;
    }

    void stop()
    {
        if (!s_initialized) return;

        enableOutput(false);

        // Disable TIM1 and update DMA request
        TIM1->CR1 &= ~TIM_CR1_CEN;
        TIM1->DIER &= ~TIM_DIER_UDE;

        // Disable DMA2 Stream 5
        DMA2_Stream5->CR &= ~DMA_SxCR_EN;
        while (DMA2_Stream5->CR & DMA_SxCR_EN) { }
        DMA2->HIFCR = 0xF40U; // Clear Stream 5 interrupt flags

        s_initialized = false;
        s_enabled = false;
    }

    void begin(float initialFreqHz)
    {
        if (s_initialized) {
            stop();
        }

        // 1. Enable Clocks: GPIOA, DMA2 (AHB1) and TIM1 (APB2)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

        // 2. Configure PA8 as TIM1_CH1 (Alternate Function AF1, Very High Speed)
        GPIOA->MODER   = (GPIOA->MODER & ~(3U << (8 * 2))) | (2U << (8 * 2)); // AF mode
        GPIOA->OSPEEDR |= (3U << (8 * 2));                                    // Very High Speed
        GPIOA->AFR[1]  = (GPIOA->AFR[1] & ~(0xFU << ((8 - 8) * 4))) | (1U << ((8 - 8) * 4)); // AF1

        // 3. Compute initial dither buffer
        setFrequency(initialFreqHz);

        // 4. TIM1 Setup: PWM Mode 1 on CH1, PSC = 0 (168 MHz tick)
        TIM1->CR1   = 0;
        TIM1->PSC   = 0;
        TIM1->ARR   = s_arr_buf[0];
        TIM1->CCR1  = (uint16_t)(s_activeBase / 2);
        TIM1->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos)   // PWM Mode 1
                    | TIM_CCMR1_OC1PE;            // CCR1 Preload Enable
        TIM1->CCER  = TIM_CCER_CC1E;              // Enable CH1 output
        TIM1->BDTR  = TIM_BDTR_MOE;               // Main Output Enable (advanced timer)
        TIM1->CR1   = TIM_CR1_ARPE;               // ARR Preload Enable
        TIM1->EGR   = TIM_EGR_UG;                 // Force update generation to transfer preloads

        // 5. DMA2 Stream 5, Channel 6 = TIM1_UP (RM0090 DMA2 Request Mapping)
        DMA2_Stream5->CR = 0;
        while (DMA2_Stream5->CR & DMA_SxCR_EN) { } // Wait until stream is disabled
        DMA2->HIFCR = 0xF40U;                      // Clear all Stream 5 flags (bits 6, 8..11)

        DMA2_Stream5->PAR  = (uint32_t)&TIM1->ARR; // Destination: Timer ARR register
        DMA2_Stream5->M0AR = (uint32_t)s_arr_buf;  // Source: RAM dither buffer
        DMA2_Stream5->NDTR = N_BUF;
        DMA2_Stream5->CR   = (6U << DMA_SxCR_CHSEL_Pos) // Channel 6
                           | DMA_SxCR_PL_1              // High priority
                           | DMA_SxCR_MSIZE_0           // Memory: 16-bit
                           | DMA_SxCR_PSIZE_0           // Peripheral: 16-bit
                           | DMA_SxCR_MINC              // Memory increment
                           | DMA_SxCR_CIRC              // Circular mode
                           | DMA_SxCR_DIR_0;            // Memory-to-peripheral
        DMA2_Stream5->CR  |= DMA_SxCR_EN;

        // 6. Start: Update event triggers DMA transfer into ARR, enable counter
        TIM1->DIER |= TIM_DIER_UDE;
        TIM1->CR1  |= TIM_CR1_CEN;

        s_initialized = true;
        s_enabled     = true;
    }

} // namespace dma_sqw

