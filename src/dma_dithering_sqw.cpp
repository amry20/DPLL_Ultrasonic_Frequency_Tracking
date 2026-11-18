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
    static uint16_t s_arr_buf[2][N_BUF];
    static uint8_t s_arrBank = 0;
    static uint32_t s_prescalerDivider = 1;
    static uint32_t s_totalTicks = 0;

    static float    s_activeFreq  = 28000.0f;
    static uint32_t s_activeBase  = 0;
    static bool     s_initialized = false;
    static bool     s_enabled     = false;

    float getResolution(float freqHz)
    {
        DitherPlan plan{};
        if (!makeDitherPlan(freqHz, plan)) return 0.0f;
        return freqHz * freqHz * static_cast<float>(plan.divider) /
               (static_cast<float>(F_TIM_HZ) * N_BUF);
    }

    void setFrequency(float freqHz)
    {
        DitherPlan plan{};
        if (!makeDitherPlan(freqHz, plan)) return;
        if (s_initialized && plan.totalTicks == s_totalTicks &&
            plan.divider == s_prescalerDivider) return;

        const uint8_t nextBank = s_arrBank ^ 1U;
        for (uint32_t i = 0; i < N_BUF; ++i) s_arr_buf[nextBank][i] = plan.arr[i];
        __DMB();
        if (s_initialized) {
            const uint32_t irqState = __get_PRIMASK();
            __disable_irq();
            // Hold shadow-register updates while replacing the DMA descriptor.
            // The counter and current output period keep running throughout.
            TIM1->CR1 |= TIM_CR1_UDIS;
            TIM1->DIER &= ~TIM_DIER_UDE;
            DMA2_Stream5->CR &= ~DMA_SxCR_EN;
            uint32_t attempts = 1000;
            while ((DMA2_Stream5->CR & DMA_SxCR_EN) && --attempts) { }
            if (attempts == 0) {
                TIM1->DIER |= TIM_DIER_UDE;
                TIM1->CR1 &= ~TIM_CR1_UDIS;
                __set_PRIMASK(irqState);
                return;
            }
            DMA2->HIFCR = 0xF40U;
            DMA2_Stream5->M0AR = reinterpret_cast<uint32_t>(s_arr_buf[nextBank]);
            DMA2_Stream5->NDTR = N_BUF;
            TIM1->ARR = plan.arr[0];
            TIM1->CCR1 = plan.base / 2U;
            TIM1->PSC = plan.divider - 1U;
            __DMB();
            DMA2_Stream5->CR |= DMA_SxCR_EN;
            TIM1->DIER |= TIM_DIER_UDE;
            TIM1->CR1 &= ~TIM_CR1_UDIS;
            __set_PRIMASK(irqState);
        }
        s_arrBank = nextBank;
        s_activeBase = plan.base;
        s_prescalerDivider = plan.divider;
        s_totalTicks = plan.totalTicks;
        s_activeFreq = plan.frequency;
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

        // 4. TIM1 Setup: PWM Mode 1 on CH1, adaptive prescaler (PSC=0 at ultrasonic frequencies)
        TIM1->CR1   = 0;
        TIM1->PSC   = s_prescalerDivider - 1U;
        TIM1->ARR   = s_arr_buf[s_arrBank][0];
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
        DMA2_Stream5->M0AR = (uint32_t)s_arr_buf[s_arrBank];  // Source: RAM dither buffer
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

