/*
 * goertzel_capture.cpp
 *
 * Dual-Channel Simultaneous ADC + Goertzel DFT Front-End for STM32F407.
 */

#include "goertzel_capture.h"
#include <Arduino.h>
#include <stm32f4xx_hal.h>
#include <cmath>

namespace goertzel_capture {

namespace {

constexpr float PI_F = 3.14159265358979323846f;
constexpr float TWO_PI_F = 6.28318530717958647692f;

// DMA Circular Buffer: 2 blocks of 1024 sample pairs.
// In STM32 Dual Regular Simultaneous mode (DMA mode 2):
// each 32-bit word holds: (ADC2 << 16) | ADC1
// Bottom 16 bits: ADC1 (PC4, Current sense)
// Top 16 bits:    ADC2 (PA7, Drive reference)
alignas(32) static uint32_t s_adcDmaBuf[2 * N_SAMPLES];

// Precomputed Hann window and window sum placed in CCMRAM (accessible by Cortex-M4 core)
__attribute__((section(".ccmram"))) static float s_hannWindow[N_SAMPLES];
static float s_hannSum = 0.0f;
static bool  s_windowInit = false;

// Hardware handles
static ADC_HandleTypeDef s_hadc1;
static ADC_HandleTypeDef s_hadc2;
static DMA_HandleTypeDef s_hdma_adc;
static TIM_HandleTypeDef s_htim3;

static volatile const uint32_t* s_readyBlock = nullptr;
static volatile uint32_t s_dmaBlocksCaptured = 0;
static bool s_running = false;

// Calibration parameters
// R_burden = 50.0 ohm, C_burden = 6.8 nF, L_sec = 24 mH, 1:100 CT ratio, input divider = 0.4545 (10k / (10k + 12k))
static float s_rBurden = 50.0f;
static float s_cBurden = 6.8e-9f;
static float s_lSec    = 24.0e-3f;
static float s_ctTurns = 100.0f;
static float s_frontEndAtten = 0.454545f; // 10k / 22k

static GoertzelMetrics s_latestMetrics{};

void initHannWindow()
{
    if (s_windowInit) return;
    s_hannSum = 0.0f;
    for (uint16_t n = 0; n < N_SAMPLES; n++) {
        // Hann window: 0.5 * (1 - cos(2*pi*n / N))
        float w = 0.5f * (1.0f - cosf(TWO_PI_F * (float)n / (float)N_SAMPLES));
        s_hannWindow[n] = w;
        s_hannSum += w;
    }
    s_windowInit = true;
}

} // namespace

// DMA Interrupt Callbacks
extern "C" void DMA2_Stream0_IRQHandler(void)
{
    // Half Transfer Complete (First half ready: s_adcDmaBuf[0 .. N_SAMPLES-1])
    if (__HAL_DMA_GET_FLAG(&s_hdma_adc, DMA_FLAG_HTIF0_4)) {
        __HAL_DMA_CLEAR_FLAG(&s_hdma_adc, DMA_FLAG_HTIF0_4);
        s_readyBlock = &s_adcDmaBuf[0];
        s_dmaBlocksCaptured++;
    }
    // Full Transfer Complete (Second half ready: s_adcDmaBuf[N_SAMPLES .. 2*N_SAMPLES-1])
    if (__HAL_DMA_GET_FLAG(&s_hdma_adc, DMA_FLAG_TCIF0_4)) {
        __HAL_DMA_CLEAR_FLAG(&s_hdma_adc, DMA_FLAG_TCIF0_4);
        s_readyBlock = &s_adcDmaBuf[N_SAMPLES];
        s_dmaBlocksCaptured++;
    }
}

void begin()
{
    if (s_running) return;

    initHannWindow();

    // 1. Enable Clocks: GPIOA, GPIOC, TIM3, ADC1, ADC2, DMA2
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    // 2. Configure GPIO Pins: PC4 (ADC1_IN14) and PA7 (ADC2_IN7) as Analog
    GPIO_InitTypeDef gpio{};
    gpio.Pin  = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin  = GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    // 3. Configure TIM3 for 750 kHz TRGO
    // APB1 timer clock on STM32F407 with 168 MHz SYSCLK is 84 MHz.
    // 84,000,000 / 112 = 750,000 Hz.
    // Prescaler = 0, Period = 112 - 1 = 111.
    s_htim3.Instance               = TIM3;
    s_htim3.Init.Prescaler         = 0;
    s_htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim3.Init.Period            = 111;
    s_htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim3.Init.RepetitionCounter = 0;
    s_htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&s_htim3);

    TIM_MasterConfigTypeDef sMasterConfig{};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&s_htim3, &sMasterConfig);

    // 4. Configure DMA2 Stream 0 Channel 0 for ADC Common Regular Data Register
    s_hdma_adc.Instance                 = DMA2_Stream0;
    s_hdma_adc.Init.Channel             = DMA_CHANNEL_0;
    s_hdma_adc.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_hdma_adc.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdma_adc.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    s_hdma_adc.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    s_hdma_adc.Init.Mode                = DMA_CIRCULAR;
    s_hdma_adc.Init.Priority            = DMA_PRIORITY_HIGH;
    s_hdma_adc.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&s_hdma_adc);

    // Enable DMA Interrupt in NVIC
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    // 5. Configure ADC Common: Dual Regular Simultaneous Mode, DMA Mode 2
    // Prescaler = Div4 (84 MHz APB2 / 4 = 21 MHz ADC clock)
    // On STM32F4 HAL, common configuration register is ADC->CCR
    uint32_t ccr = 0;
    ccr |= ADC_DUALMODE_REGSIMULT;              // Dual regular simultaneous
    ccr |= ADC_CLOCK_SYNC_PCLK_DIV4;            // PCLK2 / 4 = 21 MHz
    ccr |= ADC_DMAACCESSMODE_2;                 // DMA mode 2: 32-bit word = (ADC2 << 16) | ADC1
    ccr |= ADC_TWOSAMPLINGDELAY_5CYCLES;        // 5 cycles delay between sampling phases
    ADC->CCR = ccr;

    // 6. Configure ADC1 (Master - PC4, Channel 14)
    s_hadc1.Instance                   = ADC1;
    s_hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    s_hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    s_hadc1.Init.ScanConvMode          = DISABLE;
    s_hadc1.Init.ContinuousConvMode    = DISABLE;
    s_hadc1.Init.DiscontinuousConvMode = DISABLE;
    s_hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    s_hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T3_TRGO; // TIM3 TRGO
    s_hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    s_hadc1.Init.NbrOfConversion       = 1;
    s_hadc1.Init.DMAContinuousRequests = ENABLE;
    s_hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&s_hadc1);

    ADC_ChannelConfTypeDef sConfig{};
    sConfig.Channel      = ADC_CHANNEL_14; // PC4
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // 15 + 12 = 27 cycles @ 21 MHz = 1.286 us (< 1.333 us)
    HAL_ADC_ConfigChannel(&s_hadc1, &sConfig);

    // 7. Configure ADC2 (Slave - PA7, Channel 7)
    s_hadc2.Instance                   = ADC2;
    s_hadc2.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    s_hadc2.Init.Resolution            = ADC_RESOLUTION_12B;
    s_hadc2.Init.ScanConvMode          = DISABLE;
    s_hadc2.Init.ContinuousConvMode    = DISABLE;
    s_hadc2.Init.DiscontinuousConvMode = DISABLE;
    s_hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; // Slave is triggered simultaneously by Master
    s_hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    s_hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    s_hadc2.Init.NbrOfConversion       = 1;
    s_hadc2.Init.DMAContinuousRequests = ENABLE;
    s_hadc2.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&s_hadc2);

    sConfig.Channel      = ADC_CHANNEL_7; // PA7
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(&s_hadc2, &sConfig);

    // Link DMA with ADC1
    __HAL_LINKDMA(&s_hadc1, DMA_Handle, s_hdma_adc);

    // 8. Start DMA and ADCs
    HAL_DMA_Start_IT(&s_hdma_adc, (uint32_t)&ADC->CDR, (uint32_t)s_adcDmaBuf, 2 * N_SAMPLES);
    s_hadc1.Instance->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;
    
    // Enable ADC2 (slave) first, then ADC1 (master)
    __HAL_ADC_ENABLE(&s_hadc2);
    __HAL_ADC_ENABLE(&s_hadc1);

    // Start TIM3 trigger
    HAL_TIM_Base_Start(&s_htim3);

    s_running = true;
}

bool poll(float f_drive)
{
    if (!s_running || s_readyBlock == nullptr) {
        return false;
    }

    // Atomically grab pointer to finished half-block and reset flag
    const uint32_t* blk = (const uint32_t*)s_readyBlock;
    s_readyBlock = nullptr;

    if (f_drive < 1000.0f || f_drive > 100000.0f) {
        f_drive = 33000.0f; // Fallback sane frequency
    }

    // Goertzel parameters for this block
    const float omega = TWO_PI_F * f_drive / FS_HZ;
    const float coeff = 2.0f * cosf(omega);
    const float cosW  = cosf(omega);
    const float sinW  = sinf(omega);

    float cur_s1 = 0.0f, cur_s2 = 0.0f;
    float ref_s1 = 0.0f, ref_s2 = 0.0f;

    bool clipped = false;

    // Execute Goertzel recurrence for both channels simultaneously in a single cache-friendly pass
    for (uint16_t n = 0; n < N_SAMPLES; n++) {
        const uint32_t pair = blk[n];
        const uint16_t raw_cur = (uint16_t)(pair & 0x0FFFU);
        const uint16_t raw_ref = (uint16_t)((pair >> 16) & 0x0FFFU);

        if (raw_cur <= 5 || raw_cur >= 4090 || raw_ref <= 5 || raw_ref >= 4090) {
            clipped = true;
        }

        const float win = s_hannWindow[n];

        // Zero-center around midscale (2048 counts) and apply Hann window
        const float x_cur = ((float)raw_cur - ADC_MID_F) * win;
        const float x_ref = ((float)raw_ref - ADC_MID_F) * win;

        // Current Goertzel step
        const float cur_s0 = x_cur + coeff * cur_s1 - cur_s2;
        cur_s2 = cur_s1;
        cur_s1 = cur_s0;

        // Ref Goertzel step
        const float ref_s0 = x_ref + coeff * ref_s1 - ref_s2;
        ref_s2 = ref_s1;
        ref_s1 = ref_s0;
    }

    // Final complex vectors (Real & Imaginary)
    const float cur_re = cur_s1 - cosW * cur_s2;
    const float cur_im = sinW * cur_s2;
    const float ref_re = ref_s1 - cosW * ref_s2;
    const float ref_im = sinW * ref_s2;

    const float cur_mag = sqrtf(cur_re * cur_re + cur_im * cur_im);
    const float ref_mag = sqrtf(ref_re * ref_re + ref_im * ref_im);

    // Peak AC voltage amplitude at ADC pins (V_pk = 2 * Mag / (WindowSum * 4095/3.3))
    const float invWsum = 2.0f / (s_hannSum > 0.0f ? s_hannSum : 1.0f);
    const float cur_pk_v = (cur_mag * invWsum) * (3.3f / 4095.0f);
    const float ref_pk_v = (ref_mag * invWsum) * (3.3f / 4095.0f);

    // Phase angles relative to block start
    const float cur_phi = atan2f(cur_im, cur_re);
    const float ref_phi = atan2f(ref_im, ref_re);

    // Phase difference (Current vs Drive Reference)
    float dphi = cur_phi - ref_phi;
    while (dphi >  PI_F) dphi -= TWO_PI_F;
    while (dphi < -PI_F) dphi += TWO_PI_F;

    // Software compensation for CT Burden (R_burden || C_burden || L_sec)
    // Burden phase shift: phi_burden = -atan(R_burden * (2*pi*f*C - 1/(2*pi*f*L)))
    const float omegaDrive = TWO_PI_F * f_drive;
    const float bNet = (omegaDrive * s_cBurden) - (1.0f / (omegaDrive * s_lSec > 1e-6f ? omegaDrive * s_lSec : 1e-6f));
    const float phiBurdenRad = -atanf(s_rBurden * bNet);

    // Corrected true phase at transducer terminal
    float trueDphi = dphi - phiBurdenRad;
    while (trueDphi >  PI_F) trueDphi -= TWO_PI_F;
    while (trueDphi < -PI_F) trueDphi += TWO_PI_F;

    const float phaseDeg = trueDphi * (180.0f / PI_F);
    const float phaseNs  = (trueDphi / omegaDrive) * 1e9f;

    // Transducer Peak Current:
    // V_burden = V_adc_pin / frontEndAtten
    // I_sec = V_burden * sqrt((1/R)^2 + B_net^2)
    // I_primary = I_sec * ctTurns
    const float vBurdenPk = (s_frontEndAtten > 1e-4f) ? (cur_pk_v / s_frontEndAtten) : cur_pk_v;
    const float yBurdenMag = sqrtf((1.0f / (s_rBurden * s_rBurden)) + (bNet * bNet));
    const float iSecPk = vBurdenPk * yBurdenMag;
    const float iPrimaryPkA = iSecPk * s_ctTurns;

    // Validate metrics
    const bool refValid = (ref_pk_v >= 0.20f);
    const bool curValid = (cur_pk_v >= 0.015f) && !clipped;

    s_latestMetrics.currentAmpV     = cur_pk_v;
    s_latestMetrics.currentAmpA     = iPrimaryPkA;
    s_latestMetrics.currentPhaseRad = cur_phi;
    s_latestMetrics.refAmpV         = ref_pk_v;
    s_latestMetrics.refPhaseRad     = ref_phi;
    s_latestMetrics.phaseDiffRad    = trueDphi;
    s_latestMetrics.phaseDiffDeg    = phaseDeg;
    s_latestMetrics.phaseDiffNs     = phaseNs;
    s_latestMetrics.driveFreqHz     = f_drive;
    s_latestMetrics.blockCount      = s_dmaBlocksCaptured;
    s_latestMetrics.refValid        = refValid;
    s_latestMetrics.currentValid    = curValid;
    s_latestMetrics.clipped         = clipped;
    s_latestMetrics.valid           = refValid && curValid;

    return true;
}

GoertzelMetrics getMetrics()
{
    return s_latestMetrics;
}

void setBurdenModel(float rBurdenOhms, float cBurdenFarads, float lSecHenries, float ctTurnsRatio, float frontEndAtten)
{
    if (rBurdenOhms > 0.1f) s_rBurden = rBurdenOhms;
    if (cBurdenFarads >= 0.0f) s_cBurden = cBurdenFarads;
    if (lSecHenries > 1e-5f) s_lSec = lSecHenries;
    if (ctTurnsRatio > 0.1f) s_ctTurns = ctTurnsRatio;
    if (frontEndAtten > 0.01f && frontEndAtten <= 1.0f) s_frontEndAtten = frontEndAtten;
}

bool isRunning()
{
    return s_running;
}

} // namespace goertzel_capture
