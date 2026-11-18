/*
 * goertzel_capture.cpp
 *
 * Dual-Channel Simultaneous ADC + Goertzel DFT Front-End for STM32F407.
 */

#include "goertzel_capture.h"
#include "dma_buffer_pool.h"
#include <Arduino.h>
#include <stm32f4xx_hal.h>
#include <cmath>

namespace goertzel_capture {

namespace {

constexpr float PI_F = 3.14159265358979323846f;
constexpr float TWO_PI_F = 6.28318530717958647692f;

// Two hardware DMA targets plus separate ready and CPU-owned blocks. Only
// inactive DMA addresses are exchanged in the ISR; the CPU block stays stable
// throughout Goertzel processing and subsequent waveform snapshots.
alignas(32) static uint32_t s_adcDmaBuf[4][N_SAMPLES];
static DmaBufferPool s_bufferPool;
static volatile uint8_t s_dmaSlotBuffer[2] = {0, 1};

// Precomputed Hann window and window sum placed in CCMRAM (accessible by Cortex-M4 core)
__attribute__((section(".ccmram"))) static float s_hannWindow[N_SAMPLES];
static float s_hannSum = 0.0f;
static bool  s_windowInit = false;

// Hardware handles
static ADC_HandleTypeDef s_hadc1;
static ADC_HandleTypeDef s_hadc2;
static DMA_HandleTypeDef s_hdma_adc;
static TIM_HandleTypeDef s_htim3;

static volatile uint32_t s_dmaBlocksCaptured = 0;
static volatile uint32_t s_readySequence = 0;
static volatile uint32_t s_readyCapturedMs = 0;
static volatile bool s_dmaFault = false;
static bool s_running = false;
static bool s_haveMetrics = false;
constexpr uint32_t kMaxMetricAgeMs = 5;

// Calibration parameters
// R_burden = 50.0 ohm, C_burden = 6.8 nF, L_sec = 24 mH, 1:100 CT ratio, input divider = 0.3333 (10k / (20k + 10k))
static float s_rBurden = 50.0f;
static float s_cBurden = 6.8e-9f;
static float s_lSec    = 24.0e-3f;
static float s_ctTurns = 100.0f;
static float s_frontEndAtten = 0.333333f; // 10k / (20k + 10k) for 20k series resistor
static float s_phaseOffsetDeg = 180.0f;   // 180 deg for inverted CT polarity relative to PA7
static bool  s_burdenCompEnabled = false; // Optional reactive burden compensation (disabled by default)

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

// DBM switches CT in hardware at each completed 1024-pair block. Redirect
// only the completed (now inactive) slot before publishing its former buffer.
extern "C" void DMA2_Stream0_IRQHandler(void)
{
    const uint32_t flags = DMA2->LISR;
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 |
                  DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;
    if (flags & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0)) {
        s_dmaFault = true;
        TIM3->CR1 &= ~TIM_CR1_CEN;
        DMA2_Stream0->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
        DMA2_Stream0->FCR &= ~DMA_SxFCR_FEIE;
        return;
    }
    if ((flags & DMA_LISR_TCIF0) == 0) return;

    const uint8_t slot = (DMA2_Stream0->CR & DMA_SxCR_CT) ? 0U : 1U;
    const uint8_t completed = s_dmaSlotBuffer[slot];
    const uint8_t replacement = s_bufferPool.complete(completed);
    if (slot == 0) DMA2_Stream0->M0AR = reinterpret_cast<uint32_t>(s_adcDmaBuf[replacement]);
    else           DMA2_Stream0->M1AR = reinterpret_cast<uint32_t>(s_adcDmaBuf[replacement]);
    s_dmaSlotBuffer[slot] = replacement;
    s_readySequence = ++s_dmaBlocksCaptured;
    s_readyCapturedMs = millis();
}

void begin()
{
    if (s_running) return;
    s_dmaFault = false;
    s_haveMetrics = false;
    s_bufferPool.reset();
    s_dmaSlotBuffer[0] = 0;
    s_dmaSlotBuffer[1] = 1;

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
    ccr |= ADC_CCR_DDS;                        // Continuous requests in multi-ADC mode
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
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 |
                  DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;
    if (HAL_DMAEx_MultiBufferStart(&s_hdma_adc, (uint32_t)&ADC->CDR,
            (uint32_t)s_adcDmaBuf[0], (uint32_t)s_adcDmaBuf[1], N_SAMPLES) != HAL_OK) {
        s_dmaFault = true;
        return;
    }
    DMA2_Stream0->CR |= DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    DMA2_Stream0->FCR |= DMA_SxFCR_FEIE;
    s_hadc1.Instance->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;
    
    // Enable ADC2 (slave) first, then ADC1 (master)
    __HAL_ADC_ENABLE(&s_hadc2);
    __HAL_ADC_ENABLE(&s_hadc1);

    // Allow both ADCs to stabilize before the first trigger.
    delayMicroseconds(3);
    HAL_TIM_Base_Start(&s_htim3);

    s_running = true;
}

bool poll(float f_drive)
{
    if (!s_running || s_dmaFault) return false;
    if ((ADC1->SR & ADC_SR_OVR) || (ADC2->SR & ADC_SR_OVR)) {
        s_dmaFault = true;
        TIM3->CR1 &= ~TIM_CR1_CEN;
        return false;
    }

    const uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    const uint8_t block = s_bufferPool.acquire();
    const uint32_t sequence = s_readySequence;
    const uint32_t capturedMs = s_readyCapturedMs;
    __set_PRIMASK(irqState);
    if (block == DmaBufferPool::None) return false;
    const uint32_t* blk = s_adcDmaBuf[block];

    if (!std::isfinite(f_drive) || f_drive < 1000.0f || f_drive > 100000.0f) {
        s_haveMetrics = false;
        return false;
    }

    // Goertzel parameters for this block
    const float omega = TWO_PI_F * f_drive / FS_HZ;
    const float cosW  = cosf(omega);
    const float coeff = 2.0f * cosW;
    const float sinW  = sinf(omega);

    float cur_s1 = 0.0f, cur_s2 = 0.0f;
    float ref_s1 = 0.0f, ref_s2 = 0.0f;

    uint16_t curRailCount = 0;

    // Execute Goertzel recurrence for both channels simultaneously in a single cache-friendly pass
    for (uint16_t n = 0; n < N_SAMPLES; n++) {
        const uint32_t pair = blk[n];
        const uint16_t raw_cur = (uint16_t)(pair & 0x0FFFU);
        const uint16_t raw_ref = (uint16_t)((pair >> 16) & 0x0FFFU);

        if (raw_cur <= 5 || raw_cur >= 4090) {
            curRailCount++;
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

    // Only the current channel is checked for clipping: the PA7 drive reference is a
    // square wave from PA8 that legitimately sits at the rails.
    const bool clipped = (curRailCount >= 3);

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

    // Phase difference (Drive Reference minus Current so lagging current = positive phase,
    // matching ZCD t_ZCD - t_REF convention).
    float dphi = ref_phi - cur_phi;
    while (dphi >  PI_F) dphi -= TWO_PI_F;
    while (dphi < -PI_F) dphi += TWO_PI_F;

    // Optional software compensation for CT Burden (R_burden || C_burden || L_sec)
    // When disabled (default), only resistive burden scaling (1/R_burden) and
    // fixed phaseOffsetDeg are applied without reactive phase shift.
    const float omegaDrive = TWO_PI_F * f_drive;
    float bNet = 0.0f;
    float phiBurdenRad = 0.0f;
    if (s_burdenCompEnabled) {
        bNet = (omegaDrive * s_cBurden) - (1.0f / (omegaDrive * s_lSec > 1e-6f ? omegaDrive * s_lSec : 1e-6f));
        phiBurdenRad = atanf(s_rBurden * bNet);
    }

    // Corrected true phase at transducer terminal + configurable polarity/offset
    float trueDphi = dphi - phiBurdenRad + (s_phaseOffsetDeg * (PI_F / 180.0f));
    while (trueDphi >  PI_F) trueDphi -= TWO_PI_F;
    while (trueDphi < -PI_F) trueDphi += TWO_PI_F;

    const float phaseDeg = trueDphi * (180.0f / PI_F);
    const float phaseNs  = (trueDphi / omegaDrive) * 1e9f;

    // Transducer Peak Current:
    // V_burden = V_adc_pin / frontEndAtten
    // I_sec = V_burden * sqrt((1/R)^2 + B_net^2)  (B_net=0 when burden comp disabled)
    // I_primary = I_sec * ctTurns
    const float vBurdenPk = (s_frontEndAtten > 1e-4f) ? (cur_pk_v / s_frontEndAtten) : cur_pk_v;
    const float yBurdenMag = s_burdenCompEnabled
        ? sqrtf((1.0f / (s_rBurden * s_rBurden)) + (bNet * bNet))
        : (1.0f / s_rBurden);
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
    s_latestMetrics.blockCount      = sequence;
    s_latestMetrics.capturedAtMs    = capturedMs;
    s_latestMetrics.droppedBlocks   = s_bufferPool.dropped();
    s_latestMetrics.refValid        = refValid;
    s_latestMetrics.currentValid    = curValid;
    s_latestMetrics.clipped         = clipped;
    s_latestMetrics.valid           = refValid && curValid;
    s_haveMetrics = true;

    return true;
}

GoertzelMetrics getMetrics()
{
    GoertzelMetrics result = s_latestMetrics;
    if (!s_running || !s_haveMetrics || s_dmaFault ||
        millis() - result.capturedAtMs > kMaxMetricAgeMs) {
        result.refValid = result.currentValid = result.valid = false;
    }
    return result;
}

void setBurdenModel(float rBurdenOhms, float cBurdenFarads, float lSecHenries, float ctTurnsRatio, float frontEndAtten, float phaseOffsetDeg, bool burdenCompEnabled)
{
    if (std::isfinite(rBurdenOhms) && rBurdenOhms > 0.1f && rBurdenOhms <= 1e6f) s_rBurden = rBurdenOhms;
    if (std::isfinite(cBurdenFarads) && cBurdenFarads >= 0.0f && cBurdenFarads <= 1.0f) s_cBurden = cBurdenFarads;
    if (std::isfinite(lSecHenries) && lSecHenries > 1e-6f && lSecHenries <= 100.0f) s_lSec = lSecHenries;
    if (std::isfinite(ctTurnsRatio) && ctTurnsRatio > 0.1f && ctTurnsRatio <= 100000.0f) s_ctTurns = ctTurnsRatio;
    if (std::isfinite(frontEndAtten) && frontEndAtten > 0.001f && frontEndAtten <= 1.0f) s_frontEndAtten = frontEndAtten;
    if (std::isfinite(phaseOffsetDeg) && phaseOffsetDeg >= -360.0f && phaseOffsetDeg <= 360.0f) s_phaseOffsetDeg = phaseOffsetDeg;
    s_burdenCompEnabled = burdenCompEnabled;
}

BurdenModel getBurdenModel()
{
    return BurdenModel{ s_rBurden, s_cBurden, s_lSec, s_ctTurns, s_frontEndAtten, s_phaseOffsetDeg, s_burdenCompEnabled };
}

void resetBurdenModel()
{
    s_rBurden           = 50.0f;
    s_cBurden           = 6.8e-9f;
    s_lSec              = 24.0e-3f;
    s_ctTurns           = 100.0f;
    s_frontEndAtten     = 0.333333f;
    s_phaseOffsetDeg    = 180.0f;
    s_burdenCompEnabled = false;
}

bool isRunning()
{
    return s_running;
}

bool captureSnapshot(uint16_t* curOut, uint16_t* refOut, uint16_t maxCount, uint16_t& actualCount, float& driveFreqHz, float& frontEndAtten)
{
    actualCount = 0;
    const uint8_t block = s_bufferPool.processed();
    if (!s_running || !s_haveMetrics || s_dmaFault || curOut == nullptr ||
        block == DmaBufferPool::None || maxCount == 0 ||
        millis() - s_latestMetrics.capturedAtMs > kMaxMetricAgeMs) return false;

    const uint32_t* blk = s_adcDmaBuf[block];
    driveFreqHz = s_latestMetrics.driveFreqHz;
    frontEndAtten = s_frontEndAtten;

    uint16_t nSamples = (maxCount > 240) ? 240 : maxCount;
    if (nSamples > N_SAMPLES) nSamples = N_SAMPLES;

    // Edge trigger search: find rising edge crossing mid-scale (2048) on reference channel
    // within the first 200 samples so trace is rock-steady on screen
    uint16_t trigIdx = 0;
    for (uint16_t i = 1; i < 200 && (i + nSamples) < N_SAMPLES; i++) {
        uint16_t vPrev = (uint16_t)((blk[i - 1] >> 16) & 0x0FFF);
        uint16_t vCurr = (uint16_t)((blk[i] >> 16) & 0x0FFF);
        if (vPrev < 2048 && vCurr >= 2048) {
            trigIdx = i;
            break;
        }
    }

    // If reference didn't trigger, fallback to current channel edge trigger
    if (trigIdx == 0) {
        for (uint16_t i = 1; i < 200 && (i + nSamples) < N_SAMPLES; i++) {
            uint16_t vPrev = (uint16_t)(blk[i - 1] & 0x0FFF);
            uint16_t vCurr = (uint16_t)(blk[i] & 0x0FFF);
            if (vPrev < 2048 && vCurr >= 2048) {
                trigIdx = i;
                break;
            }
        }
    }

    for (uint16_t i = 0; i < nSamples; i++) {
        uint32_t pair = blk[trigIdx + i];
        curOut[i] = (uint16_t)(pair & 0x0FFF);
        if (refOut != nullptr) {
            refOut[i] = (uint16_t)((pair >> 16) & 0x0FFF);
        }
    }

    actualCount = nSamples;
    return true;
}

} // namespace goertzel_capture
