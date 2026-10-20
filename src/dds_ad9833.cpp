/*
 * dds_ad9833.cpp
 *
 * Driver and hardware controller for AD9833 DDS generator.
 * Configures PA8 as MCO1 (HSE 8MHz / 4 = 2.000 MHz), initializes SPI1,
 * and manages the AD9833 DDS chip with 64-bit double precision frequency stepping.
 */

#include "dds_ad9833.h"

namespace dds
{
    static AD9833 s_dds(PIN_SS, MCLK_FREQ_HZ);
    static double s_currentFreq = 28000.0;
    static WaveformType s_currentWave = SQUARE_WAVE;
    static bool s_enabled = false;
    static bool s_initialized = false;

    void begin(double defaultFreqHz, WaveformType wave)
    {
        // 1. Configure PA8 as MCO1 hardware clock output:
        //    Source: HSE (8.000 MHz external crystal)
        //    Prescaler: /1 -> 8.000.000 Hz MCLK to AD9833
        //    Provides 4x lower period jitter (125 ns period) with 0.0298 Hz resolution.
        HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);

        // 2. Set up SPI hardware pins for SPI1 (PA5=SCK, PA7=MOSI)
        SPI.setSCLK(PIN_SCK);
        SPI.setMOSI(PIN_MOSI);
        SPI.begin();

        // 3. Initialize AD9833 with SPI bus
        s_dds.Begin(&SPI);

        s_currentFreq = defaultFreqHz;
        s_currentWave = wave;

        // Apply initial frequency and waveform to REG0
        s_dds.ApplySignal(s_currentWave, REG0, s_currentFreq);

        // Un-reset and enable output immediately!
        s_dds.EnableOutput(true);
        s_enabled = true;
        s_initialized = true;
    }

    void setFrequency(double freqHz)
    {
        if (freqHz < 1.0) freqHz = 1.0;
        if (freqHz > ((double)MCLK_FREQ_HZ / 2.0)) freqHz = (double)MCLK_FREQ_HZ / 2.0;

        // Avoid spamming SPI writes if frequency has not changed.
        // Rewriting the AD9833 frequency registers resets phase accumulation states
        // and creates oscilloscope jitter / breathing artifacts.
        if (s_initialized && fabs(freqHz - s_currentFreq) < 0.005) {
            return;
        }

        s_currentFreq = freqHz;
        s_dds.SetFrequency(REG0, freqHz);
    }

    double getFrequency()
    {
        return s_currentFreq;
    }

    void setWaveform(WaveformType wave)
    {
        s_currentWave = wave;
        s_dds.SetWaveform(REG0, wave);
    }

    WaveformType getWaveform()
    {
        return s_currentWave;
    }

    void enableOutput(bool on)
    {
        s_enabled = on;
        s_dds.EnableOutput(on);
    }

    bool isEnabled()
    {
        return s_enabled;
    }

    double getResolution()
    {
        return (double)MCLK_FREQ_HZ / 268435456.0; // ~0.0298023 Hz
    }

    DdsStatus getStatus(float measuredRefFreqHz)
    {
        if (!s_initialized) {
            return DDS_STATUS_ERROR;
        }

        // Complementary check: if output is enabled and measured frequency on PA0
        // matches the programmed DDS frequency within ±5%, signal is physically verified!
        if (s_enabled && measuredRefFreqHz > 100.0f) {
            double diff = fabs((double)measuredRefFreqHz - s_currentFreq);
            if (diff <= (s_currentFreq * 0.05)) {
                return DDS_STATUS_SIGNAL_OK;
            }
        }

        return DDS_STATUS_READY;
    }
} // namespace dds
