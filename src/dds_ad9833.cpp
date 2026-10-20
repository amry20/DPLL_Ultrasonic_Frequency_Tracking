/*
 * dds_ad9833.cpp
 *
 * Driver and hardware controller for AD9833 DDS generator.
 * Configures PA8 as MCO1 hardware clock output with selectable clock source & prescaler
 * (allowing user to balance frequency resolution and phase stability/jitter),
 * initializes SPI1, and manages the AD9833 DDS chip with 64-bit double precision frequency stepping.
 */

#include "dds_ad9833.h"

namespace dds
{
    static const McoClockInfo s_clockTable[MCO_CLOCK_COUNT] = {
        { MCO_CLOCK_HSE_8MHZ,   "HSE 8MHz (/1)",   8000000UL,  8000000.0  / 268435456.0, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1, "Crystal (High Stability)", "Crystal 8.0 MHz, low phase jitter (125ns), res 0.0298 Hz [Default]" },
        { MCO_CLOCK_HSE_4MHZ,   "HSE 4MHz (/2)",   4000000UL,  4000000.0  / 268435456.0, RCC_MCO1SOURCE_HSE, RCC_MCODIV_2, "Crystal (High Stability)", "Crystal 4.0 MHz, 2x finer resolution (0.0149 Hz), jitter 250ns" },
        { MCO_CLOCK_HSE_2MHZ,   "HSE 2MHz (/4)",   2000000UL,  2000000.0  / 268435456.0, RCC_MCO1SOURCE_HSE, RCC_MCODIV_4, "Crystal (High Stability)", "Crystal 2.0 MHz, high resolution (0.00745 Hz), jitter 500ns" },
        { MCO_CLOCK_HSE_1_6MHZ, "HSE 1.6MHz (/5)", 1600000UL,  1600000.0  / 268435456.0, RCC_MCO1SOURCE_HSE, RCC_MCODIV_5, "Crystal (High Stability)", "Crystal 1.6 MHz, ultra-fine resolution (0.00596 Hz), jitter 625ns" },
        { MCO_CLOCK_HSI_16MHZ,  "HSI 16MHz (/1)",  16000000UL, 16000000.0 / 268435456.0, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1, "Internal RC (~1% Drift)", "Internal RC 16.0 MHz, fastest edge (62.5ns), res 0.0596 Hz" },
        { MCO_CLOCK_HSI_8MHZ,   "HSI 8MHz (/2)",   8000000UL,  8000000.0  / 268435456.0, RCC_MCO1SOURCE_HSI, RCC_MCODIV_2, "Internal RC (~1% Drift)", "Internal RC 8.0 MHz, standard resolution (0.0298 Hz)" },
        { MCO_CLOCK_HSI_4MHZ,   "HSI 4MHz (/4)",   4000000UL,  4000000.0  / 268435456.0, RCC_MCO1SOURCE_HSI, RCC_MCODIV_4, "Internal RC (~1% Drift)", "Internal RC 4.0 MHz, high resolution (0.0149 Hz)" }
    };

    static McoClockSelection s_currentClockSel = MCO_CLOCK_HSE_8MHZ;
    static uint32_t s_currentMclkHz = DEFAULT_MCLK_FREQ_HZ;
    static AD9833 s_dds(PIN_SS, DEFAULT_MCLK_FREQ_HZ);
    static double s_currentFreq = 28000.0;
    static WaveformType s_currentWave = SQUARE_WAVE;
    static bool s_enabled = false;
    static bool s_initialized = false;

    bool setMcoClock(McoClockSelection clockSel)
    {
        if (clockSel >= MCO_CLOCK_COUNT) {
            return false;
        }

        s_currentClockSel = clockSel;
        const McoClockInfo &info = s_clockTable[clockSel];
        s_currentMclkHz = info.freqHz;

        // 1. Configure PA8 as MCO1 hardware clock output with chosen source & prescaler
        HAL_RCC_MCOConfig(RCC_MCO1, info.rccSource, info.rccDiv);

        // 2. Update AD9833 reference frequency so frequency calculations use the new clock
        s_dds.SetRefClock(s_currentMclkHz);

        // 3. If initialized, re-apply signal to frequency registers so output transitions smoothly
        if (s_initialized) {
            s_dds.ApplySignal(s_currentWave, REG0, s_currentFreq);
            s_dds.EnableOutput(s_enabled);
        }

        return true;
    }

    McoClockSelection getMcoClock()
    {
        return s_currentClockSel;
    }

    uint32_t getMclkFreq()
    {
        return s_currentMclkHz;
    }

    const char* getMcoClockName()
    {
        return s_clockTable[s_currentClockSel].name;
    }

    const McoClockInfo& getMcoClockInfo(McoClockSelection clockSel)
    {
        if (clockSel >= MCO_CLOCK_COUNT) {
            return s_clockTable[0];
        }
        return s_clockTable[clockSel];
    }

    McoClockSelection parseMcoClock(const char* str)
    {
        if (!str || str[0] == '\0') {
            return MCO_CLOCK_COUNT;
        }

        // If single digit 0-6
        if (str[1] == '\0' && str[0] >= '0' && str[0] <= '6') {
            return (McoClockSelection)(str[0] - '0');
        }

        String s = str;
        s.toLowerCase();
        s.trim();

        if (s == "0" || s == "8" || s == "8mhz" || s == "8m" || s == "hse8" || s == "hse8mhz" || s == "hse/1" || s == "hse_8mhz")
            return MCO_CLOCK_HSE_8MHZ;
        if (s == "1" || s == "4" || s == "4mhz" || s == "4m" || s == "hse4" || s == "hse4mhz" || s == "hse/2" || s == "hse_4mhz")
            return MCO_CLOCK_HSE_4MHZ;
        if (s == "2" || s == "2mhz" || s == "2m" || s == "hse2" || s == "hse2mhz" || s == "hse/4" || s == "hse_2mhz")
            return MCO_CLOCK_HSE_2MHZ;
        if (s == "3" || s == "1.6" || s == "1.6mhz" || s == "1.6m" || s == "hse1.6" || s == "hse1.6mhz" || s == "hse/5" || s == "hse_1_6mhz")
            return MCO_CLOCK_HSE_1_6MHZ;
        if (s == "4" || s == "16" || s == "16mhz" || s == "16m" || s == "hsi16" || s == "hsi16mhz" || s == "hsi/1" || s == "hsi_16mhz")
            return MCO_CLOCK_HSI_16MHZ;
        if (s == "5" || s == "hsi8" || s == "hsi8mhz" || s == "hsi/2" || s == "hsi_8mhz")
            return MCO_CLOCK_HSI_8MHZ;
        if (s == "6" || s == "hsi4" || s == "hsi4mhz" || s == "hsi/4" || s == "hsi_4mhz")
            return MCO_CLOCK_HSI_4MHZ;

        return MCO_CLOCK_COUNT;
    }

    void begin(double defaultFreqHz, WaveformType wave, McoClockSelection clockSel)
    {
        // 1. Configure PA8 as MCO1 hardware clock output with chosen clock preset
        setMcoClock(clockSel);

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
        double maxF = (double)s_currentMclkHz / 2.0;
        if (freqHz > maxF) freqHz = maxF;

        if (!s_initialized) {
            s_currentFreq = freqHz;
            return;
        }

        // Avoid spamming SPI writes if frequency change is smaller than half a step
        // Rewriting the AD9833 frequency registers resets phase accumulation states
        // and creates oscilloscope jitter / breathing artifacts.
        double minDelta = getResolution() * 0.5;
        if (minDelta < 0.002) minDelta = 0.002;
        if (fabs(freqHz - s_currentFreq) < minDelta) {
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
        if (s_initialized) {
            s_dds.SetWaveform(REG0, wave);
        }
    }

    WaveformType getWaveform()
    {
        return s_currentWave;
    }

    void enableOutput(bool on)
    {
        s_enabled = on;
        if (s_initialized) {
            s_dds.EnableOutput(on);
        }
    }

    bool isEnabled()
    {
        return s_enabled;
    }

    double getResolution()
    {
        return (double)s_currentMclkHz / 268435456.0;
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
