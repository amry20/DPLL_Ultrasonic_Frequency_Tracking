/*
 * dds_ad9833.h
 *
 * Driver and hardware controller for AD9833 DDS generator.
 * Connections on STM32F407:
 *   - PC4 : FSYNC / SS (Chip Select, active LOW)
 *   - PA5 : SCLK (SPI1_SCK)
 *   - PA7 : SDATA (SPI1_MOSI)
 *   - PA8 : MCLK (MCO1 hardware clock output = HSE 8MHz / 1 = 8.000 MHz)
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "AD9833.h"

namespace dds
{
    // Hardware pin definitions
    static constexpr uint32_t PIN_SS   = PC4;
    static constexpr uint32_t PIN_SCK  = PA5;
    static constexpr uint32_t PIN_MOSI = PA7;
    static constexpr uint32_t PIN_MCLK = PA8; // MCO1

    // MCO1 Hardware Clock Selection for AD9833 DDS:
    // Enables the user to balance between frequency resolution and phase stability/jitter:
    // - Lower MCLK (e.g. 1.6 MHz - 2 MHz) -> Finer frequency step resolution (0.00596 - 0.00745 Hz)
    // - Higher MCLK (e.g. 8 MHz - 16 MHz) -> Shorter clock period (125 ns - 62.5 ns) & faster update
    // - HSE (External Quartz Crystal) -> High stability (~10-20 ppm, low thermal drift)
    // - HSI (Internal RC Oscillator)  -> Available if external crystal is absent (~1% tolerance)
    enum McoClockSelection : uint8_t {
        MCO_CLOCK_HSE_8MHZ   = 0, // HSE / 1 = 8.0 MHz  (Res: 0.0298 Hz, Crystal stability, jitter 125ns) [Default]
        MCO_CLOCK_HSE_4MHZ   = 1, // HSE / 2 = 4.0 MHz  (Res: 0.0149 Hz, Crystal stability, 2x resolution, jitter 250ns)
        MCO_CLOCK_HSE_2MHZ   = 2, // HSE / 4 = 2.0 MHz  (Res: 0.00745 Hz, Crystal stability, high resolution, jitter 500ns)
        MCO_CLOCK_HSE_1_6MHZ = 3, // HSE / 5 = 1.6 MHz  (Res: 0.00596 Hz, Crystal stability, ultra-fine resolution, jitter 625ns)
        MCO_CLOCK_HSI_16MHZ  = 4, // HSI / 1 = 16.0 MHz (Res: 0.0596 Hz, Internal RC, fastest edge 62.5ns)
        MCO_CLOCK_HSI_8MHZ   = 5, // HSI / 2 = 8.0 MHz  (Res: 0.0298 Hz, Internal RC, standard resolution)
        MCO_CLOCK_HSI_4MHZ   = 6, // HSI / 4 = 4.0 MHz  (Res: 0.0149 Hz, Internal RC, 2x resolution)
        MCO_CLOCK_COUNT      = 7
    };

    struct McoClockInfo {
        McoClockSelection id;
        const char* name;
        uint32_t freqHz;
        double resolutionHz;
        uint32_t rccSource;
        uint32_t rccDiv;
        const char* stability;
        const char* description;
    };

    static constexpr uint32_t DEFAULT_MCLK_FREQ_HZ = 8000000UL;
    static constexpr uint32_t MCLK_FREQ_HZ = DEFAULT_MCLK_FREQ_HZ; // Backwards compatibility

    /**
     * @brief Initialize PA8 MCO1 hardware clock, SPI1 bus, and AD9833 DDS chip.
     * @param defaultFreqHz Initial frequency in Hz (default 28000.0 Hz)
     * @param wave Initial waveform (default SQUARE_WAVE for ultrasonic transducer driver)
     * @param clockSel Initial MCO clock selection (default MCO_CLOCK_HSE_8MHZ)
     */
    void begin(double defaultFreqHz = 28000.0, WaveformType wave = SQUARE_WAVE, McoClockSelection clockSel = MCO_CLOCK_HSE_8MHZ);

    /**
     * @brief Select and configure PA8 MCO1 hardware clock source & prescaler for AD9833 DDS.
     * Updates MCO1 hardware output, AD9833 reference frequency, and seamlessly recalculates
     * the active frequency register.
     * @param clockSel Clock selection enum
     * @return true if successfully changed, false if invalid selection
     */
    bool setMcoClock(McoClockSelection clockSel);

    /**
     * @brief Get currently active MCO clock selection.
     */
    McoClockSelection getMcoClock();

    /**
     * @brief Get active MCLK frequency in Hz.
     */
    uint32_t getMclkFreq();

    /**
     * @brief Get human-readable name of current MCO clock.
     */
    const char* getMcoClockName();

    /**
     * @brief Get metadata for a specific MCO clock selection.
     */
    const McoClockInfo& getMcoClockInfo(McoClockSelection clockSel);

    /**
     * @brief Parse string into McoClockSelection (supports index "0"-"6" or names like "8mhz", "2mhz", etc.).
     * @return Valid McoClockSelection or MCO_CLOCK_COUNT if unknown.
     */
    McoClockSelection parseMcoClock(const char* str);

    /**
     * @brief Set output frequency with sub-Hz precision (resolution 0.00745 Hz).
     * @param freqHz Target frequency in Hz (e.g. 28000.05 Hz)
     */
    void setFrequency(double freqHz);

    /**
     * @brief Get currently programmed frequency in Hz.
     */
    double getFrequency();

    /**
     * @brief Change output waveform (SINE_WAVE, SQUARE_WAVE, TRIANGLE_WAVE).
     */
    void setWaveform(WaveformType wave);

    /**
     * @brief Get currently configured output waveform.
     */
    WaveformType getWaveform();

    /**
     * @brief Enable or disable DDS output (muting / unmuting signal).
     */
    void enableOutput(bool on);

    /**
     * @brief Return true if output is currently enabled.
     */
    bool isEnabled();

    /**
     * @brief Return calculated frequency resolution in Hz (0.00745 Hz).
     */
    double getResolution();

    enum DdsStatus : uint8_t {
        DDS_STATUS_ERROR       = 0, // Uninitialized or hardware init error
        DDS_STATUS_READY       = 1, // MCLK running, SPI initialized, driver ready
        DDS_STATUS_SIGNAL_OK   = 2  // DDS output active AND signal verified on PA0
    };

    /**
     * @brief Get current DDS hardware and operational status.
     * @param measuredRefFreqHz Current measured frequency on PA0 (from phase_capture), or 0.0f
     * @return DdsStatus (0=Error, 1=Ready/OK, 2=Signal Verified)
     */
    DdsStatus getStatus(float measuredRefFreqHz = 0.0f);

} // namespace dds
