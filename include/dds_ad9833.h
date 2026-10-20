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

    // 8.000.000 Hz MCLK (HSE / 1) gives 4x lower period jitter (125 ns clock period)
    // and sub-0.03 Hz step resolution: 8,000,000 / 2^28 = 0.0298 Hz!
    static constexpr uint32_t MCLK_FREQ_HZ = 8000000UL;

    /**
     * @brief Initialize PA8 MCO1 hardware clock, SPI1 bus, and AD9833 DDS chip.
     * @param defaultFreqHz Initial frequency in Hz (default 28000.0 Hz)
     * @param wave Initial waveform (default SQUARE_WAVE for ultrasonic transducer driver)
     */
    void begin(double defaultFreqHz = 28000.0, WaveformType wave = SQUARE_WAVE);

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
