/*
 * goertzel_capture.h
 *
 * Dual-Channel Simultaneous ADC + Goertzel DFT Front-End for STM32F407.
 *
 * Current sense:   PC4 (ADC1_IN14) - from T2 burden op-amp buffer (centred on VREF 1.65V)
 * Drive reference: PA7 (ADC2_IN7)  - from PA8 TIM1 PWM via RC filter (1k + 1nF)
 *
 * Sampling: 750 kSPS triggered by TIM3 TRGO.
 * DMA:      DMA2 Stream 0 Channel 0 in circular mode, two 1024-pair DMA targets and two ownership buffers.
 * Method:   Single-bin Goertzel DFT with Hann windowing at f_drive.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

namespace goertzel_capture {

constexpr uint16_t N_SAMPLES   = 1024;        // Samples per block (~1.37 ms at 750 kSPS)
constexpr float    FS_HZ       = 750000.0f;   // ADC sampling rate (Hz)
constexpr float    ADC_MID_F   = 2048.0f;     // Mid-scale VREF 1.65 V in 12-bit ADC counts

struct GoertzelMetrics {
    float currentAmpV;       // Peak AC voltage at ADC pin PC4 (Volts)
    float currentAmpA;       // Peak AC current at transducer / burden (Amperes)
    float currentPhaseRad;   // Current fundamental phase relative to block (radians)
    float refAmpV;           // Peak AC fundamental voltage at PA7 (Volts)
    float refPhaseRad;       // Reference fundamental phase relative to block (radians)
    float phaseDiffRad;      // cur.phase - ref.phase, wrapped to [-pi, +pi]
    float phaseDiffDeg;      // Phase difference (degrees)
    float phaseDiffNs;       // Phase difference in time (nanoseconds)
    float driveFreqHz;       // Frequency at which Goertzel was computed
    uint32_t blockCount;     // Sequence of this processed block
    uint32_t capturedAtMs;   // DMA completion timestamp; used to reject stale metrics
    uint32_t droppedBlocks; // Completed blocks replaced before the main loop acquired them
    bool  refValid;          // True if refAmpV > threshold (~0.2 V)
    bool  currentValid;      // True if currentAmpV > threshold and not clipped
    bool  clipped;           // True if either channel saturated ADC rails
    bool  valid;             // True if both channels are valid
};

/**
 * @brief Initialize TIM3 TRGO (750 kHz), PC4 (ADC1_IN14), PA7 (ADC2_IN7), Dual Simultaneous ADC, and DMA2.
 * Precomputes the 1024-point Hann window lookup table.
 */
void begin();

/**
 * @brief Check if a new DMA half/full block is ready, compute Goertzel DFT at f_drive, and update metrics.
 * Call frequently from the main loop.
 * @param f_drive Current driving frequency in Hz (from TIM1 PWM dither)
 * @return true if a fresh block was processed, false otherwise
 */
bool poll(float f_drive);

/**
 * @brief Get the latest calculated Goertzel metrics snapshot.
 */
GoertzelMetrics getMetrics();

struct BurdenModel {
    float rBurdenOhms;
    float cBurdenFarads;
    float lSecHenries;
    float ctTurnsRatio;
    float frontEndAtten;
    float phaseOffsetDeg;
    bool  burdenCompEnabled;
};

/**
 * @brief Configure CT burden model parameters for software phase & amplitude calibration.
 * @param rBurdenOhms Burden resistor (e.g. 50.0 ohms)
 * @param cBurdenFarads Burden parallel capacitance (e.g. 6.8e-9 F)
 * @param lSecHenries Burden secondary inductance (e.g. 24.0e-3 H)
 * @param ctTurnsRatio Turns ratio (e.g. 100.0 for 1:100 CT)
 * @param frontEndAtten Attenuation factor of input divider (e.g. 0.333333 for 10k/(20k+10k))
 * @param phaseOffsetDeg Fixed phase offset in degrees (e.g. 180.0 if CT polarity is inverted)
 * @param burdenCompEnabled Enable reactive burden compensation (C_burden || L_sec). Default false.
 */
void setBurdenModel(float rBurdenOhms, float cBurdenFarads, float lSecHenries, float ctTurnsRatio, float frontEndAtten, float phaseOffsetDeg = 180.0f, bool burdenCompEnabled = false);

/**
 * @brief Get the active CT burden model parameters.
 */
BurdenModel getBurdenModel();

/**
 * @brief Restore factory default CT burden model parameters.
 */
void resetBurdenModel();

/**
 * @brief Check whether Goertzel capture hardware is initialized and running.
 */
bool isRunning();

/**
 * @brief Capture a synchronous snapshot of the latest CPU-owned raw ADC waveform (PC4 current & PA7 drive ref).
 * Uses edge-triggering around mid-scale (2048) to stabilize display.
 * @param curOut Array to receive PC4 current sense 12-bit ADC raw samples
 * @param refOut Array to receive PA7 drive ref 12-bit ADC raw samples (can be nullptr)
 * @param maxCount Max samples to copy (e.g. 240)
 * @param actualCount Returns number of samples copied
 * @param driveFreqHz Returns active drive frequency
 * @param frontEndAtten Returns input attenuator ratio
 * @return true if successful
 */
bool captureSnapshot(uint16_t* curOut, uint16_t* refOut, uint16_t maxCount, uint16_t& actualCount, float& driveFreqHz, float& frontEndAtten);

} // namespace goertzel_capture

