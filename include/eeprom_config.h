/*
 * eeprom_config.h
 *
 * Persistent storage of DPLL configuration in 24LC16B EEPROM.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

namespace eeprom_config
{
    static constexpr uint32_t CONFIG_MAGIC = 0x44504C31; // "DPL1" in ASCII
    static constexpr uint16_t CONFIG_VERSION = 4;
    static constexpr uint16_t EEPROM_CONFIG_ADDR = 0x0000;

    struct PackedConfig
    {
        uint32_t magic;
        uint16_t version;
        uint16_t length;
        float    kp;
        float    ki;
        float    kd;
        float    centerVoltage;
        float    targetPhase;
        float    maxSlew;
        uint32_t loopPeriodMs;
        float    lockThresholdNs;
        uint32_t lockHoldCycles;
        uint32_t lockMemoryTimeoutMs;
        uint32_t streamPeriodMs;
        uint8_t  signalLossBehavior;
        uint8_t  generatorSource;       // 0=DAC, 1=DDS
        uint8_t  ddsMcoClock;           // McoClockSelection (0=8MHz HSE, 1=4MHz HSE, etc.)
        uint8_t  captureRefEdge;        // 0=Rising, 1=Falling, 2=Both
        uint8_t  captureZcdEdge;        // 0=Rising, 1=Falling, 2=Both
        uint8_t  trackingDirection;     // 0=Normal (Series fs), 1=Inverted (Parallel fp)
        float    ddsCenterFreq;        // Hz
        float    ddsMinFreq;           // Hz
        float    ddsMaxFreq;           // Hz
        float    ddsMaxSlew;           // Hz/s
        uint32_t crc32;
    } __attribute__((packed));

    /**
     * @brief Save the currently active DPLL controller configuration to EEPROM.
     * @return true if written and verified successfully, false otherwise.
     */
    bool saveCurrentConfig();

    /**
     * @brief Load DPLL controller configuration from EEPROM and apply it to controller.
     * @return true if valid configuration was found and applied, false otherwise.
     */
    bool loadSavedConfig();

    /**
     * @brief Check whether a valid configuration exists in EEPROM (valid magic & CRC).
     * @return true if valid, false otherwise.
     */
    bool hasValidConfig();

    /**
     * @brief Erase / invalidate the configuration header in EEPROM.
     * @return true on success, false otherwise.
     */
    bool eraseConfig();

    /**
     * @brief Reset EEPROM configuration (wiping stored config with 0xFF)
     *        and revert the active DPLL controller, DDS, and DAC to default baseline parameters.
     * @return true on success, false otherwise.
     */
    bool resetConfig();

} // namespace eeprom_config
