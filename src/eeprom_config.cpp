/*
 * eeprom_config.cpp
 *
 * Implementation of DPLL configuration persistence in 24LC16B EEPROM.
 */

#include "eeprom_config.h"
#include "eeprom_24lc16b.h"
#include "dpll_controller.h"
#include <string.h>
#include <math.h>

namespace eeprom_config
{
    static uint32_t calcCrc32(const uint8_t *data, size_t len)
    {
        uint32_t crc = 0xFFFFFFFF;
        for (size_t i = 0; i < len; ++i)
        {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; ++j)
            {
                crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
            }
        }
        return ~crc;
    }

    bool saveCurrentConfig()
    {
        if (!eeprom::isConnected())
        {
            return false;
        }

        PackedConfig cfg;
        memset(&cfg, 0, sizeof(cfg));

        cfg.magic                = CONFIG_MAGIC;
        cfg.version              = CONFIG_VERSION;
        cfg.length               = sizeof(PackedConfig);
        cfg.kp                   = dpll::getKp();
        cfg.ki                   = dpll::getKi();
        cfg.kd                   = dpll::getKd();
        cfg.centerVoltage        = dpll::getCenterVoltage();
        cfg.targetPhase          = dpll::getTargetPhase();
        cfg.maxSlew              = dpll::getMaxSlew();
        cfg.loopPeriodMs         = dpll::getLoopPeriodMs();
        cfg.lockThresholdNs      = dpll::getLockThresholdNs();
        cfg.lockHoldCycles       = dpll::getLockHoldCycles();
        cfg.lockMemoryTimeoutMs  = dpll::getLockMemoryTimeoutMs();
        cfg.streamPeriodMs       = dpll::getStreamPeriodMs();
        cfg.signalLossBehavior   = (uint8_t)dpll::getSignalLossBehavior();

        // Calculate CRC32 over all fields except the crc32 field itself
        size_t crcLen = offsetof(PackedConfig, crc32);
        cfg.crc32 = calcCrc32(reinterpret_cast<const uint8_t *>(&cfg), crcLen);

        if (!eeprom::writeBuffer(EEPROM_CONFIG_ADDR, reinterpret_cast<const uint8_t *>(&cfg), sizeof(cfg)))
        {
            return false;
        }

        // Verify write by reading back
        PackedConfig verify;
        if (!eeprom::readBuffer(EEPROM_CONFIG_ADDR, reinterpret_cast<uint8_t *>(&verify), sizeof(verify)))
        {
            return false;
        }

        return (memcmp(&cfg, &verify, sizeof(cfg)) == 0);
    }

    bool loadSavedConfig()
    {
        if (!eeprom::isConnected())
        {
            return false;
        }

        PackedConfig cfg;
        if (!eeprom::readBuffer(EEPROM_CONFIG_ADDR, reinterpret_cast<uint8_t *>(&cfg), sizeof(cfg)))
        {
            return false;
        }

        if (cfg.magic != CONFIG_MAGIC || cfg.version != CONFIG_VERSION || cfg.length != sizeof(PackedConfig))
        {
            return false;
        }

        size_t crcLen = offsetof(PackedConfig, crc32);
        uint32_t expectedCrc = calcCrc32(reinterpret_cast<const uint8_t *>(&cfg), crcLen);
        if (cfg.crc32 != expectedCrc)
        {
            return false;
        }

        // Sanity checks on loaded values
        if (isnan(cfg.kp) || isnan(cfg.ki) || isnan(cfg.kd) ||
            isnan(cfg.centerVoltage) || cfg.centerVoltage < 0.0f || cfg.centerVoltage > 3.3f ||
            cfg.loopPeriodMs < 1 || cfg.loopPeriodMs > 1000)
        {
            return false;
        }

        // Apply loaded configuration to DPLL controller
        dpll::begin(cfg.centerVoltage, cfg.kp, cfg.ki, cfg.kd);
        dpll::setTargetPhase(cfg.targetPhase);
        dpll::setMaxSlew(cfg.maxSlew);
        dpll::setLoopPeriodMs(cfg.loopPeriodMs);
        dpll::setLockThresholdNs(cfg.lockThresholdNs);
        dpll::setLockHoldCycles(cfg.lockHoldCycles);
        dpll::setLockMemoryTimeoutMs(cfg.lockMemoryTimeoutMs);
        dpll::setStreamPeriodMs(cfg.streamPeriodMs);
        if (cfg.signalLossBehavior <= 2)
        {
            dpll::setSignalLossBehavior((dpll::SignalLossBehavior)cfg.signalLossBehavior);
        }

        return true;
    }

    bool hasValidConfig()
    {
        if (!eeprom::isConnected())
        {
            return false;
        }

        PackedConfig cfg;
        if (!eeprom::readBuffer(EEPROM_CONFIG_ADDR, reinterpret_cast<uint8_t *>(&cfg), sizeof(cfg)))
        {
            return false;
        }

        if (cfg.magic != CONFIG_MAGIC || cfg.version != CONFIG_VERSION || cfg.length != sizeof(PackedConfig))
        {
            return false;
        }

        size_t crcLen = offsetof(PackedConfig, crc32);
        uint32_t expectedCrc = calcCrc32(reinterpret_cast<const uint8_t *>(&cfg), crcLen);
        return (cfg.crc32 == expectedCrc);
    }

    bool eraseConfig()
    {
        if (!eeprom::isConnected())
        {
            return false;
        }

        uint8_t zeros[4] = {0, 0, 0, 0};
        return eeprom::writeBuffer(EEPROM_CONFIG_ADDR, zeros, sizeof(zeros));
    }

} // namespace eeprom_config
