/*
 * eeprom_config.cpp
 *
 * Implementation of DPLL configuration persistence in 24LC16B EEPROM.
 */

#include "eeprom_config.h"
#include "eeprom_24lc16b.h"
#include "dpll_controller.h"
#include "dds_ad9833.h"
#include "dac_output.h"
#include "phase_capture.h"
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
        cfg.generatorSource      = (uint8_t)dpll::getGeneratorSource();
        cfg.ddsMcoClock          = (uint8_t)dds::getMcoClock();
        cfg.captureRefEdge       = (uint8_t)phase_capture::getRefEdge();
        cfg.captureZcdEdge       = (uint8_t)phase_capture::getZcdEdge();
        cfg.trackingDirection    = (uint8_t)dpll::getTrackingDirection();
        cfg.ddsCenterFreq        = dpll::getDdsCenterFreq();
        cfg.ddsMinFreq           = dpll::getDdsMinFreq();
        cfg.ddsMaxFreq           = dpll::getDdsMaxFreq();
        cfg.ddsMaxSlew           = dpll::getDdsMaxSlew();

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

        if (cfg.magic != CONFIG_MAGIC || (cfg.version != 1 && cfg.version != 2 && cfg.version != 3 && cfg.version != 4))
        {
            return false;
        }

        size_t crcLen = (cfg.version >= 2) ? offsetof(PackedConfig, crc32) : 56;
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

        // Apply DDS & generator parameters first if version >= 2
        if (cfg.version >= 2)
        {
            if (cfg.generatorSource <= 1)
            {
                dpll::setGeneratorSource((dpll::GeneratorSource)cfg.generatorSource);
            }
            dpll::setDdsFreqLimits(cfg.ddsMinFreq, cfg.ddsMaxFreq);
            dpll::setDdsMaxSlew(cfg.ddsMaxSlew);
            dpll::setDdsCenterFreq(cfg.ddsCenterFreq);
            if (cfg.ddsMcoClock < (uint8_t)dds::MCO_CLOCK_COUNT)
            {
                dds::setMcoClock((dds::McoClockSelection)cfg.ddsMcoClock);
            }
        }

        // Apply capture edge polarities if version >= 3
        if (cfg.version >= 3)
        {
            if (cfg.captureRefEdge <= 2)
            {
                phase_capture::setRefEdge((phase_capture::EdgePolarity)cfg.captureRefEdge);
            }
            if (cfg.captureZcdEdge <= 2)
            {
                phase_capture::setZcdEdge((phase_capture::EdgePolarity)cfg.captureZcdEdge);
            }
        }
        else
        {
            phase_capture::setRefEdge(phase_capture::CAPTURE_RISING);
            phase_capture::setZcdEdge(phase_capture::CAPTURE_RISING);
        }

        // Apply tracking direction if version >= 4
        if (cfg.version >= 4)
        {
            if (cfg.trackingDirection <= 1)
            {
                dpll::setTrackingDirection((dpll::TrackingDirection)cfg.trackingDirection);
            }
        }
        else
        {
            dpll::setTrackingDirection(dpll::TRACKING_DIR_NORMAL);
        }

        dpll::begin(cfg.centerVoltage, cfg.kp, cfg.ki, cfg.kd);
        dpll::setTargetPhase(cfg.targetPhase);
        if (cfg.version >= 2 && cfg.generatorSource == 1)
        {
            dpll::setDdsMaxSlew(cfg.ddsMaxSlew);
        }
        else
        {
            dpll::setMaxSlew(cfg.maxSlew);
        }
        dpll::setLoopPeriodMs(cfg.loopPeriodMs);
        dpll::setLockThresholdNs(cfg.lockThresholdNs);
        dpll::setLockHoldCycles(cfg.lockHoldCycles);
        dpll::setLockMemoryTimeoutMs(cfg.lockMemoryTimeoutMs);
        dpll::setStreamPeriodMs(cfg.streamPeriodMs);
        if (cfg.signalLossBehavior <= 2)
        {
            dpll::setSignalLossBehavior((dpll::SignalLossBehavior)cfg.signalLossBehavior);
        }

        // Ensure DDS output frequency matches the center frequency loaded from EEPROM
        dds::setFrequency((double)dpll::getDdsCenterFreq());

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

        // Fill entire PackedConfig area with 0xFF (factory unprogrammed state)
        return eeprom::fill(EEPROM_CONFIG_ADDR, 0xFF, sizeof(PackedConfig));
    }

    bool resetConfig()
    {
        bool ok = eraseConfig();

        // Restore factory baseline configuration to DPLL controller & hardware peripherals
        dpll::setGeneratorSource(dpll::GENERATOR_SOURCE_DAC);
        dpll::setDdsFreqLimits(20000.0f, 40000.0f);
        dpll::setDdsMaxSlew(20000.0f);
        dpll::setDdsCenterFreq(28000.0f);
        dds::setMcoClock(dds::MCO_CLOCK_HSE_8MHZ);

        dpll::begin(1.65f, 0.000002f, 0.0002f, 0.0f);
        dpll::setTargetPhase(0.0f);
        dpll::setOutputLimits(0.0f, 3.3f);
        dpll::setMaxSlew(30.0f);
        dpll::setLoopPeriodMs(20);
        dpll::setLockThresholdNs(500.0f);
        dpll::setLockHoldCycles(10);
        dpll::setLockMemoryTimeoutMs(5000);
        dpll::setStreamPeriodMs(10);
        dpll::setSignalLossBehavior(dpll::SIGNAL_LOSS_FREEZE);

        dac::setVoltage(1.65f);
        dds::setFrequency(28000.0);
        phase_capture::setRefEdge(phase_capture::CAPTURE_RISING);
        phase_capture::setZcdEdge(phase_capture::CAPTURE_RISING);
        dpll::setTrackingDirection(dpll::TRACKING_DIR_NORMAL);

        return ok;
    }

} // namespace eeprom_config
