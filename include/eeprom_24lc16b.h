/*
 * eeprom_24lc16b.h
 *
 * Driver for Microchip 24LC16B (16 Kbit / 2048 Byte) I2C EEPROM.
 * Connected to STM32F407:
 *   - SCL : PB6
 *   - SDA : PB7
 *
 * Architecture of 24LC16B:
 *   - Total capacity: 2048 bytes (2 KB) organized as 8 blocks of 256 bytes.
 *   - Page write buffer: 16 bytes.
 *   - Addressing: 7-bit base address 0x50. The 3 block select bits (B2, B1, B0)
 *     are packed into the lower 3 bits of the I2C device address (0x50 - 0x57),
 *     followed by an 8-bit word address within that block.
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <stddef.h>

namespace eeprom
{
    // 24LC16B Physical Constants
    static constexpr uint16_t CAPACITY_BYTES   = 2048; // 16 Kbit = 2048 bytes
    static constexpr uint8_t  PAGE_SIZE        = 16;   // 16-byte write page buffer
    static constexpr uint16_t BLOCK_SIZE       = 256;  // 256 bytes per block (8 blocks total)
    static constexpr uint8_t  NUM_BLOCKS       = 8;    // 8 blocks
    static constexpr uint8_t  I2C_BASE_ADDR    = 0x50; // Base 7-bit I2C address (0x50 - 0x57)
    static constexpr uint32_t WRITE_TIMEOUT_MS = 10;   // Max write cycle timeout (t_WC datasheet max 5ms)

    /**
     * @brief Initialize the I2C bus and the 24LC16B EEPROM driver.
     * @param sclPin SCL pin (default PB6)
     * @param sdaPin SDA pin (default PB7)
     * @param clockSpeed I2C clock frequency in Hz (default 400000 = 400 kHz)
     * @param wireInstance Pointer to TwoWire instance (default &Wire)
     * @return true if initialized and EEPROM acknowledges on the bus, false otherwise.
     */
    bool begin(uint32_t sclPin = PB6, uint32_t sdaPin = PB7, uint32_t clockSpeed = 400000, TwoWire *wireInstance = &Wire);

    /**
     * @brief Check whether the EEPROM is connected and acknowledging on the I2C bus.
     * @return true if device responds with ACK, false otherwise.
     */
    bool isConnected();

    /**
     * @brief Read a single byte from the specified memory address.
     * @param memAddr Memory address (0 - 2047)
     * @param ok Optional pointer to bool flag indicating success/failure
     * @return Byte read from EEPROM (or 0xFF if error)
     */
    uint8_t readByte(uint16_t memAddr, bool *ok = nullptr);

    /**
     * @brief Write a single byte to the specified memory address.
     * @param memAddr Memory address (0 - 2047)
     * @param data Data byte to write
     * @return true on success, false on error
     */
    bool writeByte(uint16_t memAddr, uint8_t data);

    /**
     * @brief Read multiple sequential bytes into a buffer across page/block boundaries.
     * @param memAddr Starting memory address (0 - 2047)
     * @param dest Destination buffer
     * @param length Number of bytes to read
     * @return true on success, false on error
     */
    bool readBuffer(uint16_t memAddr, uint8_t *dest, size_t length);

    /**
     * @brief Write multiple bytes from a buffer, handling 16-byte page boundaries
     *        and hardware write cycle completion automatically.
     * @param memAddr Starting memory address (0 - 2047)
     * @param src Source buffer
     * @param length Number of bytes to write
     * @return true on success, false on error
     */
    bool writeBuffer(uint16_t memAddr, const uint8_t *src, size_t length);

    /**
     * @brief Fill a memory region with a constant byte value.
     * @param memAddr Starting address (0 - 2047)
     * @param val Byte value to fill with (e.g. 0x00 or 0xFF)
     * @param length Number of bytes to fill
     * @return true on success, false on error
     */
    bool fill(uint16_t memAddr, uint8_t val, size_t length);

    /**
     * @brief Perform a non-destructive self-test:
     *        Saves original data, writes test patterns, verifies read-back, and restores data.
     * @param testAddr Starting address to test (default 0x07F0, last page)
     * @return true if test passes, false on communication or verification error.
     */
    bool selfTest(uint16_t testAddr = 0x07F0);

    /**
     * @brief Write any trivial/POD struct or variable into EEPROM.
     */
    template <typename T>
    bool writeObject(uint16_t memAddr, const T &value)
    {
        return writeBuffer(memAddr, reinterpret_cast<const uint8_t *>(&value), sizeof(T));
    }

    /**
     * @brief Read any trivial/POD struct or variable from EEPROM.
     */
    template <typename T>
    bool readObject(uint16_t memAddr, T &value)
    {
        return readBuffer(memAddr, reinterpret_cast<uint8_t *>(&value), sizeof(T));
    }

} // namespace eeprom
