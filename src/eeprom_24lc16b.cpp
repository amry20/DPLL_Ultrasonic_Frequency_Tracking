/*
 * eeprom_24lc16b.cpp
 *
 * Implementation of Microchip 24LC16B I2C EEPROM driver.
 */

#include "eeprom_24lc16b.h"

namespace eeprom
{
    static TwoWire *_wire = nullptr;

    static inline uint8_t calcDevAddr(uint16_t memAddr)
    {
        // 24LC16B: Upper 3 bits (bits 10..8) of the 11-bit address select the 256-byte block (0..7)
        return I2C_BASE_ADDR | ((memAddr >> 8) & 0x07);
    }

    static bool waitForWriteComplete(uint8_t devAddr)
    {
        if (!_wire)
        {
            return false;
        }

        uint32_t start = millis();
        while ((millis() - start) <= WRITE_TIMEOUT_MS)
        {
            _wire->beginTransmission(devAddr);
            if (_wire->endTransmission() == 0)
            {
                return true; // ACK received, internal write cycle finished
            }
            delayMicroseconds(200);
        }
        return false; // Timed out waiting for EEPROM write cycle
    }

    bool begin(uint32_t sclPin, uint32_t sdaPin, uint32_t clockSpeed, TwoWire *wireInstance)
    {
        _wire = (wireInstance != nullptr) ? wireInstance : &Wire;

        _wire->setSCL(sclPin);
        _wire->setSDA(sdaPin);
        _wire->begin();
        _wire->setClock(clockSpeed);

        return isConnected();
    }

    bool isConnected()
    {
        if (!_wire)
        {
            return false;
        }

        // Ping base block 0 (address 0x50)
        _wire->beginTransmission(I2C_BASE_ADDR);
        return (_wire->endTransmission() == 0);
    }

    uint8_t readByte(uint16_t memAddr, bool *ok)
    {
        if (ok)
        {
            *ok = false;
        }

        if (!_wire || memAddr >= CAPACITY_BYTES)
        {
            return 0xFF;
        }

        uint8_t devAddr = calcDevAddr(memAddr);
        uint8_t wordAddr = (uint8_t)(memAddr & 0xFF);

        _wire->beginTransmission(devAddr);
        _wire->write(wordAddr);
        if (_wire->endTransmission(false) != 0)
        {
            return 0xFF;
        }

        uint8_t count = _wire->requestFrom(devAddr, (uint8_t)1);
        if (count != 1 || _wire->available() < 1)
        {
            return 0xFF;
        }

        uint8_t val = _wire->read();
        if (ok)
        {
            *ok = true;
        }
        return val;
    }

    bool writeByte(uint16_t memAddr, uint8_t data)
    {
        if (!_wire || memAddr >= CAPACITY_BYTES)
        {
            return false;
        }

        uint8_t devAddr = calcDevAddr(memAddr);
        uint8_t wordAddr = (uint8_t)(memAddr & 0xFF);

        _wire->beginTransmission(devAddr);
        _wire->write(wordAddr);
        _wire->write(data);
        if (_wire->endTransmission() != 0)
        {
            return false;
        }

        return waitForWriteComplete(devAddr);
    }

    bool readBuffer(uint16_t memAddr, uint8_t *dest, size_t length)
    {
        if (!_wire || dest == nullptr || length == 0)
        {
            return false;
        }

        if ((uint32_t)memAddr + length > CAPACITY_BYTES)
        {
            return false;
        }

        size_t remaining = length;
        uint8_t *p = dest;

        while (remaining > 0)
        {
            // Chunk reads up to 16 bytes and do not cross 256-byte block boundary without re-addressing
            size_t bytesInBlock = BLOCK_SIZE - (memAddr % BLOCK_SIZE);
            size_t chunk = (remaining < 16) ? remaining : 16;
            if (chunk > bytesInBlock)
            {
                chunk = bytesInBlock;
            }

            uint8_t devAddr = calcDevAddr(memAddr);
            uint8_t wordAddr = (uint8_t)(memAddr & 0xFF);

            _wire->beginTransmission(devAddr);
            _wire->write(wordAddr);
            if (_wire->endTransmission(false) != 0)
            {
                return false;
            }

            uint8_t count = _wire->requestFrom(devAddr, (uint8_t)chunk);
            if (count != chunk)
            {
                return false;
            }

            for (size_t i = 0; i < chunk; ++i)
            {
                if (_wire->available() < 1)
                {
                    return false;
                }
                *p++ = _wire->read();
            }

            memAddr += chunk;
            remaining -= chunk;
        }

        return true;
    }

    bool writeBuffer(uint16_t memAddr, const uint8_t *src, size_t length)
    {
        if (!_wire || src == nullptr || length == 0)
        {
            return false;
        }

        if ((uint32_t)memAddr + length > CAPACITY_BYTES)
        {
            return false;
        }

        size_t remaining = length;
        const uint8_t *p = src;

        while (remaining > 0)
        {
            // Must not cross 16-byte page boundary in a single I2C transmission
            uint8_t pageOffset = memAddr % PAGE_SIZE;
            size_t bytesInPage = PAGE_SIZE - pageOffset;
            size_t chunk = (remaining < bytesInPage) ? remaining : bytesInPage;

            uint8_t devAddr = calcDevAddr(memAddr);
            uint8_t wordAddr = (uint8_t)(memAddr & 0xFF);

            _wire->beginTransmission(devAddr);
            _wire->write(wordAddr);
            _wire->write(p, chunk);
            if (_wire->endTransmission() != 0)
            {
                return false;
            }

            if (!waitForWriteComplete(devAddr))
            {
                return false;
            }

            p += chunk;
            memAddr += chunk;
            remaining -= chunk;
        }

        return true;
    }

    bool fill(uint16_t memAddr, uint8_t val, size_t length)
    {
        if (!_wire || length == 0)
        {
            return false;
        }

        if ((uint32_t)memAddr + length > CAPACITY_BYTES)
        {
            return false;
        }

        uint8_t temp[PAGE_SIZE];
        memset(temp, val, sizeof(temp));

        size_t remaining = length;
        while (remaining > 0)
        {
            uint8_t pageOffset = memAddr % PAGE_SIZE;
            size_t bytesInPage = PAGE_SIZE - pageOffset;
            size_t chunk = (remaining < bytesInPage) ? remaining : bytesInPage;

            if (!writeBuffer(memAddr, temp, chunk))
            {
                return false;
            }

            memAddr += chunk;
            remaining -= chunk;
        }

        return true;
    }

    bool selfTest(uint16_t testAddr)
    {
        if (testAddr + PAGE_SIZE > CAPACITY_BYTES)
        {
            testAddr = CAPACITY_BYTES - PAGE_SIZE;
        }

        uint8_t orig[PAGE_SIZE];
        uint8_t readback[PAGE_SIZE];
        uint8_t pattern1[PAGE_SIZE];
        uint8_t pattern2[PAGE_SIZE];

        // 1. Read and backup original data
        if (!readBuffer(testAddr, orig, PAGE_SIZE))
        {
            return false;
        }

        // Generate patterns
        for (uint8_t i = 0; i < PAGE_SIZE; ++i)
        {
            pattern1[i] = (uint8_t)(0x5A ^ i);
            pattern2[i] = (uint8_t)(~pattern1[i]);
        }

        // 2. Write pattern 1 and verify
        if (!writeBuffer(testAddr, pattern1, PAGE_SIZE))
        {
            writeBuffer(testAddr, orig, PAGE_SIZE); // attempt restore
            return false;
        }

        if (!readBuffer(testAddr, readback, PAGE_SIZE) || memcmp(pattern1, readback, PAGE_SIZE) != 0)
        {
            writeBuffer(testAddr, orig, PAGE_SIZE); // attempt restore
            return false;
        }

        // 3. Write pattern 2 and verify
        if (!writeBuffer(testAddr, pattern2, PAGE_SIZE))
        {
            writeBuffer(testAddr, orig, PAGE_SIZE); // attempt restore
            return false;
        }

        if (!readBuffer(testAddr, readback, PAGE_SIZE) || memcmp(pattern2, readback, PAGE_SIZE) != 0)
        {
            writeBuffer(testAddr, orig, PAGE_SIZE); // attempt restore
            return false;
        }

        // 4. Restore original data
        if (!writeBuffer(testAddr, orig, PAGE_SIZE))
        {
            return false;
        }

        // 5. Verify restored data
        if (!readBuffer(testAddr, readback, PAGE_SIZE) || memcmp(orig, readback, PAGE_SIZE) != 0)
        {
            return false;
        }

        return true;
    }

} // namespace eeprom
