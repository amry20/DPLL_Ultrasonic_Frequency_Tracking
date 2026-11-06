#pragma once
/*
 * com.h
 *
 * Serial communication for the DPLL project.
 *
 * This module handles the UART interface for status reporting and command input.
 * It is initialized in main.cpp setup() and runs in the main loop.
 *
 * The STM32F407 has multiple USARTs; this project uses USART1 (PA9=TX, PA10=RX).
 * Protocol:
 *   *   - Header 8 byte
 *   *   *   - Byte 0: 0xAA (start byte)
 *   *   *   - Byte 1 - 2: Opcode (uint16_t, little-endian)
 *   *   *   - Byte 3 - 4: Address (uint16_t, little-endian)
 *   *   *   - Byte 5 - 6: Length + 1 check sum (uint16_t, little-endian)
 *   *   *   - Byte 7: End byte (0xBB)
 *   *   - Payload variable length
 */
#include <Arduino.h>
#include "Opcode.h"
#define COM_HEADER_SIZE 8
#define COM_PAYLOAD_MAX_SIZE 512
#define START_BYTE 0xAA
#define END_BYTE 0xBB
typedef struct ComHeader
{
    uint8_t startByte;
    uint16_t opcode;
    uint16_t address;
    uint16_t payloadLength;
    uint8_t endByte;
} __attribute__((packed)) ComHeader;
typedef struct ComPacket
{
    ComHeader header;
    uint8_t payload[COM_PAYLOAD_MAX_SIZE];
} __attribute__((packed)) ComPacket;
typedef struct dpllStatusData
{
    float   ReferenceFrequencyHz;  // 4 bytes — all floats first, naturally aligned
    float   PhaseError_ns;         // 4 bytes (last-valid value when PhaseStale=1)
    float   DACVoltage_V;          // 4 bytes
    float   CurrentMagnitude_mA;   // 4 bytes — Transducer peak current magnitude from Goertzel (mA)
    uint8_t LockStatus;            // 1 byte  (0=NO_REF, 1=WAIT_ZCD, 2=TRACK, 3=LOCK)
    uint8_t PhaseStale;            // 1 byte  (0=fresh measurement, 1=ZCD/Goertzel absent)
    uint8_t EepromStatus;          // 1 byte  (0=Not detected / error, 1=Connected & OK)
    uint8_t DdsStatus;             // 1 byte  (0=Error, 1=Ready/OK, 2=Signal Verified)
}__attribute__((packed)) dpllStatusData;

typedef struct BodeSweepPointPacket
{
    uint8_t  state;               // 0=IDLE/COMPLETED, 1=RUNNING, 2=ABORTED
    uint8_t  valid;               // 1=valid phase captured, 0=no signal
    uint16_t stepIndex;           // 1-based step index
    uint16_t totalSteps;          // total steps in sweep
    uint16_t sampleCount;         // accepted samples in window
    float    targetFreqHz;        // commanded generator frequency (Hz)
    float    measuredFreqHz;      // measured REF frequency (Hz)
    float    phaseDiffNs;         // measured phase difference (ns)
    float    phaseDegrees;        // measured phase difference (deg)
    float    currentMagnitude_mA; // measured peak current amplitude (mA)
} __attribute__((packed)) BodeSweepPointPacket;
namespace com
{
    void begin();
    void receive_command();
    // Build a ComPacket from the given fields and push it onto the TX queue.
    // Returns true if the packet was queued, false if the payload is too large
    // or the queue is full.
    bool sendPacket(uint16_t opcode, uint16_t address,
                    const uint8_t *payload, uint16_t payloadLength);
    Opcode getAvailableRxPackets(ComPacket *packets);
    void SetAllowSendStream(bool allow);
    bool GetAllowSendStream();
    void FlushTxQueue();

}