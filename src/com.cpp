#include <Arduino.h>
#include "com.h"
#include "CircularBuffer.h"
#include <USBSerial.h>
namespace com
{
    namespace
    {
        uint8_t rxBuffer[COM_PAYLOAD_MAX_SIZE];
        CircularBuffer<ComPacket, 10> RxOpcodeQueue;
        CircularBuffer<ComPacket, 50> TxOpcodeQueue;
        bool AllowSendStream = false;

        uint8_t calculate_sum(uint8_t *bytes, int len)
        {
            uint16_t i;
            uint8_t checksum = 0x00;
            for (i = 0; i < len; i++)
                checksum += bytes[i];
            checksum = (checksum ^ 0xFF); // one's complement (~sum)
            checksum = checksum + 1;      // +1 -> two's complement (-sum)
            return checksum;
        }
        uint8_t calculate_sum_address(uint16_t AddressOffset, uint8_t *bytes, int len)
        {
            uint16_t i;
            uint8_t checksum = 0x00;
            for (i = 0; i < len; i++)
                checksum += bytes[i + AddressOffset];
            checksum = (checksum ^ 0xFF); // one's complement (~sum)
            checksum = checksum + 1;      // +1 -> two's complement (-sum)
            return checksum;
        }
    }
    void begin()
    {
        SerialUSB.begin(115200);
        RxOpcodeQueue.clear();
        TxOpcodeQueue.clear();
    }
    void end()
    {
        SerialUSB.end();
    }
    void receive_command()
    {
        if (SerialUSB.available() >= COM_HEADER_SIZE)
        {
            ComPacket packet;
            // Read the header
            SerialUSB.readBytes((uint8_t *)&packet.header, COM_HEADER_SIZE);
            // Validate header
            if (packet.header.startByte != START_BYTE || packet.header.endByte != END_BYTE)
            {
                return;
            }
            // Read the payload and checksum
            uint16_t payloadLength = packet.header.payloadLength - 1; // length includes checksum byte
            if (payloadLength > COM_PAYLOAD_MAX_SIZE)
            {
                return;
            }
            SerialUSB.readBytes(packet.payload, payloadLength + 1); // read payload + checksum
            // Validate checksum
            uint8_t calculatedChecksum = calculate_sum(packet.payload, payloadLength);
            if (calculatedChecksum != packet.payload[payloadLength])
            {
                return;
            }
            // Push the packet onto the RX queue
            if (!RxOpcodeQueue.push(packet))
            {
                return;
            }
        }
    }
    
    bool sendPacket(uint16_t opcode, uint16_t address, const uint8_t *payload, uint16_t payloadLength)
    {
        ComPacket packet;
        uint16_t idx = 0;

        // --- Header (8 bytes) ---
        packet.header.startByte = START_BYTE; // [0] start byte
        packet.header.opcode = opcode;         // [1..2] opcode (uint16 LE)
        packet.header.address = address;       // [3..4] address (uint16 LE)
        packet.header.payloadLength = payloadLength + 1; // [5..6] length = payloadLength + 1 (the +1 counts the checksum byte).
        packet.header.endByte = END_BYTE;     // [7] end byte

        //copy payload
        if (payload != nullptr && payloadLength > 0 && payloadLength <= COM_PAYLOAD_MAX_SIZE)
        {
            memcpy(&packet.payload[idx], payload, payloadLength);
            idx += payloadLength;
        }
        else
        {
            return false; // No payload to send
        }
        // --- Payload + checksum ---
        uint8_t checksum = 0;
        checksum = calculate_sum(&packet.payload[0], payloadLength); // compute checksum of payload
        packet.payload[idx++] = checksum; // checksum byte
        if (TxOpcodeQueue.isFull())
        {
            return false; // Queue is full, cannot send
        }
        // Push the packet onto the TX queue
        if (!TxOpcodeQueue.push(packet))
        {
            return false; // Failed to push packet onto queue
        }

        return true;
    }
    Opcode getAvailableRxPackets(ComPacket *packets)
    {
        if (RxOpcodeQueue.isEmpty())
        {
            return ILEGAL_OPCODE; // No packets available
        }
        ComPacket packet;
        if (!RxOpcodeQueue.pop(packet))
        {
            return ILEGAL_OPCODE; // Failed to pop packet
        }
        return static_cast<Opcode>(packet.header.opcode);
    }
    void SetAllowSendStream(bool allow)
    {
        AllowSendStream = allow;
    }
    bool GetAllowSendStream()
    {
        return AllowSendStream;
    }
}