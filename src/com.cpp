#include <Arduino.h>
#include "com.h"
#include "circular_buffer.h"
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
        // Read one byte at a time so we can resync cleanly. SerialUSB carries
        // ONLY binary frames (the ASCII debug protocol runs on the separate
        // DebugPort), but a host might still send stray bytes on connect or
        // partial/corrupt frames, so we discard bytes until we hit the 0xAA
        // start marker, then validate the rest of the header, payload length
        // and checksum. This byte-wise scan is also robust if the host and
        // this code ever disagree about frame boundaries.
        while (SerialUSB.available() > 0)
        {
            uint8_t first = SerialUSB.read();
            if (first != START_BYTE)
            {
                // Not a frame start — discard and keep scanning.
                continue;
            }

            // Found the start byte; need 7 more bytes for the full header.
            if (SerialUSB.available() < 7)
            {
                // Not enough data yet. The 0xAA is consumed; if the rest of a
                // real packet follows it will be misaligned, but the host
                // resyncs on its side (it scans for 0xAA) and retransmits on
                // refresh, so this is acceptable for the shared port.
                break;
            }

            ComPacket packet;
            packet.header.startByte = first;
            SerialUSB.readBytes(((uint8_t *)&packet.header) + 1, 7);

            if (packet.header.endByte != END_BYTE)
            {
                // Bad frame — rescan from the next byte.
                continue;
            }

            uint16_t payloadLength = packet.header.payloadLength - 1; // length includes checksum byte
            if (payloadLength > COM_PAYLOAD_MAX_SIZE)
            {
                continue;
            }

            SerialUSB.readBytes(packet.payload, payloadLength + 1); // read payload + checksum

            // Validate checksum
            uint8_t calculatedChecksum = calculate_sum(packet.payload, payloadLength);
            if (calculatedChecksum != packet.payload[payloadLength])
            {
                continue;
            }

            // Push the packet onto the RX queue
            RxOpcodeQueue.push(packet);
            // Keep draining in case more packets are already buffered.
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
        // IMPORTANT: copy the popped packet to the caller's buffer — the old
        // code returned the opcode but never filled *packets, so the caller
        // always saw stale/zero data.
        if (packets != nullptr)
        {
            *packets = packet;
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
    void FlushTxQueue()
    {
        if (TxOpcodeQueue.isEmpty())
        {
            return; // No packets to send
        }
        ComPacket packet;
        TxOpcodeQueue.pop(packet);
        // Send the packet over SerialUSB
        uint8_t Buffer[COM_HEADER_SIZE + COM_PAYLOAD_MAX_SIZE];
        memcpy(Buffer, &packet.header, COM_HEADER_SIZE);
        memcpy(Buffer + COM_HEADER_SIZE, packet.payload, packet.header.payloadLength);
        SerialUSB.write(Buffer, COM_HEADER_SIZE + packet.header.payloadLength);
    }
}