#include <Arduino.h>
#include "com.h"
#include "circular_buffer.h"
#include <USBSerial.h>
namespace com
{
    namespace
    {
        // RX queue sized for host bursts: stream-enable + a 13-opcode GET
        // refresh arrive back-to-back. commandProccessor() drains the whole
        // queue each loop cycle, so a modest capacity is plenty.
        CircularBuffer<ComPacket, 32> RxOpcodeQueue;
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
            // Read the header first
            SerialUSB.readBytes((uint8_t *)&packet.header, COM_HEADER_SIZE);
            // Validate the header
            if (packet.header.startByte != START_BYTE || packet.header.endByte != END_BYTE)
            {
                // Invalid header, discard and return
                return;
            }
            // Read the payload based on the length specified in the header
            uint16_t payloadLength = packet.header.payloadLength - 1; // Exclude checksum byte
            if (payloadLength > COM_PAYLOAD_MAX_SIZE)
            {
                // Payload too large, discard and return
                return;
            }
            // Wait for the full payload and checksum to be available, with a
            // bounded timeout so a truncated frame cannot hang the control loop.
            uint32_t waitStart = millis();
            while (SerialUSB.available() < payloadLength + 1)
            {
                if (millis() - waitStart > 100)
                {
                    return; // timeout — drop this frame and wait for the next frame
                }
            }
            SerialUSB.readBytes(packet.payload, payloadLength + 1); // +1 for checksum byte
            // Validate checksum
            uint8_t calculatedChecksum = calculate_sum(packet.payload, payloadLength);
            if (calculatedChecksum != packet.payload[payloadLength])
            {
                // Checksum mismatch, discard and return
                return;
            }
            // Push the valid packet onto the RX queue
            RxOpcodeQueue.push(packet);
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
        // Drain every queued reply in one shot so the host's paced request-
        // response sequence never stalls waiting for a reply that is stuck
        // behind other packets.  This is safe: SerialUSB.write() enqueues
        // into the USB CDC TX ring (512 B after our build_flag enlargement)
        // and returns immediately; the USB interrupt drains it in the
        // background without touching the DPLL control path.
        ComPacket packet;
        while (!TxOpcodeQueue.isEmpty())
        {
            TxOpcodeQueue.pop(packet);
            uint8_t Buffer[COM_HEADER_SIZE + COM_PAYLOAD_MAX_SIZE];
            memcpy(Buffer, &packet.header, COM_HEADER_SIZE);
            memcpy(Buffer + COM_HEADER_SIZE, packet.payload, packet.header.payloadLength);
            SerialUSB.write(Buffer, COM_HEADER_SIZE + packet.header.payloadLength);
        }
    }
}