#include <Arduino.h>
#include "com.h"
#include "circular_buffer.h"
#include <USBSerial.h>
#include <algorithm>

namespace com {
namespace {
CircularBuffer<ComPacket, 32> RxOpcodeQueue;
CircularBuffer<ComPacket, 50> TxOpcodeQueue;
bool AllowSendStream = false;

ComPacket s_rxPacket{};
uint16_t s_headerBytes = 0;
uint16_t s_payloadBytes = 0;
uint32_t s_lastRxMs = 0;
constexpr uint32_t kRxTimeoutMs = 100;
constexpr size_t kIoByteBudget = 128;

uint8_t s_txBytes[COM_HEADER_SIZE + COM_PAYLOAD_MAX_SIZE];
size_t s_txLength = 0;
size_t s_txOffset = 0;

uint8_t calculate_sum(const uint8_t* bytes, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) sum += bytes[i];
    return static_cast<uint8_t>(0U - sum);
}

void resetRx() { s_headerBytes = s_payloadBytes = 0; }

void receiveByte(uint8_t byte) {
    if (s_headerBytes < COM_HEADER_SIZE) {
        if (s_headerBytes == 0 && byte != START_BYTE) return;
        auto* header = reinterpret_cast<uint8_t*>(&s_rxPacket.header);
        header[s_headerBytes++] = byte;
        if (s_headerBytes != COM_HEADER_SIZE) return;

        const uint16_t length = s_rxPacket.header.payloadLength;
        if (s_rxPacket.header.endByte != END_BYTE || length == 0 ||
            length > COM_PAYLOAD_MAX_SIZE) {
            // Preserve a possible header start inside the rejected header.
            size_t next = 1;
            while (next < COM_HEADER_SIZE && header[next] != START_BYTE) ++next;
            s_headerBytes = static_cast<uint16_t>(COM_HEADER_SIZE - next);
            memmove(header, header + next, s_headerBytes);
        }
        return;
    }

    s_rxPacket.payload[s_payloadBytes++] = byte;
    if (s_payloadBytes == s_rxPacket.header.payloadLength) {
        const size_t dataLength = s_payloadBytes - 1U;
        if (calculate_sum(s_rxPacket.payload, dataLength) == s_rxPacket.payload[dataLength]) {
            RxOpcodeQueue.push(s_rxPacket);
        }
        resetRx();
    }
}
} // namespace

void begin() {
    SerialUSB.begin(115200);
    RxOpcodeQueue.clear();
    TxOpcodeQueue.clear();
    resetRx();
    s_txLength = s_txOffset = 0;
}

void end() { SerialUSB.end(); }

void receive_command() {
    const uint32_t now = millis();
    if (s_headerBytes != 0 && now - s_lastRxMs > kRxTimeoutMs) resetRx();
    // Never wait for bytes or drain an unlimited host burst in one loop pass.
    for (size_t count = 0; count < kIoByteBudget && SerialUSB.available() > 0; ++count) {
        const int byte = SerialUSB.read();
        if (byte < 0) break;
        s_lastRxMs = now;
        receiveByte(static_cast<uint8_t>(byte));
    }
}

bool sendPacket(uint16_t opcode, uint16_t address, const uint8_t* payload, uint16_t payloadLength) {
    if (payload == nullptr || payloadLength == 0 ||
        payloadLength > COM_DATA_MAX_SIZE || TxOpcodeQueue.isFull()) return false;
    ComPacket packet{};
    packet.header = {START_BYTE, opcode, address,
                     static_cast<uint16_t>(payloadLength + 1U), END_BYTE};
    memcpy(packet.payload, payload, payloadLength);
    packet.payload[payloadLength] = calculate_sum(payload, payloadLength);
    return TxOpcodeQueue.push(packet);
}

Opcode getAvailableRxPackets(ComPacket* packets) {
    ComPacket packet;
    if (!RxOpcodeQueue.pop(packet)) return ILEGAL_OPCODE;
    if (packets != nullptr) *packets = packet;
    return static_cast<Opcode>(packet.header.opcode);
}

void SetAllowSendStream(bool allow) { AllowSendStream = allow; }
bool GetAllowSendStream() { return AllowSendStream; }

void FlushTxQueue() {
    if (s_txOffset == s_txLength) {
        ComPacket packet;
        if (!TxOpcodeQueue.pop(packet)) return;
        s_txLength = COM_HEADER_SIZE + packet.header.payloadLength;
        s_txOffset = 0;
        memcpy(s_txBytes, &packet.header, COM_HEADER_SIZE);
        memcpy(s_txBytes + COM_HEADER_SIZE, packet.payload, packet.header.payloadLength);
    }
    // USBSerial::write waits when its ring is full. Limit each write to space
    // already available and keep the unsent tail for the next loop iteration.
    const int available = SerialUSB.availableForWrite();
    if (available <= 0) return;
    const size_t count = std::min({s_txLength - s_txOffset,
                                   static_cast<size_t>(available), kIoByteBudget});
    s_txOffset += SerialUSB.write(s_txBytes + s_txOffset, count);
}
} // namespace com
