#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>
#include <USBSerial.h>
#include "com.h"
#include "dma_buffer_pool.h"
#include "pwm_dither_plan.h"
// Exercise the real estimator and ISR sample writer, with fake timer registers.
#include "../../src/phase_capture.cpp"

#define CHECK(expression) do { if (!(expression)) { \
    std::fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #expression); \
    std::abort(); } } while (false)

FakeUsbSerial SerialUSB;
uint32_t nowMs = 1000;
uint32_t millis() { return nowMs; }

std::vector<uint8_t> frame(const std::vector<uint8_t>& data, uint16_t opcode = 0x1234) {
    const uint16_t length = static_cast<uint16_t>(data.size() + 1);
    std::vector<uint8_t> bytes = {START_BYTE, uint8_t(opcode), uint8_t(opcode >> 8),
        0, 0, uint8_t(length), uint8_t(length >> 8), END_BYTE};
    uint8_t sum = 0;
    for (auto byte : data) { bytes.push_back(byte); sum += byte; }
    bytes.push_back(static_cast<uint8_t>(0U - sum));
    return bytes;
}

void resetCom() { SerialUSB = {}; com::begin(); }
void feed(const std::vector<uint8_t>& bytes) {
    SerialUSB.rx.insert(SerialUSB.rx.end(), bytes.begin(), bytes.end());
    while (!SerialUSB.rx.empty()) {
        const auto before = SerialUSB.rx.size();
        com::receive_command();
        CHECK(before - SerialUSB.rx.size() <= 128);
    }
}
ComPacket take(uint16_t opcode) {
    ComPacket packet{};
    CHECK(static_cast<uint16_t>(com::getAvailableRxPackets(&packet)) == opcode);
    return packet;
}
void emptyRx() { CHECK(com::getAvailableRxPackets(nullptr) == ILEGAL_OPCODE); }

void testCom() {
    resetCom();
    const auto packet = frame({1, 2, 3, 4});
    for (auto byte : packet) { feed({byte}); ++nowMs; }
    CHECK(take(0x1234).header.payloadLength == 5);
    emptyRx();

    resetCom();
    feed(std::vector<uint8_t>(packet.begin(), packet.begin() + 9));
    for (int i = 0; i < 20; ++i) com::receive_command();
    emptyRx();
    nowMs += 101;
    feed(packet);
    take(0x1234);

    resetCom();
    feed({0x55, 0x22, START_BYTE, 0, 0, 0, 0, 0, 0, END_BYTE}); // zero length
    feed({START_BYTE, 0, 0, 0, 0, 1, 2, END_BYTE}); // 513 bytes
    feed(packet);
    take(0x1234);
    emptyRx();

    resetCom();
    auto damaged = packet;
    damaged.back() ^= 1;
    feed(damaged);
    emptyRx();
    feed(packet);
    take(0x1234);

    resetCom();
    std::vector<uint8_t> maximum(COM_DATA_MAX_SIZE);
    for (size_t i = 0; i < maximum.size(); ++i) maximum[i] = uint8_t(i);
    feed(frame(maximum));
    CHECK(take(0x1234).header.payloadLength == COM_PAYLOAD_MAX_SIZE);
    emptyRx();
    CHECK(!com::sendPacket(0x1234, 0, maximum.data(), COM_PAYLOAD_MAX_SIZE));
    CHECK(com::sendPacket(0x1234, 0, maximum.data(), maximum.size()));
    com::FlushTxQueue(); // no USB space: return without consuming data
    CHECK(SerialUSB.tx.empty());
    SerialUSB.writeLimit = 7; // exercise partial writes and persistent tails
    for (int i = 0; i < 200 && SerialUSB.tx.size() < 520; ++i) {
        SerialUSB.capacity = (i % 3 == 0) ? 0 : 17;
        com::FlushTxQueue();
    }
    CHECK(SerialUSB.tx == frame(maximum));

    resetCom();
    std::mt19937 random(12);
    for (int i = 0; i < 1000; ++i) {
        std::vector<uint8_t> data(1 + random() % COM_DATA_MAX_SIZE);
        for (auto& byte : data) byte = uint8_t(random());
        const auto bytes = frame(data);
        size_t offset = 0;
        while (offset < bytes.size()) {
            const size_t chunk = std::min<size_t>(1 + random() % 31, bytes.size() - offset);
            feed(std::vector<uint8_t>(bytes.begin() + offset, bytes.begin() + offset + chunk));
            offset += chunk;
        }
        const auto received = take(0x1234);
        CHECK(memcmp(data.data(), received.payload, data.size()) == 0);
    }
    std::puts("PASS: fragmented/max-size/malformed USB frames and TX backpressure");
}

void testBufferOwnership() {
    DmaBufferPool pool;
    pool.reset();
    uint8_t targets[2] = {0, 1};
    uint32_t contents[4] = {};
    uint8_t cpu = DmaBufferPool::None;
    uint32_t cpuValue = 0;
    std::mt19937 random(42);
    for (uint32_t sequence = 1; sequence <= 200000; ++sequence) {
        const uint8_t slot = sequence & 1U;
        CHECK(targets[slot] != cpu);
        contents[targets[slot]] = sequence;
        targets[slot] = pool.complete(targets[slot]);
        CHECK(targets[0] != targets[1]);
        CHECK(targets[0] != cpu && targets[1] != cpu);
        if (cpu != DmaBufferPool::None) CHECK(contents[cpu] == cpuValue);
        if (random() % 7 == 0) {
            cpu = pool.acquire();
            CHECK(cpu != DmaBufferPool::None);
            cpuValue = contents[cpu];
            CHECK(cpuValue == sequence); // always newest, including long consumer stalls
            CHECK(cpu != targets[0] && cpu != targets[1]);
            CHECK(pool.acquire() == DmaBufferPool::None);
        }
    }
    CHECK(pool.dropped() > 0);
    std::puts("PASS: DMA ownership under 200000 completions and delayed consumers");
}

void testFrequencyPlans() {
    using namespace dma_sqw;
    DitherPlan plan{};
    for (float frequency = MIN_FREQUENCY_HZ; frequency <= MAX_FREQUENCY_HZ; frequency += 7.25f) {
        CHECK(makeDitherPlan(frequency, plan));
        uint32_t ticks = 0;
        for (auto arr : plan.arr) {
            ticks += uint32_t(arr) + 1;
            CHECK(plan.base / 2U < uint32_t(arr) + 1);
        }
        CHECK(ticks == plan.totalTicks);
        const double actual = double(F_TIM_HZ) * N_BUF / (double(plan.divider) * ticks);
        const double step = double(frequency) * frequency * plan.divider / (double(F_TIM_HZ) * N_BUF);
        CHECK(std::fabs(actual - frequency) <= step * 0.51 + 0.01);
    }
    for (float frequency : {1000.0f, 2000.0f, 2500.0f, 2563.4765f, 28000.0f, 40000.0f, 100000.0f}) {
        CHECK(makeDitherPlan(frequency, plan));
        CHECK(std::fabs(plan.frequency - frequency) < 1.0f);
    }
    CHECK(!makeDitherPlan(999.0f, plan));
    CHECK(!makeDitherPlan(100001.0f, plan));
    CHECK(!makeDitherPlan(std::numeric_limits<float>::quiet_NaN(), plan));
    CHECK(!makeDitherPlan(std::numeric_limits<float>::infinity(), plan));
    std::puts("PASS: PWM periods, duty and resolution across the full 1-100 kHz range");
}

void testPhaseCapture() {
    using namespace phase_capture;
    begin();
    auto collect = [](uint32_t count, int32_t delay, uint32_t first = 10000) {
        s_refValid = true;
        s_periodTicks = 3000;
        for (uint32_t i = 0; i < count; ++i) {
            s_lastRefCapture = first + i * 3000;
            storePhaseSampleFromIsr(s_lastRefCapture + delay);
        }
    };
    collect(600, 30);
    auto window = consumeWindowData();
    CHECK(window.trustworthy && std::fabs(window.phaseDiffNs - 357.14285f) < 1);
    clearWindowData();
    resetFilterState();
    collect(3000, 300);
    collect(2000, 60, 9010000);
    window = consumeWindowData();
    CHECK(window.overflow && window.sampleCount == 1536 && window.trustworthy);
    CHECK(std::fabs(window.phaseDiffNs - 714.2857f) < 1); // latest tail, not the stale prefix

    clearWindowData();
    s_periodTicks = 3000;
    s_lastRefCapture = 0;
    storePhaseSampleFromIsr(8400000); // ZCD persists 100ms after REF stops
    CHECK(s_bankCount[s_activeBank] == 0 && !s_haveSynchronizedDiff);
    s_lastRefCapture = 0xffffff00U;
    storePhaseSampleFromIsr(s_lastRefCapture + 300U); // rollover
    CHECK(s_bankCount[s_activeBank] == 1 && s_lastSynchronizedDiff == 300);
    setEnabled(false);
    CHECK(!s_timer->running && !getData().valid);
    setEnabled(true);
    CHECK(s_timer->running && !getData().valid);
    std::puts("PASS: ZCD filtering, long-window freshness, REF loss and timer rollover");
}

int main() {
    testCom();
    testBufferOwnership();
    testFrequencyPlans();
    testPhaseCapture();
    std::puts("All native regression checks passed.");
}
