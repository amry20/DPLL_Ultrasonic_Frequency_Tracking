#pragma once
#include <stdint.h>
#include <stddef.h>
#include <deque>
#include <vector>
#include <algorithm>
#include <cstdlib>
struct FakeUsbSerial {
    std::deque<uint8_t> rx;
    std::vector<uint8_t> tx;
    size_t capacity = 0;
    size_t writeLimit = 10000;
    void begin(int) {}
    void end() {}
    int available() { return static_cast<int>(rx.size()); }
    int read() { if (rx.empty()) return -1; auto b = rx.front(); rx.pop_front(); return b; }
    int availableForWrite() { return static_cast<int>(capacity); }
    size_t write(const uint8_t* bytes, size_t count) {
        if (count > capacity || count > 128) std::abort();
        count = std::min(count, writeLimit);
        tx.insert(tx.end(), bytes, bytes + count);
        capacity -= count;
        return count;
    }
};
extern FakeUsbSerial SerialUSB;
