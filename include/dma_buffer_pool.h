#pragma once
#include <stdint.h>

// Four buffers: two DMA targets, the newest completed block, and a CPU-owned
// block. Call acquire() with the DMA IRQ masked; complete() runs in the ISR.
// A CPU-owned block is never returned to DMA until the next acquire().
class DmaBufferPool {
public:
    static constexpr uint8_t None = 0xff;

    void reset() {
        ready_ = processed_ = None;
        free_[0] = 2;
        free_[1] = 3;
        freeCount_ = 2;
        dropped_ = 0;
    }

    uint8_t complete(uint8_t completed) {
        uint8_t replacement;
        if (ready_ != None) {
            replacement = ready_;
            ++dropped_;
        } else if (freeCount_ != 0) {
            replacement = free_[--freeCount_];
        } else {
            ++dropped_;
            return completed;
        }
        ready_ = completed;
        return replacement;
    }

    uint8_t acquire() {
        if (ready_ == None) return None;
        if (processed_ != None) free_[freeCount_++] = processed_;
        processed_ = ready_;
        ready_ = None;
        return processed_;
    }

    uint8_t processed() const { return processed_; }
    uint32_t dropped() const { return dropped_; }

private:
    volatile uint8_t ready_ = None;
    volatile uint8_t processed_ = None;
    volatile uint8_t free_[2] = {2, 3};
    volatile uint8_t freeCount_ = 2;
    volatile uint32_t dropped_ = 0;
};
