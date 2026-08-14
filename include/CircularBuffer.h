#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H
#include "Arduino.h"
#include <type_traits>
#include <cstring>
template <typename T, size_t Size>
class CircularBuffer {
  public:
    CircularBuffer() : head(0), tail(0), count(0) {}
    size_t head;
    size_t tail;
    size_t count;
    bool push(const T& item) {
      if (isFull()) return false;
      buffer[head] = item;
      head = (head + 1) % Size;
      count++;
      return true;
    }
    
    // Push item to front of buffer (tail position)
    bool pushFront(const T& item) {
      if (isFull()) return false;
      tail = (tail + Size - 1) % Size; // Move tail backwards
      buffer[tail] = item;
      count++;
      return true;
    }
    // Tambahan: Push array
    bool push(const T* items, size_t len) {
        if (Size - count < len) return false; // tidak cukup ruang

        for (size_t i = 0; i < len; ++i) {
            buffer[head] = items[i];
            head = (head + 1) % Size;
            count++;
        }
        return true;
    }
    
    bool pop(T& item) {
      if (isEmpty()) return false;
      item = buffer[tail];
      tail = (tail + 1) % Size;
      count--;
      return true;
    }

    bool peek(T& item) {
      if (isEmpty()) return false;
      item = buffer[tail];
      return true;
    }

    bool isEmpty() const {
      return count == 0;
    }

    bool isFull() const {
      return count == Size;
    }

    size_t available() const {
      return count;
    }
    size_t space_available()
    {
      return Size - count;
    }
    void clear() {
      head = 0;
      tail = 0;
      count = 0;
    }
    bool peekAt(T& item, size_t index) const {
        if (index >= count) return false;
        item = buffer[(tail + index) % Size];
        return true;
    }
    size_t CopyToBuffer(T* dest, size_t max_items) const {
        size_t items_to_copy = (max_items < count) ? max_items : count;
        for (size_t i = 0; i < items_to_copy; i++) {
            dest[i] = buffer[(tail + i) % Size];
        }
        return items_to_copy;
    }
    
    // OPTIMIZED: Bulk pop operation
    size_t popMultiple(size_t num_items) {
        size_t items_to_pop = (num_items < count) ? num_items : count;
        tail = (tail + items_to_pop) % Size;
        count -= items_to_pop;
        return items_to_pop;
    }
    
    // OPTIMIZED: Fast bulk copy without individual indexing
    size_t fastCopyToBuffer(T* dest, size_t max_items) const {
        if (count == 0) return 0;
        
        size_t items_to_copy = (max_items < count) ? max_items : count;
        size_t first_chunk = Size - tail;
        
        if (first_chunk >= items_to_copy) {
            // No wrapping - single memcpy for primitive types
            if constexpr (std::is_trivially_copyable_v<T>) {
                memcpy(dest, &buffer[tail], items_to_copy * sizeof(T));
            } else {
                for (size_t i = 0; i < items_to_copy; i++) {
                    dest[i] = buffer[tail + i];
                }
            }
        } else {
            // Wrapping case - two chunks
            if constexpr (std::is_trivially_copyable_v<T>) {
                memcpy(dest, &buffer[tail], first_chunk * sizeof(T));
                memcpy(&dest[first_chunk], &buffer[0], (items_to_copy - first_chunk) * sizeof(T));
            } else {
                for (size_t i = 0; i < first_chunk; i++) {
                    dest[i] = buffer[tail + i];
                }
                for (size_t i = 0; i < (items_to_copy - first_chunk); i++) {
                    dest[first_chunk + i] = buffer[i];
                }
            }
        }
        return items_to_copy;
    }
  private:
    T buffer[Size];
    
};

#endif
