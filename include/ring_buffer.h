// ring_buffer.h - Templatized fixed-size ring buffer
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <Arduino.h>

template<typename T, uint8_t Capacity>
class RingBuffer {
public:
    RingBuffer() : m_head(0), m_tail(0), m_count(0) {}

    bool enqueue(const T& item) {
        if (isFull()) {
            return false;
        }
        m_buffer[m_head] = item;
        m_head = (m_head + 1) % Capacity;
        m_count++;
        return true;
    }

    bool dequeue(T& item) {
        if (isEmpty()) {
            return false;
        }
        item = m_buffer[m_tail];
        m_tail = (m_tail + 1) % Capacity;
        m_count--;
        return true;
    }

    bool isFull() const { return m_count == Capacity; }
    bool isEmpty() const { return m_count == 0; }
    uint8_t size() const { return m_count; }

private:
    T m_buffer[Capacity];
    uint8_t m_head;
    uint8_t m_tail;
    uint8_t m_count;
};

#endif // RING_BUFFER_H
