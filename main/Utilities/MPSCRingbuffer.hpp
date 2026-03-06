#pragma once

#include <atomic>
#include <cstddef>
#include <new>

namespace bldc {
    template <typename T, size_t E>
    class MPSCRingbuffer {
        public:
            static constexpr size_t kBufferSize = 1 << E;

            size_t size() const { return kBufferSize; }

            bool empty() const;
            bool full() const;
            size_t remaining() const;

            void clear();

            void push(const T& value);
            template <typename... Args>
            const T* emplace(Args&&... args);
            const T* peek() const;
            void pop(); 

        private:
            static constexpr size_t kIndexMask = kBufferSize - 1;
            static_assert((kBufferSize & kIndexMask) == 0, "Ringbuffer size must be a power of 2");

            struct Entry {
                T data;
                std::atomic<bool> ready = false;
            };

            Entry _buffer[kBufferSize];
            std::atomic<size_t> _writeIndex = 0;
            std::atomic<size_t> _readIndex = 0;
    };

    // Implementation

    template <typename T, size_t E>
    bool MPSCRingbuffer<T, E>::empty() const {
        return _writeIndex.load(std::memory_order_acquire) - _readIndex.load(std::memory_order_relaxed) == 0;
    }

    template <typename T, size_t E>
    bool MPSCRingbuffer<T, E>::full() const {
        return _writeIndex.load(std::memory_order_relaxed) - _readIndex.load(std::memory_order_acquire) >= kBufferSize;
    }

    template <typename T, size_t E>
    size_t MPSCRingbuffer<T, E>::remaining() const {
        return _writeIndex.load(std::memory_order_relaxed) - _readIndex.load(std::memory_order_relaxed);
    }

    template <typename T, size_t E>
    void MPSCRingbuffer<T, E>::clear() {
        _readIndex.store(0, std::memory_order_release);
        _writeIndex.store(0, std::memory_order_release);

        for (Entry& entry : _buffer) {
            entry.ready.store(false, std::memory_order_relaxed);
        }
    }

    template <typename T, size_t E>
    void MPSCRingbuffer<T, E>::push(const T& value) {
        if (full()) {
            return;
        }

        const size_t slot = _writeIndex.fetch_add(1, std::memory_order_release) & kIndexMask;
        _buffer[slot].data = value;
        _buffer[slot].ready.store(true, std::memory_order_release);
    }

    template <typename T, size_t E>
    template <typename... Args>
    const T* MPSCRingbuffer<T, E>::emplace(Args&&... args) {
        if (full()) {
            return nullptr;
        }

        const size_t slot = _writeIndex.fetch_add(1, std::memory_order_release) & kIndexMask;
        T* result = new (&_buffer[slot].data) T(std::forward<Args>(args)...);
        _buffer[slot].ready.store(true, std::memory_order_release);

        return result;
    }

    template <typename T, size_t E>
    const T* MPSCRingbuffer<T, E>::peek() const {
        if (empty()) {
            return nullptr;
        }

        const size_t slot = _readIndex.load(std::memory_order_relaxed) & kIndexMask;
        if (!_buffer[slot].ready.load(std::memory_order_acquire)) {
            return nullptr;
        }

        return &_buffer[slot].data;
    }

    template <typename T, size_t E>
    void MPSCRingbuffer<T, E>::pop() {
        if (empty()) {
            return;
        }

        const size_t slot = _readIndex.load(std::memory_order_relaxed) & kIndexMask;
        if (!_buffer[slot].ready.load(std::memory_order_acquire)) {
            return;
        }

        _buffer[slot].ready.store(false, std::memory_order_relaxed);

        _readIndex.fetch_add(1, std::memory_order_release);
    }
}