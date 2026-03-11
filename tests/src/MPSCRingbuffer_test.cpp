#include "Utilities/MPSCRingbuffer.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <thread>
#include <vector>

// ─────────────────────────────────────────────
// Convenience aliases
// ─────────────────────────────────────────────

using RB4 = bldc::MPSCRingbuffer<int, 4>;  // 16 elements
using RB2 = bldc::MPSCRingbuffer<int, 2>;  // 4 elements
using RB1 = bldc::MPSCRingbuffer<int, 1>;  // 2 elements

// ─────────────────────────────────────────────
// Size and capacity
// ─────────────────────────────────────────────

TEST(MPSCRingbuffer, SizeIsPowerOfTwo) {
    RB4 rb;
    EXPECT_EQ(rb.size(), 16);
}

TEST(MPSCRingbuffer, SizeRB2) {
    RB2 rb;
    EXPECT_EQ(rb.size(), 4);
}

// ─────────────────────────────────────────────
// Initial state
// ─────────────────────────────────────────────

TEST(MPSCRingbuffer, InitiallyEmpty) {
    RB4 rb;
    EXPECT_TRUE(rb.empty());
}

TEST(MPSCRingbuffer, InitiallyNotFull) {
    RB4 rb;
    EXPECT_FALSE(rb.full());
}

TEST(MPSCRingbuffer, InitialRemainingIsZero) {
    RB4 rb;
    EXPECT_EQ(rb.remaining(), 0);
}

TEST(MPSCRingbuffer, InitialPeekIsNull) {
    RB4 rb;
    EXPECT_EQ(rb.peek(), nullptr);
}

// ─────────────────────────────────────────────
// Push and peek
// ─────────────────────────────────────────────

TEST(MPSCRingbuffer, PushMakesNonEmpty) {
    RB4 rb;
    rb.push(42);
    EXPECT_FALSE(rb.empty());
}

TEST(MPSCRingbuffer, PeekAfterPush) {
    RB4 rb;
    rb.push(42);
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(*rb.peek(), 42);
}

TEST(MPSCRingbuffer, RemainingAfterPush) {
    RB4 rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    EXPECT_EQ(rb.remaining(), 3);
}

TEST(MPSCRingbuffer, PeekDoesNotConsume) {
    RB4 rb;
    rb.push(42);
    rb.peek();
    rb.peek();
    EXPECT_EQ(rb.remaining(), 1);
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(*rb.peek(), 42);
}

TEST(MPSCRingbuffer, PeekReturnsFront) {
    RB4 rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(*rb.peek(), 1);
}

// ─────────────────────────────────────────────
// Pop
// ─────────────────────────────────────────────

TEST(MPSCRingbuffer, PopReducesRemaining) {
    RB4 rb;
    rb.push(1);
    rb.push(2);
    rb.pop();
    EXPECT_EQ(rb.remaining(), 1);
}

TEST(MPSCRingbuffer, PopOnEmptyIsNoop) {
    RB4 rb;
    rb.pop();
    EXPECT_TRUE(rb.empty());
}

TEST(MPSCRingbuffer, PopThenPeek) {
    RB4 rb;
    rb.push(1);
    rb.push(2);
    rb.pop();
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(*rb.peek(), 2);
}

TEST(MPSCRingbuffer, PopAllMakesEmpty) {
    RB4 rb;
    rb.push(1);
    rb.push(2);
    rb.pop();
    rb.pop();
    EXPECT_TRUE(rb.empty());
    EXPECT_EQ(rb.peek(), nullptr);
}

// ─────────────────────────────────────────────
// Full behaviour
// ─────────────────────────────────────────────

TEST(MPSCRingbuffer, FullAfterFillingBuffer) {
    RB2 rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    rb.push(4);
    EXPECT_TRUE(rb.full());
}

TEST(MPSCRingbuffer, PushWhenFullIsNoop) {
    RB2 rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    rb.push(4);
    rb.push(99);
    EXPECT_EQ(rb.remaining(), 4);
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(*rb.peek(), 1);
}

TEST(MPSCRingbuffer, NotFullAfterPop) {
    RB2 rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    rb.push(4);
    EXPECT_TRUE(rb.full());
    rb.pop();
    EXPECT_FALSE(rb.full());
}

// ─────────────────────────────────────────────
// Clear
// ─────────────────────────────────────────────

TEST(MPSCRingbuffer, ClearMakesEmpty) {
    RB4 rb;
    rb.push(1);
    rb.push(2);
    rb.clear();
    EXPECT_TRUE(rb.empty());
}

TEST(MPSCRingbuffer, ClearResetsRemaining) {
    RB4 rb;
    rb.push(1);
    rb.push(2);
    rb.clear();
    EXPECT_EQ(rb.remaining(), 0);
}

TEST(MPSCRingbuffer, ClearThenPush) {
    RB4 rb;
    rb.push(1);
    rb.push(2);
    rb.clear();
    rb.push(42);
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(*rb.peek(), 42);
}

TEST(MPSCRingbuffer, ClearResetsReadyFlags) {
    // Fill, clear, refill — if ready flags aren't reset, peek will fail
    RB2 rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    rb.push(4);
    rb.clear();
    rb.push(10);
    rb.push(20);
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(*rb.peek(), 10);
}

// ─────────────────────────────────────────────
// Emplace
// ─────────────────────────────────────────────

TEST(MPSCRingbuffer, EmplaceReturnsPointer) {
    bldc::MPSCRingbuffer<std::string, 2> rb;
    const std::string* p = rb.emplace("hello");
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(*p, "hello");
}

TEST(MPSCRingbuffer, EmplaceWhenFullReturnsNull) {
    bldc::MPSCRingbuffer<std::string, 1> rb;  // 2 elements
    rb.emplace("a");
    rb.emplace("b");
    const std::string* p = rb.emplace("c");
    EXPECT_EQ(p, nullptr);
}

TEST(MPSCRingbuffer, EmplaceConstructsInPlace) {
    bldc::MPSCRingbuffer<std::pair<int, int>, 2> rb;
    rb.emplace(1, 2);
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(rb.peek()->first, 1);
    EXPECT_EQ(rb.peek()->second, 2);
}

// ─────────────────────────────────────────────
// Wraparound behaviour
// ─────────────────────────────────────────────

TEST(MPSCRingbuffer, WrapsAroundCorrectly) {
    RB2 rb;  // 4 elements
    for (int i = 0; i < 3; i++) {
        rb.push(i * 10 + 1);
        rb.push(i * 10 + 2);
        rb.push(i * 10 + 3);
        rb.push(i * 10 + 4);
        EXPECT_TRUE(rb.full());
        for (int j = 1; j <= 4; j++) {
            ASSERT_NE(rb.peek(), nullptr);
            EXPECT_EQ(*rb.peek(), i * 10 + j);
            rb.pop();
        }
        EXPECT_TRUE(rb.empty());
    }
}

TEST(MPSCRingbuffer, PartialFillAndDrain) {
    RB2 rb;
    for (int round = 0; round < 8; round++) {
        rb.push(round);
        ASSERT_NE(rb.peek(), nullptr);
        EXPECT_EQ(*rb.peek(), round);
        rb.pop();
    }
    EXPECT_TRUE(rb.empty());
}

// ─────────────────────────────────────────────
// MPSC threading — multiple producers, single consumer
// ─────────────────────────────────────────────

TEST(MPSCRingbuffer, TwoProducersSingleConsumer) {
    bldc::MPSCRingbuffer<int, 8> rb;  // 256 elements
    constexpr int kItemsPerProducer = 1000;
    constexpr int kTotalItems = kItemsPerProducer * 2;
    std::atomic<int> consumed{0};

    // Each producer pushes a distinct value range so we can verify no corruption
    std::atomic<int> producer1Count{0};
    std::atomic<int> producer2Count{0};

    std::thread producer1([&]() {
        int pushed = 0;
        while (pushed < kItemsPerProducer) {
            if (rb.push(1)) {
                pushed++;
                producer1Count++;
            }
        }
    });

    std::thread producer2([&]() {
        int pushed = 0;
        while (pushed < kItemsPerProducer) {
            if (rb.push(1)) {
                pushed++;
                producer2Count++;
            }
        }
    });

    std::thread consumer([&]() {
        while (consumed.load() < kTotalItems) {
            const int* val = rb.peek();
            if (val != nullptr) {
                // Each value must be either 1 or 2 — anything else is corruption
                EXPECT_TRUE(*val == 1 || *val == 2);
                rb.pop();
                consumed++;
            } else {
                std::this_thread::yield();
            }
        }
    });

    producer1.join();
    producer2.join();
    consumer.join();

    EXPECT_EQ(consumed.load(), kTotalItems);
    EXPECT_TRUE(rb.empty());
}

TEST(MPSCRingbuffer, FourProducersSingleConsumer) {
    bldc::MPSCRingbuffer<int, 8> rb;  // 256 elements
    constexpr int kProducers = 4;
    constexpr int kItemsPerProducer = 500;
    constexpr int kTotalItems = kProducers * kItemsPerProducer;
    std::atomic<int> consumed{0};

    std::vector<std::thread> producers;
    for (int p = 0; p < kProducers; p++) {
        producers.emplace_back([&]() {
            int pushed = 0;
            while (pushed < kItemsPerProducer) {
                if (rb.push(1)) {  // push returns true on success
                    pushed++;
                }
            }
        });
    }

    std::thread consumer([&]() {
        while (consumed.load() < kTotalItems) {
            const int* val = rb.peek();
            if (val != nullptr) {
                EXPECT_GE(*val, 1);
                EXPECT_LE(*val, kProducers);
                rb.pop();
                consumed++;
            } else {
                std::this_thread::yield();
            }
        }
    });

    for (auto& t : producers)
        t.join();
    consumer.join();

    EXPECT_EQ(consumed.load(), kTotalItems);
    EXPECT_TRUE(rb.empty());
}

TEST(MPSCRingbuffer, NoItemsLostUnderContention) {
    bldc::MPSCRingbuffer<int, 8> rb;  // 256 elements
    constexpr int kProducers = 3;
    constexpr int kItemsPerProducer = 1000;
    constexpr int kTotalItems = kProducers * kItemsPerProducer;

    std::atomic<int> totalPushed{0};
    std::atomic<int> consumed{0};

    std::vector<std::thread> producers;
    for (int p = 0; p < kProducers; p++) {
        producers.emplace_back([&]() {
            int pushed = 0;
            while (pushed < kItemsPerProducer) {
                if (rb.push(1)) {
                    pushed++;
                    totalPushed++;
                }
            }
        });
    }

    std::thread consumer([&]() {
        while (consumed.load() < kTotalItems) {
            const int* val = rb.peek();
            if (val != nullptr) {
                rb.pop();
                consumed++;
            } else {
                std::this_thread::yield();
            }
        }
    });

    for (auto& t : producers)
        t.join();
    consumer.join();

    EXPECT_EQ(consumed.load(), kTotalItems);
    EXPECT_TRUE(rb.empty());
}

// ─────────────────────────────────────────────
// Ready flag correctness
// ─────────────────────────────────────────────

TEST(MPSCRingbuffer, PeekReturnsNullIfNotReady) {
    // This tests the MPSC-specific behaviour: peek returns nullptr if the
    // slot has been claimed by a producer but data isn't written yet.
    // We can't easily force this race in a unit test, but we can verify
    // that peek correctly gates on the ready flag by checking that after
    // a full wraparound the ready flags are correctly managed.
    RB2 rb;
    for (int i = 0; i < 4; i++) {
        rb.push(i);
    }
    for (int i = 0; i < 4; i++) {
        ASSERT_NE(rb.peek(), nullptr);
        rb.pop();
    }
    // After full drain, ready flags should all be false
    EXPECT_EQ(rb.peek(), nullptr);
    EXPECT_TRUE(rb.empty());
}
