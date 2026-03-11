#include "Utilities/SPSCRingbuffer.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <thread>
#include <vector>

// ─────────────────────────────────────────────
// Convenience aliases
// ─────────────────────────────────────────────

using RB4 = bldc::SPSCRingbuffer<int, 4>;  // 16 elements
using RB2 = bldc::SPSCRingbuffer<int, 2>;  // 4 elements
using RB1 = bldc::SPSCRingbuffer<int, 1>;  // 2 elements

// ─────────────────────────────────────────────
// Size and capacity
// ─────────────────────────────────────────────

TEST(SPSCRingbuffer, SizeIsPowerOfTwo) {
    RB4 rb;
    EXPECT_EQ(rb.size(), 16);
}

TEST(SPSCRingbuffer, SizeRB2) {
    RB2 rb;
    EXPECT_EQ(rb.size(), 4);
}

TEST(SPSCRingbuffer, SizeRB1) {
    RB1 rb;
    EXPECT_EQ(rb.size(), 2);
}

// ─────────────────────────────────────────────
// Initial state
// ─────────────────────────────────────────────

TEST(SPSCRingbuffer, InitiallyEmpty) {
    RB4 rb;
    EXPECT_TRUE(rb.empty());
}

TEST(SPSCRingbuffer, InitiallyNotFull) {
    RB4 rb;
    EXPECT_FALSE(rb.full());
}

TEST(SPSCRingbuffer, InitialRemainingIsZero) {
    RB4 rb;
    EXPECT_EQ(rb.remaining(), 0);
}

TEST(SPSCRingbuffer, InitialPeekIsNull) {
    RB4 rb;
    EXPECT_EQ(rb.peek(), nullptr);
}

// ─────────────────────────────────────────────
// Push and peek
// ─────────────────────────────────────────────

TEST(SPSCRingbuffer, PushMakesNonEmpty) {
    RB4 rb;
    rb.push(42);
    EXPECT_FALSE(rb.empty());
}

TEST(SPSCRingbuffer, PeekAfterPush) {
    RB4 rb;
    rb.push(42);
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(*rb.peek(), 42);
}

TEST(SPSCRingbuffer, RemainingAfterPush) {
    RB4 rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    EXPECT_EQ(rb.remaining(), 3);
}

TEST(SPSCRingbuffer, PeekDoesNotConsume) {
    RB4 rb;
    rb.push(42);
    rb.peek();
    rb.peek();
    EXPECT_EQ(rb.remaining(), 1);
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(*rb.peek(), 42);
}

TEST(SPSCRingbuffer, PeekReturnsFront) {
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

TEST(SPSCRingbuffer, PopReducesRemaining) {
    RB4 rb;
    rb.push(1);
    rb.push(2);
    rb.pop();
    EXPECT_EQ(rb.remaining(), 1);
}

TEST(SPSCRingbuffer, PopOnEmptyIsNoop) {
    RB4 rb;
    rb.pop();  // should not crash
    EXPECT_TRUE(rb.empty());
}

TEST(SPSCRingbuffer, PopThenPeek) {
    RB4 rb;
    rb.push(1);
    rb.push(2);
    rb.pop();
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(*rb.peek(), 2);
}

TEST(SPSCRingbuffer, PopAllMakesEmpty) {
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

TEST(SPSCRingbuffer, FullAfterFillingBuffer) {
    RB2 rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    rb.push(4);
    EXPECT_TRUE(rb.full());
}

TEST(SPSCRingbuffer, PushWhenFullIsNoop) {
    RB2 rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    rb.push(4);
    rb.push(99);  // should be dropped
    EXPECT_EQ(rb.remaining(), 4);
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(*rb.peek(), 1);  // first element unchanged
}

TEST(SPSCRingbuffer, NotFullAfterPop) {
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

TEST(SPSCRingbuffer, ClearMakesEmpty) {
    RB4 rb;
    rb.push(1);
    rb.push(2);
    rb.clear();
    EXPECT_TRUE(rb.empty());
}

TEST(SPSCRingbuffer, ClearResetsRemaining) {
    RB4 rb;
    rb.push(1);
    rb.push(2);
    rb.clear();
    EXPECT_EQ(rb.remaining(), 0);
}

TEST(SPSCRingbuffer, ClearThenPush) {
    RB4 rb;
    rb.push(1);
    rb.push(2);
    rb.clear();
    rb.push(42);
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(*rb.peek(), 42);
}

TEST(SPSCRingbuffer, ClearOnEmptyIsNoop) {
    RB4 rb;
    rb.clear();
    EXPECT_TRUE(rb.empty());
}

// ─────────────────────────────────────────────
// Emplace
// ─────────────────────────────────────────────

TEST(SPSCRingbuffer, EmplaceReturnsPointer) {
    bldc::SPSCRingbuffer<std::string, 2> rb;
    const std::string* p = rb.emplace("hello");
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(*p, "hello");
}

TEST(SPSCRingbuffer, EmplaceWhenFullReturnsNull) {
    bldc::SPSCRingbuffer<std::string, 1> rb;  // 2 elements
    rb.emplace("a");
    rb.emplace("b");
    const std::string* p = rb.emplace("c");
    EXPECT_EQ(p, nullptr);
}

TEST(SPSCRingbuffer, EmplaceConstructsInPlace) {
    bldc::SPSCRingbuffer<std::pair<int, int>, 2> rb;
    rb.emplace(1, 2);
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(rb.peek()->first, 1);
    EXPECT_EQ(rb.peek()->second, 2);
}

TEST(SPSCRingbuffer, EmplaceThenPop) {
    bldc::SPSCRingbuffer<std::string, 2> rb;
    rb.emplace("first");
    rb.emplace("second");
    rb.pop();
    ASSERT_NE(rb.peek(), nullptr);
    EXPECT_EQ(*rb.peek(), "second");
}

// ─────────────────────────────────────────────
// Wraparound behaviour
// ─────────────────────────────────────────────

TEST(SPSCRingbuffer, WrapsAroundCorrectly) {
    RB2 rb;  // 4 elements
    // fill and drain several times to force wraparound
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

TEST(SPSCRingbuffer, PartialFillAndDrain) {
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
// SPSC threading
// ─────────────────────────────────────────────

TEST(SPSCRingbuffer, SingleProducerSingleConsumer) {
    bldc::SPSCRingbuffer<int, 8> rb;  // 256 elements
    constexpr int kItemCount = 10000;
    std::atomic<int> consumed{0};

    std::thread producer([&]() {
        int pushed = 0;
        while (pushed < kItemCount) {
            if (!rb.full()) {
                rb.push(pushed);
                pushed++;
            }
        }
    });

    std::thread consumer([&]() {
        int expected = 0;
        while (expected < kItemCount) {
            const int* val = rb.peek();
            if (val != nullptr) {
                EXPECT_EQ(*val, expected);
                rb.pop();
                expected++;
                consumed++;
            }
        }
    });

    producer.join();
    consumer.join();

    EXPECT_EQ(consumed.load(), kItemCount);
    EXPECT_TRUE(rb.empty());
}

TEST(SPSCRingbuffer, SPSCNoItemsLost) {
    bldc::SPSCRingbuffer<int, 6> rb;  // 64 elements
    constexpr int kItemCount = 5000;
    std::vector<int> received;
    received.reserve(kItemCount);

    std::thread producer([&]() {
        int pushed = 0;
        while (pushed < kItemCount) {
            if (!rb.full()) {
                rb.push(pushed++);
            }
        }
    });

    std::thread consumer([&]() {
        while ((int)received.size() < kItemCount) {
            const int* val = rb.peek();
            if (val != nullptr) {
                received.push_back(*val);
                rb.pop();
            }
        }
    });

    producer.join();
    consumer.join();

    ASSERT_EQ((int)received.size(), kItemCount);
    for (int i = 0; i < kItemCount; i++) {
        EXPECT_EQ(received[i], i);
    }
}
