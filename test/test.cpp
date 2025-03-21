#include <gtest/gtest.h>
#include <atomic>
#include <chrono>

// Demonstrate some basic assertions.
TEST(HelloTest, BasicAssertions) {
    // Expect two strings not to be equal.
    EXPECT_STRNE("hello", "world");
    // Expect equality.
    EXPECT_EQ(7 * 6, 42);
}

TEST(DurationConversion, DurationConversion) {
    using namespace std::chrono;
    auto duration = 200ms;
    EXPECT_FLOAT_EQ(duration_cast<std::chrono::duration<float>>(duration).count(), 0.2);
    duration = 0ms;
    EXPECT_FLOAT_EQ(duration_cast<std::chrono::duration<float>>(duration).count(), 0);
    auto duration_us = 3us;
    EXPECT_FLOAT_EQ(duration_cast<std::chrono::duration<float>>(duration_us).count(), 0.000003);
}