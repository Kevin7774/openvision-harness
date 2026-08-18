#include <cstdint>

#include <gtest/gtest.h>

#include "astrabot_rtc/runtime/media_rate_tracker.h"

namespace astrabot::rtc::runtime {
namespace {

TEST(MediaRateTrackerTest, ReportsApplicationRatesForLatestWindow) {
    MediaRateTracker tracker;
    EXPECT_FALSE(tracker.update(1000000000U, MediaCumulativeCounters{10U, 1000U, 20U, 2000U}).available);

    const auto window = tracker.update(2000000000U, MediaCumulativeCounters{40U, 5000U, 80U, 10000U});
    EXPECT_TRUE(window.available);
    EXPECT_EQ(window.duration_ms, 1000U);
    EXPECT_DOUBLE_EQ(window.encoded_fps, 30.0);
    EXPECT_EQ(window.encoded_bitrate_bps, 32000U);
    EXPECT_DOUBLE_EQ(window.peer_send_fps, 60.0);
    EXPECT_EQ(window.peer_send_bitrate_bps, 64000U);
}

TEST(MediaRateTrackerTest, CounterRegressionRebuildsBaseline) {
    MediaRateTracker tracker;
    EXPECT_FALSE(tracker.update(1000000000U, MediaCumulativeCounters{10U, 100U, 10U, 100U}).available);
    EXPECT_FALSE(tracker.update(2000000000U, MediaCumulativeCounters{1U, 10U, 1U, 10U}).available);

    const auto window = tracker.update(3000000000U, MediaCumulativeCounters{2U, 20U, 3U, 30U});
    EXPECT_TRUE(window.available);
    EXPECT_DOUBLE_EQ(window.encoded_fps, 1.0);
    EXPECT_EQ(window.encoded_bitrate_bps, 80U);
    EXPECT_DOUBLE_EQ(window.peer_send_fps, 2.0);
    EXPECT_EQ(window.peer_send_bitrate_bps, 160U);
}

TEST(MediaRateTrackerTest, ResetClearsWindowBaseline) {
    MediaRateTracker tracker;
    EXPECT_FALSE(tracker.update(1000000000U, {}).available);
    tracker.reset();
    EXPECT_FALSE(tracker.update(2000000000U, MediaCumulativeCounters{1U, 10U, 1U, 10U}).available);
}

}  // namespace
}  // namespace astrabot::rtc::runtime
