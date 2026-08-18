#include <cstdint>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "astrabot_rtc/media/latest_frame_buffer.h"

namespace astrabot::rtc::media {
namespace {

std::shared_ptr<const RawVideoFrame> makeRawFrame(std::uint8_t value) {
    auto frame = std::make_shared<RawVideoFrame>();
    frame->track_id = "right_eye";
    frame->data = std::make_shared<const std::vector<std::uint8_t>>(std::vector<std::uint8_t>{value});
    return frame;
}

TEST(LatestFrameBufferTest, OverwritesOldFrameWithoutGrowingQueue) {
    LatestFrameBuffer buffer;
    ASSERT_TRUE(buffer.push(makeRawFrame(1U)).ok());
    ASSERT_TRUE(buffer.push(makeRawFrame(2U)).ok());

    const auto frame = buffer.takeLatest();
    ASSERT_TRUE(frame);
    EXPECT_EQ(frame->data->front(), 2U);
    EXPECT_EQ(buffer.overwrittenCount(), 1U);
    EXPECT_FALSE(buffer.takeLatest());
}

}  // namespace
}  // namespace astrabot::rtc::media
