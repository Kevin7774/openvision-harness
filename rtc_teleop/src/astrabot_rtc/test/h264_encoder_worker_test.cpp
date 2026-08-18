#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "astrabot_rtc/media/h264_encoder_worker.h"

namespace astrabot::rtc::media {
namespace {

using namespace std::chrono_literals;

class BlockingFakeEncoder final : public IH264Encoder {
  public:
    Status start() override {
        running_ = true;
        return Status::success();
    }

    void stop() override {
        running_ = false;
    }

    Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>> encode(const RawVideoFrame &frame) override {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (encode_count_ == 0U) {
                first_encode_entered_ = true;
                condition_.notify_all();
                condition_.wait(lock, [this]() { return release_first_encode_; });
            }
            ++encode_count_;
        }
        auto encoded = std::make_shared<EncodedVideoFrame>();
        encoded->track_id = frame.track_id;
        encoded->codec = "h264";
        encoded->capture_time_ns = frame.capture_time_ns;
        const std::uint8_t value = frame.data->empty() ? static_cast<std::uint8_t>(0U) : frame.data->front();
        encoded->data = std::make_shared<const std::vector<std::uint8_t>>(std::vector<std::uint8_t>{value});
        std::vector<std::shared_ptr<const EncodedVideoFrame>> output;
        output.push_back(std::move(encoded));
        return Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>>::success(std::move(output));
    }

    std::uint32_t activeFrameRate() const override {
        return running_ ? 30U : 0U;
    }

    const std::string &encoderName() const override {
        return encoder_name_;
    }

    bool waitForFirstEncode() {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, 2s, [this]() { return first_encode_entered_; });
    }

    void releaseFirstEncode() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            release_first_encode_ = true;
        }
        condition_.notify_all();
    }

  private:
    std::string encoder_name_{"blocking_fake"};
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    bool running_{false};
    bool first_encode_entered_{false};
    bool release_first_encode_{false};
    std::uint64_t encode_count_{0U};
};

std::shared_ptr<const RawVideoFrame> makeRawFrame(std::uint8_t value) {
    auto frame = std::make_shared<RawVideoFrame>();
    frame->track_id = "right_eye";
    frame->capture_time_ns = value;
    frame->width = 1U;
    frame->height = 1U;
    frame->row_step = 1U;
    frame->encoding = "mono8";
    frame->data = std::make_shared<const std::vector<std::uint8_t>>(std::vector<std::uint8_t>{value});
    return frame;
}

TEST(H264EncoderWorkerTest, OverwritesPendingFrameAndStopsIdempotently) {
    auto encoder = std::make_unique<BlockingFakeEncoder>();
    BlockingFakeEncoder *encoder_observer = encoder.get();
    H264EncoderWorker worker("right_eye", std::move(encoder));

    std::mutex output_mutex;
    std::condition_variable output_condition;
    std::vector<std::uint8_t> output_values;
    ASSERT_TRUE(worker
                    .start([&](std::shared_ptr<const EncodedVideoFrame> frame) {
                        {
                            std::lock_guard<std::mutex> lock(output_mutex);
                            output_values.push_back(frame->data->front());
                        }
                        output_condition.notify_all();
                        return Status::success();
                    })
                    .ok());
    ASSERT_TRUE(worker.submit(makeRawFrame(1U)).ok());
    ASSERT_TRUE(encoder_observer->waitForFirstEncode());
    ASSERT_TRUE(worker.submit(makeRawFrame(2U)).ok());
    ASSERT_TRUE(worker.submit(makeRawFrame(3U)).ok());
    encoder_observer->releaseFirstEncode();

    {
        std::unique_lock<std::mutex> lock(output_mutex);
        ASSERT_TRUE(output_condition.wait_for(lock, 2s, [&output_values]() { return output_values.size() == 2U; }));
    }
    worker.stop();
    worker.stop();

    ASSERT_EQ(output_values.size(), 2U);
    EXPECT_EQ(output_values[0], 1U);
    EXPECT_EQ(output_values[1], 3U);
    const auto metrics = worker.metrics();
    EXPECT_EQ(metrics.submitted_frames, 3U);
    EXPECT_EQ(metrics.overwritten_frames, 1U);
    EXPECT_EQ(metrics.encoded_frames, 2U);
    EXPECT_EQ(metrics.encoded_bytes, 2U);
    EXPECT_EQ(metrics.encode_failures, 0U);
    EXPECT_EQ(metrics.sink_failures, 0U);
    EXPECT_EQ(worker.submit(makeRawFrame(4U)).code(), ErrorCode::kFailedPrecondition);
}

TEST(H264EncoderWorkerTest, StopInterruptsIdleMailboxWait) {
    auto encoder = std::make_unique<BlockingFakeEncoder>();
    H264EncoderWorker worker("right_eye", std::move(encoder));
    ASSERT_TRUE(worker.start([](std::shared_ptr<const EncodedVideoFrame>) { return Status::success(); }).ok());

    const auto start = std::chrono::steady_clock::now();
    worker.stop();
    const auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_LT(elapsed, 500ms);
}

TEST(H264EncoderWorkerTest, StopWaitsForInFlightSinkAndPreventsLaterCallbacks) {
    auto encoder = std::make_unique<BlockingFakeEncoder>();
    BlockingFakeEncoder *encoder_observer = encoder.get();
    encoder_observer->releaseFirstEncode();
    H264EncoderWorker worker("right_eye", std::move(encoder));

    std::mutex sink_mutex;
    std::condition_variable sink_condition;
    bool sink_entered = false;
    bool release_sink = false;
    std::atomic<std::uint64_t> callback_count{0U};
    ASSERT_TRUE(worker
                    .start([&](std::shared_ptr<const EncodedVideoFrame>) {
                        std::unique_lock<std::mutex> lock(sink_mutex);
                        ++callback_count;
                        sink_entered = true;
                        sink_condition.notify_all();
                        sink_condition.wait(lock, [&release_sink]() { return release_sink; });
                        return Status::success();
                    })
                    .ok());
    ASSERT_TRUE(worker.submit(makeRawFrame(1U)).ok());
    {
        std::unique_lock<std::mutex> lock(sink_mutex);
        ASSERT_TRUE(sink_condition.wait_for(lock, 2s, [&sink_entered]() { return sink_entered; }));
    }

    std::mutex stop_mutex;
    std::condition_variable stop_condition;
    bool stop_started = false;
    std::atomic<bool> stop_completed{false};
    std::thread stopper([&]() {
        {
            std::lock_guard<std::mutex> lock(stop_mutex);
            stop_started = true;
        }
        stop_condition.notify_all();
        worker.stop();
        stop_completed.store(true);
    });
    {
        std::unique_lock<std::mutex> lock(stop_mutex);
        EXPECT_TRUE(stop_condition.wait_for(lock, 2s, [&stop_started]() { return stop_started; }));
    }
    EXPECT_FALSE(stop_completed.load());
    {
        std::lock_guard<std::mutex> lock(sink_mutex);
        release_sink = true;
    }
    sink_condition.notify_all();
    stopper.join();

    EXPECT_TRUE(stop_completed.load());
    EXPECT_EQ(callback_count.load(), 1U);
    EXPECT_EQ(worker.submit(makeRawFrame(2U)).code(), ErrorCode::kFailedPrecondition);
    EXPECT_EQ(callback_count.load(), 1U);
}

}  // namespace
}  // namespace astrabot::rtc::media
