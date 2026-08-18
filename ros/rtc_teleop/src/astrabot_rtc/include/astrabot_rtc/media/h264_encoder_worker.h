#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "astrabot_rtc/common/status.h"
#include "astrabot_rtc/media/h264_encoder.h"

namespace astrabot::rtc::media {

/**
 * @brief 单 track 编码 worker 的有界指标快照。
 */
struct H264EncoderWorkerMetrics {
    std::uint64_t submitted_frames{0U};
    std::uint64_t overwritten_frames{0U};
    std::uint64_t encoded_frames{0U};
    std::uint64_t encoded_bytes{0U};
    std::uint64_t encode_failures{0U};
    std::uint64_t sink_failures{0U};
};

/**
 * @brief 将单 track 原始帧通过容量 1 latest-wins mailbox 交给 H.264 encoder。
 *
 * submit 可由 ROS callback 并发调用；start/stop 必须由同一控制线程串行调用。stop 会唤醒空闲 worker、丢弃待编码帧并
 * 等待已经进入的 encode/sink 返回，返回后不再触发 sink。
 */
class H264EncoderWorker final {
  public:
    using EncodedFrameSink = std::function<Status(std::shared_ptr<const EncodedVideoFrame>)>;

    H264EncoderWorker(std::string track_id, std::unique_ptr<IH264Encoder> encoder);
    ~H264EncoderWorker();

    H264EncoderWorker(const H264EncoderWorker &) = delete;
    H264EncoderWorker &operator=(const H264EncoderWorker &) = delete;

    /**
     * @brief 启动 encoder 和 worker；重复调用保持幂等。
     */
    Status start(EncodedFrameSink sink);

    /**
     * @brief 停止接收新帧并打断 mailbox 等待。
     */
    void stop();

    /**
     * @brief 提交最新原始帧；已有待处理帧时覆盖旧帧。
     */
    Status submit(std::shared_ptr<const RawVideoFrame> frame);

    /**
     * @brief 返回有界队列和编码结果累计指标。
     */
    H264EncoderWorkerMetrics metrics() const;

    /**
     * @brief 返回实际 encoder 帧率。
     */
    std::uint32_t activeFrameRate() const;

    /**
     * @brief 返回显式 encoder 名称。
     */
    const std::string &encoderName() const;

  private:
    void run();

    const std::string track_id_;
    std::unique_ptr<IH264Encoder> encoder_;
    mutable std::mutex mutex_;
    std::condition_variable frame_available_;
    bool running_{false};
    std::shared_ptr<const RawVideoFrame> latest_frame_;
    EncodedFrameSink sink_;
    std::thread worker_;
    std::atomic<std::uint64_t> submitted_frames_{0U};
    std::atomic<std::uint64_t> overwritten_frames_{0U};
    std::atomic<std::uint64_t> encoded_frames_{0U};
    std::atomic<std::uint64_t> encoded_bytes_{0U};
    std::atomic<std::uint64_t> encode_failures_{0U};
    std::atomic<std::uint64_t> sink_failures_{0U};
};

}  // namespace astrabot::rtc::media
