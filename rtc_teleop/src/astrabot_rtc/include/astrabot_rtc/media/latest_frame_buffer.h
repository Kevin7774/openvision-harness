#pragma once

#include <cstdint>
#include <memory>
#include <mutex>

#include "astrabot_rtc/common/status.h"
#include "astrabot_rtc/media/video_frame.h"

namespace astrabot::rtc::media {

/**
 * @brief 容量固定为 1 的原始帧 latest-wins 缓冲区。
 *
 * producer 永不因 consumer 变慢而排队；新帧覆盖旧帧并递增覆盖计数。
 */
class LatestFrameBuffer {
  public:
    /**
     * @brief 写入最新帧。
     */
    Status push(std::shared_ptr<const RawVideoFrame> frame);

    /**
     * @brief 取出当前帧；无帧时返回空指针。
     */
    std::shared_ptr<const RawVideoFrame> takeLatest();

    /**
     * @brief 清空缓存。
     */
    void clear();

    /**
     * @brief 返回累计覆盖旧帧数量。
     */
    std::uint64_t overwrittenCount() const;

  private:
    mutable std::mutex mutex_;
    std::shared_ptr<const RawVideoFrame> latest_;
    std::uint64_t overwritten_count_{0U};
};

}  // namespace astrabot::rtc::media
