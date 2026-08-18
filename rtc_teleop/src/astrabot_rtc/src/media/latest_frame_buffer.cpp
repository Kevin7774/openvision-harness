#include "astrabot_rtc/media/latest_frame_buffer.h"

#include <utility>

namespace astrabot::rtc::media {

Status LatestFrameBuffer::push(std::shared_ptr<const RawVideoFrame> frame) {
    if (!frame || frame->track_id.empty() || !frame->data) {
        return Status::error(ErrorCode::kInvalidArgument, "raw video frame is incomplete");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_) {
        ++overwritten_count_;
    }
    latest_ = std::move(frame);
    return Status::success();
}

std::shared_ptr<const RawVideoFrame> LatestFrameBuffer::takeLatest() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto frame = std::move(latest_);
    latest_.reset();
    return frame;
}

void LatestFrameBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.reset();
}

std::uint64_t LatestFrameBuffer::overwrittenCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return overwritten_count_;
}

}  // namespace astrabot::rtc::media
