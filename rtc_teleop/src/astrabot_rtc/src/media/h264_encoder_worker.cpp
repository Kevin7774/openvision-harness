#include "astrabot_rtc/media/h264_encoder_worker.h"

#include <utility>

namespace astrabot::rtc::media {

H264EncoderWorker::H264EncoderWorker(std::string track_id, std::unique_ptr<IH264Encoder> encoder)
    : track_id_(std::move(track_id)), encoder_(std::move(encoder)) {}

H264EncoderWorker::~H264EncoderWorker() {
    stop();
}

Status H264EncoderWorker::start(EncodedFrameSink sink) {
    if (track_id_.empty() || !encoder_ || !sink) {
        return Status::error(ErrorCode::kInvalidArgument, "H264 encoder worker configuration is incomplete");
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (running_) {
            return Status::success();
        }
    }

    const Status encoder_status = encoder_->start();
    if (!encoder_status.ok()) {
        return encoder_status;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        sink_ = std::move(sink);
        running_ = true;
    }
    worker_ = std::thread(&H264EncoderWorker::run, this);
    return Status::success();
}

void H264EncoderWorker::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        latest_frame_.reset();
    }
    frame_available_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sink_ = {};
    }
    if (encoder_) {
        encoder_->stop();
    }
}

Status H264EncoderWorker::submit(std::shared_ptr<const RawVideoFrame> frame) {
    if (!frame || frame->track_id != track_id_ || !frame->data) {
        return Status::error(ErrorCode::kInvalidArgument, "raw frame does not match H264 encoder worker track");
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) {
            return Status::error(ErrorCode::kFailedPrecondition, "H264 encoder worker is stopped");
        }
        if (latest_frame_) {
            ++overwritten_frames_;
        }
        latest_frame_ = std::move(frame);
        ++submitted_frames_;
    }
    frame_available_.notify_one();
    return Status::success();
}

H264EncoderWorkerMetrics H264EncoderWorker::metrics() const {
    return H264EncoderWorkerMetrics{submitted_frames_.load(), overwritten_frames_.load(), encoded_frames_.load(),
                                    encoded_bytes_.load(),    encode_failures_.load(),    sink_failures_.load()};
}

std::uint32_t H264EncoderWorker::activeFrameRate() const {
    return encoder_ ? encoder_->activeFrameRate() : 0U;
}

const std::string &H264EncoderWorker::encoderName() const {
    static const std::string empty_name;
    return encoder_ ? encoder_->encoderName() : empty_name;
}

void H264EncoderWorker::run() {
    while (true) {
        std::shared_ptr<const RawVideoFrame> raw_frame;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            frame_available_.wait(lock, [this]() { return !running_ || latest_frame_ != nullptr; });
            if (!running_) {
                latest_frame_.reset();
                return;
            }
            raw_frame = std::move(latest_frame_);
            latest_frame_.reset();
        }

        auto encoded_result = encoder_->encode(*raw_frame);
        if (!encoded_result.ok()) {
            ++encode_failures_;
            continue;
        }
        auto encoded_frames = encoded_result.takeValue();
        for (auto &encoded_frame : encoded_frames) {
            EncodedFrameSink sink;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!running_) {
                    return;
                }
                sink = sink_;
            }
            if (!encoded_frame || encoded_frame->track_id != track_id_ || !encoded_frame->data || !sink) {
                ++encode_failures_;
                continue;
            }
            ++encoded_frames_;
            encoded_bytes_.fetch_add(encoded_frame->data->size(), std::memory_order_relaxed);
            if (!sink(std::move(encoded_frame)).ok()) {
                ++sink_failures_;
            }
        }
    }
}

}  // namespace astrabot::rtc::media
