#include "astrabot_rtc/media/ffmpeg_h264_encoder.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

namespace astrabot::rtc::media {
namespace {

constexpr std::size_t kMaximumPendingFrames = 4U;
constexpr std::size_t kMaximumEncodedFrameBytes = 8U * 1024U * 1024U;

struct SourceLayout {
    AVPixelFormat pixel_format{AV_PIX_FMT_NONE};
    std::uint32_t bytes_per_pixel{0U};
    bool semiplanar{false};
};

struct PendingFrame {
    std::int64_t pts{0};
    std::string track_id;
    std::uint64_t capture_time_ns{0U};
};

struct PacketAggregate {
    std::int64_t pts{AV_NOPTS_VALUE};
    bool key_frame{false};
    std::vector<std::uint8_t> data;
};

std::string ffmpegError(int error) {
    std::array<char, AV_ERROR_MAX_STRING_SIZE> buffer{};
    if (av_strerror(error, buffer.data(), buffer.size()) < 0) {
        return "unknown FFmpeg error";
    }
    return std::string(buffer.data());
}

bool startsWithAnnexBStartCode(const std::vector<std::uint8_t> &data) {
    if (data.size() >= 4U && data[0] == 0U && data[1] == 0U && data[2] == 0U && data[3] == 1U) {
        return true;
    }
    return data.size() >= 3U && data[0] == 0U && data[1] == 0U && data[2] == 1U;
}

Result<SourceLayout> sourceLayout(const std::string &encoding) {
    if (encoding == "rgb8") {
        return Result<SourceLayout>::success(SourceLayout{AV_PIX_FMT_RGB24, 3U, false});
    }
    if (encoding == "bgr8") {
        return Result<SourceLayout>::success(SourceLayout{AV_PIX_FMT_BGR24, 3U, false});
    }
    if (encoding == "rgba8") {
        return Result<SourceLayout>::success(SourceLayout{AV_PIX_FMT_RGBA, 4U, false});
    }
    if (encoding == "bgra8") {
        return Result<SourceLayout>::success(SourceLayout{AV_PIX_FMT_BGRA, 4U, false});
    }
    if (encoding == "mono8") {
        return Result<SourceLayout>::success(SourceLayout{AV_PIX_FMT_GRAY8, 1U, false});
    }
    if (encoding == "yuv422_yuy2" || encoding == "yuyv") {
        return Result<SourceLayout>::success(SourceLayout{AV_PIX_FMT_YUYV422, 2U, false});
    }
    if (encoding == "uyvy") {
        return Result<SourceLayout>::success(SourceLayout{AV_PIX_FMT_UYVY422, 2U, false});
    }
    if (encoding == "nv12") {
        return Result<SourceLayout>::success(SourceLayout{AV_PIX_FMT_NV12, 1U, true});
    }
    return Result<SourceLayout>::failure(
        Status::error(ErrorCode::kInvalidArgument, "raw video encoding is unsupported by the H264 encoder"));
}

Status validateSettings(const H264EncoderSettings &settings) {
    if (settings.encoder_name.empty() || settings.pixel_format != "nv12" || settings.output_width == 0U ||
        settings.output_height == 0U || settings.output_width % 2U != 0U || settings.output_height % 2U != 0U ||
        settings.output_width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        settings.output_height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        settings.output_width > 3840U || settings.output_height > 2160U) {
        return Status::error(ErrorCode::kInvalidArgument, "H264 encoder dimensions, name or pixel format are invalid");
    }
    if (settings.primary_frame_rate == 0U || settings.fallback_frame_rate == 0U ||
        settings.fallback_frame_rate > settings.primary_frame_rate ||
        settings.primary_frame_rate > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
        return Status::error(ErrorCode::kInvalidArgument, "H264 encoder frame rates are invalid");
    }
    if (settings.bitrate_bps == 0U ||
        settings.bitrate_bps > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
        settings.gop_size_frames == 0U ||
        settings.gop_size_frames > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        settings.max_encoded_frame_bytes == 0U || settings.max_encoded_frame_bytes > kMaximumEncodedFrameBytes ||
        settings.max_encoder_surfaces == 0U || settings.max_encoder_surfaces > 4U || settings.preset.empty() ||
        settings.tune.empty() || settings.profile.empty() || settings.level.empty()) {
        return Status::error(ErrorCode::kInvalidArgument, "H264 encoder rate, queue or codec options are invalid");
    }
    return Status::success();
}

Status validateRawFrame(const RawVideoFrame &frame, const SourceLayout &layout) {
    if (frame.track_id.empty() || frame.width == 0U || frame.height == 0U || frame.row_step == 0U || !frame.data) {
        return Status::error(ErrorCode::kInvalidArgument, "raw video frame is incomplete");
    }
    if (frame.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        frame.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        frame.row_step > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
        return Status::error(ErrorCode::kInvalidArgument, "raw video dimensions exceed FFmpeg integer limits");
    }
    if ((layout.pixel_format == AV_PIX_FMT_YUYV422 || layout.pixel_format == AV_PIX_FMT_UYVY422) &&
        frame.width % 2U != 0U) {
        return Status::error(ErrorCode::kInvalidArgument, "packed YUV raw video width must be even");
    }
    if (layout.semiplanar && (frame.width % 2U != 0U || frame.height % 2U != 0U)) {
        return Status::error(ErrorCode::kInvalidArgument, "subsampled raw video dimensions must be even");
    }
    if (frame.width > std::numeric_limits<std::uint32_t>::max() / layout.bytes_per_pixel) {
        return Status::error(ErrorCode::kInvalidArgument, "raw video row size overflows uint32");
    }
    const std::uint32_t minimum_row_step = frame.width * layout.bytes_per_pixel;
    if (frame.row_step < minimum_row_step) {
        return Status::error(ErrorCode::kInvalidArgument, "raw video row_step is smaller than one active row");
    }

    const std::size_t row_step = static_cast<std::size_t>(frame.row_step);
    const std::size_t height = static_cast<std::size_t>(frame.height);
    if (height > std::numeric_limits<std::size_t>::max() / row_step) {
        return Status::error(ErrorCode::kPayloadTooLarge, "raw video payload size overflows size_t");
    }
    std::size_t required_bytes = row_step * (height - 1U) + static_cast<std::size_t>(minimum_row_step);
    if (layout.semiplanar) {
        const std::size_t luma_bytes = row_step * height;
        const std::size_t chroma_rows = height / 2U;
        if (chroma_rows > (std::numeric_limits<std::size_t>::max() - luma_bytes) / row_step) {
            return Status::error(ErrorCode::kPayloadTooLarge, "NV12 payload size overflows size_t");
        }
        required_bytes = luma_bytes + row_step * chroma_rows;
    }
    if (frame.data->size() < required_bytes) {
        return Status::error(ErrorCode::kInvalidArgument, "raw video payload is shorter than its declared layout");
    }
    return Status::success();
}

Status setStringOption(void *private_data, const char *name, const std::string &value) {
    const int result = av_opt_set(private_data, name, value.c_str(), 0);
    if (result < 0) {
        return Status::error(ErrorCode::kInvalidArgument,
                             std::string("FFmpeg rejected encoder option ") + name + ": " + ffmpegError(result));
    }
    return Status::success();
}

Status setOptionalIntegerOption(void *private_data, const char *name, std::int64_t value) {
    const int result = av_opt_set_int(private_data, name, value, 0);
    if (result == AVERROR_OPTION_NOT_FOUND) {
        return Status::success();
    }
    if (result < 0) {
        return Status::error(ErrorCode::kInvalidArgument,
                             std::string("FFmpeg rejected encoder option ") + name + ": " + ffmpegError(result));
    }
    return Status::success();
}

}  // namespace

class FfmpegH264Encoder::Implementation final {
  public:
    explicit Implementation(H264EncoderSettings settings) : settings_(std::move(settings)) {}

    ~Implementation() {
        stop();
    }

    Status start() {
        if (started_) {
            return Status::success();
        }
        const Status settings_status = validateSettings(settings_);
        if (!settings_status.ok()) {
            return settings_status;
        }

        const AVCodec *codec = avcodec_find_encoder_by_name(settings_.encoder_name.c_str());
        if (codec == nullptr) {
            return Status::error(ErrorCode::kNotFound, "configured FFmpeg encoder was not found");
        }
        if (codec->id != AV_CODEC_ID_H264) {
            return Status::error(ErrorCode::kInvalidArgument, "configured FFmpeg encoder is not H264");
        }
        if (settings_.require_hardware && (codec->capabilities & AV_CODEC_CAP_HARDWARE) == 0) {
            return Status::error(ErrorCode::kFailedPrecondition,
                                 "configured FFmpeg encoder is not marked as hardware-backed");
        }

        Status primary_status = openCodec(codec, settings_.primary_frame_rate);
        if (primary_status.ok()) {
            return Status::success();
        }
        releaseCodecResources();
        if (settings_.fallback_frame_rate == settings_.primary_frame_rate) {
            return primary_status;
        }

        const Status fallback_status = openCodec(codec, settings_.fallback_frame_rate);
        if (!fallback_status.ok()) {
            releaseCodecResources();
            return Status::error(ErrorCode::kUnavailable,
                                 "configured FFmpeg encoder failed at primary and fallback frame rates: " +
                                     primary_status.message() + "; " + fallback_status.message());
        }
        return Status::success();
    }

    void stop() {
        releaseCodecResources();
    }

    Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>> encode(const RawVideoFrame &frame) {
        if (!started_ || codec_context_ == nullptr || converted_frame_ == nullptr || packet_ == nullptr) {
            return Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>>::failure(
                Status::error(ErrorCode::kFailedPrecondition, "FFmpeg H264 encoder is stopped"));
        }
        auto layout_result = sourceLayout(frame.encoding);
        if (!layout_result.ok()) {
            return Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>>::failure(layout_result.status());
        }
        const SourceLayout layout = layout_result.takeValue();
        const Status frame_status = validateRawFrame(frame, layout);
        if (!frame_status.ok()) {
            return Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>>::failure(frame_status);
        }
        if (pending_frames_.size() >= kMaximumPendingFrames) {
            return Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>>::failure(
                Status::error(ErrorCode::kResourceExhausted, "FFmpeg encoder pending frame bound was reached"));
        }

        const Status conversion_status = convertFrame(frame, layout);
        if (!conversion_status.ok()) {
            return Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>>::failure(conversion_status);
        }

        const std::int64_t pts = next_pts_++;
        converted_frame_->pts = pts;
        converted_frame_->pict_type = pts == 0 || pts % static_cast<std::int64_t>(settings_.gop_size_frames) == 0
                                          ? AV_PICTURE_TYPE_I
                                          : AV_PICTURE_TYPE_NONE;
        pending_frames_.push_back(PendingFrame{pts, frame.track_id, frame.capture_time_ns});

        const int send_result = avcodec_send_frame(codec_context_, converted_frame_);
        if (send_result < 0) {
            pending_frames_.pop_back();
            return Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>>::failure(Status::error(
                ErrorCode::kInternal, "FFmpeg failed to accept a raw frame: " + ffmpegError(send_result)));
        }
        return receivePackets();
    }

    std::uint32_t activeFrameRate() const {
        return active_frame_rate_;
    }

    const std::string &encoderName() const {
        return settings_.encoder_name;
    }

  private:
    Status openCodec(const AVCodec *codec, std::uint32_t frame_rate) {
        codec_context_ = avcodec_alloc_context3(codec);
        if (codec_context_ == nullptr) {
            return Status::error(ErrorCode::kResourceExhausted, "FFmpeg could not allocate an encoder context");
        }

        codec_context_->codec_type = AVMEDIA_TYPE_VIDEO;
        codec_context_->codec_id = AV_CODEC_ID_H264;
        codec_context_->width = static_cast<int>(settings_.output_width);
        codec_context_->height = static_cast<int>(settings_.output_height);
        codec_context_->pix_fmt = AV_PIX_FMT_NV12;
        codec_context_->time_base = AVRational{1, static_cast<int>(frame_rate)};
        codec_context_->framerate = AVRational{static_cast<int>(frame_rate), 1};
        codec_context_->pkt_timebase = codec_context_->time_base;
        codec_context_->bit_rate = static_cast<std::int64_t>(settings_.bitrate_bps);
        codec_context_->rc_max_rate = codec_context_->bit_rate;
        const std::uint64_t frame_budget = settings_.bitrate_bps / frame_rate;
        const std::uint64_t bounded_frame_budget =
            std::min<std::uint64_t>(frame_budget, static_cast<std::uint64_t>(std::numeric_limits<int>::max()) / 2U);
        codec_context_->rc_buffer_size = static_cast<int>(std::max<std::uint64_t>(bounded_frame_budget * 2U, 1U));
        codec_context_->gop_size = static_cast<int>(settings_.gop_size_frames);
        codec_context_->max_b_frames = 0;
        codec_context_->thread_count = 1;
        codec_context_->flags |= AV_CODEC_FLAG_LOW_DELAY;

        Status status = setStringOption(codec_context_->priv_data, "preset", settings_.preset);
        if (!status.ok()) {
            return status;
        }
        status = setStringOption(codec_context_->priv_data, "tune", settings_.tune);
        if (!status.ok()) {
            return status;
        }
        status = setStringOption(codec_context_->priv_data, "profile", settings_.profile);
        if (!status.ok()) {
            return status;
        }
        status = setStringOption(codec_context_->priv_data, "level", settings_.level);
        if (!status.ok()) {
            return status;
        }
        status = setOptionalIntegerOption(codec_context_->priv_data, "rc-lookahead", 0);
        if (!status.ok()) {
            return status;
        }
        status = setOptionalIntegerOption(codec_context_->priv_data, "delay", 0);
        if (!status.ok()) {
            return status;
        }
        status = setOptionalIntegerOption(codec_context_->priv_data, "surfaces", settings_.max_encoder_surfaces);
        if (!status.ok()) {
            return status;
        }
        status = setOptionalIntegerOption(codec_context_->priv_data, "zerolatency", 1);
        if (!status.ok()) {
            return status;
        }
        status = setOptionalIntegerOption(codec_context_->priv_data, "forced-idr", 1);
        if (!status.ok()) {
            return status;
        }
        status = setOptionalIntegerOption(codec_context_->priv_data, "repeat_headers", 1);
        if (!status.ok()) {
            return status;
        }

        const int open_result = avcodec_open2(codec_context_, codec, nullptr);
        if (open_result < 0) {
            return Status::error(ErrorCode::kUnavailable,
                                 "FFmpeg could not open the configured encoder: " + ffmpegError(open_result));
        }

        converted_frame_ = av_frame_alloc();
        packet_ = av_packet_alloc();
        if (converted_frame_ == nullptr || packet_ == nullptr) {
            return Status::error(ErrorCode::kResourceExhausted, "FFmpeg could not allocate frame or packet storage");
        }
        converted_frame_->format = codec_context_->pix_fmt;
        converted_frame_->width = codec_context_->width;
        converted_frame_->height = codec_context_->height;
        const int buffer_result = av_frame_get_buffer(converted_frame_, 32);
        if (buffer_result < 0) {
            return Status::error(ErrorCode::kResourceExhausted,
                                 "FFmpeg could not allocate NV12 frame storage: " + ffmpegError(buffer_result));
        }

        active_frame_rate_ = frame_rate;
        next_pts_ = 0;
        started_ = true;
        return Status::success();
    }

    Status convertFrame(const RawVideoFrame &frame, const SourceLayout &layout) {
        const int writable_result = av_frame_make_writable(converted_frame_);
        if (writable_result < 0) {
            return Status::error(ErrorCode::kInternal,
                                 "FFmpeg output frame is not writable: " + ffmpegError(writable_result));
        }

        const std::uint8_t *source_data[4]{frame.data->data(), nullptr, nullptr, nullptr};
        int source_linesize[4]{static_cast<int>(frame.row_step), 0, 0, 0};
        if (layout.semiplanar) {
            const std::size_t chroma_offset = static_cast<std::size_t>(frame.row_step) * frame.height;
            source_data[1] = frame.data->data() + chroma_offset;
            source_linesize[1] = static_cast<int>(frame.row_step);
        }

        sws_context_ = sws_getCachedContext(sws_context_, static_cast<int>(frame.width), static_cast<int>(frame.height),
                                            layout.pixel_format, codec_context_->width, codec_context_->height,
                                            codec_context_->pix_fmt, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
        if (sws_context_ == nullptr) {
            return Status::error(ErrorCode::kResourceExhausted, "FFmpeg could not allocate a pixel conversion context");
        }
        const int converted_rows =
            sws_scale(sws_context_, source_data, source_linesize, 0, static_cast<int>(frame.height),
                      converted_frame_->data, converted_frame_->linesize);
        if (converted_rows != codec_context_->height) {
            return Status::error(ErrorCode::kInternal, "FFmpeg pixel conversion returned an incomplete frame");
        }
        return Status::success();
    }

    Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>> receivePackets() {
        std::vector<PacketAggregate> aggregates;
        while (true) {
            const int receive_result = avcodec_receive_packet(codec_context_, packet_);
            if (receive_result == AVERROR(EAGAIN) || receive_result == AVERROR_EOF) {
                break;
            }
            if (receive_result < 0) {
                av_packet_unref(packet_);
                return Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>>::failure(Status::error(
                    ErrorCode::kInternal, "FFmpeg failed to produce H264 data: " + ffmpegError(receive_result)));
            }
            const std::int64_t packet_pts = packet_->pts;
            auto aggregate = std::find_if(aggregates.begin(), aggregates.end(),
                                          [packet_pts](const auto &candidate) { return candidate.pts == packet_pts; });
            if (aggregate == aggregates.end()) {
                aggregates.push_back(PacketAggregate{packet_pts, false, {}});
                aggregate = std::prev(aggregates.end());
            }
            if (packet_->size <= 0 || aggregate->data.size() > settings_.max_encoded_frame_bytes ||
                static_cast<std::size_t>(packet_->size) > settings_.max_encoded_frame_bytes - aggregate->data.size()) {
                av_packet_unref(packet_);
                pending_frames_.clear();
                return Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>>::failure(
                    Status::error(ErrorCode::kPayloadTooLarge, "encoded H264 frame exceeded its configured bound"));
            }
            aggregate->data.insert(aggregate->data.end(), packet_->data, packet_->data + packet_->size);
            aggregate->key_frame = aggregate->key_frame || (packet_->flags & AV_PKT_FLAG_KEY) != 0;
            av_packet_unref(packet_);
        }

        std::vector<std::shared_ptr<const EncodedVideoFrame>> frames;
        frames.reserve(aggregates.size());
        for (auto &aggregate : aggregates) {
            auto pending = aggregate.pts == AV_NOPTS_VALUE
                               ? pending_frames_.begin()
                               : std::find_if(pending_frames_.begin(), pending_frames_.end(),
                                              [&aggregate](const auto &item) { return item.pts == aggregate.pts; });
            if (pending == pending_frames_.end()) {
                pending_frames_.clear();
                return Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>>::failure(
                    Status::error(ErrorCode::kInternal, "FFmpeg returned an unknown frame timestamp"));
            }
            if (aggregate.data.empty() || !startsWithAnnexBStartCode(aggregate.data)) {
                pending_frames_.erase(pending);
                return Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>>::failure(
                    Status::error(ErrorCode::kInternal, "FFmpeg H264 output is not an Annex-B access unit"));
            }

            auto encoded_frame = std::make_shared<EncodedVideoFrame>();
            encoded_frame->track_id = pending->track_id;
            encoded_frame->codec = "h264";
            encoded_frame->capture_time_ns = pending->capture_time_ns;
            encoded_frame->key_frame = aggregate.key_frame;
            encoded_frame->data = std::make_shared<const std::vector<std::uint8_t>>(std::move(aggregate.data));
            pending_frames_.erase(pending);
            frames.push_back(std::move(encoded_frame));
        }
        return Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>>::success(std::move(frames));
    }

    void releaseCodecResources() {
        if (sws_context_ != nullptr) {
            sws_freeContext(sws_context_);
            sws_context_ = nullptr;
        }
        av_packet_free(&packet_);
        av_frame_free(&converted_frame_);
        avcodec_free_context(&codec_context_);
        pending_frames_.clear();
        active_frame_rate_ = 0U;
        next_pts_ = 0;
        started_ = false;
    }

    H264EncoderSettings settings_;
    AVCodecContext *codec_context_{nullptr};
    AVFrame *converted_frame_{nullptr};
    AVPacket *packet_{nullptr};
    SwsContext *sws_context_{nullptr};
    std::deque<PendingFrame> pending_frames_;
    std::uint32_t active_frame_rate_{0U};
    std::int64_t next_pts_{0};
    bool started_{false};
};

FfmpegH264Encoder::FfmpegH264Encoder(H264EncoderSettings settings)
    : implementation_(std::make_unique<Implementation>(std::move(settings))) {}

FfmpegH264Encoder::~FfmpegH264Encoder() = default;

Status FfmpegH264Encoder::start() {
    return implementation_->start();
}

void FfmpegH264Encoder::stop() {
    implementation_->stop();
}

Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>> FfmpegH264Encoder::encode(const RawVideoFrame &frame) {
    return implementation_->encode(frame);
}

std::uint32_t FfmpegH264Encoder::activeFrameRate() const {
    return implementation_->activeFrameRate();
}

const std::string &FfmpegH264Encoder::encoderName() const {
    return implementation_->encoderName();
}

}  // namespace astrabot::rtc::media
