#include "astrabot_rtc/config/rtc_config.h"

#include <arpa/inet.h>
#include <algorithm>
#include <cctype>
#include <charconv>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string_view>
#include <utility>

namespace astrabot::rtc::config {
namespace {

constexpr std::size_t kMaximumViewerCount = 2U;
constexpr std::size_t kMaximumPeerCount = 16U;
constexpr std::size_t kMaximumDataChannelCount = 32U;
constexpr std::size_t kMaximumDataPayloadBytes = 16384U;
constexpr std::size_t kMaximumSignalingPayloadBytes = 64U * 1024U;
constexpr std::size_t kMaximumBufferedAmountBytes = 4U * 1024U * 1024U;
constexpr std::size_t kMaximumMediaBufferedAmountBytes = 8U * 1024U * 1024U;
constexpr std::size_t kMaximumMediaTrackCount = 8U;
constexpr std::uint32_t kMaximumVideoWidth = 3840U;
constexpr std::uint32_t kMaximumVideoHeight = 2160U;
constexpr std::uint32_t kMaximumFrameRate = 120U;
constexpr std::uint64_t kMinimumBitrateBps = 100000U;
constexpr std::uint64_t kMaximumBitrateBps = 50000000U;
constexpr std::size_t kMaximumEncodedFrameBytes = 8U * 1024U * 1024U;
constexpr std::uint32_t kMaximumEncoderSurfaces = 4U;

const std::set<std::string> &knownKeys() {
    static const std::set<std::string> keys{
        "topics.gateway_command",
        "topics.gateway_report",
        "topics.peer_event",
        "topics.data_received",
        "topics.authorize_channel_service",
        "topics.close_peer_service",
        "topics.diagnostics",
        "runtime.max_viewers",
        "runtime.max_peers",
        "runtime.max_data_channels",
        "runtime.max_payload_bytes",
        "runtime.dispatch_period_ms",
        "runtime.authorization_timeout_ms",
        "runtime.diagnostics_period_ms",
        "signaling.max_payload_bytes",
        "transport.backend",
        "transport.bind_address",
        "transport.max_buffered_amount_bytes",
        "transport.max_media_buffered_amount_bytes",
        "media.enabled",
        "media.max_encoded_subscribers",
        "media.encoder_name",
        "media.require_hardware",
        "media.output_width",
        "media.output_height",
        "media.frame_rate",
        "media.fallback_frame_rate",
        "media.bitrate_bps",
        "media.gop_size_frames",
        "media.max_encoded_frame_bytes",
        "media.max_encoder_surfaces",
        "media.pixel_format",
        "media.preset",
        "media.tune",
        "media.profile",
        "media.level",
        "media.track_ids",
        "media.image_topics",
        "media.camera_info_topics",
    };
    return keys;
}

bool isKnownSection(const std::string &section) {
    const std::string prefix = section + '.';
    return std::any_of(knownKeys().begin(), knownKeys().end(),
                       [&prefix](const std::string &key) { return key.compare(0U, prefix.size(), prefix) == 0; });
}

std::string trim(std::string value) {
    const auto not_space = [](unsigned char character) { return !std::isspace(character); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

std::string unquote(std::string value) {
    if (value.size() >= 2U &&
        ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\''))) {
        return value.substr(1U, value.size() - 2U);
    }
    return value;
}

Result<std::map<std::string, std::string>> parseYamlScalars(const std::string &path) {
    std::ifstream input(path);
    if (!input) {
        return Result<std::map<std::string, std::string>>::failure(
            Status::error(ErrorCode::kNotFound, "rtc config file is not readable"));
    }

    std::map<std::string, std::string> values;
    std::vector<std::pair<int, std::string>> sections;
    std::string line;
    std::size_t line_number = 0U;
    while (std::getline(input, line)) {
        ++line_number;
        const std::size_t comment_position = line.find('#');
        if (comment_position != std::string::npos) {
            line = line.substr(0U, comment_position);
        }
        if (trim(line).empty()) {
            continue;
        }

        int indentation = 0;
        while (indentation < static_cast<int>(line.size()) && line[static_cast<std::size_t>(indentation)] == ' ') {
            ++indentation;
        }
        const std::string content = trim(line);
        const std::size_t separator = content.find(':');
        if (separator == std::string::npos) {
            return Result<std::map<std::string, std::string>>::failure(
                Status::error(ErrorCode::kInvalidArgument,
                              "rtc config contains a line without ':' at line " + std::to_string(line_number)));
        }

        const std::string key = trim(content.substr(0U, separator));
        const std::string value = trim(content.substr(separator + 1U));
        if (key.empty()) {
            return Result<std::map<std::string, std::string>>::failure(
                Status::error(ErrorCode::kInvalidArgument, "rtc config contains an empty key"));
        }

        while (!sections.empty() && sections.back().first >= indentation) {
            sections.pop_back();
        }
        if (value.empty()) {
            std::string full_section;
            for (const auto &section : sections) {
                if (!full_section.empty()) {
                    full_section += '.';
                }
                full_section += section.second;
            }
            if (!full_section.empty()) {
                full_section += '.';
            }
            full_section += key;
            if (!isKnownSection(full_section)) {
                return Result<std::map<std::string, std::string>>::failure(
                    Status::error(ErrorCode::kInvalidArgument, "rtc config contains unknown section: " + full_section));
            }
            sections.emplace_back(indentation, key);
            continue;
        }

        std::string full_key;
        for (const auto &section : sections) {
            if (!full_key.empty()) {
                full_key += '.';
            }
            full_key += section.second;
        }
        if (!full_key.empty()) {
            full_key += '.';
        }
        full_key += key;
        if (knownKeys().count(full_key) == 0U) {
            return Result<std::map<std::string, std::string>>::failure(
                Status::error(ErrorCode::kInvalidArgument, "rtc config contains unknown key: " + full_key));
        }
        if (!values.emplace(full_key, unquote(value)).second) {
            return Result<std::map<std::string, std::string>>::failure(
                Status::error(ErrorCode::kAlreadyExists, "rtc config contains duplicate key: " + full_key));
        }
    }

    return Result<std::map<std::string, std::string>>::success(std::move(values));
}

Result<std::size_t> parseSize(const std::string &text, const std::string &key) {
    std::size_t value = 0U;
    const char *begin = text.data();
    const char *end = text.data() + text.size();
    const auto result = std::from_chars(begin, end, value);
    if (result.ec != std::errc() || result.ptr != end) {
        return Result<std::size_t>::failure(
            Status::error(ErrorCode::kInvalidArgument, "rtc config expects an unsigned integer for " + key));
    }
    return Result<std::size_t>::success(value);
}

Result<std::int64_t> parseI64(const std::string &text, const std::string &key) {
    std::int64_t value = 0;
    const char *begin = text.data();
    const char *end = text.data() + text.size();
    const auto result = std::from_chars(begin, end, value);
    if (result.ec != std::errc() || result.ptr != end) {
        return Result<std::int64_t>::failure(
            Status::error(ErrorCode::kInvalidArgument, "rtc config expects an integer for " + key));
    }
    return Result<std::int64_t>::success(value);
}

Result<std::uint64_t> parseU64(const std::string &text, const std::string &key) {
    std::uint64_t value = 0U;
    const char *begin = text.data();
    const char *end = text.data() + text.size();
    const auto result = std::from_chars(begin, end, value);
    if (result.ec != std::errc() || result.ptr != end) {
        return Result<std::uint64_t>::failure(
            Status::error(ErrorCode::kInvalidArgument, "rtc config expects an unsigned integer for " + key));
    }
    return Result<std::uint64_t>::success(value);
}

Result<bool> parseBool(const std::string &text, const std::string &key) {
    std::string normalized;
    normalized.reserve(text.size());
    for (const char character : text) {
        normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(character))));
    }
    if (normalized == "true" || normalized == "yes" || normalized == "on" || normalized == "1") {
        return Result<bool>::success(true);
    }
    if (normalized == "false" || normalized == "no" || normalized == "off" || normalized == "0") {
        return Result<bool>::success(false);
    }
    return Result<bool>::failure(Status::error(ErrorCode::kInvalidArgument, "rtc config expects a boolean for " + key));
}

std::vector<std::string> splitCsv(const std::string &text) {
    std::vector<std::string> values;
    std::istringstream stream(text);
    std::string value;
    while (std::getline(stream, value, ',')) {
        value = trim(value);
        if (!value.empty()) {
            values.push_back(value);
        }
    }
    return values;
}

const std::string *findValue(const std::map<std::string, std::string> &values, const std::string &key) {
    const auto iterator = values.find(key);
    return iterator == values.end() ? nullptr : &iterator->second;
}

void setString(const std::map<std::string, std::string> &values, const std::string &key, std::string &output) {
    if (const std::string *value = findValue(values, key)) {
        output = *value;
    }
}

Status setSize(const std::map<std::string, std::string> &values, const std::string &key, std::size_t &output) {
    const std::string *value = findValue(values, key);
    if (value == nullptr) {
        return Status::success();
    }
    auto parsed = parseSize(*value, key);
    if (!parsed.ok()) {
        return parsed.status();
    }
    output = parsed.takeValue();
    return Status::success();
}

Status setI64(const std::map<std::string, std::string> &values, const std::string &key, std::int64_t &output) {
    const std::string *value = findValue(values, key);
    if (value == nullptr) {
        return Status::success();
    }
    auto parsed = parseI64(*value, key);
    if (!parsed.ok()) {
        return parsed.status();
    }
    output = parsed.takeValue();
    return Status::success();
}

Status setU32(const std::map<std::string, std::string> &values, const std::string &key, std::uint32_t &output) {
    const std::string *value = findValue(values, key);
    if (value == nullptr) {
        return Status::success();
    }
    auto parsed = parseU64(*value, key);
    if (!parsed.ok()) {
        return parsed.status();
    }
    if (parsed.value() > std::numeric_limits<std::uint32_t>::max()) {
        return Status::error(ErrorCode::kInvalidArgument, "rtc config integer exceeds uint32 range for " + key);
    }
    output = static_cast<std::uint32_t>(parsed.value());
    return Status::success();
}

Status setU64(const std::map<std::string, std::string> &values, const std::string &key, std::uint64_t &output) {
    const std::string *value = findValue(values, key);
    if (value == nullptr) {
        return Status::success();
    }
    auto parsed = parseU64(*value, key);
    if (!parsed.ok()) {
        return parsed.status();
    }
    output = parsed.takeValue();
    return Status::success();
}

Status setBool(const std::map<std::string, std::string> &values, const std::string &key, bool &output) {
    const std::string *value = findValue(values, key);
    if (value == nullptr) {
        return Status::success();
    }
    auto parsed = parseBool(*value, key);
    if (!parsed.ok()) {
        return parsed.status();
    }
    output = parsed.takeValue();
    return Status::success();
}

RtcConfig defaultConfig() {
    RtcConfig config;
    config.media.tracks = {
        MediaTrackSettings{"left_eye", "/astrabot/data_sources/image/left_eye",
                           "/astrabot/data_sources/camera_info/left_eye"},
        MediaTrackSettings{"right_eye", "/astrabot/data_sources/image/right_eye",
                           "/astrabot/data_sources/camera_info/right_eye"},
    };
    return config;
}

Status applyValues(const std::map<std::string, std::string> &values, RtcConfig &config) {
    setString(values, "topics.gateway_command", config.topics.gateway_command);
    setString(values, "topics.gateway_report", config.topics.gateway_report);
    setString(values, "topics.peer_event", config.topics.peer_event);
    setString(values, "topics.data_received", config.topics.data_received);
    setString(values, "topics.authorize_channel_service", config.topics.authorize_channel_service);
    setString(values, "topics.close_peer_service", config.topics.close_peer_service);
    setString(values, "topics.diagnostics", config.topics.diagnostics);
    setString(values, "transport.backend", config.transport.backend);
    setString(values, "transport.bind_address", config.transport.bind_address);
    setString(values, "media.encoder_name", config.media.encoder.encoder_name);
    setString(values, "media.pixel_format", config.media.encoder.pixel_format);
    setString(values, "media.preset", config.media.encoder.preset);
    setString(values, "media.tune", config.media.encoder.tune);
    setString(values, "media.profile", config.media.encoder.profile);
    setString(values, "media.level", config.media.encoder.level);

    Status status = setSize(values, "runtime.max_viewers", config.runtime.max_viewers);
    if (!status.ok()) {
        return status;
    }
    status = setSize(values, "runtime.max_peers", config.runtime.max_peers);
    if (!status.ok()) {
        return status;
    }
    status = setSize(values, "runtime.max_data_channels", config.runtime.max_data_channels);
    if (!status.ok()) {
        return status;
    }
    status = setSize(values, "runtime.max_payload_bytes", config.runtime.max_payload_bytes);
    if (!status.ok()) {
        return status;
    }
    status = setI64(values, "runtime.dispatch_period_ms", config.runtime.dispatch_period_ms);
    if (!status.ok()) {
        return status;
    }
    status = setI64(values, "runtime.authorization_timeout_ms", config.runtime.authorization_timeout_ms);
    if (!status.ok()) {
        return status;
    }
    status = setI64(values, "runtime.diagnostics_period_ms", config.runtime.diagnostics_period_ms);
    if (!status.ok()) {
        return status;
    }
    status = setSize(values, "signaling.max_payload_bytes", config.signaling.max_payload_bytes);
    if (!status.ok()) {
        return status;
    }
    status = setSize(values, "transport.max_buffered_amount_bytes", config.transport.max_buffered_amount_bytes);
    if (!status.ok()) {
        return status;
    }
    status =
        setSize(values, "transport.max_media_buffered_amount_bytes", config.transport.max_media_buffered_amount_bytes);
    if (!status.ok()) {
        return status;
    }
    status = setBool(values, "media.enabled", config.media.enabled);
    if (!status.ok()) {
        return status;
    }
    status = setSize(values, "media.max_encoded_subscribers", config.media.max_encoded_subscribers);
    if (!status.ok()) {
        return status;
    }
    status = setBool(values, "media.require_hardware", config.media.encoder.require_hardware);
    if (!status.ok()) {
        return status;
    }
    status = setU32(values, "media.output_width", config.media.encoder.output_width);
    if (!status.ok()) {
        return status;
    }
    status = setU32(values, "media.output_height", config.media.encoder.output_height);
    if (!status.ok()) {
        return status;
    }
    status = setU32(values, "media.frame_rate", config.media.encoder.frame_rate);
    if (!status.ok()) {
        return status;
    }
    status = setU32(values, "media.fallback_frame_rate", config.media.encoder.fallback_frame_rate);
    if (!status.ok()) {
        return status;
    }
    status = setU64(values, "media.bitrate_bps", config.media.encoder.bitrate_bps);
    if (!status.ok()) {
        return status;
    }
    status = setU32(values, "media.gop_size_frames", config.media.encoder.gop_size_frames);
    if (!status.ok()) {
        return status;
    }
    status = setSize(values, "media.max_encoded_frame_bytes", config.media.encoder.max_encoded_frame_bytes);
    if (!status.ok()) {
        return status;
    }
    status = setU32(values, "media.max_encoder_surfaces", config.media.encoder.max_encoder_surfaces);
    if (!status.ok()) {
        return status;
    }

    const std::string *track_ids_text = findValue(values, "media.track_ids");
    const std::string *image_topics_text = findValue(values, "media.image_topics");
    const std::string *camera_info_topics_text = findValue(values, "media.camera_info_topics");
    if (track_ids_text != nullptr || image_topics_text != nullptr || camera_info_topics_text != nullptr) {
        if (track_ids_text == nullptr || image_topics_text == nullptr || camera_info_topics_text == nullptr) {
            return Status::error(
                ErrorCode::kInvalidArgument,
                "media.track_ids, media.image_topics and media.camera_info_topics must be set together");
        }
        const auto track_ids = splitCsv(*track_ids_text);
        const auto image_topics = splitCsv(*image_topics_text);
        const auto camera_info_topics = splitCsv(*camera_info_topics_text);
        if (track_ids.size() != image_topics.size() || track_ids.size() != camera_info_topics.size()) {
            return Status::error(ErrorCode::kInvalidArgument, "media track CSV lists must have equal length");
        }
        config.media.tracks.clear();
        for (std::size_t index = 0U; index < track_ids.size(); ++index) {
            config.media.tracks.push_back(
                MediaTrackSettings{track_ids[index], image_topics[index], camera_info_topics[index]});
        }
    }
    return Status::success();
}

bool isValidRosName(const std::string &name) {
    return !name.empty() && name.front() == '/' && name.find_first_of(" \t\r\n") == std::string::npos;
}

bool isSafeEncoderOption(const std::string &value) {
    return !value.empty() && value.size() <= 64U &&
           std::all_of(value.begin(), value.end(), [](unsigned char character) {
               return std::isalnum(character) || character == '_' || character == '-' || character == '.';
           });
}

bool isSafeTrackId(const std::string &value) {
    return !value.empty() && value.size() <= 64U &&
           std::none_of(value.begin(), value.end(),
                        [](unsigned char character) { return character < 0x20U || character == 0x7FU; });
}

}  // namespace

Result<RtcConfig> RtcConfigLoader::load(const std::string &path) const {
    RtcConfig config = defaultConfig();
    if (path.empty()) {
        const Status validation = validate(config);
        if (!validation.ok()) {
            return Result<RtcConfig>::failure(validation);
        }
        return Result<RtcConfig>::success(std::move(config));
    }

    auto parsed = parseYamlScalars(path);
    if (!parsed.ok()) {
        return Result<RtcConfig>::failure(parsed.status());
    }
    const Status applied = applyValues(parsed.value(), config);
    if (!applied.ok()) {
        return Result<RtcConfig>::failure(applied);
    }
    const Status validation = validate(config);
    if (!validation.ok()) {
        return Result<RtcConfig>::failure(validation);
    }
    return Result<RtcConfig>::success(std::move(config));
}

Status RtcConfigLoader::validate(const RtcConfig &config) const {
    const std::vector<std::string> ros_names{
        config.topics.gateway_command,
        config.topics.gateway_report,
        config.topics.peer_event,
        config.topics.data_received,
        config.topics.authorize_channel_service,
        config.topics.close_peer_service,
        config.topics.diagnostics,
    };
    if (std::any_of(ros_names.begin(), ros_names.end(),
                    [](const std::string &name) { return !isValidRosName(name); })) {
        return Status::error(ErrorCode::kInvalidArgument, "rtc topic and service names must be absolute ROS names");
    }
    if (config.runtime.max_viewers == 0U || config.runtime.max_viewers > kMaximumViewerCount) {
        return Status::error(ErrorCode::kInvalidArgument, "runtime.max_viewers must be in [1, 2]");
    }
    if (config.runtime.max_peers < config.runtime.max_viewers || config.runtime.max_peers > kMaximumPeerCount) {
        return Status::error(ErrorCode::kInvalidArgument, "runtime.max_peers is outside the supported range");
    }
    if (config.runtime.max_data_channels == 0U || config.runtime.max_data_channels > kMaximumDataChannelCount) {
        return Status::error(ErrorCode::kInvalidArgument, "runtime.max_data_channels is outside the supported range");
    }
    if (config.runtime.max_payload_bytes == 0U || config.runtime.max_payload_bytes > kMaximumDataPayloadBytes) {
        return Status::error(ErrorCode::kInvalidArgument, "runtime.max_payload_bytes must be in [1, 16384]");
    }
    if (config.signaling.max_payload_bytes == 0U ||
        config.signaling.max_payload_bytes > kMaximumSignalingPayloadBytes) {
        return Status::error(ErrorCode::kInvalidArgument, "signaling.max_payload_bytes is outside the supported range");
    }
    if (config.runtime.dispatch_period_ms < 1 || config.runtime.dispatch_period_ms > 1000 ||
        config.runtime.authorization_timeout_ms < 1 || config.runtime.authorization_timeout_ms > 5000 ||
        config.runtime.diagnostics_period_ms < 100 || config.runtime.diagnostics_period_ms > 60000) {
        return Status::error(ErrorCode::kInvalidArgument, "rtc runtime periods are outside the supported range");
    }
    if (config.transport.backend != "disabled" && config.transport.backend != "libdatachannel") {
        return Status::error(ErrorCode::kInvalidArgument, "transport.backend is unsupported");
    }
    if (!config.transport.bind_address.empty()) {
        in_addr ipv4{};
        in6_addr ipv6{};
        if (inet_pton(AF_INET, config.transport.bind_address.c_str(), &ipv4) != 1 &&
            inet_pton(AF_INET6, config.transport.bind_address.c_str(), &ipv6) != 1) {
            return Status::error(ErrorCode::kInvalidArgument,
                                 "transport.bind_address must be an IPv4 or IPv6 literal");
        }
    }
    if (config.transport.max_buffered_amount_bytes < config.runtime.max_payload_bytes ||
        config.transport.max_buffered_amount_bytes > kMaximumBufferedAmountBytes) {
        return Status::error(ErrorCode::kInvalidArgument,
                             "transport.max_buffered_amount_bytes must cover one payload and stay below 4 MiB");
    }
    if (config.transport.max_media_buffered_amount_bytes == 0U ||
        config.transport.max_media_buffered_amount_bytes > kMaximumMediaBufferedAmountBytes) {
        return Status::error(ErrorCode::kInvalidArgument,
                             "transport.max_media_buffered_amount_bytes must be in [1, 8 MiB]");
    }
    if (config.media.max_encoded_subscribers == 0U || config.media.max_encoded_subscribers > kMaximumViewerCount ||
        config.media.max_encoded_subscribers > config.runtime.max_viewers) {
        return Status::error(ErrorCode::kInvalidArgument,
                             "media.max_encoded_subscribers must be in [1, 2] and not exceed runtime.max_viewers");
    }
    const auto &encoder = config.media.encoder;
    if (!isSafeEncoderOption(encoder.encoder_name) || !isSafeEncoderOption(encoder.preset) ||
        !isSafeEncoderOption(encoder.tune) || !isSafeEncoderOption(encoder.profile) ||
        !isSafeEncoderOption(encoder.level)) {
        return Status::error(ErrorCode::kInvalidArgument, "media encoder name and options contain unsafe characters");
    }
    if (encoder.pixel_format != "nv12") {
        return Status::error(ErrorCode::kInvalidArgument, "media.pixel_format currently only supports nv12");
    }
    if (encoder.output_width == 0U || encoder.output_width > kMaximumVideoWidth || encoder.output_height == 0U ||
        encoder.output_height > kMaximumVideoHeight || encoder.output_width % 2U != 0U ||
        encoder.output_height % 2U != 0U) {
        return Status::error(ErrorCode::kInvalidArgument, "media output dimensions must be even and within 3840x2160");
    }
    if (encoder.frame_rate == 0U || encoder.frame_rate > kMaximumFrameRate || encoder.fallback_frame_rate == 0U ||
        encoder.fallback_frame_rate > encoder.frame_rate) {
        return Status::error(ErrorCode::kInvalidArgument,
                             "media frame rates must satisfy 1 <= fallback_frame_rate <= frame_rate <= 120");
    }
    if (encoder.bitrate_bps < kMinimumBitrateBps || encoder.bitrate_bps > kMaximumBitrateBps) {
        return Status::error(ErrorCode::kInvalidArgument, "media.bitrate_bps must be in [100000, 50000000]");
    }
    if (encoder.gop_size_frames == 0U || encoder.gop_size_frames > encoder.frame_rate * 10U) {
        return Status::error(ErrorCode::kInvalidArgument,
                             "media.gop_size_frames must be within ten seconds of configured frame rate");
    }
    if (encoder.max_encoded_frame_bytes == 0U || encoder.max_encoded_frame_bytes > kMaximumEncodedFrameBytes) {
        return Status::error(ErrorCode::kInvalidArgument, "media.max_encoded_frame_bytes must be in [1, 8388608]");
    }
    if (encoder.max_encoder_surfaces == 0U || encoder.max_encoder_surfaces > kMaximumEncoderSurfaces) {
        return Status::error(ErrorCode::kInvalidArgument, "media.max_encoder_surfaces must be in [1, 4]");
    }
    if (config.media.tracks.empty() || config.media.tracks.size() > kMaximumMediaTrackCount) {
        return Status::error(ErrorCode::kInvalidArgument, "media track count must be in [1, 8]");
    }
    std::set<std::string> track_ids;
    for (const auto &track : config.media.tracks) {
        if (!isSafeTrackId(track.track_id) || !isValidRosName(track.image_topic) ||
            !isValidRosName(track.camera_info_topic)) {
            return Status::error(ErrorCode::kInvalidArgument, "media track contains an invalid id or ROS topic");
        }
        if (!track_ids.insert(track.track_id).second) {
            return Status::error(ErrorCode::kAlreadyExists, "media track ids must be unique");
        }
    }
    return Status::success();
}

}  // namespace astrabot::rtc::config
