#include "astrabot_rtc/transport/webrtc/libdatachannel_signaling.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

namespace astrabot::rtc::transport {
namespace {

constexpr std::size_t kMaxPeerIdBytes = 128U;
constexpr std::size_t kMaxSessionIdBytes = 128U;
constexpr std::size_t kMaxRoomBytes = 256U;
constexpr std::size_t kMaxRunIdBytes = 64U;
constexpr std::size_t kMaxResourceIdBytes = 64U;
constexpr std::size_t kMaxPurposeBytes = 16U;
constexpr std::size_t kMaxTrackIdBytes = 64U;
constexpr std::size_t kMaxSdpBytes = 1024U * 1024U;
constexpr std::size_t kMaxCandidateBytes = 4096U;
constexpr std::size_t kMaxCandidateMidBytes = 64U;
constexpr std::size_t kMaxAuthorizationTokenBytes = 8192U;
constexpr std::size_t kMaxIceServers = 8U;
constexpr std::size_t kMaxIceUrlsPerServer = 4U;
constexpr std::size_t kMaxIceUrlBytes = 2048U;
constexpr std::size_t kMaxIceCredentialBytes = 512U;
constexpr std::size_t kMaxMediaTracks = 8U;
constexpr std::size_t kMaxReasonCodeBytes = 128U;
constexpr std::size_t kTeleopMaxPayloadBytes = 16384U;

bool isReasonCodeCharacter(unsigned char character) {
    return std::isalnum(character) != 0 || character == '_' || character == '-' || character == '.' || character == ':';
}

bool containsAsciiControl(const std::string &value) {
    return std::any_of(value.begin(), value.end(),
                       [](unsigned char character) { return character < 0x20U || character == 0x7FU; });
}

std::string normalizeSdpLineEndings(const std::string &sdp) {
    std::string normalized;
    normalized.reserve(sdp.size());
    for (std::size_t index = 0U; index < sdp.size(); ++index) {
        const char character = sdp[index];
        if (character == '\r') {
            normalized.append("\r\n");
            if (index + 1U < sdp.size() && sdp[index + 1U] == '\n') {
                ++index;
            }
        } else if (character == '\n') {
            normalized.append("\r\n");
        } else {
            normalized.push_back(character);
        }
    }
    return normalized;
}

std::string sanitizedReasonCode(const std::string &reason_code, const char *fallback) {
    if (reason_code.empty()) {
        return fallback;
    }
    if (reason_code.size() > kMaxReasonCodeBytes ||
        !std::all_of(reason_code.begin(), reason_code.end(),
                     [](unsigned char character) { return isReasonCodeCharacter(character); })) {
        return fallback;
    }
    return reason_code;
}

const std::string *stringField(const nlohmann::json &value, const char *key) {
    const auto iterator = value.find(key);
    if (iterator == value.end() || !iterator->is_string()) {
        return nullptr;
    }
    return &iterator->get_ref<const std::string &>();
}

Status validateOptionalString(const nlohmann::json &value, const char *key, std::size_t max_bytes) {
    const auto iterator = value.find(key);
    if (iterator == value.end()) {
        return Status::success();
    }
    if (!iterator->is_string() || iterator->get_ref<const std::string &>().size() > max_bytes) {
        return Status::error(ErrorCode::kInvalidArgument, "WebRTC signaling contains an invalid string field");
    }
    return Status::success();
}

Status validateOptionalSafeString(const nlohmann::json &value, const char *key, std::size_t max_bytes) {
    const Status string_status = validateOptionalString(value, key, max_bytes);
    if (!string_status.ok()) {
        return string_status;
    }
    const std::string *field = stringField(value, key);
    if (field != nullptr && containsAsciiControl(*field)) {
        return Status::error(ErrorCode::kInvalidArgument, "WebRTC signaling string contains control characters");
    }
    return Status::success();
}

std::string stringOrEmpty(const nlohmann::json &value, const char *key) {
    const std::string *field = stringField(value, key);
    return field == nullptr ? std::string{} : *field;
}

std::string peerId(const nlohmann::json &signal) {
    std::string peer_id = stringOrEmpty(signal, "peer_id");
    if (peer_id.empty()) {
        peer_id = stringOrEmpty(signal, "from");
    }
    return peer_id;
}

bool startsWith(const std::string &value, const char *prefix) {
    const std::string prefix_value(prefix);
    return value.compare(0U, prefix_value.size(), prefix_value) == 0;
}

bool isSupportedIceUrl(const std::string &url) {
    return startsWith(url, "stun:") || startsWith(url, "stuns:") || startsWith(url, "turn:") ||
           startsWith(url, "turns:");
}

Status validateVersion(const nlohmann::json &value) {
    for (const char *key : {"version", "protocol_version"}) {
        const auto iterator = value.find(key);
        if (iterator == value.end()) {
            continue;
        }
        bool supported = false;
        if (iterator->is_number_unsigned()) {
            supported = iterator->get<std::uint64_t>() == 1U;
        } else if (iterator->is_number_integer()) {
            supported = iterator->get<std::int64_t>() == 1;
        }
        if (!supported) {
            return Status::error(ErrorCode::kInvalidArgument, "WebRTC signaling version is unsupported");
        }
    }
    return Status::success();
}

std::optional<std::int64_t> integerValue(const nlohmann::json &value) {
    if (value.is_number_unsigned()) {
        const std::uint64_t unsigned_value = value.get<std::uint64_t>();
        if (unsigned_value > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
            return std::nullopt;
        }
        return static_cast<std::int64_t>(unsigned_value);
    }
    if (value.is_number_integer()) {
        return value.get<std::int64_t>();
    }
    return std::nullopt;
}

Result<std::vector<WebRtcIceServerSettings>> parseIceServers(const nlohmann::json &signal) {
    const auto servers = signal.find("ice_servers");
    if (servers == signal.end()) {
        return Result<std::vector<WebRtcIceServerSettings>>::success({});
    }
    if (!servers->is_array() || servers->size() > kMaxIceServers) {
        return Result<std::vector<WebRtcIceServerSettings>>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC signaling contains invalid ICE servers"));
    }

    std::vector<WebRtcIceServerSettings> parsed_servers;
    parsed_servers.reserve(servers->size());
    for (const auto &server : *servers) {
        if (!server.is_object()) {
            return Result<std::vector<WebRtcIceServerSettings>>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC signaling contains invalid ICE server entry"));
        }
        const auto urls = server.find("urls");
        if (urls == server.end()) {
            return Result<std::vector<WebRtcIceServerSettings>>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC ICE server is missing urls"));
        }

        WebRtcIceServerSettings parsed;
        if (urls->is_string()) {
            parsed.urls.push_back(urls->get_ref<const std::string &>());
        } else if (urls->is_array() && urls->size() <= kMaxIceUrlsPerServer) {
            for (const auto &url : *urls) {
                if (!url.is_string()) {
                    return Result<std::vector<WebRtcIceServerSettings>>::failure(
                        Status::error(ErrorCode::kInvalidArgument, "WebRTC ICE server contains a non-string URL"));
                }
                parsed.urls.push_back(url.get_ref<const std::string &>());
            }
        } else {
            return Result<std::vector<WebRtcIceServerSettings>>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC ICE server urls are invalid"));
        }
        if (parsed.urls.empty()) {
            return Result<std::vector<WebRtcIceServerSettings>>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC ICE server has no URL"));
        }
        for (const auto &url : parsed.urls) {
            if (url.empty() || url.size() > kMaxIceUrlBytes || containsAsciiControl(url) || !isSupportedIceUrl(url)) {
                return Result<std::vector<WebRtcIceServerSettings>>::failure(
                    Status::error(ErrorCode::kInvalidArgument, "WebRTC ICE server URL is unsupported"));
            }
        }

        const Status username_status = validateOptionalSafeString(server, "username", kMaxIceCredentialBytes);
        const Status credential_status = validateOptionalSafeString(server, "credential", kMaxIceCredentialBytes);
        if (!username_status.ok() || !credential_status.ok()) {
            return Result<std::vector<WebRtcIceServerSettings>>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC ICE credential is invalid"));
        }
        parsed.username = stringOrEmpty(server, "username");
        parsed.credential = stringOrEmpty(server, "credential");
        if (parsed.username.empty() != parsed.credential.empty()) {
            return Result<std::vector<WebRtcIceServerSettings>>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC ICE username and credential must appear together"));
        }
        parsed_servers.push_back(std::move(parsed));
    }
    return Result<std::vector<WebRtcIceServerSettings>>::success(std::move(parsed_servers));
}

Result<std::vector<std::string>> parseMediaTracks(const nlohmann::json &signal) {
    const auto tracks = signal.find("media_tracks");
    if (tracks == signal.end()) {
        return Result<std::vector<std::string>>::success({});
    }
    if (!tracks->is_array() || tracks->size() > kMaxMediaTracks) {
        return Result<std::vector<std::string>>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC peer media_tracks are invalid"));
    }
    std::set<std::string> unique_tracks;
    std::vector<std::string> parsed_tracks;
    parsed_tracks.reserve(tracks->size());
    for (const auto &track : *tracks) {
        if (!track.is_string()) {
            return Result<std::vector<std::string>>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC peer media_tracks contain a non-string id"));
        }
        const std::string &track_id = track.get_ref<const std::string &>();
        if (track_id.empty() || track_id.size() > kMaxTrackIdBytes || containsAsciiControl(track_id) ||
            !unique_tracks.insert(track_id).second) {
            return Result<std::vector<std::string>>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC peer media_tracks contain an invalid id"));
        }
        parsed_tracks.push_back(track_id);
    }
    return Result<std::vector<std::string>>::success(std::move(parsed_tracks));
}

Result<std::optional<WebRtcDataChannelSettings>> parseDataChannel(const nlohmann::json &signal) {
    const auto channel = signal.find("data_channel");
    if (channel == signal.end()) {
        return Result<std::optional<WebRtcDataChannelSettings>>::success(std::nullopt);
    }
    if (!channel->is_object() || !channel->contains("label") || !channel->contains("ordered") ||
        !channel->contains("max_packet_lifetime_ms")) {
        return Result<std::optional<WebRtcDataChannelSettings>>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC DataChannel contract is invalid"));
    }
    const auto &label = (*channel)["label"];
    const auto &ordered = (*channel)["ordered"];
    const auto &lifetime = (*channel)["max_packet_lifetime_ms"];
    const auto lifetime_value = integerValue(lifetime);
    if (!label.is_string() || !ordered.is_boolean() || !lifetime_value.has_value()) {
        return Result<std::optional<WebRtcDataChannelSettings>>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC DataChannel options have invalid types"));
    }

    const std::string &label_value = label.get_ref<const std::string &>();
    const bool ordered_value = ordered.get<bool>();
    if (label_value != "astrabot.teleop" || ordered_value || *lifetime_value != 20) {
        return Result<std::optional<WebRtcDataChannelSettings>>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC DataChannel label or reliability is unsupported"));
    }
    const auto payload_limit = channel->find("max_payload_bytes");
    if (channel->size() != 4U || payload_limit == channel->end()) {
        return Result<std::optional<WebRtcDataChannelSettings>>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC DataChannel fields do not match the contract"));
    }
    const auto payload_limit_value = integerValue(*payload_limit);
    if (!payload_limit_value.has_value() || *payload_limit_value != static_cast<std::int64_t>(kTeleopMaxPayloadBytes)) {
        return Result<std::optional<WebRtcDataChannelSettings>>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC DataChannel payload limit is unsupported"));
    }
    return Result<std::optional<WebRtcDataChannelSettings>>::success(WebRtcDataChannelSettings{
        label_value, ordered_value, static_cast<std::uint32_t>(*lifetime_value), kTeleopMaxPayloadBytes});
}

Result<std::string> authorizationToken(const nlohmann::json &signal) {
    const auto grant = signal.find("teleop_grant");
    const auto generic = signal.find("authorization_token");
    if (grant != signal.end() && generic != signal.end()) {
        return Result<std::string>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC peer contains conflicting authorization fields"));
    }
    const auto token = grant != signal.end() ? grant : generic;
    if (token == signal.end()) {
        return Result<std::string>::success({});
    }
    if (!token->is_string() || token->get_ref<const std::string &>().empty() ||
        token->get_ref<const std::string &>().size() > kMaxAuthorizationTokenBytes ||
        containsAsciiControl(token->get_ref<const std::string &>())) {
        return Result<std::string>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC peer authorization token is invalid"));
    }
    return Result<std::string>::success(token->get_ref<const std::string &>());
}

Result<const nlohmann::json *> normalizeSignal(const nlohmann::json &root, nlohmann::json &decoded_string) {
    const nlohmann::json *signal = &root;
    const auto payload = root.find("payload");
    if (payload != root.end() && payload->is_object()) {
        const auto wrapped = payload->find("signal");
        if (wrapped != payload->end()) {
            signal = &(*wrapped);
        }
    } else if (!root.contains("type")) {
        const auto wrapped = root.find("signal");
        if (wrapped != root.end()) {
            signal = &(*wrapped);
        }
    }
    if (signal->is_string()) {
        decoded_string = nlohmann::json::parse(signal->get_ref<const std::string &>(), nullptr, false);
        signal = &decoded_string;
    }
    if (signal->is_discarded() || !signal->is_object()) {
        return Result<const nlohmann::json *>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC signaling JSON shape is invalid"));
    }
    return Result<const nlohmann::json *>::success(signal);
}

Status validateRouteFields(const nlohmann::json &signal) {
    for (const auto &field : {std::pair<const char *, std::size_t>{"session_id", kMaxSessionIdBytes},
                              {"peer_id", kMaxPeerIdBytes},
                              {"from", kMaxPeerIdBytes},
                              {"to", kMaxPeerIdBytes},
                              {"room", kMaxRoomBytes},
                              {"run_id", kMaxRunIdBytes},
                              {"resource_id", kMaxResourceIdBytes},
                              {"purpose", kMaxPurposeBytes}}) {
        const Status status = validateOptionalSafeString(signal, field.first, field.second);
        if (!status.ok()) {
            return status;
        }
    }
    return Status::success();
}

Result<std::string> checkedDump(nlohmann::json value, std::size_t max_payload_bytes) {
    const std::string payload = value.dump();
    if (payload.size() > max_payload_bytes) {
        return Result<std::string>::failure(
            Status::error(ErrorCode::kPayloadTooLarge, "WebRTC signaling report exceeds the configured limit"));
    }
    return Result<std::string>::success(payload);
}

nlohmann::json routeEnvelope(const std::string &local_peer_id, const std::string &room, const std::string &session_id,
                             const std::string &peer_id) {
    nlohmann::json envelope{{"type", "signal"}, {"version", 1},       {"from", local_peer_id},
                            {"to", peer_id},    {"peer_id", peer_id}, {"room", room}};
    if (!session_id.empty()) {
        envelope["session_id"] = session_id;
    }
    return envelope;
}

}  // namespace

LibDataChannelSignalingCodec::LibDataChannelSignalingCodec(std::size_t max_payload_bytes)
    : max_payload_bytes_(max_payload_bytes) {}

Result<WebRtcSignalingCommand> LibDataChannelSignalingCodec::parse(const std::string &payload) const {
    if (payload.empty()) {
        return Result<WebRtcSignalingCommand>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC signaling payload is empty"));
    }
    if (payload.size() > max_payload_bytes_) {
        return Result<WebRtcSignalingCommand>::failure(
            Status::error(ErrorCode::kPayloadTooLarge, "WebRTC signaling payload exceeds the configured limit"));
    }
    const nlohmann::json root = nlohmann::json::parse(payload, nullptr, false);
    if (root.is_discarded() || !root.is_object()) {
        return Result<WebRtcSignalingCommand>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC signaling payload is not a JSON object"));
    }
    const Status root_version = validateVersion(root);
    if (!root_version.ok()) {
        return Result<WebRtcSignalingCommand>::failure(root_version);
    }
    const auto wrapped_payload = root.find("payload");
    if (wrapped_payload != root.end() && wrapped_payload->is_object()) {
        const Status payload_version = validateVersion(*wrapped_payload);
        if (!payload_version.ok()) {
            return Result<WebRtcSignalingCommand>::failure(payload_version);
        }
    }
    nlohmann::json decoded_string;
    auto normalized = normalizeSignal(root, decoded_string);
    if (!normalized.ok()) {
        return Result<WebRtcSignalingCommand>::failure(normalized.status());
    }
    const nlohmann::json &signal = *normalized.value();
    const Status signal_version = validateVersion(signal);
    if (!signal_version.ok()) {
        return Result<WebRtcSignalingCommand>::failure(signal_version);
    }
    const Status route_status = validateRouteFields(signal);
    if (!route_status.ok()) {
        return Result<WebRtcSignalingCommand>::failure(route_status);
    }
    const std::string type = stringOrEmpty(signal, "type");
    if (type.empty()) {
        return Result<WebRtcSignalingCommand>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC signaling type is missing"));
    }

    WebRtcSignalingCommand command;
    command.room = stringOrEmpty(signal, "room");
    command.session_id = stringOrEmpty(signal, "session_id");
    command.peer_id = peerId(signal);
    command.source_peer_id = stringOrEmpty(signal, "from");
    command.target_peer_id = stringOrEmpty(signal, "to");
    const std::string explicit_peer_id = stringOrEmpty(signal, "peer_id");
    if (!explicit_peer_id.empty() && !command.source_peer_id.empty() && explicit_peer_id != command.source_peer_id) {
        return Result<WebRtcSignalingCommand>::failure(
            Status::error(ErrorCode::kPermissionDenied, "WebRTC signaling contains conflicting peer routes"));
    }

    if (type == "registered") {
        command.type = WebRtcSignalingCommandType::kRegistered;
        command.local_peer_id = stringOrEmpty(signal, "id");
        if (command.local_peer_id.empty() || command.local_peer_id.size() > kMaxPeerIdBytes ||
            containsAsciiControl(command.local_peer_id) || command.room.empty()) {
            return Result<WebRtcSignalingCommand>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC registered identity is incomplete"));
        }
        auto ice_servers = parseIceServers(signal);
        if (!ice_servers.ok()) {
            return Result<WebRtcSignalingCommand>::failure(ice_servers.status());
        }
        command.ice_servers = ice_servers.takeValue();
        return Result<WebRtcSignalingCommand>::success(std::move(command));
    }

    if (type == "peer_joined") {
        command.type = WebRtcSignalingCommandType::kPeerJoined;
        command.peer_join.session_id = command.session_id;
        command.peer_join.peer_id = command.peer_id;
        command.peer_join.purpose = stringOrEmpty(signal, "purpose");
        command.peer_join.run_id = stringOrEmpty(signal, "run_id");
        command.peer_join.resource_id = stringOrEmpty(signal, "resource_id");
        if (command.peer_join.peer_id.empty() || command.room.empty() || command.peer_join.purpose.empty()) {
            return Result<WebRtcSignalingCommand>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC peer_joined identity is incomplete"));
        }
        auto tracks = parseMediaTracks(signal);
        auto data_channel = parseDataChannel(signal);
        auto token = authorizationToken(signal);
        if (!tracks.ok()) {
            return Result<WebRtcSignalingCommand>::failure(tracks.status());
        }
        if (!data_channel.ok()) {
            return Result<WebRtcSignalingCommand>::failure(data_channel.status());
        }
        if (!token.ok()) {
            return Result<WebRtcSignalingCommand>::failure(token.status());
        }
        command.peer_join.media_tracks = tracks.takeValue();
        command.peer_join.data_channel = data_channel.takeValue();
        command.peer_join.authorization_token = token.takeValue();
        if (command.peer_join.purpose == "video") {
            if (command.peer_join.data_channel.has_value() || !command.peer_join.authorization_token.empty() ||
                !command.peer_join.run_id.empty() || !command.peer_join.resource_id.empty()) {
                return Result<WebRtcSignalingCommand>::failure(Status::error(
                    ErrorCode::kInvalidArgument, "WebRTC video viewer unexpectedly contains control fields"));
            }
        } else if (command.peer_join.purpose == "teleop") {
            if (command.peer_join.session_id.empty() || command.peer_join.resource_id.empty() ||
                command.peer_join.authorization_token.empty() || !command.peer_join.data_channel.has_value()) {
                return Result<WebRtcSignalingCommand>::failure(Status::error(
                    ErrorCode::kInvalidArgument, "WebRTC teleop peer is missing its authorization binding"));
            }
        } else {
            return Result<WebRtcSignalingCommand>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC peer purpose is unsupported"));
        }
        return Result<WebRtcSignalingCommand>::success(std::move(command));
    }

    if (type == "peer_left" || type == "hangup") {
        command.type = WebRtcSignalingCommandType::kPeerLeft;
        command.reason_code = sanitizedReasonCode(stringOrEmpty(signal, "message"), "remote_peer_left");
        if (command.peer_id.empty()) {
            return Result<WebRtcSignalingCommand>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC peer close is missing peer identity"));
        }
        return Result<WebRtcSignalingCommand>::success(std::move(command));
    }

    const nlohmann::json *media_signal = &signal;
    if (type == "signal") {
        const auto nested = signal.find("signal");
        if (nested == signal.end() || !nested->is_object()) {
            return Result<WebRtcSignalingCommand>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC nested signal is missing"));
        }
        media_signal = &(*nested);
    }
    const Status nested_version = validateVersion(*media_signal);
    if (!nested_version.ok()) {
        return Result<WebRtcSignalingCommand>::failure(nested_version);
    }
    const std::string media_type = type == "signal" ? stringOrEmpty(*media_signal, "type") : type;
    if (command.peer_id.empty()) {
        return Result<WebRtcSignalingCommand>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC SDP/ICE signal is missing peer identity"));
    }
    if (media_type == "offer" || media_type == "answer") {
        const Status sdp_status = validateOptionalString(*media_signal, "sdp", kMaxSdpBytes);
        command.sdp = stringOrEmpty(*media_signal, "sdp");
        if (!sdp_status.ok() || command.sdp.empty()) {
            return Result<WebRtcSignalingCommand>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC remote description is invalid"));
        }
        command.type = media_type == "offer" ? WebRtcSignalingCommandType::kRemoteOffer
                                             : WebRtcSignalingCommandType::kRemoteAnswer;
        return Result<WebRtcSignalingCommand>::success(std::move(command));
    }
    if (media_type == "candidate") {
        command.type = WebRtcSignalingCommandType::kRemoteCandidate;
        const auto candidate = media_signal->find("candidate");
        if (candidate == media_signal->end()) {
            return Result<WebRtcSignalingCommand>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC remote candidate is missing"));
        }
        const Status outer_mid_status = validateOptionalSafeString(*media_signal, "sdpMid", kMaxCandidateMidBytes);
        if (!outer_mid_status.ok()) {
            return Result<WebRtcSignalingCommand>::failure(outer_mid_status);
        }
        if (candidate->is_string()) {
            command.candidate = candidate->get_ref<const std::string &>();
            command.candidate_mid = stringOrEmpty(*media_signal, "sdpMid");
        } else if (candidate->is_object()) {
            const Status nested_mid_status = validateOptionalSafeString(*candidate, "sdpMid", kMaxCandidateMidBytes);
            if (!nested_mid_status.ok()) {
                return Result<WebRtcSignalingCommand>::failure(nested_mid_status);
            }
            command.candidate = stringOrEmpty(*candidate, "candidate");
            command.candidate_mid = stringOrEmpty(*candidate, "sdpMid");
            if (command.candidate_mid.empty()) {
                command.candidate_mid = stringOrEmpty(*media_signal, "sdpMid");
            }
        }
        if (command.candidate.empty() || command.candidate.size() > kMaxCandidateBytes ||
            containsAsciiControl(command.candidate) || command.candidate_mid.size() > kMaxCandidateMidBytes ||
            containsAsciiControl(command.candidate_mid)) {
            return Result<WebRtcSignalingCommand>::failure(
                Status::error(ErrorCode::kInvalidArgument, "WebRTC remote candidate is invalid"));
        }
        return Result<WebRtcSignalingCommand>::success(std::move(command));
    }

    return Result<WebRtcSignalingCommand>::failure(
        Status::error(ErrorCode::kInvalidArgument, "WebRTC signaling type is unsupported"));
}

Result<std::string>
LibDataChannelSignalingCodec::makeDescriptionReport(const std::string &local_peer_id, const std::string &room,
                                                    const std::string &session_id, const std::string &peer_id,
                                                    const std::string &description_type, const std::string &sdp) const {
    if (local_peer_id.empty() || room.empty() || peer_id.empty() || sdp.empty() || sdp.size() > kMaxSdpBytes ||
        (description_type != "offer" && description_type != "answer")) {
        return Result<std::string>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC local description report is incomplete"));
    }
    nlohmann::json report = routeEnvelope(local_peer_id, room, session_id, peer_id);
    report["signal"] = {{"type", description_type}, {"sdp", normalizeSdpLineEndings(sdp)}, {"version", 1}};
    return checkedDump(std::move(report), max_payload_bytes_);
}

Result<std::string>
LibDataChannelSignalingCodec::makeCandidateReport(const std::string &local_peer_id, const std::string &room,
                                                  const std::string &session_id, const std::string &peer_id,
                                                  const std::string &candidate, const std::string &mid) const {
    if (local_peer_id.empty() || room.empty() || peer_id.empty() || candidate.empty() ||
        candidate.size() > kMaxCandidateBytes || mid.size() > kMaxCandidateMidBytes ||
        containsAsciiControl(candidate) || containsAsciiControl(mid)) {
        return Result<std::string>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC local candidate report is incomplete"));
    }
    nlohmann::json report = routeEnvelope(local_peer_id, room, session_id, peer_id);
    report["signal"] = {{"type", "candidate"}, {"candidate", candidate}, {"version", 1}};
    if (!mid.empty()) {
        report["signal"]["sdpMid"] = mid;
    }
    return checkedDump(std::move(report), max_payload_bytes_);
}

Result<std::string> LibDataChannelSignalingCodec::makePeerLeftReport(const std::string &local_peer_id,
                                                                     const std::string &room,
                                                                     const std::string &session_id,
                                                                     const std::string &peer_id,
                                                                     const std::string &reason_code) const {
    if (local_peer_id.empty() || room.empty() || peer_id.empty()) {
        return Result<std::string>::failure(
            Status::error(ErrorCode::kInvalidArgument, "WebRTC peer close report is incomplete"));
    }
    nlohmann::json report{{"type", "peer_left"},
                          {"version", 1},
                          {"from", local_peer_id},
                          {"to", peer_id},
                          {"peer_id", peer_id},
                          {"room", room},
                          {"message", sanitizedReasonCode(reason_code, "local_close")}};
    if (!session_id.empty()) {
        report["session_id"] = session_id;
    }
    return checkedDump(std::move(report), max_payload_bytes_);
}

}  // namespace astrabot::rtc::transport
