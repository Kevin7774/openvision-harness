#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "astrabot_rtc/common/result.hpp"

namespace astrabot::rtc::transport {

/**
 * @brief libdatachannel adapter 支持的下行信令类型。
 */
enum class WebRtcSignalingCommandType {
    kRegistered,
    kPeerJoined,
    kPeerLeft,
    kRemoteOffer,
    kRemoteAnswer,
    kRemoteCandidate,
};

/**
 * @brief 平台下发的一组 STUN/TURN 地址及可选凭据。
 */
struct WebRtcIceServerSettings {
    std::vector<std::string> urls;
    std::string username;
    std::string credential;
};

/**
 * @brief 平台冻结的 DataChannel label 和可靠性参数。
 */
struct WebRtcDataChannelSettings {
    std::string label;
    bool ordered{true};
    std::uint32_t max_packet_lifetime_ms{0U};
    std::size_t max_payload_bytes{16384U};
};

/**
 * @brief 一个 peer_joined 信令携带的会话身份和能力。
 */
struct WebRtcPeerJoinSettings {
    std::string session_id;
    std::string peer_id;
    std::string purpose;
    std::string run_id;
    std::string resource_id;
    std::vector<std::string> media_tracks;
    std::optional<WebRtcDataChannelSettings> data_channel;
    std::string authorization_token;
};

/**
 * @brief 已严格校验、可交给 transport 执行的信令命令。
 */
struct WebRtcSignalingCommand {
    WebRtcSignalingCommandType type{WebRtcSignalingCommandType::kRegistered};
    std::string local_peer_id;
    std::string room;
    std::vector<WebRtcIceServerSettings> ice_servers;
    WebRtcPeerJoinSettings peer_join;
    std::string session_id;
    std::string peer_id;
    std::string source_peer_id;
    std::string target_peer_id;
    std::string sdp;
    std::string candidate;
    std::string candidate_mid;
    std::string reason_code;
};

/**
 * @brief 解析和生成 Gateway 已冻结的 WebRTC signaling v1 JSON。
 *
 * 解析器默认拒绝未知类型、超限字段和不完整身份；不会在错误信息中包含 SDP、ICE credential 或 grant。
 */
class LibDataChannelSignalingCodec {
  public:
    explicit LibDataChannelSignalingCodec(std::size_t max_payload_bytes);

    /**
     * @brief 解析一条下行信令。
     */
    Result<WebRtcSignalingCommand> parse(const std::string &payload) const;

    /**
     * @brief 生成 device 到指定 peer 的 offer/answer 上行信令。
     */
    Result<std::string> makeDescriptionReport(const std::string &local_peer_id, const std::string &room,
                                              const std::string &session_id, const std::string &peer_id,
                                              const std::string &description_type, const std::string &sdp) const;

    /**
     * @brief 生成一条 trickle ICE candidate 上行信令。
     */
    Result<std::string> makeCandidateReport(const std::string &local_peer_id, const std::string &room,
                                            const std::string &session_id, const std::string &peer_id,
                                            const std::string &candidate, const std::string &mid) const;

    /**
     * @brief 生成主动关闭 peer 的低频上行信令。
     */
    Result<std::string> makePeerLeftReport(const std::string &local_peer_id, const std::string &room,
                                           const std::string &session_id, const std::string &peer_id,
                                           const std::string &reason_code) const;

  private:
    std::size_t max_payload_bytes_;
};

}  // namespace astrabot::rtc::transport
