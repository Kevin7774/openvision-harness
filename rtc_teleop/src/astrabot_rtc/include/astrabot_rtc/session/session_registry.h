#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "astrabot_rtc/common/status.h"

namespace astrabot::rtc::session {

/**
 * @brief RTC transport 的通用 peer 状态，不包含上层应用状态。
 */
enum class PeerState : std::uint8_t {
    kUnknown = 0,
    kConnecting = 1,
    kConnected = 2,
    kDisconnected = 3,
    kFailed = 4,
    kClosed = 5,
};

/**
 * @brief 一个远端 PeerConnection 的稳定身份和能力快照。
 */
struct PeerDescriptor {
    std::string session_id;
    std::string peer_id;
    std::string purpose;
    std::string run_id;
    std::string resource_id;
    std::vector<std::string> media_tracks;
    std::vector<std::string> data_channels;
    PeerState state{PeerState::kUnknown};
    std::uint64_t steady_time_ns{0U};
    std::string reason_code;
};

/**
 * @brief 线程安全地维护有界 peer 集合和 viewer 数量。
 *
 * media_tracks 非空的 peer 被计入 viewer；重复 peer key 会被拒绝，不会静默覆盖。平台 video-only viewer 可以没有
 * application session_id，此时以空 session_id + peer_id 建立稳定 key；任何控制 peer 仍必须有 session_id。
 */
class SessionRegistry {
  public:
    SessionRegistry(std::size_t max_peers, std::size_t max_viewers);

    /**
     * @brief 注册一个新 peer。
     */
    Status addPeer(const PeerDescriptor &peer);

    /**
     * @brief 更新已注册 peer 的 transport 状态和 DataChannel 列表。
     *
     * purpose、run、resource 和媒体 track 在 PeerConnection 生命周期内不可改变。
     */
    Status updatePeer(const PeerDescriptor &peer);

    /**
     * @brief 更新已注册 peer 的连接状态。
     */
    Status updatePeerState(const std::string &session_id, const std::string &peer_id, PeerState state,
                           std::uint64_t steady_time_ns, const std::string &reason_code);

    /**
     * @brief 删除 peer；目标不存在时返回 NotFound。
     */
    Status removePeer(const std::string &session_id, const std::string &peer_id);

    /**
     * @brief 查询 peer 快照。
     */
    std::optional<PeerDescriptor> findPeer(const std::string &session_id, const std::string &peer_id) const;

    /**
     * @brief 返回当前全部 peer 的稳定快照。
     */
    std::vector<PeerDescriptor> snapshot() const;

    /**
     * @brief 返回 peer 数量。
     */
    std::size_t peerCount() const;

    /**
     * @brief 返回媒体 viewer 数量。
     */
    std::size_t viewerCount() const;

    /**
     * @brief 清空全部运行态；用于 runtime stop。
     */
    void clear();

  private:
    static std::string makeKey(const std::string &session_id, const std::string &peer_id);

    const std::size_t max_peers_;
    const std::size_t max_viewers_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, PeerDescriptor> peers_;
    std::size_t viewer_count_{0U};
};

}  // namespace astrabot::rtc::session
