#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "astrabot_rtc/common/clock.h"
#include "astrabot_rtc/common/status.h"
#include "astrabot_rtc/protocol/data_channel_contract.h"

namespace astrabot::rtc::session {

/**
 * @brief 一个 DataChannel 的不可歧义路由 key。
 */
struct DataChannelKey {
    std::string session_id;
    std::string peer_id;
    std::string channel_label;
};

/**
 * @brief 授权结果；expires_at 使用机器人本机 steady clock 纳秒，必须是未来的非零 deadline。
 */
struct ChannelAuthorization {
    bool allowed{false};
    std::string reason_code;
    std::uint64_t expires_at{0U};
};

/**
 * @brief DataChannelRouter 的丢弃与覆盖计数快照。
 */
struct DataChannelRouterMetrics {
    std::uint64_t accepted_packets{0U};
    std::uint64_t overwritten_packets{0U};
    std::uint64_t unauthorized_packets{0U};
    std::uint64_t oversized_packets{0U};
    std::uint64_t stopped_packets{0U};
    std::uint64_t expired_queued_packets{0U};
};

/**
 * @brief 授权后才允许入队的有界 latest-wins DataChannel 路由器。
 *
 * 每个已授权 channel 最多保存一个最新 packet。所有方法线程安全；start/stop 幂等，stop 会清除授权和待转发数据。
 */
class DataChannelRouter {
  public:
    DataChannelRouter(std::size_t max_channels, std::size_t max_payload_bytes, std::shared_ptr<const IClock> clock);

    /**
     * @brief 启动接收；重复调用返回成功。
     */
    Status start();

    /**
     * @brief 停止并清空授权和缓存；重复调用安全。
     */
    void stop();

    /**
     * @brief 记录上层授权服务的允许结果。
     */
    Status authorize(const DataChannelKey &key, const ChannelAuthorization &authorization);

    /**
     * @brief 查询 channel 当前是否仍处于有效授权期。
     */
    bool isAuthorized(const DataChannelKey &key) const;

    /**
     * @brief 撤销 channel 授权和待转发 packet。
     */
    Status revoke(const DataChannelKey &key);

    /**
     * @brief 撤销指定 peer 的全部 channel 授权和待转发 packet。
     *
     * @return 实际撤销的 channel 数量。
     */
    std::size_t revokePeer(const std::string &session_id, const std::string &peer_id);

    /**
     * @brief 校验并压入最新 packet；未授权、过期或超限输入会被拒绝。
     */
    Status pushIncoming(protocol::DataChannelPacket packet);

    /**
     * @brief 取出指定 channel 当前最新 packet；授权在出队前过期时删除 channel 并丢弃缓存。
     */
    std::optional<protocol::DataChannelPacket> takeLatest(const DataChannelKey &key);

    /**
     * @brief 一次取出全部仍在授权期内的最新 packet，返回数量不超过 max_channels。
     */
    std::vector<protocol::DataChannelPacket> takeAllLatest();

    /**
     * @brief 返回稳定计数快照。
     */
    DataChannelRouterMetrics metrics() const;

    /**
     * @brief 返回当前授权 channel 数量。
     */
    std::size_t authorizedChannelCount() const;

  private:
    struct AuthorizedChannel {
        DataChannelKey key;
        std::uint64_t expires_at{0U};
        std::optional<protocol::DataChannelPacket> latest;
    };

    static std::string makeKey(const DataChannelKey &key);
    static DataChannelKey packetKey(const protocol::DataChannelPacket &packet);

    const std::size_t max_channels_;
    const protocol::DataChannelContract contract_;
    const std::shared_ptr<const IClock> clock_;
    mutable std::mutex mutex_;
    bool running_{false};
    std::unordered_map<std::string, AuthorizedChannel> channels_;
    DataChannelRouterMetrics metrics_;
};

}  // namespace astrabot::rtc::session
