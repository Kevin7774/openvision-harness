#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "astrabot_rtc/common/status.h"
#include "astrabot_rtc/media/video_frame.h"
#include "astrabot_rtc/protocol/data_channel_contract.h"
#include "astrabot_rtc/session/session_registry.h"

namespace astrabot::rtc::transport {

/**
 * @brief transport 在创建 DataChannel 后上报的通用授权请求。
 *
 * 该请求不代表 SDK channel 已经 open。authorization_token 只允许在授权 service 请求的短生命周期内存在，不得写入日志或
 * 持久化状态。
 */
struct DataChannelOpenRequest {
    std::string session_id;
    std::string peer_id;
    std::string purpose;
    std::string run_id;
    std::string resource_id;
    std::string channel_label;
    std::string authorization_token;
};

/**
 * @brief WebRTC SDK 确认 DataChannel 已实际进入 open 状态的通用事件。
 *
 * 该事件不携带 grant；runtime 只有在同一路由授权成功后，才会向上层应用发布可连接事件。
 */
struct DataChannelReadyEvent {
    std::string session_id;
    std::string peer_id;
    std::string channel_label;
};

/**
 * @brief transport 对外声明的真实能力。
 */
struct WebRtcTransportCapabilities {
    std::string backend;
    bool peer_connections{false};
    bool media_tracks{false};
    bool data_channels{false};
};

/**
 * @brief transport 高频发送路径的累计指标快照。
 *
 * 每个计数器在 transport 对象生命周期内单调递增。媒体拥塞按 peer purpose 区分 Teleop 与普通 viewer，
 * 便于确认 WebOps 降级没有拖慢 Quest 链路。
 */
struct WebRtcTransportMetrics {
    std::uint64_t media_peer_sends{0U};
    std::uint64_t media_peer_bytes{0U};
    std::uint64_t media_teleop_congestion_drops{0U};
    std::uint64_t media_viewer_congestion_drops{0U};
    std::uint64_t media_buffer_query_failures{0U};
    std::uint64_t reconnect_count{0U};
};

/**
 * @brief WebRTC SDK adapter 向 runtime 提交的回调集合。
 */
struct WebRtcTransportCallbacks {
    std::function<void(const session::PeerDescriptor &)> on_peer_event;
    std::function<void(DataChannelOpenRequest)> on_data_channel_open;
    std::function<void(DataChannelReadyEvent)> on_data_channel_ready;
    std::function<void(protocol::DataChannelPacket)> on_data_channel_packet;
    std::function<void(std::string)> on_signaling_report;
};

/**
 * @brief 隔离具体 WebRTC SDK、线程和 callback 生命周期的最小 transport seam。
 *
 * 上层 runtime 不得依赖 libdatachannel、GStreamer 或其他 SDK 类型。实现必须在 stop 返回前停止新回调。
 */
class IWebRtcTransport {
  public:
    virtual ~IWebRtcTransport() = default;

    /**
     * @brief 启动 transport；重复调用语义由实现保持幂等。
     */
    virtual Status start(WebRtcTransportCallbacks callbacks) = 0;

    /**
     * @brief 停止 transport 并注销回调。
     */
    virtual void stop() = 0;

    /**
     * @brief 处理 Gateway 转发的原始信令 envelope。
     */
    virtual Status handleSignaling(const std::string &payload) = 0;

    /**
     * @brief 关闭 peer 或指定 channel；channel_label 为空表示关闭整个 peer。
     */
    virtual Status closePeer(const std::string &session_id, const std::string &peer_id,
                             const std::string &channel_label, const std::string &reason_code) = 0;

    /**
     * @brief 向已打开的通用 DataChannel 发送一个有界 payload。
     */
    virtual Status sendDataChannelPacket(const protocol::DataChannelPacket &packet) = 0;

    /**
     * @brief 将共享编码帧送入真实媒体 sender。
     */
    virtual Status sendEncodedFrame(std::shared_ptr<const media::EncodedVideoFrame> frame) = 0;

    /**
     * @brief 返回实现实际具备的能力，不允许用配置伪造 true。
     */
    virtual WebRtcTransportCapabilities capabilities() const = 0;

    /**
     * @brief 返回无阻塞的累计指标快照。
     */
    virtual WebRtcTransportMetrics metrics() const = 0;
};

}  // namespace astrabot::rtc::transport
