#pragma once

#include <memory>

#include "astrabot_rtc/common/result.hpp"
#include "astrabot_rtc/config/rtc_config.h"
#include "astrabot_rtc/transport/webrtc/webrtc_transport.h"

namespace astrabot::rtc::transport {

/**
 * @brief 根据严格配置创建编译进当前二进制的 WebRTC backend。
 */
class WebRtcTransportFactory {
  public:
    /**
     * @brief 创建 backend；请求未编译的 backend 时 fail closed。
     */
    static Result<std::unique_ptr<IWebRtcTransport>> create(const config::RtcConfig &config);
};

}  // namespace astrabot::rtc::transport
