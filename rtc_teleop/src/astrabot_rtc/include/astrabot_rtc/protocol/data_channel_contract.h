#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "astrabot_rtc/common/status.h"

namespace astrabot::rtc::protocol {

/**
 * @brief RTC 层可见的通用 DataChannel payload。
 *
 * RTC 只校验大小和路由身份，不解释 payload 的业务语义。
 */
struct DataChannelPacket {
    std::string session_id;
    std::string peer_id;
    std::string channel_label;
    std::uint64_t receive_steady_time_ns{0U};
    std::vector<std::uint8_t> payload;
};

/**
 * @brief 对通用 DataChannel 标识和 payload 实施固定资源边界。
 */
class DataChannelContract {
  public:
    explicit DataChannelContract(std::size_t max_payload_bytes);

    /**
     * @brief 校验 packet 的稳定路由字段和 payload 大小。
     */
    Status validate(const DataChannelPacket &packet) const;

    /**
     * @brief 返回配置后的 payload 上限。
     */
    std::size_t maxPayloadBytes() const;

  private:
    std::size_t max_payload_bytes_;
};

}  // namespace astrabot::rtc::protocol
