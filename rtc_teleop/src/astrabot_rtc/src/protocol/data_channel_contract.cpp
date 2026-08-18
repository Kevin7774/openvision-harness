#include "astrabot_rtc/protocol/data_channel_contract.h"

namespace astrabot::rtc::protocol {

DataChannelContract::DataChannelContract(std::size_t max_payload_bytes) : max_payload_bytes_(max_payload_bytes) {}

Status DataChannelContract::validate(const DataChannelPacket &packet) const {
    if (packet.session_id.empty() || packet.peer_id.empty() || packet.channel_label.empty()) {
        return Status::error(ErrorCode::kInvalidArgument, "data channel packet has an empty route identity");
    }
    if (packet.payload.size() > max_payload_bytes_) {
        return Status::error(ErrorCode::kPayloadTooLarge, "data channel payload exceeds the configured limit");
    }
    return Status::success();
}

std::size_t DataChannelContract::maxPayloadBytes() const {
    return max_payload_bytes_;
}

}  // namespace astrabot::rtc::protocol
