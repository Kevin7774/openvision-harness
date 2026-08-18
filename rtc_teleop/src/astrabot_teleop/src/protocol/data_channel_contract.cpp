// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/protocol/data_channel_contract.h"

namespace astrabot::teleop {

Result<DataChannelContract> DataChannelContracts::find(const std::string &label) {
    if (label == kLabel) {
        return Result<DataChannelContract>::success(DataChannelContract{label, false, 20, 16384});
    }
    return Result<DataChannelContract>::failure(
        Status::error(ErrorCode::kInvalidArgument, "unsupported teleop data channel label"));
}

}  // namespace astrabot::teleop
