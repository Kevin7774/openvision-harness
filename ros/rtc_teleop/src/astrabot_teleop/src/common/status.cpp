// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/common/status.h"

#include <utility>

namespace astrabot::teleop {

Status Status::success() {
    return Status(ErrorCode::kOk, {});
}

Status Status::error(const ErrorCode code, std::string message) {
    return Status(code, std::move(message));
}

bool Status::ok() const {
    return code_ == ErrorCode::kOk;
}

ErrorCode Status::code() const {
    return code_;
}

const std::string &Status::message() const {
    return message_;
}

Status::Status(const ErrorCode code, std::string message) : code_(code), message_(std::move(message)) {}

}  // namespace astrabot::teleop
