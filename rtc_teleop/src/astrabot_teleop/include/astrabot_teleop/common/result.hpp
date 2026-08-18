// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <optional>
#include <utility>

#include "astrabot_teleop/common/status.h"

namespace astrabot::teleop {

/**
 * @brief 携带成功值或显式 Status 的轻量结果类型。
 *
 * 调用方必须先检查 ok()，再访问 value()。该类型不使用异常传递错误。
 */
template <typename T> class Result {
  public:
    /** @brief 构造成功结果。 */
    static Result<T> success(T value) {
        return Result<T>(std::move(value));
    }

    /** @brief 构造失败结果。 */
    static Result<T> failure(Status status) {
        return Result<T>(std::move(status));
    }

    /** @brief 返回结果是否成功。 */
    bool ok() const {
        return value_.has_value();
    }

    /** @brief 返回失败状态；成功时返回成功状态。 */
    const Status &status() const {
        return status_;
    }

    /** @brief 返回只读成功值；调用前必须检查 ok()。 */
    const T &value() const {
        return *value_;
    }

    /** @brief 移出成功值；调用前必须检查 ok()。 */
    T takeValue() {
        return std::move(*value_);
    }

  private:
    explicit Result(T value) : value_(std::move(value)), status_(Status::success()) {}
    explicit Result(Status status) : status_(std::move(status)) {}

    std::optional<T> value_;
    Status status_{Status::error(ErrorCode::kInternal, "result has no value")};
};

}  // namespace astrabot::teleop
