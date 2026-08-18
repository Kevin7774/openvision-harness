// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <string>

namespace astrabot::teleop {

/**
 * @brief 模块内部稳定错误码。
 */
enum class ErrorCode {
    kOk = 0,
    kInvalidArgument,
    kUnavailable,
    kUnauthorized,
    kConflict,
    kDataLoss,
    kFailedPrecondition,
    kDeadlineExceeded,
    kResourceExhausted,
    kInternal,
};

/**
 * @brief 不依赖异常的错误返回值。
 */
class Status {
  public:
    /** @brief 构造成功状态。 */
    static Status success();

    /** @brief 构造带错误码和可安全记录消息的失败状态。 */
    static Status error(ErrorCode code, std::string message);

    /** @brief 返回操作是否成功。 */
    bool ok() const;

    /** @brief 返回稳定错误码。 */
    ErrorCode code() const;

    /** @brief 返回不包含密钥或 token 的错误说明。 */
    const std::string &message() const;

  private:
    Status(ErrorCode code, std::string message);

    ErrorCode code_{ErrorCode::kOk};
    std::string message_;
};

}  // namespace astrabot::teleop
