#pragma once

#include <string>
#include <utility>

namespace astrabot::rtc {

/**
 * @brief RTC 模块统一错误码。
 */
enum class ErrorCode {
    kOk = 0,
    kInvalidArgument,
    kAlreadyExists,
    kNotFound,
    kResourceExhausted,
    kFailedPrecondition,
    kPermissionDenied,
    kPayloadTooLarge,
    kUnavailable,
    kInternal,
};

/**
 * @brief 表达无返回值操作的成功或失败状态。
 */
class Status {
  public:
    /**
     * @brief 创建成功状态。
     */
    static Status success() {
        return Status();
    }

    /**
     * @brief 创建包含错误码和上下文的失败状态。
     */
    static Status error(ErrorCode code, std::string message) {
        return Status(code, std::move(message));
    }

    /**
     * @brief 判断操作是否成功。
     */
    bool ok() const {
        return code_ == ErrorCode::kOk;
    }

    /**
     * @brief 返回稳定错误码。
     */
    ErrorCode code() const {
        return code_;
    }

    /**
     * @brief 返回不包含敏感输入的错误描述。
     */
    const std::string &message() const {
        return message_;
    }

  private:
    Status() = default;
    Status(ErrorCode code, std::string message) : code_(code), message_(std::move(message)) {}

    ErrorCode code_{ErrorCode::kOk};
    std::string message_;
};

}  // namespace astrabot::rtc
