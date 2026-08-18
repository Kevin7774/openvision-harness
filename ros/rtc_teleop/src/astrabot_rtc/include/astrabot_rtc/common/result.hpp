#pragma once

#include <cassert>
#include <optional>
#include <utility>

#include "astrabot_rtc/common/status.h"

namespace astrabot::rtc {

/**
 * @brief 在不使用异常的前提下携带返回值或失败状态。
 *
 * @tparam T 成功结果类型。
 */
template <typename T> class Result {
  public:
    /**
     * @brief 创建成功结果。
     */
    static Result<T> success(T value) {
        return Result<T>(std::move(value));
    }

    /**
     * @brief 创建失败结果。
     */
    static Result<T> failure(Status status) {
        return Result<T>(std::move(status));
    }

    /**
     * @brief 判断结果是否包含成功值。
     */
    bool ok() const {
        return status_.ok();
    }

    /**
     * @brief 返回操作状态。
     */
    const Status &status() const {
        return status_;
    }

    /**
     * @brief 读取成功值；调用前必须先检查 ok()。
     */
    const T &value() const {
        assert(value_.has_value());
        return *value_;
    }

    /**
     * @brief 移出成功值；调用前必须先检查 ok()。
     */
    T takeValue() {
        assert(value_.has_value());
        return std::move(*value_);
    }

  private:
    explicit Result(T value) : value_(std::move(value)) {}
    explicit Result(Status status) : status_(std::move(status)) {}

    Status status_{Status::success()};
    std::optional<T> value_;
};

}  // namespace astrabot::rtc
