// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

namespace astrabot::teleop {

/**
 * @brief 容量固定为 1 的 latest-wins mailbox。
 *
 * push() 与 take() 可由不同线程调用。新值覆盖未消费旧值时递增 overwriteCount()，不会形成历史动作队列。
 */
template <typename T> class LatestMailbox {
  public:
    /** @brief 写入最新值，必要时覆盖尚未消费的旧值。 */
    void push(T value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (value_.has_value()) {
            ++overwrite_count_;
        }
        value_ = std::move(value);
    }

    /** @brief 取走当前最新值；没有值时返回 std::nullopt。 */
    std::optional<T> take() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!value_.has_value()) {
            return std::nullopt;
        }
        std::optional<T> result(std::move(value_));
        value_.reset();
        return result;
    }

    /** @brief 清空未消费值。 */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        value_.reset();
    }

    /** @brief 返回累计覆盖次数。 */
    std::uint64_t overwriteCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return overwrite_count_;
    }

  private:
    mutable std::mutex mutex_;
    std::optional<T> value_;
    std::uint64_t overwrite_count_{0};
};

}  // namespace astrabot::teleop
