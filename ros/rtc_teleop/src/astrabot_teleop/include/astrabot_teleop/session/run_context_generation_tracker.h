// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <deque>
#include <string>
#include <unordered_set>

namespace astrabot::teleop {

/** @brief 描述一次 run context publisher generation 观察结果。 */
enum class RunContextGenerationObservation {
    kInvalid,
    kInitial,
    kCurrent,
    kChanged,
    kRetired,
};

/**
 * @brief 跟踪当前 run context 发布代次，并拒绝最近已经退休的代次重新生效。
 *
 * UUID 只能判断相等，不能比较先后顺序。本对象按观察顺序保存最近 32 个已退休 generation，避免 A→B 后迟到的
 * transient-local A 被误认成新的发布方换代。对象本身非线程安全，调用方必须在同一锁下串行调用。
 */
class RunContextGenerationTracker final {
  public:
    static constexpr std::size_t kMaxRetiredGenerations = 32U;

    /**
     * @brief 观察一个 generation；切换到未见过的新 generation 时自动退休当前 generation。
     *
     * @param publisher_generation 已由边界层完成长度和字符校验的非空 generation。
     * @return 空值返回 kInvalid；已退休值返回 kRetired 且不改变当前 generation。
     */
    RunContextGenerationObservation observe(const std::string &publisher_generation);

    /** @brief 返回当前 generation；尚未观察到有效值时为空。 */
    const std::string &currentGeneration() const;

    /** @brief 返回当前保留的已退休 generation 数量。 */
    std::size_t retiredGenerationCount() const;

    /** @brief 清空当前 generation 和有界退休历史。 */
    void clear();

  private:
    void retireCurrentGeneration();

    std::string current_generation_;
    std::deque<std::string> retired_generation_order_;
    std::unordered_set<std::string> retired_generations_;
};

}  // namespace astrabot::teleop
