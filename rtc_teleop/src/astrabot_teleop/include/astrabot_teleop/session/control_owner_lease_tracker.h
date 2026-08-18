// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "astrabot_teleop/common/result.hpp"

namespace astrabot::teleop {

/** @brief arbitration owner 的完整身份绑定。 */
struct ControlOwnerIdentity {
    std::string session_id;
    std::string run_id;
    std::string resource_id;
    std::string source_id;
};

/** @brief 已确认且尚未本地撤销的 arbitration owner lease。 */
struct ControlOwnerLease {
    ControlOwnerIdentity identity;
    std::uint64_t owner_epoch{0U};
    std::uint64_t expires_at_steady_ns{0U};
};

/** @brief 异步 owner 请求类型。 */
enum class ControlOwnerOperationKind : std::uint8_t {
    kAcquire = 0,
    kRenew = 1,
    kRelease = 2,
};

/** @brief 用于拒绝超时、取消或旧 session callback 的异步操作令牌。 */
struct ControlOwnerOperation {
    ControlOwnerOperationKind kind{ControlOwnerOperationKind::kAcquire};
    std::uint64_t operation_id{0U};
    std::uint64_t generation{0U};
    ControlOwnerIdentity identity;
    std::uint64_t owner_epoch{0U};
    std::uint64_t deadline_steady_ns{0U};
};

/**
 * @brief 维护单一 owner lease 与单个有界 pending 操作，不执行 ROS IO。
 *
 * release 开始时立即本地撤销 epoch；即使服务超时，生产命令也不会继续携带旧 epoch。所有完成回调都必须携带
 * begin*() 返回的令牌，generation 不匹配时按旧 callback 拒绝。
 */
class ControlOwnerLeaseTracker {
  public:
    /** @brief 开始 acquire；已有 lease 或 pending 时 fail closed。 */
    Result<ControlOwnerOperation> beginAcquire(const ControlOwnerIdentity &identity, std::uint64_t steady_now_ns,
                                               std::uint64_t timeout_ns);

    /** @brief 接受 acquire 响应；epoch 为 0、过期响应或旧 callback 均拒绝。 */
    Status completeAcquire(const ControlOwnerOperation &operation, bool granted, std::uint64_t owner_epoch,
                           std::uint64_t expires_at_steady_ns, std::uint64_t steady_now_ns);

    /** @brief 为当前 lease 开始 renew。 */
    Result<ControlOwnerOperation> beginRenew(std::uint64_t steady_now_ns, std::uint64_t timeout_ns);

    /** @brief 接受 renew 响应；失败时立即撤销本地 lease。 */
    Status completeRenew(const ControlOwnerOperation &operation, bool renewed, std::uint64_t owner_epoch,
                         std::uint64_t expires_at_steady_ns, std::uint64_t steady_now_ns);

    /** @brief 开始 release，并在发请求前立即本地撤销 epoch。 */
    Result<ControlOwnerOperation> beginRelease(std::uint64_t steady_now_ns, std::uint64_t timeout_ns);

    /** @brief 接受 release 响应；本地 epoch 无论结果如何都保持撤销。 */
    Status completeRelease(const ControlOwnerOperation &operation, bool released);

    /** @brief 取消当前 pending；renew 取消时保留 lease，调用方可紧接着 release。 */
    std::optional<ControlOwnerOperation> cancelPending();

    /** @brief 到期时移除 pending；renew 超时会同步撤销 lease。 */
    std::optional<ControlOwnerOperation> expirePending(std::uint64_t steady_now_ns);

    /** @brief 返回指定 steady 时间仍有效的 lease 快照。 */
    std::optional<ControlOwnerLease> activeLease(std::uint64_t steady_now_ns) const;

    /** @brief 返回 lease 快照，不隐式忽略过期，供诊断与显式 fail-closed 判断。 */
    std::optional<ControlOwnerLease> leaseSnapshot() const;

    /** @brief 返回当前 pending 操作快照。 */
    std::optional<ControlOwnerOperation> pendingOperation() const;

    /** @brief 撤销 lease 和 pending，并使所有旧 callback 失效。 */
    void invalidate();

  private:
    Result<ControlOwnerOperation> makeOperationLocked(ControlOwnerOperationKind kind,
                                                      const ControlOwnerIdentity &identity, std::uint64_t owner_epoch,
                                                      std::uint64_t steady_now_ns, std::uint64_t timeout_ns);
    bool matchesPendingLocked(const ControlOwnerOperation &operation) const;
    void advanceGenerationLocked();

    mutable std::mutex mutex_;
    std::optional<ControlOwnerLease> lease_;
    std::optional<ControlOwnerOperation> pending_;
    std::uint64_t next_operation_id_{1U};
    std::uint64_t generation_{1U};
};

}  // namespace astrabot::teleop
