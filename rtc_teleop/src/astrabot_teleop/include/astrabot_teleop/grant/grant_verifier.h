// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "astrabot_teleop/common/result.hpp"
#include "astrabot_teleop/config/teleop_config.h"

namespace astrabot::teleop {

/** @brief 已验签且完成字段绑定的 Teleop grant claims。 */
struct GrantClaims {
    std::uint32_t version{0};
    std::string key_id;
    std::string session_id;
    std::string run_id;
    std::string user_id;
    std::string device_id;
    std::string resource_id;
    /** @brief grant wire claim，单位为 Unix epoch 秒；不得直接与 steady clock 比较。 */
    std::uint64_t expires_at_epoch_sec{0};
    std::string nonce;
};

/** @brief grant 必须绑定的端侧上下文。 */
struct ExpectedGrantBinding {
    std::string session_id;
    std::string run_id;
    std::string device_id;
    std::string resource_id;
};

/**
 * @brief Ed25519 Teleop grant 验证器和有界、进程生命周期内不遗忘 nonce 的 replay guard。
 *
 * token 只在该边界内短暂存在，不会进入日志、普通 topic、metrics 或持久化。已消费 nonce 不因 grant 到期而删除；缓存
 * 达到上限时 fail closed，避免旧 session/token 在进程内复活。该类可并发调用。
 */
class GrantVerifier {
  public:
    explicit GrantVerifier(std::size_t max_nonce_entries = 1024);
    ~GrantVerifier();

    GrantVerifier(const GrantVerifier &) = delete;
    GrantVerifier &operator=(const GrantVerifier &) = delete;

    /** @brief 加载公钥；OpenSSL 或合法公钥不可用时返回显式失败。 */
    Status initialize(const std::vector<GrantPublicKeyConfig> &public_keys);

    /** @brief 验签、校验绑定与过期时间，并原子消费 nonce。 */
    Result<GrantClaims> verifyAndConsume(const std::string &token, const ExpectedGrantBinding &expected,
                                         std::uint64_t system_now_sec);

    /**
     * @brief 计算不泄漏原始 grant 的 SHA-256 指纹。
     *
     * 指纹只用于判断当前活动授权服务重试是否携带完全相同的 token；不得用它替代验签。
     */
    static Result<std::string> tokenFingerprint(const std::string &token);

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace astrabot::teleop
