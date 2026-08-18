// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

#include "astrabot_teleop/common/result.hpp"
#include "astrabot_teleop/mapping/command_mapper.h"

namespace astrabot::teleop {

/** @brief 机器人末端位姿采样的新鲜度与四元数校验配置。 */
struct ArmOriginMappingConfig {
    std::uint64_t max_robot_pose_age_ns{100000000U};
    double quaternion_norm_tolerance{0.05};
};

/**
 * @brief 将 Quest 相对手臂位姿映射为机器人 base frame 下的绝对目标。
 *
 * 每只手在 deadman 上升沿独立捕获一次机器人当前末端位姿，并在本次按压期间固定使用该原点。
 * 机器人位姿过期、缺失或非法时默认拒绝；头部控制尚未完成机器人原点闭环，因此本类始终关闭 head_valid。
 */
class ArmOriginMapper {
  public:
    explicit ArmOriginMapper(ArmOriginMappingConfig config);

    /** @brief 更新左右末端在机器人 base frame 下的最新位姿快照。 */
    Status updateRobotPose(const PoseSample &right_arm, const PoseSample &left_arm,
                           std::uint64_t receive_steady_time_ns);

    /**
     * @brief 根据每臂 deadman 状态捕获原点并生成绝对目标。
     *
     * 该方法会更新内部按压状态；调用方需要原子校验时，应在对象副本上调用并仅在完整安全链通过后提交副本。
     */
    Result<MappedCommand> map(const MappedCommand &command, std::uint64_t steady_now_ns);

    /** @brief 新 session 或停止时清空按压原点，同时保留最新机器人位姿快照。 */
    void resetControlState();

  private:
    struct RobotPoseSnapshot {
        PoseSample right_arm;
        PoseSample left_arm;
        std::uint64_t receive_steady_time_ns{0};
        bool available{false};
    };

    Status validateFreshSnapshot(std::uint64_t steady_now_ns) const;
    Result<PoseSample> mapArm(const PoseSample &relative_pose, bool deadman, bool *active, PoseSample *origin,
                              bool *origin_captured, std::uint64_t steady_now_ns, bool right_arm);

    ArmOriginMappingConfig config_;
    RobotPoseSnapshot robot_pose_;
    PoseSample right_origin_;
    PoseSample left_origin_;
    bool right_active_{false};
    bool left_active_{false};
};

}  // namespace astrabot::teleop
