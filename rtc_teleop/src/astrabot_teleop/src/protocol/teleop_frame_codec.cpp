// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/protocol/teleop_frame_codec.h"

#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include "TeleopFrame.pb.h"
#include "astrabot_teleop/protocol/crc32.h"

namespace astrabot::teleop {
namespace {

PoseSample toPoseSample(const PoseAction &pose) {
    PoseSample sample;
    sample.present = true;
    sample.x = pose.pos_x();
    sample.y = pose.pos_y();
    sample.z = pose.pos_z();
    sample.qx = pose.rot_x();
    sample.qy = pose.rot_y();
    sample.qz = pose.rot_z();
    sample.qw = pose.rot_w();
    return sample;
}

bool poseIsFinite(const PoseSample &pose) {
    return !pose.present ||
           (std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) && std::isfinite(pose.qx) &&
            std::isfinite(pose.qy) && std::isfinite(pose.qz) && std::isfinite(pose.qw));
}

bool messageHasUnknownFields(const google::protobuf::Message &message) {
    const auto *reflection = message.GetReflection();
    if (reflection->GetUnknownFields(message).field_count() != 0) {
        return true;
    }

    std::vector<const google::protobuf::FieldDescriptor *> fields;
    reflection->ListFields(message, &fields);
    for (const auto *field : fields) {
        if (field->cpp_type() != google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE) {
            continue;
        }
        if (field->is_repeated()) {
            const int count = reflection->FieldSize(message, field);
            for (int index = 0; index < count; ++index) {
                if (messageHasUnknownFields(reflection->GetRepeatedMessage(message, field, index))) {
                    return true;
                }
            }
        } else if (reflection->HasField(message, field) &&
                   messageHasUnknownFields(reflection->GetMessage(message, field))) {
            return true;
        }
    }
    return false;
}

bool skeletonIsFinite(const OpenXRHandSkeleton &skeleton) {
    for (const auto &joint : skeleton.joints()) {
        if (!std::isfinite(joint.x()) || !std::isfinite(joint.y()) || !std::isfinite(joint.z())) {
            return false;
        }
    }
    return true;
}

}  // namespace

TeleopFrameCodec::TeleopFrameCodec(const std::size_t max_frame_bytes) : max_frame_bytes_(max_frame_bytes) {}

Result<DecodedTeleopFrame> TeleopFrameCodec::decode(const std::vector<std::uint8_t> &payload) const {
    if (payload.empty()) {
        return Result<DecodedTeleopFrame>::failure(
            Status::error(ErrorCode::kInvalidArgument, "teleop frame payload is empty"));
    }
    if (payload.size() > max_frame_bytes_ ||
        payload.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        return Result<DecodedTeleopFrame>::failure(
            Status::error(ErrorCode::kResourceExhausted, "teleop frame exceeds configured size limit"));
    }

    TeleopFrame frame;
    if (!frame.ParseFromArray(payload.data(), static_cast<int>(payload.size()))) {
        return Result<DecodedTeleopFrame>::failure(Status::error(ErrorCode::kDataLoss, "teleop protobuf parse failed"));
    }
    if (!frame.has_header() || !frame.has_data_body()) {
        return Result<DecodedTeleopFrame>::failure(
            Status::error(ErrorCode::kInvalidArgument, "teleop frame requires header and data_body"));
    }
    if (messageHasUnknownFields(frame)) {
        return Result<DecodedTeleopFrame>::failure(
            Status::error(ErrorCode::kInvalidArgument, "teleop frame contains unknown protobuf fields"));
    }

    std::string body_bytes;
    if (!frame.data_body().SerializeToString(&body_bytes)) {
        return Result<DecodedTeleopFrame>::failure(
            Status::error(ErrorCode::kInternal, "failed to serialize data_body for CRC validation"));
    }
    const auto data_body_crc =
        Crc32::compute(reinterpret_cast<const std::uint8_t *>(body_bytes.data()), body_bytes.size());

    TeleopFrame legacy_envelope;
    legacy_envelope.mutable_data_body()->CopyFrom(frame.data_body());
    std::string legacy_bytes;
    if (!legacy_envelope.SerializeToString(&legacy_bytes)) {
        return Result<DecodedTeleopFrame>::failure(
            Status::error(ErrorCode::kInternal, "failed to serialize legacy CRC envelope"));
    }
    const auto legacy_crc =
        Crc32::compute(reinterpret_cast<const std::uint8_t *>(legacy_bytes.data()), legacy_bytes.size());
    const auto expected_crc = frame.header().crc32_checksum();
    if (expected_crc != data_body_crc && expected_crc != legacy_crc) {
        return Result<DecodedTeleopFrame>::failure(Status::error(ErrorCode::kDataLoss, "teleop frame CRC mismatch"));
    }

    DecodedTeleopFrame decoded;
    decoded.sequence = frame.header().frame_index();
    decoded.timestamp_ms = frame.header().timestamp_ms();
    decoded.crc32_checksum = expected_crc;
    decoded.crc_variant = expected_crc == data_body_crc ? CrcVariant::kDataBody : CrcVariant::kQuestLegacyEnvelope;
    decoded.pose_valid_right = frame.header().pose_valid_right();
    decoded.pose_valid_left = frame.header().pose_valid_left();
    decoded.pose_valid_head = frame.header().pose_valid_head();
    decoded.hand_skeleton_valid_right = frame.header().hand_skeleton_valid_right();
    decoded.hand_skeleton_valid_left = frame.header().hand_skeleton_valid_left();

    const auto &body = frame.data_body();
    if (body.has_action_right()) {
        decoded.action_right = toPoseSample(body.action_right());
    }
    if (body.has_action_left()) {
        decoded.action_left = toPoseSample(body.action_left());
    }
    if (body.has_action_head()) {
        decoded.action_head = toPoseSample(body.action_head());
    }
    if (body.has_calibration_poses()) {
        const auto &calibration = body.calibration_poses();
        if (calibration.has_pose_to_calibration_base_right()) {
            decoded.calibration_right = toPoseSample(calibration.pose_to_calibration_base_right());
        }
        if (calibration.has_pose_to_calibration_base_left()) {
            decoded.calibration_left = toPoseSample(calibration.pose_to_calibration_base_left());
        }
        if (calibration.has_pose_to_calibration_base_head()) {
            decoded.calibration_head = toPoseSample(calibration.pose_to_calibration_base_head());
        }
    }
    if (body.has_axes()) {
        const auto &axes = body.axes();
        decoded.axes.present = true;
        decoded.axes.trigger_right = axes.axis_trig_right();
        decoded.axes.trigger_left = axes.axis_trig_left();
        decoded.axes.grip_right = axes.axis_grip_right();
        decoded.axes.grip_left = axes.axis_grip_left();
        decoded.axes.joystick_right_x = axes.axis_js_right_x();
        decoded.axes.joystick_right_y = axes.axis_js_right_y();
        decoded.axes.joystick_left_x = axes.axis_js_left_x();
        decoded.axes.joystick_left_y = axes.axis_js_left_y();
    }
    if (body.has_safety()) {
        decoded.safety_present = true;
        decoded.safety_right = body.safety().is_safe_right();
        decoded.safety_left = body.safety().is_safe_left();
        decoded.safety_head = body.safety().is_active_head();
    }
    if (body.has_hand_skeleton_right()) {
        decoded.hand_skeleton_right_joint_count = static_cast<std::size_t>(body.hand_skeleton_right().joints_size());
        decoded.all_values_finite = decoded.all_values_finite && skeletonIsFinite(body.hand_skeleton_right());
    }
    if (body.has_hand_skeleton_left()) {
        decoded.hand_skeleton_left_joint_count = static_cast<std::size_t>(body.hand_skeleton_left().joints_size());
        decoded.all_values_finite = decoded.all_values_finite && skeletonIsFinite(body.hand_skeleton_left());
    }

    decoded.all_values_finite = decoded.all_values_finite && poseIsFinite(decoded.action_right) &&
                                poseIsFinite(decoded.action_left) && poseIsFinite(decoded.action_head) &&
                                poseIsFinite(decoded.calibration_right) && poseIsFinite(decoded.calibration_left) &&
                                poseIsFinite(decoded.calibration_head);
    if (decoded.axes.present) {
        decoded.all_values_finite =
            decoded.all_values_finite && std::isfinite(decoded.axes.trigger_right) &&
            std::isfinite(decoded.axes.trigger_left) && std::isfinite(decoded.axes.grip_right) &&
            std::isfinite(decoded.axes.grip_left) && std::isfinite(decoded.axes.joystick_right_x) &&
            std::isfinite(decoded.axes.joystick_right_y) && std::isfinite(decoded.axes.joystick_left_x) &&
            std::isfinite(decoded.axes.joystick_left_y);
    }
    return Result<DecodedTeleopFrame>::success(std::move(decoded));
}

}  // namespace astrabot::teleop
