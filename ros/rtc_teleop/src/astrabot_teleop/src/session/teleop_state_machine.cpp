// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/session/teleop_state_machine.h"

#include <array>

namespace astrabot::teleop {
namespace {

struct Transition {
    TeleopState from;
    TeleopEvent event;
    TeleopState to;
};

constexpr std::array<Transition, 30> kTransitions{{
    {TeleopState::kIdle, TeleopEvent::kAuthorize, TeleopState::kAuthorized},
    {TeleopState::kClosed, TeleopEvent::kAuthorize, TeleopState::kAuthorized},
    {TeleopState::kAuthorized, TeleopEvent::kPeerConnected, TeleopState::kConnected},
    {TeleopState::kConnected, TeleopEvent::kPeerConnected, TeleopState::kConnected},
    {TeleopState::kConnected, TeleopEvent::kFrameValid, TeleopState::kArmed},
    {TeleopState::kConnected, TeleopEvent::kOwnerAcquired, TeleopState::kArmed},
    {TeleopState::kArmed, TeleopEvent::kFrameValid, TeleopState::kArmed},
    {TeleopState::kControlling, TeleopEvent::kFrameValid, TeleopState::kControlling},
    {TeleopState::kArmed, TeleopEvent::kDeadmanPressed, TeleopState::kControlling},
    {TeleopState::kControlling, TeleopEvent::kDeadmanPressed, TeleopState::kControlling},
    {TeleopState::kArmed, TeleopEvent::kDeadmanReleased, TeleopState::kArmed},
    {TeleopState::kControlling, TeleopEvent::kDeadmanReleased, TeleopState::kArmed},
    {TeleopState::kArmed, TeleopEvent::kOwnerReleased, TeleopState::kConnected},
    {TeleopState::kControlling, TeleopEvent::kOwnerReleased, TeleopState::kConnected},
    {TeleopState::kAuthorized, TeleopEvent::kStopRequested, TeleopState::kStopping},
    {TeleopState::kConnected, TeleopEvent::kStopRequested, TeleopState::kStopping},
    {TeleopState::kArmed, TeleopEvent::kStopRequested, TeleopState::kStopping},
    {TeleopState::kControlling, TeleopEvent::kStopRequested, TeleopState::kStopping},
    {TeleopState::kStopping, TeleopEvent::kStopRequested, TeleopState::kStopping},
    {TeleopState::kStopping, TeleopEvent::kStopCompleted, TeleopState::kClosed},
    {TeleopState::kFault, TeleopEvent::kStopCompleted, TeleopState::kClosed},
    {TeleopState::kIdle, TeleopEvent::kFault, TeleopState::kFault},
    {TeleopState::kAuthorized, TeleopEvent::kFault, TeleopState::kFault},
    {TeleopState::kConnected, TeleopEvent::kFault, TeleopState::kFault},
    {TeleopState::kArmed, TeleopEvent::kFault, TeleopState::kFault},
    {TeleopState::kControlling, TeleopEvent::kFault, TeleopState::kFault},
    {TeleopState::kStopping, TeleopEvent::kFault, TeleopState::kFault},
    {TeleopState::kClosed, TeleopEvent::kFault, TeleopState::kFault},
    {TeleopState::kFault, TeleopEvent::kFault, TeleopState::kFault},
    {TeleopState::kClosed, TeleopEvent::kStopCompleted, TeleopState::kClosed},
}};

}  // namespace

Status TeleopStateMachine::transition(const TeleopEvent event, const std::string &reason_code) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &transition : kTransitions) {
        if (transition.from == state_ && transition.event == event) {
            state_ = transition.to;
            reason_code_ = reason_code;
            return Status::success();
        }
    }
    return Status::error(ErrorCode::kFailedPrecondition, "illegal Teleop state transition");
}

TeleopState TeleopStateMachine::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

std::string TeleopStateMachine::reasonCode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return reason_code_;
}

Status TeleopStateMachine::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != TeleopState::kClosed && state_ != TeleopState::kIdle) {
        return Status::error(ErrorCode::kFailedPrecondition, "only Closed state can reset to Idle");
    }
    state_ = TeleopState::kIdle;
    reason_code_ = "reset";
    return Status::success();
}

}  // namespace astrabot::teleop
