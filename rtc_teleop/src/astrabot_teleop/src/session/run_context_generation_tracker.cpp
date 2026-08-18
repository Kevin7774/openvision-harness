// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/session/run_context_generation_tracker.h"

#include <utility>

namespace astrabot::teleop {

RunContextGenerationObservation RunContextGenerationTracker::observe(const std::string &publisher_generation) {
    if (publisher_generation.empty()) {
        return RunContextGenerationObservation::kInvalid;
    }
    if (current_generation_.empty()) {
        current_generation_ = publisher_generation;
        return RunContextGenerationObservation::kInitial;
    }
    if (publisher_generation == current_generation_) {
        return RunContextGenerationObservation::kCurrent;
    }
    if (retired_generations_.count(publisher_generation) != 0U) {
        return RunContextGenerationObservation::kRetired;
    }

    retireCurrentGeneration();
    current_generation_ = publisher_generation;
    return RunContextGenerationObservation::kChanged;
}

const std::string &RunContextGenerationTracker::currentGeneration() const {
    return current_generation_;
}

std::size_t RunContextGenerationTracker::retiredGenerationCount() const {
    return retired_generations_.size();
}

void RunContextGenerationTracker::clear() {
    current_generation_.clear();
    retired_generation_order_.clear();
    retired_generations_.clear();
}

void RunContextGenerationTracker::retireCurrentGeneration() {
    if (current_generation_.empty() || !retired_generations_.insert(current_generation_).second) {
        return;
    }
    retired_generation_order_.push_back(current_generation_);
    while (retired_generations_.size() > kMaxRetiredGenerations && !retired_generation_order_.empty()) {
        std::string oldest_generation = std::move(retired_generation_order_.front());
        retired_generation_order_.pop_front();
        retired_generations_.erase(oldest_generation);
    }
}

}  // namespace astrabot::teleop
