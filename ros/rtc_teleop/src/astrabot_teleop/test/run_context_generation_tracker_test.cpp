// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include <cstddef>
#include <string>

#include <gtest/gtest.h>

#include "astrabot_teleop/session/run_context_generation_tracker.h"

namespace astrabot::teleop {
namespace {

TEST(RunContextGenerationTrackerTest, RejectsRetiredGenerationWithoutSwitchingBack) {
    RunContextGenerationTracker tracker;

    EXPECT_EQ(tracker.observe("publisher-generation-a"), RunContextGenerationObservation::kInitial);
    EXPECT_EQ(tracker.observe("publisher-generation-b"), RunContextGenerationObservation::kChanged);
    EXPECT_EQ(tracker.observe("publisher-generation-a"), RunContextGenerationObservation::kRetired);
    EXPECT_EQ(tracker.currentGeneration(), "publisher-generation-b");
}

TEST(RunContextGenerationTrackerTest, KeepsRetiredHistoryBounded) {
    RunContextGenerationTracker tracker;
    ASSERT_EQ(tracker.observe("publisher-generation-0"), RunContextGenerationObservation::kInitial);

    for (std::size_t index = 1U; index <= RunContextGenerationTracker::kMaxRetiredGenerations + 8U; ++index) {
        ASSERT_EQ(tracker.observe("publisher-generation-" + std::to_string(index)),
                  RunContextGenerationObservation::kChanged);
        EXPECT_LE(tracker.retiredGenerationCount(), RunContextGenerationTracker::kMaxRetiredGenerations);
    }
}

TEST(RunContextGenerationTrackerTest, RejectsEmptyGenerationWithoutChangingCurrent) {
    RunContextGenerationTracker tracker;
    ASSERT_EQ(tracker.observe("publisher-generation-a"), RunContextGenerationObservation::kInitial);

    EXPECT_EQ(tracker.observe(""), RunContextGenerationObservation::kInvalid);
    EXPECT_EQ(tracker.currentGeneration(), "publisher-generation-a");
}

}  // namespace
}  // namespace astrabot::teleop
