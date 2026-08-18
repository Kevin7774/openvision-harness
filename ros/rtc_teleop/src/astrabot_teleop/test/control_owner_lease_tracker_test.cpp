// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "astrabot_teleop/session/control_owner_lease_tracker.h"

namespace astrabot::teleop {
namespace {

ControlOwnerIdentity makeIdentity(const char *session_id = "session-1") {
    return ControlOwnerIdentity{session_id, "run-1", "thor", "peer-1"};
}

ControlOwnerOperation acquireLease(ControlOwnerLeaseTracker *tracker, std::uint64_t owner_epoch = 7U) {
    auto operation = tracker->beginAcquire(makeIdentity(), 100U, 40U);
    EXPECT_TRUE(operation.ok());
    if (!operation.ok()) {
        return ControlOwnerOperation{};
    }
    EXPECT_TRUE(tracker->completeAcquire(operation.value(), true, owner_epoch, 300U, 120U).ok());
    return operation.value();
}

TEST(ControlOwnerLeaseTrackerTest, ActivatesOnlyAfterConfirmedAcquire) {
    ControlOwnerLeaseTracker tracker;
    auto operation = tracker.beginAcquire(makeIdentity(), 100U, 40U);
    ASSERT_TRUE(operation.ok());
    EXPECT_FALSE(tracker.activeLease(110U).has_value());
    ASSERT_TRUE(tracker.completeAcquire(operation.value(), true, 7U, 300U, 120U).ok());

    const auto lease = tracker.activeLease(299U);
    ASSERT_TRUE(lease.has_value());
    EXPECT_EQ(lease->owner_epoch, 7U);
    EXPECT_EQ(lease->identity.session_id, "session-1");
    EXPECT_FALSE(tracker.activeLease(300U).has_value());
}

TEST(ControlOwnerLeaseTrackerTest, RejectedAcquireClearsPendingWithoutLease) {
    ControlOwnerLeaseTracker tracker;
    auto operation = tracker.beginAcquire(makeIdentity(), 100U, 40U);
    ASSERT_TRUE(operation.ok());
    EXPECT_FALSE(tracker.completeAcquire(operation.value(), false, 0U, 0U, 120U).ok());
    EXPECT_FALSE(tracker.pendingOperation().has_value());
    EXPECT_FALSE(tracker.leaseSnapshot().has_value());
    EXPECT_TRUE(tracker.beginAcquire(makeIdentity(), 130U, 40U).ok());
}

TEST(ControlOwnerLeaseTrackerTest, RenewExtendsOnlyTheSameEpoch) {
    ControlOwnerLeaseTracker tracker;
    acquireLease(&tracker);
    auto operation = tracker.beginRenew(150U, 40U);
    ASSERT_TRUE(operation.ok());
    ASSERT_TRUE(tracker.completeRenew(operation.value(), true, 7U, 450U, 170U).ok());
    ASSERT_TRUE(tracker.activeLease(449U).has_value());
    EXPECT_EQ(tracker.activeLease(449U)->expires_at_steady_ns, 450U);

    operation = tracker.beginRenew(200U, 40U);
    ASSERT_TRUE(operation.ok());
    EXPECT_FALSE(tracker.completeRenew(operation.value(), true, 8U, 500U, 220U).ok());
    EXPECT_FALSE(tracker.leaseSnapshot().has_value());
}

TEST(ControlOwnerLeaseTrackerTest, RenewTimeoutRevokesLeaseAndRejectsLateCallback) {
    ControlOwnerLeaseTracker tracker;
    acquireLease(&tracker);
    auto operation = tracker.beginRenew(150U, 40U);
    ASSERT_TRUE(operation.ok());
    EXPECT_FALSE(tracker.expirePending(189U).has_value());
    const auto expired = tracker.expirePending(190U);
    ASSERT_TRUE(expired.has_value());
    EXPECT_EQ(expired->kind, ControlOwnerOperationKind::kRenew);
    EXPECT_FALSE(tracker.leaseSnapshot().has_value());
    EXPECT_FALSE(tracker.completeRenew(operation.value(), true, 7U, 500U, 200U).ok());
    EXPECT_FALSE(tracker.leaseSnapshot().has_value());
}

TEST(ControlOwnerLeaseTrackerTest, ReleaseRevokesEpochBeforeServiceResponse) {
    ControlOwnerLeaseTracker tracker;
    acquireLease(&tracker);
    auto operation = tracker.beginRelease(150U, 40U);
    ASSERT_TRUE(operation.ok());
    EXPECT_EQ(operation.value().owner_epoch, 7U);
    EXPECT_FALSE(tracker.activeLease(151U).has_value());
    EXPECT_FALSE(tracker.leaseSnapshot().has_value());
    EXPECT_FALSE(tracker.completeRelease(operation.value(), false).ok());
    EXPECT_FALSE(tracker.leaseSnapshot().has_value());
}

TEST(ControlOwnerLeaseTrackerTest, ReleaseTimeoutNeverRestoresEpoch) {
    ControlOwnerLeaseTracker tracker;
    acquireLease(&tracker);
    auto operation = tracker.beginRelease(150U, 40U);
    ASSERT_TRUE(operation.ok());
    ASSERT_TRUE(tracker.expirePending(190U).has_value());
    EXPECT_FALSE(tracker.leaseSnapshot().has_value());
    EXPECT_FALSE(tracker.completeRelease(operation.value(), true).ok());
    EXPECT_FALSE(tracker.leaseSnapshot().has_value());
}

TEST(ControlOwnerLeaseTrackerTest, CancelMakesOldAcquireCallbackUnableToReviveLease) {
    ControlOwnerLeaseTracker tracker;
    auto old_operation = tracker.beginAcquire(makeIdentity("session-old"), 100U, 40U);
    ASSERT_TRUE(old_operation.ok());
    ASSERT_TRUE(tracker.cancelPending().has_value());

    auto current_operation = tracker.beginAcquire(makeIdentity("session-new"), 150U, 40U);
    ASSERT_TRUE(current_operation.ok());
    EXPECT_FALSE(tracker.completeAcquire(old_operation.value(), true, 6U, 300U, 160U).ok());
    EXPECT_FALSE(tracker.leaseSnapshot().has_value());
    ASSERT_TRUE(tracker.completeAcquire(current_operation.value(), true, 7U, 350U, 170U).ok());
    ASSERT_TRUE(tracker.leaseSnapshot().has_value());
    EXPECT_EQ(tracker.leaseSnapshot()->identity.session_id, "session-new");
}

TEST(ControlOwnerLeaseTrackerTest, InvalidateRejectsOldRenewCallback) {
    ControlOwnerLeaseTracker tracker;
    acquireLease(&tracker);
    auto operation = tracker.beginRenew(150U, 40U);
    ASSERT_TRUE(operation.ok());
    tracker.invalidate();
    EXPECT_FALSE(tracker.completeRenew(operation.value(), true, 7U, 500U, 170U).ok());
    EXPECT_FALSE(tracker.leaseSnapshot().has_value());
}

TEST(ControlOwnerLeaseTrackerTest, BoundsPendingOperationsToOne) {
    ControlOwnerLeaseTracker tracker;
    auto operation = tracker.beginAcquire(makeIdentity(), 100U, 40U);
    ASSERT_TRUE(operation.ok());
    EXPECT_FALSE(tracker.beginAcquire(makeIdentity("session-2"), 101U, 40U).ok());
    const auto pending = tracker.pendingOperation();
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(pending->operation_id, operation.value().operation_id);
}

TEST(ControlOwnerLeaseTrackerTest, RejectsDeadlineOverflowWithoutPendingState) {
    ControlOwnerLeaseTracker tracker;
    const auto operation = tracker.beginAcquire(makeIdentity(), std::numeric_limits<std::uint64_t>::max() - 5U, 10U);
    EXPECT_FALSE(operation.ok());
    EXPECT_FALSE(tracker.pendingOperation().has_value());
    EXPECT_FALSE(tracker.leaseSnapshot().has_value());
}

}  // namespace
}  // namespace astrabot::teleop
