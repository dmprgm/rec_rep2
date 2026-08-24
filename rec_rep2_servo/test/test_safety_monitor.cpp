// gtest port of test/test_safety_monitor.py. The Python tests use 2
// joints; this port fixes n_joints at kNumJoints=7 to match SafetyMonitor's
// compile-time-fixed size, exercising joints 0-1 (matching the Python
// tests' "Joint 1" assertions) and padding the rest with zero/in-limits
// values so they stay inert.
#include <gtest/gtest.h>

#include "rec_rep2_servo/safety_monitor.hpp"

namespace rec_rep2_servo
{
namespace
{

SafetyMonitor make_monitor(int max_missed_cycles = 3)
{
  return SafetyMonitor(
    Vector7d::Constant(1.0), Vector7d::Constant(1.0), max_missed_cycles, 0.005, 3.0);
}

TEST(SafetyMonitor, WithinLimitsPassesThroughUnmodified)
{
  SafetyMonitor mon = make_monitor();
  Vector7d tau_cmd;
  tau_cmd << 0.5, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0;
  Vector7d q_dot;
  q_dot << 0.2, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0;

  const SafetyResult result = mon.check_and_clamp(tau_cmd, q_dot, 0.005);

  EXPECT_FALSE(result.faulted);
  EXPECT_FALSE(result.reason.has_value());
  EXPECT_TRUE(result.tau_safe.isApprox(tau_cmd));
  EXPECT_FALSE(mon.is_faulted());
}

TEST(SafetyMonitor, TorqueSaturationClampsWithoutFaulting)
{
  SafetyMonitor mon = make_monitor();
  Vector7d tau_cmd;
  tau_cmd << 2.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  const SafetyResult result = mon.check_and_clamp(tau_cmd, Vector7d::Zero(), 0.005);

  EXPECT_FALSE(result.faulted);
  Vector7d expected;
  expected << 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  EXPECT_TRUE(result.tau_safe.isApprox(expected));
  EXPECT_FALSE(mon.is_faulted());
}

TEST(SafetyMonitor, VelocityTripLatchesFaultAndZeroesTorque)
{
  SafetyMonitor mon = make_monitor();
  Vector7d tau_cmd;
  tau_cmd << 0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;
  Vector7d q_dot;
  q_dot << 0.2, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0;

  const SafetyResult result = mon.check_and_clamp(tau_cmd, q_dot, 0.005);

  ASSERT_TRUE(result.faulted);
  ASSERT_TRUE(result.reason.has_value());
  EXPECT_NE(result.reason->find("Joint 1"), std::string::npos);
  EXPECT_TRUE(result.tau_safe.isZero(0.0));
  ASSERT_TRUE(mon.is_faulted());
  ASSERT_TRUE(mon.fault_record().has_value());
  EXPECT_EQ(mon.fault_record()->fault_type, FaultType::kVelocityTrip);
  ASSERT_TRUE(mon.fault_record()->joint_index.has_value());
  EXPECT_EQ(*mon.fault_record()->joint_index, 1);
}

TEST(SafetyMonitor, FaultStaysLatchedUntilReset)
{
  SafetyMonitor mon = make_monitor();
  Vector7d q_dot_bad;
  q_dot_bad << 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  mon.check_and_clamp(Vector7d::Zero(), q_dot_bad, 0.005);
  ASSERT_TRUE(mon.is_faulted());

  // Even a perfectly safe command is now rejected -- the fault is latched.
  Vector7d safe_cmd;
  safe_cmd << 0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;
  SafetyResult result = mon.check_and_clamp(safe_cmd, Vector7d::Zero(), 0.005);
  EXPECT_TRUE(result.faulted);
  EXPECT_TRUE(result.tau_safe.isZero(0.0));

  mon.reset();
  EXPECT_FALSE(mon.is_faulted());
  EXPECT_FALSE(mon.fault_record().has_value());

  result = mon.check_and_clamp(safe_cmd, Vector7d::Zero(), 0.005);
  EXPECT_FALSE(result.faulted);
}

TEST(SafetyMonitor, LoopOverrunRequiresConsecutiveMisses)
{
  SafetyMonitor mon = make_monitor(3);
  const double over_dt = 0.005 * 3.0 * 2.0;  // well over threshold

  for (int i = 0; i < 2; ++i) {
    const SafetyResult result = mon.check_and_clamp(Vector7d::Zero(), Vector7d::Zero(), over_dt);
    EXPECT_FALSE(result.faulted);
  }

  const SafetyResult result = mon.check_and_clamp(Vector7d::Zero(), Vector7d::Zero(), over_dt);
  ASSERT_TRUE(result.faulted);
  ASSERT_TRUE(mon.fault_record().has_value());
  EXPECT_EQ(mon.fault_record()->fault_type, FaultType::kLoopOverrun);
}

TEST(SafetyMonitor, GoodCycleResetsOverrunCounter)
{
  SafetyMonitor mon = make_monitor(3);
  const double over_dt = 0.005 * 3.0 * 2.0;
  const double good_dt = 0.005;

  mon.check_and_clamp(Vector7d::Zero(), Vector7d::Zero(), over_dt);
  mon.check_and_clamp(Vector7d::Zero(), Vector7d::Zero(), over_dt);
  // A single good cycle should reset the miss counter to zero...
  mon.check_and_clamp(Vector7d::Zero(), Vector7d::Zero(), good_dt);

  // ...so two more overruns are not enough to trip.
  for (int i = 0; i < 2; ++i) {
    const SafetyResult result = mon.check_and_clamp(Vector7d::Zero(), Vector7d::Zero(), over_dt);
    EXPECT_FALSE(result.faulted);
  }
  EXPECT_FALSE(mon.is_faulted());
}

TEST(SafetyMonitor, TripExceptionLatchesFault)
{
  SafetyMonitor mon = make_monitor();
  mon.trip_exception("boom");
  ASSERT_TRUE(mon.is_faulted());
  ASSERT_TRUE(mon.fault_record().has_value());
  EXPECT_EQ(mon.fault_record()->fault_type, FaultType::kException);

  const SafetyResult result = mon.check_and_clamp(Vector7d::Zero(), Vector7d::Zero(), 0.005);
  EXPECT_TRUE(result.faulted);
}

TEST(SafetyMonitor, VelocityTripFiresOnEitherSign)
{
  for (const double sign : {1.0, -1.0}) {
    SafetyMonitor mon = make_monitor();
    Vector7d q_dot;
    q_dot << 0.0, sign * 2.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    const SafetyResult result = mon.check_and_clamp(Vector7d::Zero(), q_dot, 0.005);

    ASSERT_TRUE(result.faulted);
    ASSERT_TRUE(result.reason.has_value());
    EXPECT_NE(result.reason->find("\xc2\xb1"), std::string::npos);  // "±", UTF-8
  }
}

}  // namespace
}  // namespace rec_rep2_servo
