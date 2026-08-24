// gtest port of test/test_friction_observer.py. The Python tests use a
// variable n_joints (2 or 3); this port fixes n_joints at kNumJoints=7 to
// match FrictionObserver's compile-time-fixed size, exercising joints 0-1
// (occasionally 0-2) and padding the rest with zero residual/gain so they
// stay inert and don't affect the assertions below.
#include <gtest/gtest.h>

#include "rec_rep2_servo/friction_observer.hpp"

namespace rec_rep2_servo
{
namespace
{

TEST(FrictionObserver, ResetZeroesState)
{
  FrictionObserver obs(Vector7d::Constant(2.0), Vector7d::Constant(1.0));
  const Vector7d g_q = Vector7d::Zero();
  const Vector7d b_qdot = Vector7d::Zero();
  Vector7d tau;
  tau << 1.0, -1.0, 0.5, 0.0, 0.0, 0.0, 0.0;

  for (int i = 0; i < 50; ++i) {
    obs.update(tau, g_q, b_qdot, 0.005);
  }
  EXPECT_FALSE(obs.state().isZero(1e-9));

  obs.reset();
  EXPECT_TRUE(obs.state().isZero(0.0));
}

TEST(FrictionObserver, ZeroResidualStaysAtZero)
{
  // If tau_measured == g_q + b_qdot exactly, the residual is always zero,
  // hence the estimate never moves off its zero initial state.
  FrictionObserver obs(Vector7d::Constant(2.0), Vector7d::Constant(1.0));
  Vector7d g_q;
  g_q << 0.7, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0;
  Vector7d b_qdot;
  b_qdot << 0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0;
  const Vector7d tau_measured = g_q + b_qdot;

  Vector7d estimate = Vector7d::Zero();
  for (int i = 0; i < 200; ++i) {
    estimate = obs.update(tau_measured, g_q, b_qdot, 0.005);
  }

  EXPECT_TRUE(estimate.isZero(1e-9));
}

TEST(FrictionObserver, ConvergesToAnalyticSteadyState)
{
  // Continuous-time steady state of xi' = -(L+Lp)*xi + L*r (constant
  // residual r) is xi_ss = L*r/(L+Lp). Forward-Euler integration with a
  // small dt over many time constants should land close to it.
  Vector7d L;
  L << 2.0, 4.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  Vector7d Lp;
  Lp << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  FrictionObserver obs(L, Lp);

  Vector7d residual;
  residual << 1.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0;
  const Vector7d g_q = Vector7d::Zero();
  const Vector7d b_qdot = Vector7d::Zero();
  const Vector7d tau_measured = residual + g_q + b_qdot;

  const double dt = 1e-3;
  Vector7d estimate = Vector7d::Zero();
  for (int i = 0; i < 20000; ++i) {  // >> 1/(L+Lp) time constants for both joints
    estimate = obs.update(tau_measured, g_q, b_qdot, dt);
  }

  const Vector7d xi_ss = L.cwiseProduct(residual).cwiseQuotient(L + Lp);
  EXPECT_NEAR(estimate[0], xi_ss[0], 1e-3);
  EXPECT_NEAR(estimate[1], xi_ss[1], 1e-3);
}

TEST(FrictionObserver, JointsEvolveIndependently)
{
  // A large residual on one joint must not perturb another joint's estimate.
  FrictionObserver obs(Vector7d::Constant(2.0), Vector7d::Constant(1.0));
  const Vector7d g_q = Vector7d::Zero();
  const Vector7d b_qdot = Vector7d::Zero();
  Vector7d tau;
  tau << 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  Vector7d estimate = Vector7d::Zero();
  for (int i = 0; i < 500; ++i) {
    estimate = obs.update(tau, g_q, b_qdot, 0.005);
  }

  EXPECT_EQ(estimate[1], 0.0);
  EXPECT_NE(estimate[0], 0.0);
}

TEST(FrictionObserver, UpdateReturnsACopyNotInternalState)
{
  FrictionObserver obs(Vector7d::Constant(2.0), Vector7d::Constant(1.0));
  Vector7d tau;
  tau << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  Vector7d estimate = obs.update(tau, Vector7d::Zero(), Vector7d::Zero(), 0.005);
  estimate[0] = 999.0;

  EXPECT_NE(obs.state()[0], 999.0);
}

}  // namespace
}  // namespace rec_rep2_servo
