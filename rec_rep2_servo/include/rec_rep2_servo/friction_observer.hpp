#pragma once

#include <Eigen/Dense>

namespace rec_rep2_servo
{

// The Gen3 7-DoF arm this package targets always has exactly 7 joints, so
// (unlike the Python FrictionObserver/SafetyMonitor, which take n_joints as
// a constructor argument) the C++ port fixes it at compile time -- fixed-size
// Eigen vectors are stack-allocated with no heap churn in the 200 Hz hot
// loop, which is the whole point of this port.
constexpr int kNumJoints = 7;
using Vector7d = Eigen::Matrix<double, kNumJoints, 1>;

/// Model-free friction observer -- direct port of
/// rec_rep2/friction_observer.py's FrictionObserver. See that file's
/// docstring for the underlying theory: "Model-Free Friction Observers for
/// Flexible Joint Robots with Torque Measurements", Gaz, Cognetti, Oliva,
/// Giordano, De Luca, IEEE T-RO 2019 (DOI: 10.1109/TRO.2019.2926915).
class FrictionObserver
{
public:
  FrictionObserver() = default;
  FrictionObserver(
    const Vector7d & L,
    const Vector7d & Lp);

  /// Reset observer state to zero (call on mode entry).
  void reset();

  /// Advance the observer one step and return the friction torque estimate
  /// (Nm). Subtract this from the commanded torque to cancel friction.
  Vector7d update(
    const Vector7d & tau_measured,
    const Vector7d & g_q,
    const Vector7d & b_qdot,
    double dt);

  /// Current filtered friction estimate (same value update() last
  /// returned). Exposed for tests and optional diagnostics logging.
  const Vector7d & state() const {return xi_;}

private:
  Vector7d L_ = Vector7d::Constant(2.0);
  Vector7d Lp_ = Vector7d::Constant(1.0);
  Vector7d xi_ = Vector7d::Zero();
};

}  // namespace rec_rep2_servo
