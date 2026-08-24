#pragma once

#include <optional>
#include <string>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "rec_rep2_servo/friction_observer.hpp"  // Vector7d, kNumJoints

namespace rec_rep2_servo
{

/// Pinocchio-backed gravity compensation for the Gen3 7-DoF arm, including
/// the URDF's alternating RUBZ (unbounded revolute)/RZ joint encoding.
///
/// Direct port of the _build_q_pin/_gravity helpers in
/// rec_rep2/compliant_torque_mode.py. Only the angles->q_pin direction is
/// needed here (unlike the Python file's _q_pin_to_angles, used only by
/// its Pinocchio forward-dynamics FAKE_HARDWARE simulation, which
/// deliberately stays Python-only -- see docs/cpp_servoing.md). Keep the
/// joint-index tables in BuildQPin() in sync with that file if the Gen3
/// URDF ever changes.
class GravityModel
{
public:
  GravityModel() = default;

  /// Build the Pinocchio model from a URDF XML string. Throws
  /// std::runtime_error on failure or if the model's nv doesn't match
  /// kNumJoints.
  void load_urdf_from_string(const std::string & urdf_xml);

  /// Build the Pinocchio model from a URDF file path. Throws
  /// std::runtime_error on failure or if the model's nv doesn't match
  /// kNumJoints.
  void load_urdf_from_file(const std::string & urdf_path);

  bool is_loaded() const {return model_.has_value();}

  /// Gravity compensation torques (Nm) for the given joint angles (rad).
  /// gravity_sign corrects per-joint encoder/URDF sign mismatches -- same
  /// parameter as the Python version's compliant_gravity_sign; see
  /// config/compliant_params.yaml's verification procedure. Throws
  /// std::runtime_error if no URDF has been loaded yet.
  Vector7d compute_gravity(const Vector7d & angles_rad, const Vector7d & gravity_sign) const;

private:
  static Eigen::VectorXd build_q_pin(const Vector7d & angles_rad);

  std::optional<pinocchio::Model> model_;
  mutable std::optional<pinocchio::Data> data_;
};

}  // namespace rec_rep2_servo
