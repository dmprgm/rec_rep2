#include "rec_rep2_servo/gravity_model.hpp"

#include <cmath>
#include <stdexcept>

#include <pinocchio/algorithm/rnea.hpp>  // computeGeneralizedGravity
#include <pinocchio/parsers/urdf.hpp>

namespace rec_rep2_servo
{

namespace
{
void check_nv(const pinocchio::Model & model)
{
  if (model.nv != kNumJoints) {
    throw std::runtime_error(
      "GravityModel: expected nv=" + std::to_string(kNumJoints) +
      " for the Gen3 7-DoF arm, got nv=" + std::to_string(model.nv) +
      " -- unexpected URDF");
  }
}
}  // namespace

void GravityModel::load_urdf_from_string(const std::string & urdf_xml)
{
  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(urdf_xml, model);
  check_nv(model);
  model_ = model;
  data_ = pinocchio::Data(*model_);
}

void GravityModel::load_urdf_from_file(const std::string & urdf_path)
{
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_path, model);
  check_nv(model);
  model_ = model;
  data_ = pinocchio::Data(*model_);
}

Eigen::VectorXd GravityModel::build_q_pin(const Vector7d & angles_rad)
{
  // Gen3 URDF joint layout (idx_q in the 11-element q vector) -- mirrors
  // compliant_torque_mode.py's _build_q_pin() exactly:
  //   joint_1 (RUBZ) -> q[0:2]  = [cos th1, sin th1]
  //   joint_2 (RZ)   -> q[2]    = th2
  //   joint_3 (RUBZ) -> q[3:5]  = [cos th3, sin th3]
  //   joint_4 (RZ)   -> q[5]    = th4
  //   joint_5 (RUBZ) -> q[6:8]  = [cos th5, sin th5]
  //   joint_6 (RZ)   -> q[8]    = th6
  //   joint_7 (RUBZ) -> q[9:11] = [cos th7, sin th7]
  Eigen::VectorXd q(11);
  constexpr int kRubzJoints[4] = {0, 2, 4, 6};
  constexpr int kRubzQStart[4] = {0, 3, 6, 9};
  for (int k = 0; k < 4; ++k) {
    q[kRubzQStart[k]] = std::cos(angles_rad[kRubzJoints[k]]);
    q[kRubzQStart[k] + 1] = std::sin(angles_rad[kRubzJoints[k]]);
  }
  q[2] = angles_rad[1];
  q[5] = angles_rad[3];
  q[8] = angles_rad[5];
  return q;
}

Vector7d GravityModel::compute_gravity(
  const Vector7d & angles_rad, const Vector7d & gravity_sign) const
{
  if (!model_.has_value()) {
    throw std::runtime_error("GravityModel::compute_gravity called before a URDF was loaded");
  }
  const Eigen::VectorXd q_pin = build_q_pin(angles_rad);
  const Eigen::VectorXd & g = pinocchio::computeGeneralizedGravity(*model_, *data_, q_pin);
  return gravity_sign.cwiseProduct(Vector7d(g));
}

}  // namespace rec_rep2_servo
