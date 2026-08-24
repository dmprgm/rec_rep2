#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rec_rep2_servo/friction_observer.hpp"
#include "rec_rep2_servo/gravity_model.hpp"
#include "rec_rep2_servo/kortex_interface.hpp"
#include "rec_rep2_servo/safety_monitor.hpp"
#include "rec_rep2_servo/srv/get_status.hpp"

namespace rec_rep2_servo
{

/// C++ real-time backend for rec_rep2's compliant torque posing mode.
///
/// Reproduces rec_rep2/compliant_torque_mode.py's CompliantTorqueMode
/// control loop (enter -> ~200 Hz torque loop -> exit/safe-exit) outside
/// Python and its GIL. Driven by rec_rep2/recorder.py over the services
/// below when the `use_cpp_servo_backend` parameter is true -- see
/// docs/cpp_servoing.md for the full architecture and the safety
/// sequencing contract this node must preserve.
///
/// Node name: compliant_torque_servo.
class ComplianceTorqueNode : public rclcpp::Node
{
public:
  ComplianceTorqueNode();
  ~ComplianceTorqueNode() override;

private:
  void handle_enter(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_exit(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_reset_fault(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_get_status(
    const std::shared_ptr<srv::GetStatus::Request> request,
    std::shared_ptr<srv::GetStatus::Response> response);

  void control_loop();

  /// Load a URDF for GravityModel. Currently only the `urdf_path`
  /// parameter is supported (unlike compliant_torque_mode.py, which also
  /// tries the robot_description parameter and an xacro subprocess
  /// fallback -- not yet ported here, see docs/cpp_servoing.md). Throws
  /// std::runtime_error if urdf_path is unset or unreadable.
  std::string load_urdf() const;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enter_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr exit_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_srv_;
  rclcpp::Service<srv::GetStatus>::SharedPtr get_status_srv_;

  std::unique_ptr<KortexInterface> kortex_;
  FrictionObserver observer_;
  SafetyMonitor safety_;
  GravityModel gravity_;
  Vector7d damping_b_ = Vector7d::Constant(0.5);
  Vector7d gravity_sign_ = Vector7d::Constant(1.0);
  double dt_nominal_ = 1.0 / 200.0;

  std::thread loop_thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> running_{false};
  // Guards safety_'s fault state, which is written from both the loop
  // thread and service-callback threads (~/reset_fault, ~/get_status).
  std::mutex fault_mutex_;
};

}  // namespace rec_rep2_servo
