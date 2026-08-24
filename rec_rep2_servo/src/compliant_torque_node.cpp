#include "rec_rep2_servo/compliant_torque_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <thread>

namespace rec_rep2_servo
{

namespace
{
Vector7d declare_vector7_param(
  rclcpp::Node * node, const std::string & name, const Vector7d & default_value)
{
  std::vector<double> defaults(default_value.data(), default_value.data() + kNumJoints);
  node->declare_parameter(name, defaults);
  const std::vector<double> values = node->get_parameter(name).as_double_array();
  if (values.size() != static_cast<size_t>(kNumJoints)) {
    throw std::runtime_error(
      "Parameter '" + name + "' must have exactly " + std::to_string(kNumJoints) +
      " elements, got " + std::to_string(values.size()));
  }
  Vector7d out;
  for (int i = 0; i < kNumJoints; ++i) {
    out[i] = values[i];
  }
  return out;
}
}  // namespace

ComplianceTorqueNode::ComplianceTorqueNode()
: rclcpp::Node("compliant_torque_servo")
{
  // ── connection params ────────────────────────────────────────────────
  declare_parameter("robot_ip", std::string("192.168.1.10"));
  declare_parameter("tcp_port", 10000);
  declare_parameter("udp_port", 10001);
  declare_parameter("username", std::string("admin"));
  declare_parameter("password", std::string("admin"));
  declare_parameter("urdf_path", std::string(""));
  declare_parameter("control_rate_hz", 200.0);
  declare_parameter("max_missed_cycles", 10);

  // ── control-law / safety params (names mirror config/compliant_params.yaml
  //    on the Python side, without the "compliant_" prefix -- see
  //    docs/cpp_servoing.md's parameter table) ─────────────────────────────
  const Vector7d observer_L = declare_vector7_param(this, "observer_L", Vector7d::Constant(2.0));
  const Vector7d observer_Lp = declare_vector7_param(this, "observer_Lp", Vector7d::Constant(1.0));
  damping_b_ = declare_vector7_param(this, "damping_b", Vector7d::Constant(0.5));
  gravity_sign_ = declare_vector7_param(this, "gravity_sign", Vector7d::Constant(1.0));
  const Vector7d tau_limits = declare_vector7_param(
    this, "torque_limits",
    (Vector7d() << 22.4, 22.4, 22.4, 11.2, 11.2, 11.2, 11.2).finished());
  const Vector7d vel_limits =
    declare_vector7_param(this, "velocity_limits_rad_s", Vector7d::Constant(0.8));

  const double rate_hz = get_parameter("control_rate_hz").as_double();
  dt_nominal_ = 1.0 / rate_hz;

  observer_ = FrictionObserver(observer_L, observer_Lp);
  safety_ = SafetyMonitor(
    tau_limits, vel_limits,
    static_cast<int>(get_parameter("max_missed_cycles").as_int()), dt_nominal_);

  gravity_.load_urdf_from_string(load_urdf());

  kortex_ = std::make_unique<KortexInterface>(
    get_parameter("robot_ip").as_string(),
    static_cast<int>(get_parameter("tcp_port").as_int()),
    static_cast<int>(get_parameter("udp_port").as_int()),
    get_parameter("username").as_string(),
    get_parameter("password").as_string());

  enter_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/enter_torque_mode",
    std::bind(
      &ComplianceTorqueNode::handle_enter, this, std::placeholders::_1, std::placeholders::_2));
  exit_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/exit_torque_mode",
    std::bind(
      &ComplianceTorqueNode::handle_exit, this, std::placeholders::_1, std::placeholders::_2));
  reset_fault_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset_fault",
    std::bind(
      &ComplianceTorqueNode::handle_reset_fault, this, std::placeholders::_1,
      std::placeholders::_2));
  get_status_srv_ = create_service<srv::GetStatus>(
    "~/get_status",
    std::bind(
      &ComplianceTorqueNode::handle_get_status, this, std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "compliant_torque_servo ready.");
}

ComplianceTorqueNode::~ComplianceTorqueNode()
{
  stop_requested_ = true;
  if (loop_thread_.joinable()) {
    loop_thread_.join();
  }
  if (kortex_) {
    // Defensive: the loop already calls safe_exit()/kortex_->safe_exit()
    // on every exit path, but call again here in case the node is
    // destroyed without ~/exit_torque_mode ever having been invoked.
    kortex_->safe_exit();
    kortex_->disconnect();
  }
}

std::string ComplianceTorqueNode::load_urdf() const
{
  const std::string urdf_path = get_parameter("urdf_path").as_string();
  if (!urdf_path.empty()) {
    std::ifstream file(urdf_path);
    if (file) {
      std::ostringstream ss;
      ss << file.rdbuf();
      return ss.str();
    }
    RCLCPP_WARN(get_logger(), "urdf_path '%s' set but unreadable", urdf_path.c_str());
  }

  throw std::runtime_error(
    "GravityModel: no URDF available. Set the urdf_path parameter to a "
    "Gen3 .urdf file. Generate one once with, e.g.: "
    "ros2 run xacro xacro $(ros2 pkg prefix kortex_description)"
    "/share/kortex_description/robots/gen3.xacro dof:=7 vision:=false "
    "-o gen3.urdf -- see docs/cpp_servoing.md for the robot_description/"
    "xacro-fallback gap this leaves versus the Python implementation.");
}

void ComplianceTorqueNode::handle_enter(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (running_) {
    response->success = true;
    response->message = "Already running.";
    return;
  }
  {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    if (safety_.is_faulted()) {
      response->success = false;
      response->message = "Latched safety fault; call ~/reset_fault first.";
      return;
    }
  }

  try {
    kortex_->connect();
    kortex_->enter_low_level_torque();
  } catch (const std::exception & exc) {
    response->success = false;
    response->message = std::string("Failed to enter low-level torque mode: ") + exc.what();
    return;
  }

  observer_.reset();
  stop_requested_ = false;
  running_ = true;
  loop_thread_ = std::thread(&ComplianceTorqueNode::control_loop, this);

  response->success = true;
  response->message = "Compliant torque mode ON.";
}

void ComplianceTorqueNode::handle_exit(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  stop_requested_ = true;
  if (loop_thread_.joinable()) {
    loop_thread_.join();
  }
  response->success = true;
  response->message = "Compliant torque mode OFF. Position control restored.";
}

void ComplianceTorqueNode::handle_reset_fault(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(fault_mutex_);
  safety_.reset();
  response->success = true;
  response->message = "Safety fault cleared.";
}

void ComplianceTorqueNode::handle_get_status(
  const std::shared_ptr<srv::GetStatus::Request>,
  std::shared_ptr<srv::GetStatus::Response> response)
{
  std::lock_guard<std::mutex> lock(fault_mutex_);
  response->running = running_;
  response->faulted = safety_.is_faulted();
  response->fault_message = safety_.fault_record() ? safety_.fault_record()->message : "";
}

void ComplianceTorqueNode::control_loop()
{
  // Mirrors compliant_torque_mode.py's _control_loop(): read feedback,
  // compute tau_cmd = g(q) - tau_hat_friction - b*qdot, clamp/check via
  // SafetyMonitor, send, rate-limit. On every exit path (clean stop,
  // fault, exception) kortex_->safe_exit() runs before returning -- do
  // not add a path that skips it. See docs/cpp_servoing.md.
  constexpr double kDegToRad = M_PI / 180.0;
  auto last_t = std::chrono::steady_clock::now();

  try {
    while (!stop_requested_) {
      const auto t0 = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(t0 - last_t).count();
      last_t = t0;
      dt = std::max(dt, 1e-6);

      const FeedbackArray feedback = kortex_->refresh_feedback();

      Vector7d q_rad, v_rad, tau_meas;
      for (int i = 0; i < kNumJoints; ++i) {
        q_rad[i] = feedback[i].position_deg * kDegToRad;
        v_rad[i] = feedback[i].velocity_deg_s * kDegToRad;
        tau_meas[i] = feedback[i].torque_nm;
      }

      const Vector7d g_q = gravity_.compute_gravity(q_rad, gravity_sign_);
      const Vector7d b_qdot = damping_b_.cwiseProduct(v_rad);
      const Vector7d tau_hat_f = observer_.update(tau_meas, g_q, b_qdot, dt);
      const Vector7d tau_cmd = g_q - tau_hat_f - b_qdot;

      SafetyResult result;
      {
        std::lock_guard<std::mutex> lock(fault_mutex_);
        result = safety_.check_and_clamp(tau_cmd, v_rad, dt);
      }
      if (result.faulted) {
        RCLCPP_ERROR(get_logger(), "Safety fault: %s", result.reason->c_str());
        break;
      }

      CommandArray command;
      for (int i = 0; i < kNumJoints; ++i) {
        command[i].position_deg = feedback[i].position_deg;
        command[i].torque_nm = result.tau_safe[i];
      }
      kortex_->refresh(command);

      const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      const double sleep_s = dt_nominal_ - elapsed;
      if (sleep_s > 1e-4) {
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
      }
    }
  } catch (const std::exception & exc) {
    {
      std::lock_guard<std::mutex> lock(fault_mutex_);
      safety_.trip_exception(exc.what());
    }
    RCLCPP_ERROR(get_logger(), "Control loop exception: %s", exc.what());
  }

  kortex_->safe_exit();
  running_ = false;
}

}  // namespace rec_rep2_servo
