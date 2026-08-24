// TODO: verify against the vendor SDK. Every Kinova::Api::* type/method
// used in this file is inferred from the public Kortex C++ example
// project's structure (mirroring the Python kortex_api SDK's class names
// 1:1, e.g. BaseClient -> Base::BaseClient) and has NOT been checked
// against real headers -- this repo doesn't have the vendor SDK available.
// Before running this on hardware, diff every call site below against the
// headers under your KORTEX_API_DIR (typically include/client/,
// include/common/, include/messages/). See docs/cpp_servoing.md.
#include "rec_rep2_servo/kortex_interface.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <RouterClient.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

namespace rec_rep2_servo
{

namespace k_api = Kinova::Api;

struct KortexInterface::Impl
{
  std::string robot_ip;
  int tcp_port = 0;
  int udp_port = 0;
  std::string username;
  std::string password;

  std::unique_ptr<k_api::TransportClientTcp> tcp_transport;
  std::unique_ptr<k_api::RouterClient> tcp_router;
  std::unique_ptr<k_api::SessionManager> tcp_session;
  std::unique_ptr<k_api::TransportClientUdp> udp_transport;
  std::unique_ptr<k_api::RouterClient> udp_router;

  std::unique_ptr<k_api::Base::BaseClient> base;
  std::unique_ptr<k_api::ActuatorConfig::ActuatorConfigClient> actuator_config;
  std::unique_ptr<k_api::BaseCyclic::BaseCyclicClient> cyclic;

  // command_id per actuator, threaded through Refresh() calls -- the
  // Kortex firmware expects a monotonically-increasing id per actuator,
  // same as the Python implementation's use of feedback.command_id.
  std::array<uint32_t, kKortexNumJoints> command_ids{};
};

KortexInterface::KortexInterface(
  const std::string & robot_ip, int tcp_port, int udp_port,
  const std::string & username, const std::string & password)
: impl_(std::make_unique<Impl>())
{
  impl_->robot_ip = robot_ip;
  impl_->tcp_port = tcp_port;
  impl_->udp_port = udp_port;
  impl_->username = username;
  impl_->password = password;
}

KortexInterface::~KortexInterface() = default;

void KortexInterface::connect()
{
  // Mirrors compliant_torque_mode.py's __init__: a TCP session for
  // config/servoing-mode calls, plus a dedicated UDP connection
  // (ROBOT_PORT_RT, typically 10001) for the realtime Refresh() loop.
  impl_->tcp_transport = std::make_unique<k_api::TransportClientTcp>();
  impl_->tcp_transport->connect(impl_->robot_ip, impl_->tcp_port);
  impl_->tcp_router =
    std::make_unique<k_api::RouterClient>(impl_->tcp_transport.get(), [](k_api::KError) {});

  k_api::Session::CreateSessionInfo session_info;
  session_info.set_username(impl_->username);
  session_info.set_password(impl_->password);
  session_info.set_session_inactivity_timeout(60000);
  session_info.set_connection_inactivity_timeout(2000);
  impl_->tcp_session = std::make_unique<k_api::SessionManager>(impl_->tcp_router.get());
  impl_->tcp_session->CreateSession(session_info);

  impl_->base = std::make_unique<k_api::Base::BaseClient>(impl_->tcp_router.get());
  impl_->actuator_config =
    std::make_unique<k_api::ActuatorConfig::ActuatorConfigClient>(impl_->tcp_router.get());

  impl_->udp_transport = std::make_unique<k_api::TransportClientUdp>();
  impl_->udp_transport->connect(impl_->robot_ip, impl_->udp_port);
  impl_->udp_router =
    std::make_unique<k_api::RouterClient>(impl_->udp_transport.get(), [](k_api::KError) {});
  impl_->cyclic = std::make_unique<k_api::BaseCyclic::BaseCyclicClient>(impl_->udp_router.get());
}

void KortexInterface::enter_low_level_torque()
{
  k_api::Base::ServoingModeInformation mode;
  mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  impl_->base->SetServoingMode(mode);

  for (int dev_id = 1; dev_id <= kKortexNumJoints; ++dev_id) {  // device IDs are 1-indexed
    k_api::ActuatorConfig::ControlModeInformation cmi;
    cmi.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
    impl_->actuator_config->SetControlMode(cmi, dev_id);
  }
}

namespace
{
FeedbackArray to_feedback_array(
  const k_api::BaseCyclic::Feedback & feedback,
  std::array<uint32_t, kKortexNumJoints> & command_ids_out)
{
  FeedbackArray out;
  for (int i = 0; i < kKortexNumJoints; ++i) {
    const auto & act = feedback.actuators(i);
    out[i] = {act.position(), act.velocity(), act.torque()};
    command_ids_out[i] = act.command_id();
  }
  return out;
}
}  // namespace

FeedbackArray KortexInterface::refresh_feedback()
{
  const k_api::BaseCyclic::Feedback feedback = impl_->cyclic->RefreshFeedback();
  return to_feedback_array(feedback, impl_->command_ids);
}

FeedbackArray KortexInterface::refresh(const CommandArray & command)
{
  k_api::BaseCyclic::Command cmd;
  for (int i = 0; i < kKortexNumJoints; ++i) {
    auto * act = cmd.add_actuators();
    act->set_command_id(impl_->command_ids[i]);
    act->set_position(static_cast<float>(command[i].position_deg));
    act->set_velocity(0.0f);
    act->set_torque_joint(static_cast<float>(command[i].torque_nm));
  }
  const k_api::BaseCyclic::Feedback feedback = impl_->cyclic->Refresh(cmd);
  return to_feedback_array(feedback, impl_->command_ids);
}

void KortexInterface::safe_exit() noexcept
{
  k_api::BaseCyclic::Feedback fb;
  bool have_feedback = false;

  try {
    fb = impl_->cyclic->RefreshFeedback();
    have_feedback = true;

    // Step 1: a handful of zero-torque frames so the arm comes to rest.
    k_api::BaseCyclic::Command zero_cmd;
    for (int i = 0; i < kKortexNumJoints; ++i) {
      auto * act = zero_cmd.add_actuators();
      act->set_command_id(fb.actuators(i).command_id());
      act->set_position(fb.actuators(i).position());
      act->set_velocity(0.0f);
      act->set_torque_joint(0.0f);
    }
    for (int i = 0; i < 5; ++i) {
      fb = impl_->cyclic->Refresh(zero_cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  } catch (...) {
    // Fall through to step 2 regardless -- safe_exit() must never throw.
  }

  try {
    // Step 2: switch every actuator to POSITION mode.
    for (int dev_id = 1; dev_id <= kKortexNumJoints; ++dev_id) {
      k_api::ActuatorConfig::ControlModeInformation cmi;
      cmi.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
      impl_->actuator_config->SetControlMode(cmi, dev_id);
    }

    // Step 3: hold at the current angles (skipped if step 1's feedback
    // read never succeeded -- there's nothing safe to hold at).
    if (have_feedback) {
      k_api::BaseCyclic::Command hold_cmd;
      for (int i = 0; i < kKortexNumJoints; ++i) {
        auto * act = hold_cmd.add_actuators();
        act->set_command_id(fb.actuators(i).command_id() + 1);
        act->set_position(fb.actuators(i).position());
      }
      impl_->cyclic->Refresh(hold_cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  } catch (...) {
  }

  try {
    // Step 4: always attempt to return to high-level mode, even if the
    // above failed.
    k_api::Base::ServoingModeInformation mode;
    mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    impl_->base->SetServoingMode(mode);
  } catch (...) {
  }
}

void KortexInterface::disconnect() noexcept
{
  try {
    if (impl_->tcp_session) {impl_->tcp_session->CloseSession();}
  } catch (...) {
  }
  try {
    if (impl_->tcp_transport) {impl_->tcp_transport->disconnect();}
  } catch (...) {
  }
  try {
    if (impl_->udp_transport) {impl_->udp_transport->disconnect();}
  } catch (...) {
  }
}

}  // namespace rec_rep2_servo
