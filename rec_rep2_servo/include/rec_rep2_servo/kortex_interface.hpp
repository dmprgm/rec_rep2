#pragma once

#include <array>
#include <memory>
#include <string>

namespace rec_rep2_servo
{

// Duplicated from friction_observer.hpp rather than included, so this
// vendor-facing header stays free of Eigen -- keep in sync if it changes.
constexpr int kKortexNumJoints = 7;

struct JointFeedback
{
  double position_deg = 0.0;
  double velocity_deg_s = 0.0;
  double torque_nm = 0.0;
};

struct JointCommand
{
  double position_deg = 0.0;
  double torque_nm = 0.0;
};

using FeedbackArray = std::array<JointFeedback, kKortexNumJoints>;
using CommandArray = std::array<JointCommand, kKortexNumJoints>;

/// Wraps the Kinova Kortex C++ SDK's TCP (config) + UDP (realtime)
/// sessions and the LOW_LEVEL_SERVOING/TORQUE enter/exit sequence.
///
/// TODO: verify against the vendor SDK. kortex_interface.cpp is a
/// best-effort port of the working Python sequence in
/// rec_rep2/compliant_torque_mode.py, based on the public Kortex C++
/// example project's structure (the Kinova::Api::* namespaces) -- it has
/// NOT been compiled or run against the real SDK, since it isn't available
/// in this repo/environment. Verify every call site against the headers
/// in your KORTEX_API_DIR before trusting this on hardware. See
/// docs/cpp_servoing.md.
class KortexInterface
{
public:
  KortexInterface(
    const std::string & robot_ip, int tcp_port, int udp_port,
    const std::string & username, const std::string & password);
  ~KortexInterface();

  KortexInterface(const KortexInterface &) = delete;
  KortexInterface & operator=(const KortexInterface &) = delete;

  /// Open the TCP (config) and UDP (realtime) sessions. Throws
  /// std::runtime_error on failure.
  void connect();

  /// Enter LOW_LEVEL_SERVOING and set every actuator to TORQUE control
  /// mode. Precondition: connect() succeeded. Throws std::runtime_error
  /// on failure.
  void enter_low_level_torque();

  /// Read the latest actuator feedback over the realtime (UDP) connection.
  FeedbackArray refresh_feedback();

  /// Send one command frame and return the feedback from the same cycle
  /// (BaseCyclic::Refresh is a request/response call in the Kortex API).
  FeedbackArray refresh(const CommandArray & command);

  /// Gravity-drop protection: a handful of zero-torque frames, then
  /// POSITION mode at current angles, then back to SINGLE_LEVEL_SERVOING.
  /// Call on every exit path (clean stop, fault, exception) -- see
  /// docs/cpp_servoing.md's "safe-exit sequencing contract". This
  /// reproduces compliant_torque_mode.py's _safe_exit() step for step;
  /// the ordering must never change, even to "optimize". Never throws --
  /// swallows all errors internally, since it is the last line of defense
  /// and is called from destructors/catch blocks.
  void safe_exit() noexcept;

  void disconnect() noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rec_rep2_servo
