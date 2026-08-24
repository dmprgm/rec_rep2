#pragma once

#include <optional>
#include <string>

#include "rec_rep2_servo/friction_observer.hpp"  // Vector7d, kNumJoints

namespace rec_rep2_servo
{

/// Safety fault types -- port of rec_rep2/safety_monitor.py's FaultType.
enum class FaultType
{
  kVelocityTrip,
  kLoopOverrun,
  kException,
};

/// Record of a single safety trip -- port of safety_monitor.py's FaultRecord.
struct FaultRecord
{
  FaultType fault_type;
  std::optional<int> joint_index;
  double measured_value = 0.0;
  double threshold = 0.0;
  std::string message;
  double timestamp = 0.0;  // seconds, steady_clock-relative

  /// Human-readable description, e.g. for logging.
  std::string to_string() const;
};

/// Result of one SafetyMonitor::check_and_clamp() call, replacing the
/// Python version's (tau_safe, faulted, reason) 3-tuple return.
struct SafetyResult
{
  Vector7d tau_safe = Vector7d::Zero();
  bool faulted = false;
  std::optional<std::string> reason;
};

/// Safety monitor for the compliant torque control loop -- direct port of
/// rec_rep2/safety_monitor.py's SafetyMonitor. All checks run inside the
/// control loop before every torque command; see that file's docstring for
/// the full fault-condition reference (VELOCITY_TRIP, LOOP_OVERRUN,
/// EXCEPTION) and the torque-saturation-is-silent rationale.
class SafetyMonitor
{
public:
  SafetyMonitor() = default;
  SafetyMonitor(
    const Vector7d & tau_limits,
    const Vector7d & vel_limits,
    int max_missed_cycles = 10,
    double dt_nominal = 0.005,
    double overrun_factor = 3.0);

  bool is_faulted() const {return faulted_;}
  const std::optional<FaultRecord> & fault_record() const {return fault_record_;}

  /// Clear latched fault state; call via the ~/reset_fault service.
  void reset();

  /// Record an unhandled-exception fault (mirrors trip_exception()).
  void trip_exception(const std::string & what);

  /// Clamp torques / check for safety violations. If already faulted,
  /// returns zero torque immediately without re-evaluating watchdogs.
  SafetyResult check_and_clamp(const Vector7d & tau_cmd, const Vector7d & q_dot, double dt_actual);

private:
  SafetyResult trip(
    FaultType fault_type, std::optional<int> joint_index,
    double measured, double threshold, const std::string & message);

  Vector7d tau_limits_ = Vector7d::Constant(1.0);
  Vector7d vel_limits_ = Vector7d::Constant(1.0);
  int max_missed_cycles_ = 10;
  double dt_nominal_ = 0.005;
  double overrun_factor_ = 3.0;

  bool faulted_ = false;
  std::optional<FaultRecord> fault_record_;
  int consecutive_misses_ = 0;
};

}  // namespace rec_rep2_servo
