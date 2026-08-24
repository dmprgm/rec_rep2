#include "rec_rep2_servo/safety_monitor.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace rec_rep2_servo
{

namespace
{
double now_seconds()
{
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

const char * fault_type_name(FaultType type)
{
  switch (type) {
    case FaultType::kVelocityTrip: return "VELOCITY_TRIP";
    case FaultType::kLoopOverrun: return "LOOP_OVERRUN";
    case FaultType::kException: return "EXCEPTION";
  }
  return "UNKNOWN";
}
}  // namespace

std::string FaultRecord::to_string() const
{
  std::ostringstream oss;
  oss << "[" << fault_type_name(fault_type) << "]";
  if (joint_index.has_value()) {
    oss << " joint " << *joint_index;
  }
  oss << "  measured=" << std::fixed << std::setprecision(4) << measured_value
      << "  threshold=" << threshold
      << "  t=" << std::setprecision(3) << timestamp;
  return oss.str();
}

SafetyMonitor::SafetyMonitor(
  const Vector7d & tau_limits, const Vector7d & vel_limits, int max_missed_cycles,
  double dt_nominal, double overrun_factor)
: tau_limits_(tau_limits), vel_limits_(vel_limits),
  max_missed_cycles_(max_missed_cycles), dt_nominal_(dt_nominal), overrun_factor_(overrun_factor)
{
}

void SafetyMonitor::reset()
{
  faulted_ = false;
  fault_record_.reset();
  consecutive_misses_ = 0;
}

void SafetyMonitor::trip_exception(const std::string & what)
{
  trip(
    FaultType::kException, std::nullopt, 0.0, 0.0,
    "Unhandled exception in control loop: " + what);
}

SafetyResult SafetyMonitor::check_and_clamp(
  const Vector7d & tau_cmd, const Vector7d & q_dot, double dt_actual)
{
  if (faulted_) {
    return SafetyResult{Vector7d::Zero(), true, fault_record_->to_string()};
  }

  // ── loop-rate watchdog ──────────────────────────────────────────────
  if (dt_actual > dt_nominal_ * overrun_factor_) {
    ++consecutive_misses_;
    if (consecutive_misses_ >= max_missed_cycles_) {
      std::ostringstream msg;
      msg << "Control loop overrun: " << consecutive_misses_
          << " consecutive cycles exceeded "
          << (dt_nominal_ * overrun_factor_ * 1e3) << " ms (last dt="
          << (dt_actual * 1e3) << " ms)";
      return trip(
        FaultType::kLoopOverrun, std::nullopt, dt_actual * 1000.0,
        dt_nominal_ * overrun_factor_ * 1000.0, msg.str());
    }
  } else {
    consecutive_misses_ = 0;
  }

  // ── velocity watchdog ───────────────────────────────────────────────
  for (int i = 0; i < kNumJoints; ++i) {
    const double v = q_dot[i];
    const double lim = vel_limits_[i];
    if (std::abs(v) > lim) {
      const double signed_lim = v > 0.0 ? lim : -lim;
      std::ostringstream msg;
      msg << "Joint " << i << " velocity " << v << " rad/s exceeds watchdog limit ±"
          << lim << " rad/s";
      return trip(FaultType::kVelocityTrip, i, v, signed_lim, msg.str());
    }
  }

  // ── torque saturation (silent clamp, not a fault) ──────────────────
  const Vector7d clamped = tau_cmd.cwiseMax(-tau_limits_).cwiseMin(tau_limits_);
  return SafetyResult{clamped, false, std::nullopt};
}

SafetyResult SafetyMonitor::trip(
  FaultType fault_type, std::optional<int> joint_index, double measured, double threshold,
  const std::string & message)
{
  faulted_ = true;
  fault_record_ = FaultRecord{fault_type, joint_index, measured, threshold, message, now_seconds()};
  return SafetyResult{Vector7d::Zero(), true, message};
}

}  // namespace rec_rep2_servo
