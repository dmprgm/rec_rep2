# RecRep2: Compliant Motion Recording and Replay for a Kinova Gen3 Robotic Arm
This was made for the Humble ROS2 distro and now must be converted to the Jazzy ROS2 distro.

## External driver stack
rec_rep2 itself only talks to the Gen3 through `/joint_states` and
`joint_trajectory_controller`. To get a (real or fake) robot onto those
topics, you need to bring up Kinova's driver stack first. Bootstrap a fresh
workspace like this:

    cd ~/ros2_ws
    vcs import src < src/rec_rep2/rec_rep2.repos   # optional, see below
    rosdep install --from-paths src --ignore-src -r -y
    pip install -r src/rec_rep2/requirements.txt
    colcon build

Dependency breakdown:

- **kortex_bringup / kortex_description** (the Gen3 driver + URDF/meshes, from [Kinovarobotics/ros2_kortex](https://github.com/Kinovarobotics/ros2_kortex)) are released as Jazzy binaries
    - `rosdep install` should be all you need
    - both are declared in `package.xml`. 
    - `rec_rep2.repos` + `vcs import` is only needed if building ros2_kortex from source instead
- **pinocchio** and **python3-numpy** are declared in `package.xml` and
  installed the same way via rosdep. They're also listed in `requirements.txt` so `friction_observer.py` / `safety_monitor.py` can be unit-tested with plain `pytest` outside a
  ROS install entirely
- **kortex_api** (Kinova's Python gRPC SDK, imported directly by
  `compliant_mode.py` / `compliant_torque_mode.py`) is not a ROS package and not on PyPI, so neither rosdep nor `requirements.txt` can install it.
  Download the wheel matching Your Python version from the
  [Kinova Kortex API Artifactory](https://artifactory.kinovaapps.com/ui/repos/tree/General/generic-public/kortex/API)
  (see [python_quick_start.md](https://github.com/Kinovarobotics/kortex/blob/master/linked_md/python_quick_start.md)
  in `Kinovarobotics/kortex` for the current version) and install it by hand:

      pip install kortex_api-<version>-py3-none-any.whl

  Needed before running anything that imports `compliant_mode` or
  `compliant_torque_mode` without `FAKE_HARDWARE=1`.

## compliant_mode.py vs compliant_torque_mode.py
These are mutually exclusive! 
- Compliant_mode.py is the SAFE and FRIENDLY version that uses the official API. 
- Compliant_torque_mode.py is an experimental alternative, from-scratch compliant controller that is not guaranteed to work Yet.

Neither have worked 100% yet AFAIK. Compliant_mode.py = jerky and unsatisfactory compliance; Compliant_torque_mode.py is untested.
Supposed to be determined by set_posing_mode but I can't see through the spagheti code

Below are WIP moving the lengthy notes/documentation from each file to a consolidated reference readme.
## recorder.py
WIP

## replayer.py
### MotionReplayer class
Reads a rosbag2 trajectory bag produced by MotionRecorder (via
    trajectory_io.load_waypoints) and publishes it to the
    joint_trajectory_controller.

    The controller accepts trajectory_msgs/JointTrajectory
    over a topic or via the FollowJointTrajectory action.
    We use the simpler topic interface here.

    Reference:
      control.ros.org/jazzy/doc/ros2_controllers/
        joint_trajectory_controller/doc/userdoc.html

## compliant_torque_mode.py

### Control per joint/cycle

    tau_cmd = g(q)  -  tau_hat_friction  -  b * q_dot

  g(q)             gravity compensation torques (Pinocchio, Gen3 URDF)
  tau_hat_friction  friction observer estimate (FrictionObserver)
  b * q_dot        viscous damping to damp free swinging after release

### Kortex API sequence
Enter
    1. SetServoingMode -> LOW_LEVEL_SERVOING
    2. SetControlMode  -> TORQUE per actuator w/ device_id 1-7
    3. Start control-loop thread; call BaseCyclic.Refresh() at ~200 Hz

Exit (clean or fault)
    1. Send zero-torque frames (stop moving)
    2. SetControlMode -> POSITION  (per actuator)
    3. Refresh with current positions (position hold)
    4. SetServoingMode -> SINGLE_LEVEL_SERVOING

On any unhandled exception the same exit sequence runs from the
except clause to avoid arm crashing due to gravity

FAKE_HARDWARE=1 skips all gRPC calls; the control loop runs as a no-op
thread so the rest of the code behaves identically.

### Gravity-compensation notes

The Gen3 URDF has alternating RUBZ (unbounded revolute) and RZ joints.
Pinocchio's nq=11, nv=7 for this model.  Positions in RUBZ joints are
stored as [cos θ, sin θ] to avoid wrapping. 
The helper _build_q_pin() converts the 7-element angle vector accordingly.

Kortex API positions and velocities are in degrees / degrees-per-second.
They are converted to radians before being fed into Pinocchio or the
observer.

The gravity_sign parameter (per joint, default all +1.0) corrects for
any mismatch between Kortex encoder sign conventions and the URDF

## safety_monitor.py

Safety monitor for the compliant torque control loop.

All checks execute inside the control loop before every torque command.

After Any safety trip, this will execute the following steps:
  1. Returns a zero-torque vector (arm goes to gravity + inertia).
  2. Latches the monitor into a faulted state.
  3. Logs the offending joint index, measured value, and threshold.

The faulted state must be explicitly cleared via reset() (which is exposed as the `~/reset_fault` service on the recorder node).  This makes sure a human confirms the fault before the arm can re-enter torque mode.

Fault conditions
----------------
VELOCITY_TRIP
    Any joint's angular velocity exceeds its per-joint watchdog limit.
    Will be caused by the user letting go and a joint(s) dropping under gravity, or some kind of instability developing.

LOOP_OVERRUN:
    The control loop missed N consecutive cycles (each cycle took longer than dt_nominal * overrun_factor). Potential likely cause: OS scheduling
    jitter, or Python GIL contention. The Kortex firmware does also fault the arm independently if refresh calls stop for ~100 ms, but this
    fires first.

EXCEPTION:
    An unhandled Python exception inside the control loop. This is recorded so that the fault message is visible in the GUI log.


Torque saturation
-----------------
    Per-joint |tau_cmd| > tau_limit is silently clamped and doesnt cause a fault.
    Conservative 40 % of rated torques used by default.

### SafetyMonitor class
Check torque commands and joint velocities/clamp torques/latch faults.

    Parameters
    ----------
    tau_limits        : per-joint torque saturation limits (Nm, positive),
                        length n_joints
    vel_limits        : per-joint velocity watchdog thresholds (rad/s,
                        positive), length n_joints
    max_missed_cycles : consecutive overrun cycles that trigger a fault
    dt_nominal        : expected control loop period (seconds)
    overrun_factor    : dt > dt_nominal * overrun_factor counts as a miss
    logger            : rclpy Logger or stdlib Logger (optional)