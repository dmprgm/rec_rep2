"""
rclpy service-client backend that drives the C++ compliant_torque_servo
node (rec_rep2_servo package) instead of running the torque control loop
in-process. Duck-types CompliantTorqueMode's public surface (enter, exit,
is_faulted, reset_fault, close) so MotionRecorder can use either backend
interchangeably via the use_cpp_servo_backend parameter.

Only imported when that parameter is true, so nothing here needs to be
importable (i.e. rec_rep2_servo need not be built) for the default,
Python-only path. See docs/cpp_servoing.md.
"""

import rclpy
from rec_rep2_servo.srv import GetStatus
from std_srvs.srv import Trigger


class CppServoBackend:

    def __init__(self, node, servo_node_name: str, service_timeout_sec: float = 5.0):
        self._node = node
        self._log = node.get_logger()
        self._timeout = service_timeout_sec

        ns = f'/{servo_node_name}'
        self._enter_cli = node.create_client(Trigger, f'{ns}/enter_torque_mode')
        self._exit_cli = node.create_client(Trigger, f'{ns}/exit_torque_mode')
        self._reset_fault_cli = node.create_client(Trigger, f'{ns}/reset_fault')
        self._get_status_cli = node.create_client(GetStatus, f'{ns}/get_status')

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=self._timeout):
            raise RuntimeError(
                f'Service {client.srv_name} not available! is rec_rep2_servo\'s '
                'compliant_torque_node running (use_cpp_servo_backend:=true)?'
            )
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self._timeout)
        if future.result() is None:
            raise RuntimeError(f'Service call to {client.srv_name} timed out')
        return future.result()

    def enter(self) -> bool:
        """Switch to torque mode and start the C++ control loop."""
        try:
            resp = self._call(self._enter_cli, Trigger.Request())
        except RuntimeError as exc:
            self._log.error(f'CppServoBackend.enter() failed: {exc}')
            return False
        if not resp.success:
            self._log.error(f'compliant_torque_servo refused enter: {resp.message}')
        return resp.success

    def exit(self) -> None:
        """Stop the C++ control loop; blocks until it confirms safe-exit."""
        try:
            self._call(self._exit_cli, Trigger.Request())
        except RuntimeError as exc:
            self._log.error(f'CppServoBackend.exit() failed: {exc}')

    @property
    def is_faulted(self) -> bool:
        """Return True if the C++ node's safety monitor has a latched fault."""
        try:
            resp = self._call(self._get_status_cli, GetStatus.Request())
        except RuntimeError as exc:
            self._log.error(f'CppServoBackend.is_faulted check failed: {exc}')
            return True  # fail safe: treat an unreachable servo node as faulted
        return resp.faulted

    def reset_fault(self) -> None:
        """Clear the latched safety fault on the C++ node."""
        try:
            self._call(self._reset_fault_cli, Trigger.Request())
        except RuntimeError as exc:
            self._log.error(f'CppServoBackend.reset_fault() failed: {exc}')

    def close(self) -> None:
        """Symmetry with CompliantTorqueMode.close(); the C++ node owns its
        own Kortex connection lifecycle, so this just ensures torque mode
        is off."""
        self.exit()
