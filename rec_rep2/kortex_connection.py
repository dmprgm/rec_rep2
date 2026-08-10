"""
Shared Kortex gRPC connection settings.

Both compliant_mode.py (admittance) and compliant_torque_mode.py (torque
control loop) connect to the same Gen3 over the same TCP/UDP ports with the
same credentials. This module holds those values in one place.
"""

import os

FAKE_HARDWARE = os.environ.get('FAKE_HARDWARE', '0').lower() in ('1', 'true', 'yes')

ROBOT_IP = os.environ.get('ROBOT_IP', '192.168.0.10')  # default NOT .1.10
ROBOT_PORT = 10000
ROBOT_PORT_RT = 10001  # UDP realtime port for BaseCyclic
USERNAME = 'admin'
PASSWORD = 'admin'
