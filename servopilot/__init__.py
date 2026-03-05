"""
servopilot — Anti-Windup PID Controller with Dual-Axis Support
==========================================================
A lightweight, zero-dependency PID controller library designed
for gimbal/servo control, robotics, and drone autopilot systems.

Quick Start:
    from servopilot import PIDController, DualAxisPID

    pid = DualAxisPID(kp=0.4, ki=0.008, kd=0.25)
    signal_x, signal_y = pid.update(error_x=50, error_y=-30)
"""

from servopilot.pid_controller import PIDController, DualAxisPID
from servopilot.feedforward import FeedforwardPID, DualAxisFeedforwardPID

__version__ = "1.1.0"
__author__ = "ByIbos"
__all__ = ["PIDController", "DualAxisPID", "FeedforwardPID", "DualAxisFeedforwardPID"]
