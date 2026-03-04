"""
PID Controller — Anti-Windup with Debug Introspection
======================================================
Single-axis and dual-axis PID controllers with integral clamping,
output limiting, and per-term debug access.

Features:
    - Anti-windup protection (integral clamping)
    - Configurable output limits
    - Per-term introspection (p_term, i_term, d_term)
    - DualAxisPID for simultaneous X/Y control (gimbal, pan-tilt)
    - Zero dependencies — pure Python

Author: ByIbos
License: MIT
"""


class PIDController:
    """
    Single-axis PID controller with anti-windup protection.

    Computes a control signal from the error (setpoint - measurement)
    using the classic PID formula:

        output = Kp * error + Ki * integral(error) + Kd * d(error)/dt

    The integral term is clamped to prevent windup, and the final output
    is clamped to [-output_limit, +output_limit].

    Args:
        kp: Proportional gain. Higher = faster response, more overshoot.
        ki: Integral gain. Higher = eliminates steady-state error, risk of windup.
        kd: Derivative gain. Higher = dampens oscillations, sensitive to noise.
        output_limit: Maximum absolute output value.
        integral_limit: Maximum absolute integral accumulation (anti-windup).

    Example:
        >>> pid = PIDController(kp=0.5, ki=0.01, kd=0.3)
        >>> error = target_position - current_position
        >>> signal = pid.update(error, dt=1/30)
    """

    def __init__(self, kp=0.5, ki=0.01, kd=0.3, output_limit=1.0, integral_limit=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit

        self.prev_error = 0.0
        self.integral = 0.0
        self.output = 0.0

        # Debug / introspection values
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

    def update(self, error, dt=1/30):
        """
        Compute the control signal for a given error value.

        Args:
            error: The difference between the desired and current value.
                Positive = target is to the right/below, depending on axis.
            dt: Time step in seconds (default: 1/30 for 30 FPS).

        Returns:
            float: Normalized control signal in [-output_limit, +output_limit].
        """
        # Proportional term
        self.p_term = self.kp * error

        # Integral term with anti-windup clamping
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        self.i_term = self.ki * self.integral

        # Derivative term
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0
        self.d_term = self.kd * derivative
        self.prev_error = error

        # Total output (clamped)
        raw_output = self.p_term + self.i_term + self.d_term
        self.output = max(-self.output_limit, min(self.output_limit, raw_output))

        return self.output

    def reset(self):
        """Reset the controller state (integral, derivative, output)."""
        self.prev_error = 0.0
        self.integral = 0.0
        self.output = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0


class DualAxisPID:
    """
    Dual-axis PID controller for simultaneous X and Y control.

    Wraps two independent PIDController instances, one for each axis.
    Ideal for gimbal yaw/pitch, pan-tilt heads, or any 2D control problem.

    Args:
        kp: Proportional gain (applied to both axes).
        ki: Integral gain (applied to both axes).
        kd: Derivative gain (applied to both axes).

    Example:
        >>> pid = DualAxisPID(kp=0.4, ki=0.008, kd=0.25)
        >>> error_x = target_x - screen_center_x  # Horizontal error
        >>> error_y = target_y - screen_center_y  # Vertical error
        >>> signal_x, signal_y = pid.update(error_x, error_y, dt=1/30)
        >>> # signal_x → send to yaw servo/motor
        >>> # signal_y → send to pitch servo/motor
    """

    def __init__(self, kp=0.4, ki=0.008, kd=0.25):
        self.pid_x = PIDController(kp=kp, ki=ki, kd=kd)
        self.pid_y = PIDController(kp=kp, ki=ki, kd=kd)

    def update(self, error_x, error_y, dt=1/30):
        """
        Compute control signals for both axes.

        Args:
            error_x: Horizontal error in pixels (positive = target is to the right).
            error_y: Vertical error in pixels (positive = target is below).
            dt: Time step in seconds.

        Returns:
            tuple: (signal_x, signal_y) — each in [-1.0, +1.0].
        """
        sx = self.pid_x.update(error_x, dt)
        sy = self.pid_y.update(error_y, dt)
        return sx, sy

    def reset(self):
        """Reset both axis controllers."""
        self.pid_x.reset()
        self.pid_y.reset()
