"""
Feedforward PID Controller — PID with Known Disturbance Compensation
=====================================================================
Extends the standard PIDController with a feedforward term that allows
compensation for known/predictable disturbances (e.g., wind, gravity,
drone velocity) before the error even manifests.

Features:
    - All standard PID features (anti-windup, output clamping, debug)
    - Additive feedforward signal injected before output limiting
    - Optional feedforward gain (kf) for scaling
    - DualAxisFeedforwardPID for gimbal/pan-tilt systems

Author: ByIbos
License: MIT
"""

from servopilot.pid_controller import PIDController


class FeedforwardPID(PIDController):
    """
    PID controller with a feedforward compensation term.

    The feedforward signal is added to the PID output before clamping,
    allowing the controller to anticipate known disturbances rather than
    waiting for the error to appear.

    Output = clamp(P + I + D + kf * feedforward, -limit, +limit)

    Args:
        kp: Proportional gain.
        ki: Integral gain.
        kd: Derivative gain.
        kf: Feedforward gain (default: 1.0).
        output_limit: Maximum absolute output value (default: 1.0).

    Example:
        >>> pid = FeedforwardPID(kp=0.5, ki=0.01, kd=0.1, kf=0.3)
        >>> # Wind pushing target right at 5 px/frame:
        >>> signal = pid.update(error=20, dt=0.033, feedforward=5.0)
    """

    def __init__(self, kp=0.5, ki=0.0, kd=0.0, kf=1.0, output_limit=1.0):
        super().__init__(kp=kp, ki=ki, kd=kd, output_limit=output_limit)
        self.kf = kf
        self.ff_term = 0.0

    def update(self, error, dt, feedforward=0.0):
        """
        Compute PID + feedforward output.

        Args:
            error: Current error (setpoint - measurement).
            dt: Time delta since last update (seconds).
            feedforward: Known disturbance signal to compensate for.
                e.g., target velocity, wind speed, gravity component.

        Returns:
            float: Clamped control signal in [-output_limit, +output_limit].
        """
        if dt <= 0:
            dt = 1e-6

        # Standard PID terms
        self.p_term = self.kp * error
        self.i_term += self.ki * error * dt
        self.i_term = max(-self.output_limit, min(self.output_limit, self.i_term))

        derivative = (error - self.prev_error) / dt
        self.d_term = self.kd * derivative
        self.prev_error = error

        # Feedforward term
        self.ff_term = self.kf * feedforward

        # Combined output
        output = self.p_term + self.i_term + self.d_term + self.ff_term
        return max(-self.output_limit, min(self.output_limit, output))

    def get_debug(self):
        """Return all internal terms including feedforward."""
        return {
            'P': self.p_term,
            'I': self.i_term,
            'D': self.d_term,
            'FF': self.ff_term,
            'kf': self.kf,
        }


class DualAxisFeedforwardPID:
    """
    Dual-axis feedforward PID for gimbal/pan-tilt systems.

    Wraps two FeedforwardPID instances (X and Y axes) with independent
    feedforward signals for each axis.

    Args:
        kp, ki, kd: PID gains (shared for both axes).
        kf: Feedforward gain (default: 1.0).
        output_limit: Max absolute output (default: 1.0).

    Example:
        >>> pid = DualAxisFeedforwardPID(kp=0.4, ki=0.01, kd=0.2, kf=0.3)
        >>> sx, sy = pid.update(
        ...     error_x=30, error_y=-15,
        ...     dt=0.033,
        ...     ff_x=5.0,   # target moving right
        ...     ff_y=-2.0   # target moving up
        ... )
    """

    def __init__(self, kp=0.5, ki=0.0, kd=0.0, kf=1.0, output_limit=1.0):
        self.pid_x = FeedforwardPID(kp, ki, kd, kf, output_limit)
        self.pid_y = FeedforwardPID(kp, ki, kd, kf, output_limit)

    def update(self, error_x, error_y, dt, ff_x=0.0, ff_y=0.0):
        """
        Update both axes with error and feedforward signals.

        Args:
            error_x: Horizontal error (pixels from center).
            error_y: Vertical error (pixels from center).
            dt: Time delta (seconds).
            ff_x: Feedforward for X axis (e.g., target horizontal velocity).
            ff_y: Feedforward for Y axis (e.g., target vertical velocity).

        Returns:
            tuple: (signal_x, signal_y) — clamped control signals.
        """
        sx = self.pid_x.update(error_x, dt, feedforward=ff_x)
        sy = self.pid_y.update(error_y, dt, feedforward=ff_y)
        return sx, sy

    def reset(self):
        """Reset both axis controllers."""
        self.pid_x.reset()
        self.pid_y.reset()
