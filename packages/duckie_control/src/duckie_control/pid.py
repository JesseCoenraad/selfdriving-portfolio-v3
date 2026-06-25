"""
PIDController — simple discrete-time PID with output clamping and integral
anti-windup, used by control_node for closed-loop speed regulation.
"""


class PIDController:
    def __init__(self, Kp, Ki, Kd, output_limits=(-float("inf"), float("inf")),
                 integral_limit=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self._out_min, self._out_max = output_limits
        self._integral_limit = integral_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._has_prev = False

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._has_prev = False

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return self._clamp(self.Kp * error)

        self._integral += error * dt
        if self._integral_limit is not None:
            self._integral = max(-self._integral_limit, min(self._integral_limit, self._integral))

        derivative = (error - self._prev_error) / dt if self._has_prev else 0.0
        self._prev_error = error
        self._has_prev = True

        output = self.Kp * error + self.Ki * self._integral + self.Kd * derivative
        return self._clamp(output)

    def _clamp(self, value: float) -> float:
        return max(self._out_min, min(self._out_max, value))