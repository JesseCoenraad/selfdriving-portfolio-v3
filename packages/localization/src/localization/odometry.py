import math
from typing import Dict, List, Optional, Tuple


class DifferentialDriveOdometry:
    """
    Dead-reckoning pose estimator for a differential-drive robot.

    Converts cumulative wheel encoder ticks to a (x, y, theta) pose using
    midpoint (Runge-Kutta 2) integration, which handles curves more accurately
    than simple Euler integration.

    Coordinate convention: x forward, y left, theta CCW from x-axis.
    """

    def __init__(self, wheel_radius: float, baseline: float, ticks_per_rev: int):
        """
        Args:
            wheel_radius:  Radius of each wheel in meters.
            baseline:      Distance between the two wheel contact points in meters.
            ticks_per_rev: Encoder ticks per full wheel revolution.
        """
        self._dist_per_tick = 2.0 * math.pi * wheel_radius / ticks_per_rev
        self._baseline = baseline

        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0

        self._left_prev: Optional[int] = None
        self._right_prev: Optional[int] = None

    def update(self, left_ticks: int, right_ticks: int) -> None:
        """
        Update pose from new cumulative encoder tick counts.
        The first call initialises the reference tick counts; no pose change occurs.
        """
        if self._left_prev is None:
            self._left_prev = left_ticks
            self._right_prev = right_ticks
            return

        d_left  = (left_ticks  - self._left_prev)  * self._dist_per_tick
        d_right = (right_ticks - self._right_prev) * self._dist_per_tick
        self._left_prev  = left_ticks
        self._right_prev = right_ticks

        d      = (d_left + d_right) * 0.5
        dtheta = (d_right - d_left) / self._baseline

        # Midpoint angle for integration
        mid_theta = self.theta + dtheta * 0.5
        self.x     += d * math.cos(mid_theta)
        self.y     += d * math.sin(mid_theta)
        self.theta  = _wrap(self.theta + dtheta)

    @property
    def pose(self) -> Tuple[float, float, float]:
        """Current pose as (x [m], y [m], theta [rad])."""
        return self.x, self.y, self.theta

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """Reset pose to given values and clear stored tick references."""
        self.x, self.y, self.theta = x, y, theta
        self._left_prev  = None
        self._right_prev = None


class NodeTracker:
    """
    Tracks the robot's progress along a planned path by comparing the
    estimated pose against node coordinates.

    The robot is considered to have reached the next node once its distance
    to that node falls below `arrival_threshold`.
    """

    def __init__(
        self,
        path: List[str],
        coords: Dict[str, Tuple[float, float]],
        arrival_threshold: float,
    ):
        """
        Args:
            path:               Ordered list of node names, start to goal.
            coords:             Map of node name to (x, y) in meters.
            arrival_threshold:  Distance in meters to consider a node reached.
        """
        if not path:
            raise ValueError("Path must not be empty")
        self._path      = path
        self._coords    = coords
        self._threshold = arrival_threshold
        self._idx       = 0  # index of the current node

    def update(self, x: float, y: float) -> Tuple[str, Optional[str], List[str]]:
        """
        Update tracker with current pose and advance along the path if
        the robot is close enough to the next node.

        Returns:
            current_node:   Name of the node the robot last reached.
            next_node:      Name of the upcoming node, or None at goal.
            remaining_path: Sublist of path from current_node onward.
        """
        if self._idx < len(self._path) - 1:
            next_node = self._path[self._idx + 1]
            if next_node in self._coords:
                nx, ny = self._coords[next_node]
                dist = math.sqrt((x - nx) ** 2 + (y - ny) ** 2)
                if dist < self._threshold:
                    self._idx += 1

        current  = self._path[self._idx]
        next_n   = self._path[self._idx + 1] if self._idx < len(self._path) - 1 else None
        remaining = self._path[self._idx:]
        return current, next_n, remaining

    @property
    def at_goal(self) -> bool:
        """True when the robot has reached the final node in the path."""
        return self._idx >= len(self._path) - 1


def _wrap(theta: float) -> float:
    """Normalise angle to [-pi, pi]."""
    return math.atan2(math.sin(theta), math.cos(theta))