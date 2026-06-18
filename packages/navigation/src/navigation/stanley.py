import math
from typing import List, Tuple


class StanleyController:
    """
    Stanley path-tracking controller for a differential-drive (unicycle) robot.

    At each step it:
    1. Finds the closest path segment to the robot.
    2. Computes signed cross-track error (e) and heading error (ψ_e).
    3. Returns desired angular velocity via:
           omega = Kp * (ψ_e + atan(K * e / max(v, v_min)))

    Waypoints are (x, y) positions in metres, typically the graph-path
    node coordinates produced by the path-planning package.
    """

    def __init__(
        self,
        K: float = 2.0,
        Kp: float = 4.0,
        v_min: float = 0.05,
        max_omega: float = 3.0,
        waypoint_reach: float = 0.15,
    ):
        """
        Args:
            K:             Cross-track error gain in the atan term.
            Kp:            Heading-error to omega gain.
            v_min:         Minimum speed used in the atan denominator
                           (prevents division by zero at a standstill).
            max_omega:     Clamp on output omega [rad/s].
            waypoint_reach: Distance [m] at which a waypoint is considered reached.
        """
        self._K     = K
        self._Kp    = Kp
        self._v_min = v_min
        self._max_omega   = max_omega
        self._reach       = waypoint_reach

    def compute(
        self,
        x: float,
        y: float,
        theta: float,
        v: float,
        waypoints: List[Tuple[float, float]],
        wp_index: int,
    ) -> Tuple[float, int]:
        """
        Compute desired omega and advance the active waypoint index.

        Args:
            x, y, theta: Current robot pose (metres, radians).
            v:           Current forward speed (m/s).
            waypoints:   Ordered list of (x, y) targets.
            wp_index:    Index of the current target waypoint.

        Returns:
            (omega [rad/s], updated wp_index)
        """
        if len(waypoints) < 2 or wp_index >= len(waypoints):
            return 0.0, wp_index

        wp_index = self._advance(waypoints, wp_index, x, y, theta)
        wp_index = min(wp_index, len(waypoints) - 1)

        # Segment from previous to current target waypoint
        prev = waypoints[max(0, wp_index - 1)]
        curr = waypoints[wp_index]

        seg_dx  = curr[0] - prev[0]
        seg_dy  = curr[1] - prev[1]
        seg_len = math.hypot(seg_dx, seg_dy) + 1e-9
        seg_angle = math.atan2(seg_dy, seg_dx)

        # Signed cross-track error (positive = robot is left of the segment)
        ex  = x - prev[0]
        ey  = y - prev[1]
        cte = ex * (-seg_dy / seg_len) + ey * (seg_dx / seg_len)

        # Heading error
        heading_err = _wrap(seg_angle - theta)

        # Stanley formula
        v_eff = max(abs(v), self._v_min)
        delta = heading_err + math.atan2(self._K * cte, v_eff)
        omega = self._Kp * delta
        omega = max(-self._max_omega, min(self._max_omega, omega))

        return omega, wp_index

    def _advance(
        self,
        waypoints: List[Tuple[float, float]],
        idx: int,
        x: float,
        y: float,
        theta: float,
    ) -> int:
        """Advance past waypoints that are already reached or clearly behind."""
        fwd_x = math.cos(theta)
        fwd_y = math.sin(theta)
        while idx < len(waypoints) - 1:
            wx, wy = waypoints[idx]
            dx   = wx - x
            dy   = wy - y
            dist = math.hypot(dx, dy)
            dot  = dx * fwd_x + dy * fwd_y
            if dist < self._reach or dot < -0.05:
                idx += 1
            else:
                break
        return idx


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))