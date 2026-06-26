#!/usr/bin/env python3
"""
control_node — PID speed regulation, the final stage before actuation.

navigation_node (task 3) computes a desired forward speed and steering rate
from the Stanley path-follower, but the speed it commands is open-loop: it
is only a heuristic v * speed_factor, with no feedback on the speed the
robot is actually achieving (wheel slip, battery sag, etc. are not
corrected for). This node closes that loop. It measures actual forward
speed from consecutive localization poses and runs a PID controller to
correct the commanded v before it reaches the wheels. Steering (omega) is
passed straight through unchanged — Stanley already handles cross-track
and heading control in task 3, so this node does not touch it.

Subscribed topics:
    navigation_node/car_cmd     (duckietown_msgs/Twist2DStamped)  desired v + omega (setpoint)
    localization_node/pose      (geometry_msgs/Pose2D)            pose, used to derive actual speed

Published topics:
    car_cmd_switch_node/cmd     (duckietown_msgs/Twist2DStamped)  PID-corrected v + passthrough omega
                                 Final command to the actuator (see navigation_node's docstring).

Parameters:
    ~Kp, ~Ki, ~Kd        PID gains                              (default 1.0, 0.5, 0.0)
    ~max_correction      clamp on the PID correction term [m/s] (default 0.10)
    ~integral_limit      anti-windup clamp on the integral term (default 0.5)
    ~v_min, ~v_max       final speed envelope sent to the wheels (default 0.0, 0.30)
    ~min_dt              minimum time between speed updates [s] (default 0.05)
                          Pose2D has no timestamp, so dt comes from wall-clock
                          callback arrival time — below this floor the speed
                          estimate (distance / dt) is dominated by jitter noise.
"""

import math
import rospy

from geometry_msgs.msg import Pose2D
from duckietown_msgs.msg import Twist2DStamped

from duckie_control.pid import PIDController


class ControlNode:
    def __init__(self):
        self.node_name = rospy.get_name()

        Kp = float(rospy.get_param("~Kp", 1.0))
        Ki = float(rospy.get_param("~Ki", 0.5))
        Kd = float(rospy.get_param("~Kd", 0.0))
        max_correction = float(rospy.get_param("~max_correction", 0.10))
        integral_limit = float(rospy.get_param("~integral_limit", 0.5))
        self._v_min = float(rospy.get_param("~v_min", 0.0))
        self._v_max = float(rospy.get_param("~v_max", 0.30))
        self._min_dt = float(rospy.get_param("~min_dt", 0.05))

        self._pid = PIDController(
            Kp, Ki, Kd,
            output_limits=(-max_correction, max_correction),
            integral_limit=integral_limit,
        )

        self._setpoint_v = 0.0
        self._setpoint_omega = 0.0
        self._prev_pose = None
        self._prev_pose_time = None

        self._pub_cmd = rospy.Publisher(
            "car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1
        )

        rospy.Subscriber("navigation_node/car_cmd", Twist2DStamped, self._cb_setpoint)
        rospy.Subscriber("localization_node/pose",  Pose2D,         self._cb_pose)

        rospy.loginfo(f"[{self.node_name}] Ready.")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_setpoint(self, msg: Twist2DStamped) -> None:
        self._setpoint_v = msg.v
        self._setpoint_omega = msg.omega

    def _cb_pose(self, msg: Pose2D) -> None:
        now = rospy.Time.now()

        if self._prev_pose is None:
            self._prev_pose = msg
            self._prev_pose_time = now
            return

        dt = (now - self._prev_pose_time).to_sec()
        if dt < self._min_dt:
            # Too little wall-clock time has passed since the last sample to
            # get a reliable speed estimate — wait for it to accumulate
            # instead of dividing by a noise-dominated dt.
            return

        v_measured = math.hypot(msg.x - self._prev_pose.x, msg.y - self._prev_pose.y) / dt
        self._prev_pose = msg
        self._prev_pose_time = now

        if self._setpoint_v <= 0.0:
            # Standing still / no path yet — don't let the integral wind up
            # against a zero setpoint while idle.
            self._pid.reset()
            self._publish(0.0, self._setpoint_omega)
            return

        correction = self._pid.update(self._setpoint_v - v_measured, dt)
        v_cmd = max(self._v_min, min(self._v_max, self._setpoint_v + correction))
        self._publish(v_cmd, self._setpoint_omega)

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish(self, v: float, omega: float) -> None:
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = v
        msg.omega = omega
        self._pub_cmd.publish(msg)


if __name__ == "__main__":
    rospy.init_node("control_node", anonymous=False)
    node = ControlNode()
    rospy.spin()