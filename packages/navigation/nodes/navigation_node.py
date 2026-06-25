#!/usr/bin/env python3
"""
navigation_node — Stanley path-following with duckie avoidance.

State machine:
    IDLE            No path received yet. Robot stands still.
    FOLLOW_PATH     Stanley controller drives toward the next waypoint.
    AVOIDING_DUCKIE Duckie detected ahead. Robot stops, waits, then tries
                    a slow bypass if the duckie does not move.
    DONE            Goal node reached. Robot stands still.

Subscribed topics:
    path_planner_node/path          (path_planning/PathPlan)
    localization_node/pose          (geometry_msgs/Pose2D)
    camera_node/image/compressed    (sensor_msgs/CompressedImage)

Published topics:
    ~car_cmd    (duckietown_msgs/Twist2DStamped)   desired v [m/s] + omega [rad/s]
                 Setpoint only — task 4's control_node (duckie_control) applies PID
                 speed correction and publishes the final command to the actuator.

Parameters:
    ~model_path         path to best.onnx
    ~nominal_speed      forward speed during normal driving  (default 0.15 m/s)
    ~bypass_speed       forward speed while steering around a duckie (default 0.08 m/s)
    ~K_avoid            lateral avoidance steering gain      (default 2.0)
    ~goal_threshold     metres to consider goal reached      (default 0.20 m)
    ~detect_every_n     run detector every N camera frames   (default 5)
    ~tile_size          metres per tile                      (default 0.585)
    ~stanley_K          Stanley cross-track gain             (default 2.0)
    ~stanley_Kp         Stanley heading→omega gain           (default 4.0)
    ~map_file           path to map YAML (defaults to path_planning/config/map.yaml)
"""

import os
import math
import threading
import yaml
import rospy
import rospkg
import numpy as np

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped

from duckie_navigation.stanley import StanleyController
from duckie_navigation.duckie_detector import DuckieDetector
from path_planning.msg import PathPlan


# Navigation states
IDLE            = "IDLE"
FOLLOW_PATH     = "FOLLOW_PATH"
AVOIDING_DUCKIE = "AVOIDING_DUCKIE"
DONE            = "DONE"


class NavigationNode:
    def __init__(self):
        self.node_name = rospy.get_name()
        self._lock = threading.Lock()

        # Parameters
        self._nominal_speed  = float(rospy.get_param("~nominal_speed",  0.15))
        self._bypass_speed   = float(rospy.get_param("~bypass_speed",   0.08))
        self._K_avoid        = float(rospy.get_param("~K_avoid",        2.0))
        self._goal_threshold = float(rospy.get_param("~goal_threshold", 0.20))
        self._detect_every_n = int(rospy.get_param(  "~detect_every_n", 5))
        self._tile_size      = float(rospy.get_param("~tile_size",      0.585))

        # Stanley controller
        self._stanley = StanleyController(
            K=float(rospy.get_param("~stanley_K",  2.0)),
            Kp=float(rospy.get_param("~stanley_Kp", 4.0)),
        )

        # Map node coordinates
        map_file = rospy.get_param("~map_file", self._default_map_path())
        self._coords = self._load_coords(map_file)

        # Duckie detector (optional — gracefully disabled if model absent)
        model_path = rospy.get_param("~model_path", self._default_model_path())
        self._detector = self._init_detector(model_path)

        # Navigation state
        self._state:      str     = IDLE
        self._waypoints:  list    = []
        self._wp_index:   int     = 0
        self._pose:       Pose2D  = Pose2D()
        # _duckie_info is None when clear, or (cx_norm, cy_norm) when detected
        self._duckie_info         = None
        self._frame_counter: int  = 0

        # Publisher — desired (uncorrected) v/omega. control_node (task 4) applies
        # PID speed correction and republishes the final command to the actuator.
        self._pub_cmd = rospy.Publisher(
            "~car_cmd", Twist2DStamped, queue_size=1
        )

        # Subscribers
        rospy.Subscriber("path_planner_node/path",       PathPlan,        self._cb_path)
        rospy.Subscriber("localization_node/pose",       Pose2D,          self._cb_pose)
        rospy.Subscriber("camera_node/image/compressed", CompressedImage, self._cb_image)

        rospy.loginfo(f"[{self.node_name}] Ready. "
                      f"Detector: {'ON' if self._detector else 'OFF (no model)'}.")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_path(self, msg: PathPlan) -> None:
        if not msg.success or not msg.path:
            rospy.logwarn(f"[{self.node_name}] Received empty/failed path — ignored.")
            return

        waypoints = []
        for node in msg.path:
            if node in self._coords:
                waypoints.append(self._coords[node])
            else:
                rospy.logwarn(f"[{self.node_name}] No coordinates for node '{node}'.")

        if len(waypoints) < 2:
            rospy.logwarn(f"[{self.node_name}] Not enough known waypoints to navigate.")
            return

        with self._lock:
            self._waypoints = waypoints
            self._wp_index  = 0
            self._state     = FOLLOW_PATH

        rospy.loginfo(
            f"[{self.node_name}] New path: {' -> '.join(msg.path)} "
            f"({len(waypoints)} waypoints)."
        )

    def _cb_pose(self, msg: Pose2D) -> None:
        with self._lock:
            self._pose = msg
            self._control_step()

    def _cb_image(self, msg: CompressedImage) -> None:
        if self._detector is None:
            return
        self._frame_counter += 1
        if self._frame_counter % self._detect_every_n != 0:
            return

        image = _decode_compressed(msg)
        if image is None:
            return

        info = self._detector.get_path_duckie(image)
        with self._lock:
            self._duckie_info = info

    # ------------------------------------------------------------------
    # Control loop (called from _cb_pose, inside lock)
    # ------------------------------------------------------------------

    def _control_step(self) -> None:
        state = self._state

        if state in (IDLE, DONE):
            self._publish(v=0.0, omega=0.0)
            return

        x, y, theta = self._pose.x, self._pose.y, self._pose.theta

        # ── Check goal reached ──────────────────────────────────────────
        if self._waypoints:
            gx, gy = self._waypoints[-1]
            if math.hypot(x - gx, y - gy) < self._goal_threshold:
                self._state = DONE
                self._publish(v=0.0, omega=0.0)
                rospy.loginfo(f"[{self.node_name}] Goal reached.")
                return

        # ── AVOIDING_DUCKIE ─────────────────────────────────────────────
        if state == AVOIDING_DUCKIE:
            info = self._duckie_info

            if info is None:
                # Duckie no longer visible — resume normal path following
                self._state = FOLLOW_PATH
                rospy.loginfo(f"[{self.node_name}] Duckie passed, resuming.")
                return

            cx_norm, _ = info   # [-0.5, 0.5]: negative=left, positive=right

            # Stanley still provides the base heading toward the goal.
            # On top of that, steer away from the duckie:
            #   duckie on right (cx_norm > 0) → add positive omega (steer left)
            #   duckie on left  (cx_norm < 0) → add negative omega (steer right)
            stanley_omega, self._wp_index = self._stanley.compute(
                x, y, theta, self._bypass_speed,
                self._waypoints, self._wp_index,
            )
            avoid_omega = self._K_avoid * cx_norm
            omega = stanley_omega + avoid_omega
            omega = max(-self._stanley._max_omega, min(self._stanley._max_omega, omega))

            self._publish(v=self._bypass_speed, omega=omega)
            return

        # ── FOLLOW_PATH ─────────────────────────────────────────────────
        if self._duckie_info is not None:
            self._state = AVOIDING_DUCKIE
            rospy.loginfo(f"[{self.node_name}] Duckie detected — initiating avoidance.")
            return

        omega, self._wp_index = self._stanley.compute(
            x, y, theta, self._nominal_speed,
            self._waypoints, self._wp_index,
        )

        # Reduce speed during sharp turns
        speed_factor = max(0.5, 1.0 - abs(omega) / (self._stanley._max_omega * 2))
        v = self._nominal_speed * speed_factor

        self._publish(v=v, omega=omega)

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish(self, v: float, omega: float) -> None:
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v     = v
        msg.omega = omega
        self._pub_cmd.publish(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _init_detector(self, model_path: str):
        if not os.path.isfile(model_path):
            rospy.logwarn(
                f"[{self.node_name}] ONNX model not found at '{model_path}'. "
                "Duckie detection disabled."
            )
            return None
        try:
            detector = DuckieDetector(model_path)
            rospy.loginfo(f"[{self.node_name}] Loaded duckie detector from '{model_path}'.")
            return detector
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Failed to load detector: {e}")
            return None

    def _load_coords(self, map_file: str) -> dict:
        if not os.path.isfile(map_file):
            rospy.logwarn(f"[{self.node_name}] Map file not found: '{map_file}'.")
            return {}
        with open(map_file) as fh:
            data = yaml.safe_load(fh)
        coords = {}
        for node in data.get("nodes", []):
            if "x" in node and "y" in node:
                coords[str(node["id"])] = (
                    float(node["x"]) * self._tile_size,
                    float(node["y"]) * self._tile_size,
                )
        return coords

    @staticmethod
    def _default_map_path() -> str:
        rospack = rospkg.RosPack()
        return os.path.join(rospack.get_path("path_planning"), "config", "map.yaml")

    @staticmethod
    def _default_model_path() -> str:
        rospack = rospkg.RosPack()
        return os.path.join(rospack.get_path("duckie_navigation"), "models", "best.onnx")


# ------------------------------------------------------------------
# Image decoding
# ------------------------------------------------------------------

def _decode_compressed(msg: CompressedImage):
    try:
        import cv2
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    except Exception:
        return None


if __name__ == "__main__":
    rospy.init_node("navigation_node", anonymous=False)
    node = NavigationNode()
    rospy.spin()