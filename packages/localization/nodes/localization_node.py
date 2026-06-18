#!/usr/bin/env python3
"""
localization_node — wheel-encoder odometry localisation on a planned path.

Subscribed topics:
  left_wheel_encoder_node/tick    (duckietown_msgs/WheelEncoderStamped)
  right_wheel_encoder_node/tick   (duckietown_msgs/WheelEncoderStamped)
  path_planner_node/path          (path_planning/PathPlan)

Published topics:
  ~pose            (geometry_msgs/Pose2D)   current estimated pose in metres
  ~current_node    (std_msgs/String)        node the robot last reached
  ~next_node       (std_msgs/String)        next node on the planned path
  ~remaining_path  (path_planning/PathPlan) remaining path from current node

Parameters:
  ~wheel_radius        wheel radius in metres          (default: 0.0318)
  ~baseline            wheel-to-wheel distance in m    (default: 0.1)
  ~ticks_per_rev       encoder ticks per revolution    (default: 135)
  ~tile_size           metres per tile                 (default: 0.585)
  ~arrival_threshold   metres to consider node reached (default: 0.15)
  ~map_file            path to map YAML                (default: path_planning/config/map.yaml)
"""

import os
import threading
import yaml
import rospy
import rospkg

from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from duckietown_msgs.msg import WheelEncoderStamped

from localization.odometry import DifferentialDriveOdometry, NodeTracker
from path_planning.msg import PathPlan


class LocalizationNode:
    def __init__(self):
        self.node_name = rospy.get_name()
        self._lock = threading.Lock()

        # --- Robot parameters ---
        wheel_radius  = float(rospy.get_param("~wheel_radius",       0.0318))
        baseline      = float(rospy.get_param("~baseline",           0.1))
        ticks_per_rev = int(rospy.get_param(  "~ticks_per_rev",      135))
        self._tile_size          = float(rospy.get_param("~tile_size",          0.585))
        self._arrival_threshold  = float(rospy.get_param("~arrival_threshold",  0.15))

        self._odometry = DifferentialDriveOdometry(wheel_radius, baseline, ticks_per_rev)
        self._tracker: NodeTracker = None

        # --- Map: node coordinates ---
        map_file = rospy.get_param("~map_file", self._default_map_path())
        self._coords = self._load_coords(map_file)

        # --- Latest raw tick values ---
        self._left_ticks:  int = None
        self._right_ticks: int = None

        # --- Publishers ---
        self._pub_pose      = rospy.Publisher("~pose",           Pose2D,   queue_size=1)
        self._pub_current   = rospy.Publisher("~current_node",   String,   queue_size=1, latch=True)
        self._pub_next      = rospy.Publisher("~next_node",      String,   queue_size=1, latch=True)
        self._pub_remaining = rospy.Publisher("~remaining_path", PathPlan, queue_size=1, latch=True)

        # --- Subscribers ---
        rospy.Subscriber("left_wheel_encoder_node/tick",  WheelEncoderStamped, self._cb_left)
        rospy.Subscriber("right_wheel_encoder_node/tick", WheelEncoderStamped, self._cb_right)
        rospy.Subscriber("path_planner_node/path",        PathPlan,            self._cb_path)

        rospy.loginfo(f"[{self.node_name}] Ready. "
                      f"Loaded {len(self._coords)} node coordinates.")

    # ------------------------------------------------------------------
    # Encoder callbacks
    # ------------------------------------------------------------------

    def _cb_left(self, msg: WheelEncoderStamped) -> None:
        with self._lock:
            self._left_ticks = msg.data
            self._update()

    def _cb_right(self, msg: WheelEncoderStamped) -> None:
        with self._lock:
            self._right_ticks = msg.data
            self._update()

    def _update(self) -> None:
        """Called inside lock. Updates odometry and publishes state."""
        if self._left_ticks is None or self._right_ticks is None:
            return

        self._odometry.update(self._left_ticks, self._right_ticks)
        x, y, theta = self._odometry.pose

        self._pub_pose.publish(Pose2D(x=x, y=y, theta=theta))

        if self._tracker is None:
            return

        current, next_n, remaining = self._tracker.update(x, y)

        self._pub_current.publish(String(data=current))
        self._pub_next.publish(String(data=next_n if next_n else ""))

        remaining_msg = PathPlan()
        remaining_msg.path = remaining
        remaining_msg.success = True
        self._pub_remaining.publish(remaining_msg)

        if self._tracker.at_goal:
            rospy.loginfo_throttle(
                5.0, f"[{self.node_name}] Arrived at goal node '{current}'."
            )

    # ------------------------------------------------------------------
    # Path callback
    # ------------------------------------------------------------------

    def _cb_path(self, msg: PathPlan) -> None:
        if not msg.success or not msg.path:
            rospy.logwarn(f"[{self.node_name}] Received empty/failed path — ignored.")
            return

        start_node = msg.path[0]

        with self._lock:
            # Reset odometry origin to the start node's map coordinate
            if start_node in self._coords:
                sx, sy = self._coords[start_node]
                self._odometry.reset(x=sx, y=sy, theta=0.0)
                rospy.loginfo(
                    f"[{self.node_name}] Pose reset to '{start_node}' "
                    f"({sx:.3f} m, {sy:.3f} m)."
                )
            else:
                self._odometry.reset()
                rospy.logwarn(
                    f"[{self.node_name}] No coordinates for start node '{start_node}'. "
                    "Resetting pose to origin."
                )

            self._tracker = NodeTracker(
                list(msg.path), self._coords, self._arrival_threshold
            )

        rospy.loginfo(
            f"[{self.node_name}] Tracking path: {' -> '.join(msg.path)}"
        )

    # ------------------------------------------------------------------
    # Map loading
    # ------------------------------------------------------------------

    def _load_coords(self, map_file: str) -> dict:
        """Load node (x, y) in metres from a YAML map file."""
        if not os.path.isfile(map_file):
            rospy.logwarn(
                f"[{self.node_name}] Map file not found: '{map_file}'. "
                "Node tracking will be disabled."
            )
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
        return os.path.join(
            rospack.get_path("path_planning"), "config", "map.yaml"
        )


if __name__ == "__main__":
    rospy.init_node("localization_node", anonymous=False)
    node = LocalizationNode()
    rospy.spin()