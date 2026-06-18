#!/usr/bin/env python3
"""
path_planner_node — ROS node for Dijkstra path planning.

Subscribed topics : none (map is loaded from config at startup)
Published topics  : ~path          (path_planning/PathPlan)
Services          : ~plan_path     (path_planning/PlanPath)
Parameters        : ~map_file      path to YAML or CSV map (default: package config/map.yaml)
                    ~start         default start node  (default: "S")
                    ~goal          default goal node   (default: "T")
                    ~auto_plan     plan start→goal on startup (default: true)
"""

import os
import yaml
import rospy

from path_planning.dijkstra import (
    dijkstra,
    build_graph_from_edges,
    build_graph_from_csv,
    Graph,
)
from path_planning.msg import PathPlan
from path_planning.srv import PlanPath, PlanPathResponse


class PathPlannerNode:
    def __init__(self):
        self.node_name = rospy.get_name()

        map_file = rospy.get_param("~map_file", self._default_map_path())
        self.graph = self._load_map(map_file)

        self.pub_path = rospy.Publisher("~path", PathPlan, queue_size=1, latch=True)
        self.srv_plan = rospy.Service("~plan_path", PlanPath, self._handle_plan_path)

        rospy.loginfo(f"[{self.node_name}] Loaded map with "
                      f"{len(self.graph)} nodes from '{map_file}'.")

        if rospy.get_param("~auto_plan", True):
            start = rospy.get_param("~start", "S")
            goal = rospy.get_param("~goal", "T")
            self._plan_and_publish(start, goal)

    # ------------------------------------------------------------------
    # Service handler
    # ------------------------------------------------------------------

    def _handle_plan_path(self, req):
        path, cost = self._plan_and_publish(req.start, req.goal)

        resp = PlanPathResponse()
        if path is not None:
            resp.path = path
            resp.cost = cost
            resp.success = True
            resp.message = f"Path found: {' → '.join(path)} (cost {cost:.2f})"
        else:
            resp.path = []
            resp.cost = float("inf")
            resp.success = False
            resp.message = f"No path from '{req.start}' to '{req.goal}'"

        rospy.loginfo(f"[{self.node_name}] {resp.message}")
        return resp

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def _plan_and_publish(self, start: str, goal: str):
        path, cost = dijkstra(self.graph, start, goal)

        msg = PathPlan()
        msg.start = start
        msg.goal = goal
        msg.cost = cost if path is not None else float("inf")
        msg.success = path is not None
        msg.path = path if path is not None else []

        self.pub_path.publish(msg)
        return path, cost

    # ------------------------------------------------------------------
    # Map loading
    # ------------------------------------------------------------------

    def _load_map(self, map_file: str) -> Graph:
        if not os.path.isfile(map_file):
            rospy.logfatal(f"[{self.node_name}] Map file not found: '{map_file}'")
            rospy.signal_shutdown("Map file not found")
            return {}

        if map_file.endswith(".csv"):
            return build_graph_from_csv(map_file)
        else:
            return self._load_yaml_map(map_file)

    @staticmethod
    def _load_yaml_map(yaml_path: str) -> Graph:
        with open(yaml_path) as fh:
            data = yaml.safe_load(fh)
        return build_graph_from_edges(data.get("edges", []), bidirectional=True)

    @staticmethod
    def _default_map_path() -> str:
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg_dir, "config", "map.yaml")


if __name__ == "__main__":
    rospy.init_node("path_planner_node", anonymous=False)
    node = PathPlannerNode()
    rospy.spin()
