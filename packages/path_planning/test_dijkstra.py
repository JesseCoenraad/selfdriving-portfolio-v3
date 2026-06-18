"""
Standalone test for the Dijkstra implementation — no ROS required.
Run with: python test_dijkstra.py
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from path_planning.dijkstra import dijkstra, build_graph_from_edges, build_graph_from_csv

# ---------------------------------------------------------------------------
# Map from the assignment
#
#   A ── B ── C ── D
#   │       │   │
#   E   F ── G   H
#   │   │       │
#   S ── I ── J ── T
# ---------------------------------------------------------------------------

EDGES = [
    {"from": "S", "to": "I", "distance": 1},
    {"from": "I", "to": "J", "distance": 1},
    {"from": "J", "to": "T", "distance": 1},
    {"from": "A", "to": "B", "distance": 1},
    {"from": "B", "to": "C", "distance": 1},
    {"from": "C", "to": "D", "distance": 1},
    {"from": "F", "to": "G", "distance": 1},
    {"from": "A", "to": "E", "distance": 1},
    {"from": "E", "to": "S", "distance": 1},
    {"from": "F", "to": "I", "distance": 1},
    {"from": "C", "to": "G", "distance": 1},
    {"from": "D", "to": "H", "distance": 1},
    {"from": "H", "to": "T", "distance": 1},
]

GRAPH = build_graph_from_edges(EDGES)


def check(description, path, cost, expected_path, expected_cost):
    ok_path = path == expected_path
    ok_cost = abs(cost - expected_cost) < 1e-9
    status = "PASS" if (ok_path and ok_cost) else "FAIL"
    print(f"[{status}] {description}")
    if not ok_path:
        print(f"       path:  got {path}, expected {expected_path}")
    if not ok_cost:
        print(f"       cost:  got {cost}, expected {expected_cost}")


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

# Shortest path: S → I → J → T  (cost 3)
path, cost = dijkstra(GRAPH, "S", "T")
check("S -> T shortest path", path, cost, ["S", "I", "J", "T"], 3.0)

# Start equals goal
path, cost = dijkstra(GRAPH, "S", "S")
check("start == goal", path, cost, ["S"], 0.0)

# Longer path forced by removing direct route
restricted = {k: dict(v) for k, v in GRAPH.items()}
del restricted["I"]["J"]   # block I-J
del restricted["J"]["I"]   # block J-I
path, cost = dijkstra(restricted, "S", "T")
# Multiple optimal paths of cost 7 exist; only verify cost and endpoints
ok = path is not None and path[0] == "S" and path[-1] == "T" and abs(cost - 7.0) < 1e-9
print(f"[{'PASS' if ok else 'FAIL'}] S -> T with I-J blocked (cost 7, S..T)")

# No path exists
isolated_graph = {"X": {"Y": 1.0}, "Y": {"X": 1.0}, "Z": {}}
path, cost = dijkstra(isolated_graph, "X", "Z")
ok = path is None and cost == float("inf")
print(f"[{'PASS' if ok else 'FAIL'}] no path returns (None, inf)")

# Unknown start node
path, cost = dijkstra(GRAPH, "UNKNOWN", "T")
ok = path is None and cost == float("inf")
print(f"[{'PASS' if ok else 'FAIL'}] unknown start returns (None, inf)")

# ---------------------------------------------------------------------------
# CSV round-trip
# ---------------------------------------------------------------------------
import tempfile, csv

csv_file = tempfile.NamedTemporaryFile(mode="w", suffix=".csv",
                                       delete=False, newline="")
writer = csv.DictWriter(csv_file, fieldnames=["from_node", "to_node", "distance"])
writer.writeheader()
for e in EDGES:
    writer.writerow({"from_node": e["from"], "to_node": e["to"],
                     "distance": e["distance"]})
csv_file.close()

graph_csv = build_graph_from_csv(csv_file.name)
path, cost = dijkstra(graph_csv, "S", "T")
check("CSV load -> S -> T", path, cost, ["S", "I", "J", "T"], 3.0)
os.unlink(csv_file.name)

print("\nDone.")