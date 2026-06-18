import csv
import heapq
from typing import Dict, List, Optional, Tuple

Graph = Dict[str, Dict[str, float]]
Coords = Dict[str, Tuple[float, float]]


def dijkstra(
    graph: Graph,
    start: str,
    goal: str,
) -> Tuple[Optional[List[str]], float]:
    """
    Dijkstra's shortest-path algorithm on a weighted directed graph.

    Args:
        graph: Adjacency dict {node: {neighbor: edge_cost, ...}, ...}
        start: Name of the start node.
        goal:  Name of the goal node.

    Returns:
        (path, cost) where path is a list of node names [start, ..., goal],
        or (None, inf) when no path exists.
    """
    if start not in graph:
        return None, float("inf")
    if start == goal:
        return [start], 0.0

    # Heap entries: (cumulative_cost, node, path_so_far)
    heap: List[Tuple[float, str, List[str]]] = [(0.0, start, [start])]
    visited: set = set()

    while heap:
        cost, node, path = heapq.heappop(heap)

        if node in visited:
            continue
        visited.add(node)

        if node == goal:
            return path, cost

        for neighbor, edge_cost in graph.get(node, {}).items():
            if neighbor not in visited:
                heapq.heappush(heap, (cost + edge_cost, neighbor, path + [neighbor]))

    return None, float("inf")


# ---------------------------------------------------------------------------
# Graph construction helpers
# ---------------------------------------------------------------------------

def build_graph_from_edges(
    edges: List[Dict],
    bidirectional: bool = True,
) -> Graph:
    """
    Build an adjacency dict from a list of edge dicts.

    Each dict must have keys 'from', 'to', and 'distance'.
    """
    graph: Graph = {}
    for edge in edges:
        src = str(edge["from"])
        dst = str(edge["to"])
        cost = float(edge["distance"])
        graph.setdefault(src, {})[dst] = cost
        if bidirectional:
            graph.setdefault(dst, {})[src] = cost
    return graph


def build_graph_from_dataframe(df) -> Graph:
    """
    Build a graph from a pandas DataFrame.

    Required columns: 'from_node', 'to_node', 'distance'
    """
    graph: Graph = {}
    for _, row in df.iterrows():
        src = str(row["from_node"])
        dst = str(row["to_node"])
        cost = float(row["distance"])
        graph.setdefault(src, {})[dst] = cost
        graph.setdefault(dst, {})[src] = cost
    return graph


def build_graph_from_csv(csv_path: str) -> Graph:
    """
    Build a graph from a CSV file without requiring pandas.

    Expected header: from_node,to_node,distance
    """
    graph: Graph = {}
    with open(csv_path, newline="") as fh:
        for row in csv.DictReader(fh):
            src = row["from_node"].strip()
            dst = row["to_node"].strip()
            cost = float(row["distance"])
            graph.setdefault(src, {})[dst] = cost
            graph.setdefault(dst, {})[src] = cost
    return graph
