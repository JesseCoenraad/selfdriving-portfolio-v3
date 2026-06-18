"""
Standalone test for DifferentialDriveOdometry and NodeTracker — no ROS required.
Run with: python test_odometry.py
"""

import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from localization.odometry import DifferentialDriveOdometry, NodeTracker

WHEEL_RADIUS  = 0.0318
BASELINE      = 0.1
TICKS_PER_REV = 135
DIST_PER_TICK = 2 * math.pi * WHEEL_RADIUS / TICKS_PER_REV

TILE = 0.585  # metres per tile

def ticks_for(metres):
    return round(metres / DIST_PER_TICK)

def check(desc, got, expected, tol=1e-6):
    ok = all(abs(g - e) < tol for g, e in zip(got, expected))
    print(f"[{'PASS' if ok else 'FAIL'}] {desc}")
    if not ok:
        print(f"       got      {[round(v,4) for v in got]}")
        print(f"       expected {[round(v,4) for v in expected]}")


# ---------------------------------------------------------------------------
# Odometry tests
# ---------------------------------------------------------------------------

odo = DifferentialDriveOdometry(WHEEL_RADIUS, BASELINE, TICKS_PER_REV)

# First call initialises reference — pose stays at origin
odo.update(0, 0)
check("initial pose is origin", odo.pose, (0.0, 0.0, 0.0))

# Drive 1 tile straight ahead (equal ticks on both wheels)
d = TILE
t = ticks_for(d)
odo.update(t, t)
check("drive 1 tile forward", odo.pose, (TILE, 0.0, 0.0), tol=1e-3)

# Rotate 90 degrees left in place
# arc length each wheel: baseline/2 * pi/2
arc = BASELINE / 2 * (math.pi / 2)
left_delta  = ticks_for(-arc)
right_delta = ticks_for(+arc)
prev_left, prev_right = t, t
odo.update(prev_left + left_delta, prev_right + right_delta)
x, y, theta = odo.pose
check("rotate 90 deg left", (theta,), (math.pi / 2,), tol=0.01)

# Reset
odo.reset()
check("reset returns to origin", odo.pose, (0.0, 0.0, 0.0))

# ---------------------------------------------------------------------------
# NodeTracker tests
# ---------------------------------------------------------------------------

COORDS = {
    "S": (0 * TILE, 0 * TILE),
    "I": (1 * TILE, 0 * TILE),
    "J": (2 * TILE, 0 * TILE),
    "T": (3 * TILE, 0 * TILE),
}
PATH = ["S", "I", "J", "T"]
THRESHOLD = 0.15

tracker = NodeTracker(PATH, COORDS, THRESHOLD)

# At start — still at S
current, next_n, remaining = tracker.update(0.0, 0.0)
ok = current == "S" and next_n == "I" and remaining == ["S", "I", "J", "T"]
print(f"[{'PASS' if ok else 'FAIL'}] tracker starts at S, next=I")

# Move close to I
ix, iy = COORDS["I"]
current, next_n, remaining = tracker.update(ix + 0.05, iy)
ok = current == "I" and next_n == "J" and remaining == ["I", "J", "T"]
print(f"[{'PASS' if ok else 'FAIL'}] tracker advances to I")

# Move close to J
jx, jy = COORDS["J"]
current, next_n, remaining = tracker.update(jx + 0.05, jy)
ok = current == "J" and next_n == "T"
print(f"[{'PASS' if ok else 'FAIL'}] tracker advances to J")

# Reach goal T
tx, ty = COORDS["T"]
current, next_n, remaining = tracker.update(tx, ty)
ok = current == "T" and next_n is None and tracker.at_goal
print(f"[{'PASS' if ok else 'FAIL'}] tracker reaches goal T, at_goal=True")

print("\nDone.")