"""
Standalone test for the PIDController — no ROS required.
Run with: python test_pid.py
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from duckie_control.pid import PIDController


def check(desc, got, expected, tol=1e-6):
    ok = abs(got - expected) < tol
    print(f"[{'PASS' if ok else 'FAIL'}] {desc}")
    if not ok:
        print(f"       got {got}, expected {expected}")


# ---------------------------------------------------------------------------
# P, I, D in isolation
# ---------------------------------------------------------------------------

pid = PIDController(Kp=2.0, Ki=0.0, Kd=0.0)
out = pid.update(error=1.0, dt=0.1)
check("P-only: Kp=2.0, error=1.0 -> output=2.0", out, 2.0)

pid = PIDController(Kp=0.0, Ki=1.0, Kd=0.0)
pid.update(error=1.0, dt=1.0)
out = pid.update(error=1.0, dt=1.0)
check("I-only: accumulates error*dt over two steps", out, 2.0)

pid = PIDController(Kp=0.0, Ki=0.0, Kd=1.0)
pid.update(error=0.0, dt=1.0)
out = pid.update(error=1.0, dt=1.0)
check("D-only: error jumps 0 -> 1 over dt=1 -> output=1.0", out, 1.0)

# ---------------------------------------------------------------------------
# Output clamp and anti-windup
# ---------------------------------------------------------------------------

pid = PIDController(Kp=10.0, Ki=0.0, Kd=0.0, output_limits=(-1.0, 1.0))
out = pid.update(error=5.0, dt=0.1)
check("output clamp: large error clamped to max", out, 1.0)

pid = PIDController(Kp=0.0, Ki=1.0, Kd=0.0, integral_limit=0.5)
for _ in range(10):
    pid.update(error=1.0, dt=1.0)
out = pid.update(error=1.0, dt=1.0)
check("anti-windup: integral term clamped to limit", out, 0.5)

# ---------------------------------------------------------------------------
# Reset
# ---------------------------------------------------------------------------

pid = PIDController(Kp=1.0, Ki=1.0, Kd=1.0)
pid.update(error=1.0, dt=1.0)
pid.reset()
out = pid.update(error=0.0, dt=1.0)
check("reset clears integral/derivative memory", out, 0.0)

print("\nDone.")