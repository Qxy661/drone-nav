"""DWA Local Planner unit tests"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import math
from drone_nav.dwa_planner import DWALocalPlanner, DWAConfig, Obstacle, plan_path_with_dwa


def test_clear_path():
    config = DWAConfig()
    planner = DWALocalPlanner(config)
    state = (0.0, 0.0, 0.0, 0.3, 0.0)  # already moving forward
    goal = (5.0, 0.0)
    obstacles = []
    v, w = planner.plan(state, goal, obstacles)
    assert v >= 0, f"Should move forward, got v={v}"
    print(f"  Clear path: v={v:.3f}, w={w:.3f}")

def test_obstacle_avoidance():
    config = DWAConfig()
    planner = DWALocalPlanner(config)
    state = (0.0, 0.0, 0.0, 0.5, 0.0)
    goal = (5.0, 0.0)
    obstacles = [Obstacle(3.0, 0.0, 0.5)]
    v, w = planner.plan(state, goal, obstacles)
    print(f"  Obstacle avoid: v={v:.3f}, w={w:.3f}")
    assert abs(w) > 0.01, "Should turn to avoid"

def test_blocked_path():
    config = DWAConfig(stop_dist=1.0)
    planner = DWALocalPlanner(config)
    state = (0.0, 0.0, 0.0, 0.5, 0.0)
    goal = (5.0, 0.0)
    obstacles = [Obstacle(1.5, y, 0.3) for y in range(-3, 4)]
    v, w = planner.plan(state, goal, obstacles)
    print(f"  Blocked path: v={v:.3f}, w={w:.3f}")

def test_trajectory_planning():
    waypoints = [(2.0, 0.0), (4.0, 2.0), (4.0, 4.0)]
    start = (0.0, 0.0, 0.0, 0.0, 0.0)
    obstacles = [Obstacle(3.0, 1.0, 0.3)]
    trajectory = plan_path_with_dwa(waypoints, start, obstacles, max_steps=500)
    assert len(trajectory) > 10
    final = trajectory[-1]
    dist = math.sqrt((final[0]-waypoints[-1][0])**2 + (final[1]-waypoints[-1][1])**2)
    print(f"  Trajectory: {len(trajectory)} steps, dist_to_goal={dist:.2f}")

if __name__ == "__main__":
    print("=== DWA Planner Tests ===")
    test_clear_path()
    test_obstacle_avoidance()
    test_blocked_path()
    test_trajectory_planning()
    print("=== ALL TESTS PASSED ===")
