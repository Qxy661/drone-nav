import pytest, math, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from drone_nav.path_planner import AStarPlanner, RRTPlanner, RRTStarPlanner
from drone_nav.coordinate_utils import enu_to_ned, ned_to_enu, gps_to_enu, enu_to_gps

class TestAStarPlanner:
    def _make_grid(self, w, h, obstacles=None):
        grid = [[0]*w for _ in range(h)]
        if obstacles:
            for (r,c) in obstacles:
                grid[r][c] = 1
        return grid
    def test_simple_path(self):
        grid = self._make_grid(10, 10)
        p = AStarPlanner(grid)
        path = p.plan((0,0), (9,9))
        assert path is not None and len(path) >= 2
    def test_no_path(self):
        obs = [(i,5) for i in range(10)]
        grid = self._make_grid(10, 10, obs)
        p = AStarPlanner(grid)
        assert p.plan((0,0), (0,9)) is None
    def test_same_start_goal(self):
        grid = self._make_grid(5,5)
        p = AStarPlanner(grid)
        path = p.plan((2,2), (2,2))
        assert path is not None and len(path) == 1
    def test_manhattan(self):
        grid = self._make_grid(10,10)
        p = AStarPlanner(grid, connectivity=4, heuristic="manhattan")
        path = p.plan((0,0), (9,9))
        assert path is not None and len(path) >= 10

class TestRRTPlanner:
    def test_rrt_finds_path(self):
        grid = [[0]*50 for _ in range(50)]
        p = RRTPlanner(grid, step_size=3.0, goal_bias=0.2, max_iterations=10000, seed=42)
        path = p.plan((5.0,5.0), (45.0,45.0))
        assert path is not None and len(path) >= 2
    def test_rrt_convergence(self):
        grid = [[0]*30 for _ in range(30)]
        ok = 0
        for s in range(10):
            p = RRTPlanner(grid, step_size=2.0, goal_bias=0.3, max_iterations=5000, seed=s)
            if p.plan((2.0,2.0), (28.0,28.0)) is not None:
                ok += 1
        assert ok >= 7

class TestRRTStarPlanner:
    def test_rrt_star_finds_path(self):
        grid = [[0]*50 for _ in range(50)]
        p = RRTStarPlanner(grid, step_size=3.0, goal_bias=0.2, max_iterations=10000, seed=42)
        path = p.plan((5.0,5.0), (45.0,45.0))
        assert path is not None

class TestCoordinateUtils:
    def test_enu_ned_roundtrip(self):
        x,y,z = 10.0, 20.0, 30.0
        ned = enu_to_ned(x,y,z)
        enu = ned_to_enu(*ned)
        assert abs(enu[0]-x) < 1e-10
        assert abs(enu[1]-y) < 1e-10
        assert abs(enu[2]-z) < 1e-10
    def test_enu_ned_values(self):
        ned = enu_to_ned(5.0, 10.0, 15.0)
        assert ned == (10.0, 5.0, -15.0)
    def test_gps_enu_roundtrip(self):
        ref = (47.397742, 8.545594, 488.0)
        target = (47.398000, 8.546000, 500.0)
        enu = gps_to_enu(*target, *ref)
        gps_back = enu_to_gps(*enu, *ref)
        assert abs(gps_back[0]-target[0]) < 0.0001
        assert abs(gps_back[1]-target[1]) < 0.0001
        assert abs(gps_back[2]-target[2]) < 0.1

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
