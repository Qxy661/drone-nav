"""path_planner.py - Path planning algorithms for 2D occupancy grid maps.
Contains: AStarPlanner, RRTPlanner, RRTStarPlanner
No ROS dependency. All planners work on 2D grid (0=free, 1=obstacle).
Paths returned as list of (x, y) tuples in grid coordinates.
"""
import math, random, heapq
from typing import List, Tuple, Optional

GridPos = Tuple[int, int]
Path = List[Tuple[float, float]]

def _h_euclid(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def _h_manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def _h_chebyshev(a, b):
    return max(abs(a[0]-b[0]), abs(a[1]-b[1]))

class AStarPlanner:
    """A* search on 2D grid. f(n)=g(n)+h(n). Supports 4/8-connectivity."""
    DIRS8 = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    def __init__(self, grid, connectivity=8, heuristic="euclidean"):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        self.conn = connectivity
        hm = {"euclidean": _h_euclid, "manhattan": _h_manhattan, "chebyshev": _h_chebyshev}
        self.h = hm.get(heuristic, _h_euclid)

    def _ok(self, pos):
        r, c = pos
        return 0 <= r < self.rows and 0 <= c < self.cols and self.grid[r][c] == 0

    def _neighbors(self, pos):
        result = []
        dirs = self.DIRS8 if self.conn == 8 else self.DIRS8[:4]
        for dr, dc in dirs:
            nr, nc = pos[0]+dr, pos[1]+dc
            if self._ok((nr, nc)):
                if abs(dr)+abs(dc) == 2:
                    if self.grid[pos[0]+dr][pos[1]] == 0 or self.grid[pos[0]][pos[1]+dc] == 0:
                        result.append(((nr, nc), math.sqrt(2)))
                    else:
                        continue
                else:
                    result.append(((nr, nc), 1.0))
        return result

    def plan(self, start, goal):
        """A* search. Returns [(x,y),...] or None."""
        if not self._ok(start) or not self._ok(goal):
            return None
        open_list = [(0.0, start)]
        closed = set()
        g = {start: 0.0}
        parent = {}
        while open_list:
            _, cur = heapq.heappop(open_list)
            if cur == goal:
                path = [cur]
                while cur in parent:
                    cur = parent[cur]
                    path.append(cur)
                path.reverse()
                return [(float(p[1]), float(p[0])) for p in path]
            if cur in closed:
                continue
            closed.add(cur)
            for nb, cost in self._neighbors(cur):
                if nb in closed:
                    continue
                ng = g[cur] + cost
                if nb not in g or ng < g[nb]:
                    g[nb] = ng
                    parent[nb] = cur
                    heapq.heappush(open_list, (ng + self.h(nb, goal), nb))
        return None

class RRTPlanner:
    """RRT planner with goal biasing for 2D grids."""

    def __init__(self, grid, step_size=2.0, goal_bias=0.1, max_iterations=5000, goal_threshold=2.0, seed=None):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.max_iter = max_iterations
        self.goal_thresh = goal_threshold
        if seed is not None:
            random.seed(seed)

    def _collision_free(self, p1, p2):
        dx, dy = p2[0]-p1[0], p2[1]-p1[1]
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 1e-6:
            return True
        steps = max(int(dist*2), 1)
        for i in range(steps+1):
            t = i/steps
            x, y = p1[0]+t*dx, p1[1]+t*dy
            ix, iy = int(round(x)), int(round(y))
            if ix < 0 or ix >= self.cols or iy < 0 or iy >= self.rows:
                return False
            if self.grid[iy][ix] == 1:
                return False
        return True

    def _nearest(self, tree, point):
        min_d, min_i = float("inf"), 0
        for i, n in enumerate(tree):
            d = math.sqrt((n[0]-point[0])**2 + (n[1]-point[1])**2)
            if d < min_d:
                min_d, min_i = d, i
        return min_i

    def _steer(self, fr, to):
        dx, dy = to[0]-fr[0], to[1]-fr[1]
        d = math.sqrt(dx*dx + dy*dy)
        if d <= self.step_size:
            return to
        r = self.step_size/d
        return (fr[0]+dx*r, fr[1]+dy*r)

    def plan(self, start, goal):
        """RRT search. Returns [(x,y),...] or None."""
        tree = [start]
        parent = {0: -1}
        for _ in range(self.max_iter):
            rand = goal if random.random() < self.goal_bias else (random.uniform(0, self.cols-1), random.uniform(0, self.rows-1))
            ni = self._nearest(tree, rand)
            new = self._steer(tree[ni], rand)
            if not self._collision_free(tree[ni], new):
                continue
            idx = len(tree)
            tree.append(new)
            parent[idx] = ni
            if math.sqrt((new[0]-goal[0])**2 + (new[1]-goal[1])**2) <= self.goal_thresh:
                path = [goal]
                j = idx
                while j != -1:
                    path.append(tree[j])
                    j = parent[j]
                path.reverse()
                return path
        return None

class RRTStarPlanner:
    """RRT* planner with rewiring for asymptotic optimality."""

    def __init__(self, grid, step_size=2.0, goal_bias=0.1, max_iterations=5000, goal_threshold=2.0, search_radius=5.0, seed=None):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.max_iter = max_iterations
        self.goal_thresh = goal_threshold
        self.search_radius = search_radius
        if seed is not None:
            random.seed(seed)

    def _collision_free(self, p1, p2):
        dx, dy = p2[0]-p1[0], p2[1]-p1[1]
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 1e-6:
            return True
        steps = max(int(dist*2), 1)
        for i in range(steps+1):
            t = i/steps
            x, y = p1[0]+t*dx, p1[1]+t*dy
            ix, iy = int(round(x)), int(round(y))
            if ix < 0 or ix >= self.cols or iy < 0 or iy >= self.rows:
                return False
            if self.grid[iy][ix] == 1:
                return False
        return True

    def _dist(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def _steer(self, fr, to):
        dx, dy = to[0]-fr[0], to[1]-fr[1]
        d = math.sqrt(dx*dx + dy*dy)
        if d <= self.step_size:
            return to
        r = self.step_size/d
        return (fr[0]+dx*r, fr[1]+dy*r)

    def _near(self, tree, point):
        n = len(tree)
        r = min(self.search_radius, 2.0*math.sqrt(math.log(n+1)/(n+1)))
        r = max(r, self.step_size)
        return [i for i, nd in enumerate(tree) if self._dist(nd, point) <= r]

    def plan(self, start, goal):
        """RRT* search. Returns [(x,y),...] or None."""
        tree = [start]
        parent = {0: -1}
        cost = {0: 0.0}
        best_goal_idx = None
        best_goal_cost = float("inf")
        for _ in range(self.max_iter):
            rand = goal if random.random() < self.goal_bias else (random.uniform(0, self.cols-1), random.uniform(0, self.rows-1))
            min_d, ni = float("inf"), 0
            for i, nd in enumerate(tree):
                d = self._dist(nd, rand)
                if d < min_d:
                    min_d, ni = d, i
            new = self._steer(tree[ni], rand)
            if not self._collision_free(tree[ni], new):
                continue
            near = self._near(tree, new)
            best_pi = ni
            best_c = cost[ni] + self._dist(tree[ni], new)
            for qi in near:
                c = cost[qi] + self._dist(tree[qi], new)
                if c < best_c and self._collision_free(tree[qi], new):
                    best_pi = qi
                    best_c = c
            idx = len(tree)
            tree.append(new)
            parent[idx] = best_pi
            cost[idx] = best_c
            for qi in near:
                pc = cost[idx] + self._dist(new, tree[qi])
                if pc < cost[qi] and self._collision_free(new, tree[qi]):
                    parent[qi] = idx
                    cost[qi] = pc
            dg = self._dist(new, goal)
            if dg <= self.goal_thresh:
                tc = cost[idx] + dg
                if tc < best_goal_cost:
                    best_goal_cost = tc
                    best_goal_idx = idx
        if best_goal_idx is not None:
            path = [goal]
            j = best_goal_idx
            while j != -1:
                path.append(tree[j])
                j = parent[j]
            path.reverse()
            return path
        return None


def visualize_path(grid, path=None, start=None, goal=None, title="Path Planning", save_path=None):
    """Visualize path on grid. Requires matplotlib (optional)."""
    try:
        import matplotlib.pyplot as plt
        import matplotlib.colors as mcolors
    except ImportError:
        print("matplotlib not installed, skipping visualization.")
        return
    fig, ax = plt.subplots(figsize=(10, 10))
    cmap = mcolors.ListedColormap(["white", "black"])
    ax.imshow(grid, cmap=cmap, origin="lower", interpolation="nearest")
    if path and len(path) > 1:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, "b-", linewidth=2, label="Path")
        ax.plot(xs, ys, "bo", markersize=4)
    if start:
        ax.plot(start[0], start[1], "go", markersize=12, label="Start")
    if goal:
        ax.plot(goal[0], goal[1], "r*", markersize=15, label="Goal")
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
    else:
        plt.show()
