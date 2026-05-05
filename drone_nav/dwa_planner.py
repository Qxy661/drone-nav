"""
DWA (Dynamic Window Approach) 局部避障规划器

DWA 是工业界和学术界广泛使用的实时避障算法
应用: 移动机器人、无人机低速避障、自动驾驶局部规划

算法原理:
1. 速度空间采样: 在机器人的动态窗口内采样 (v, omega) 对
2. 轨迹前向模拟: 对每个 (v, omega) 模拟未来一段时间的轨迹
3. 轨迹评价: 对每条轨迹打分 (目标方向 + 障碍物距离 + 速度)
4. 选择最优: 选得分最高的 (v, omega) 作为控制输出

与 A*/RRT 的区别:
- A*/RRT: 全局规划, 生成路径点序列, 不考虑机器人动力学
- DWA: 局部规划, 直接输出控制命令 (v, omega), 考虑动力学约束
- 实际使用: A* 做全局路径, DWA 沿全局路径做局部避障
"""
import math
import numpy as np
from typing import List, Tuple
from dataclasses import dataclass


@dataclass
class DWAConfig:
    max_speed: float = 1.0
    min_speed: float = -0.5
    max_omega: float = 1.0
    max_accel: float = 2.0
    max_alpha: float = 3.0
    dt: float = 0.1
    predict_time: float = 2.0
    v_samples: int = 10
    w_samples: int = 20
    w_heading: float = 0.3
    w_dist: float = 0.6
    w_velocity: float = 0.1
    robot_radius: float = 0.3
    safe_distance: float = 0.5
    stop_dist: float = 0.3


@dataclass
class Obstacle:
    x: float
    y: float
    radius: float


class DWALocalPlanner:
    def __init__(self, config: DWAConfig = None):
        self.cfg = config or DWAConfig()

    def plan(self, state, goal, obstacles):
        x, y, theta, v, w = state
        Vs = self._calc_dynamic_window(v, w)
        candidates = self._sample_velocities(Vs)

        best_score = -float("inf")
        best_v, best_w = 0.0, 0.0

        for v_cand, w_cand in candidates:
            trajectory = self._simulate_trajectory(x, y, theta, v_cand, w_cand)
            score = self._evaluate_trajectory(trajectory, goal, obstacles)
            if score > best_score:
                best_score = score
                best_v = v_cand
                best_w = w_cand

        min_dist = self._min_obstacle_dist(x, y, obstacles)
        if min_dist < self.cfg.stop_dist:
            best_v = 0.0
            best_w = 0.0

        return best_v, best_w

    def _calc_dynamic_window(self, v, w):
        v_min = max(self.cfg.min_speed, v - self.cfg.max_accel * self.cfg.dt)
        v_max = min(self.cfg.max_speed, v + self.cfg.max_accel * self.cfg.dt)
        w_min = max(-self.cfg.max_omega, w - self.cfg.max_alpha * self.cfg.dt)
        w_max = min(self.cfg.max_omega, w + self.cfg.max_alpha * self.cfg.dt)
        return (v_min, v_max, w_min, w_max)

    def _sample_velocities(self, Vs):
        v_min, v_max, w_min, w_max = Vs
        vs = np.linspace(v_min, v_max, self.cfg.v_samples)
        ws = np.linspace(w_min, w_max, self.cfg.w_samples)
        return [(v, w) for v in vs for w in ws]

    def _simulate_trajectory(self, x, y, theta, v, w):
        trajectory = [(x, y, theta)]
        t = 0.0
        while t < self.cfg.predict_time:
            x += v * math.cos(theta) * self.cfg.dt
            y += v * math.sin(theta) * self.cfg.dt
            theta += w * self.cfg.dt
            trajectory.append((x, y, theta))
            t += self.cfg.dt
        return trajectory

    def _evaluate_trajectory(self, trajectory, goal, obstacles):
        end_x, end_y, end_theta = trajectory[-1]

        # heading: 基于轨迹终点到目标的方向 (不是最终朝向)
        goal_dx = goal[0] - end_x
        goal_dy = goal[1] - end_y
        dist_to_goal = math.sqrt(goal_dx**2 + goal_dy**2)

        if dist_to_goal < 0.01:
            heading_score = 1.0
        else:
            # 轨迹终点离目标越近, heading分越高
            # 同时考虑朝向
            goal_angle = math.atan2(goal_dy, goal_dx)
            angle_diff = abs(self._normalize_angle(goal_angle - end_theta))
            heading_score = math.cos(angle_diff) * 0.5 + 0.5  # 0~1

            # 额外奖励: 接近目标
            proximity = max(0, 1.0 - dist_to_goal / 10.0)
            heading_score = heading_score * 0.7 + proximity * 0.3

        # dist: 离最近障碍物的距离
        min_dist = float("inf")
        for tx, ty, _ in trajectory:
            for obs in obstacles:
                d = math.sqrt((tx - obs.x)**2 + (ty - obs.y)**2) - obs.radius
                min_dist = min(min_dist, d)

        if min_dist < self.cfg.robot_radius:
            return -float("inf")  # 碰撞

        dist_score = min(min_dist / (self.cfg.safe_distance + self.cfg.robot_radius), 1.0)

        # velocity: 鼓励前进 (v > 0)
        v_start = trajectory[0][0] if len(trajectory) > 1 else 0
        # 用起始速度, 但避免v=0永远得0分
        # 给小速度也给一定分
        vel_score = max(0, (v_start + 0.1) / (self.cfg.max_speed + 0.1))
        vel_score = min(vel_score, 1.0)

        total = (self.cfg.w_heading * heading_score +
                 self.cfg.w_dist * dist_score +
                 self.cfg.w_velocity * vel_score)
        return total

    def _min_obstacle_dist(self, x, y, obstacles):
        min_d = float("inf")
        for obs in obstacles:
            d = math.sqrt((x - obs.x)**2 + (y - obs.y)**2) - obs.radius
            min_d = min(min_d, d)
        return min_d

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def plan_path_with_dwa(waypoints, start_state, obstacles,
                       config=None, dt=0.1, max_steps=1000):
    planner = DWALocalPlanner(config)
    state = list(start_state)
    trajectory = [tuple(state)]
    wp_idx = 0

    for _ in range(max_steps):
        if wp_idx >= len(waypoints):
            break
        goal = waypoints[wp_idx]
        dist_to_goal = math.sqrt(
            (state[0] - goal[0])**2 + (state[1] - goal[1])**2)
        if dist_to_goal < 0.3:
            wp_idx += 1
            continue
        v_cmd, w_cmd = planner.plan(tuple(state), goal, obstacles)
        state[2] += w_cmd * dt
        state[0] += v_cmd * math.cos(state[2]) * dt
        state[1] += v_cmd * math.sin(state[2]) * dt
        state[3] = v_cmd
        state[4] = w_cmd
        trajectory.append(tuple(state))
        if v_cmd == 0 and w_cmd == 0 and dist_to_goal < 0.5:
            break
    return trajectory
