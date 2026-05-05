# drone-nav

ROS2 自主导航与路径规划系统。全局规划 + 局部避障 + 坐标转换。

## 功能

- A* 全局路径规划 (8方向, 欧几里得启发)
- RRT / RRT* 随机采样规划 (目标偏向, 渐近最优)
- DWA 局部避障 (速度空间采样 + 轨迹评价)
- 坐标转换: ENU <-> NED <-> Body <-> GPS
- MAVROS 航点导航 + 任务管理状态机

## 架构

```
mission_manager (状态机)
    |
    v
path_planner (A*/RRT/RRT*) -> 全局路径
    |
    v
dwa_planner (DWA) -> 局部避障 -> 控制命令
    |
    v
waypoint_navigator -> MAVROS -> 飞控
```

## Modules

| Module | 可独立使用 | 功能 |
|--------|-----------|------|
| path_planner | Yes | A*, RRT, RRT* 算法 |
| dwa_planner | Yes | DWA 局部避障 |
| coordinate_utils | Yes | ENU/NED/GPS 坐标转换 |
| waypoint_navigator | No (ROS) | MAVROS 航点导航 |
| mission_manager | No (ROS) | 任务编排状态机 |
| obstacle_map | No (ROS) | 2D 栅格障碍物地图 |

## 快速开始

```bash
# 安装依赖
sudo apt install ros-humble-mavros
pip install numpy

# 编译
cd ros2_ws && colcon build --packages-select drone_nav
source install/setup.bash

# 运行测试
python3 src/drone_nav/test/test_path_planner.py
python3 src/drone_nav/test/test_dwa.py

# 启动系统
ros2 launch drone_nav nav_test.launch.py
```

## 算法对比

| 算法 | 类型 | 最优性 | 完备性 | 适用场景 |
|------|------|--------|--------|---------|
| A* | 全局 | 最优 | 完备 | 栅格地图 |
| RRT | 全局 | 不最优 | 概率完备 | 高维空间 |
| RRT* | 全局 | 渐近最优 | 概率完备 | 需要最优路径 |
| DWA | 局部 | 局部最优 | - | 动态避障 |

## License

MIT
