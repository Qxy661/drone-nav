"""
坐标系转换工具
Coordinate Frame Conversion Utilities

常用坐标系:
- ENU: East-North-Up (ROS标准局部坐标系)
- NED: North-East-Down (飞控标准坐标系, ArduPilot/PX4内部使用)
- Body: 机体系 (x=前, y=右, z=下)
- GPS: WGS84 经纬高

注意: MAVROS 自动处理 ENU<->NED 转换，直接用 MAVROS topic 时不需要手动转
      但理解这些转换对调试和算法开发很重要
"""
import math
import numpy as np
from typing import Tuple


def enu_to_ned(x_enu: float, y_enu: float, z_enu: float) -> Tuple[float, float, float]:
    """ENU -> NED 坐标转换
    ENU: x=东, y=北, z=上
    NED: x=北, y=东, z=下
    """
    return y_enu, x_enu, -z_enu


def ned_to_enu(x_ned: float, y_ned: float, z_ned: float) -> Tuple[float, float, float]:
    """NED -> ENU 坐标转换"""
    return y_ned, x_ned, -z_ned


def body_to_enu(vx_body: float, vy_body: float, vz_body: float,
                yaw: float) -> Tuple[float, float, float]:
    """机体系速度 -> ENU 速度 (只考虑偏航角)
    机体系: x=前, y=右, z=上 (与NED的z相反)
    yaw: 从北方向顺时针旋转的角度 (弧度)
    """
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    vx_enu = vx_body * sin_yaw + vy_body * cos_yaw
    vy_enu = vx_body * cos_yaw - vy_body * sin_yaw
    return vx_enu, vy_enu, vz_body


def enu_to_body(vx_enu: float, vy_enu: float, vz_enu: float,
                yaw: float) -> Tuple[float, float, float]:
    """ENU 速度 -> 机体系速度"""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    vx_body = vx_enu * sin_yaw + vy_enu * cos_yaw
    vy_body = -vx_enu * cos_yaw + vy_enu * sin_yaw
    return vx_body, vy_body, vz_enu


def gps_to_enu(lat: float, lon: float, alt: float,
               ref_lat: float, ref_lon: float, ref_alt: float) -> Tuple[float, float, float]:
    """GPS (WGS84) -> ENU 局部坐标
    使用参考点作为局部坐标原点
    精度: 1km 内误差 < 1m
    """
    R = 6378137.0  # 地球半径 (米)
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    x_enu = d_lon * R * math.cos(math.radians(ref_lat))  # 东
    y_enu = d_lat * R                                     # 北
    z_enu = alt - ref_alt                                  # 上
    return x_enu, y_enu, z_enu


def enu_to_gps(x: float, y: float, z: float,
               ref_lat: float, ref_lon: float, ref_alt: float) -> Tuple[float, float, float]:
    """ENU 局部坐标 -> GPS (WGS84)"""
    R = 6378137.0
    lat = ref_lat + math.degrees(y / R)
    lon = ref_lon + math.degrees(x / (R * math.cos(math.radians(ref_lat))))
    alt = ref_alt + z
    return lat, lon, alt


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """2D欧氏距离"""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def distance_3d(p1: Tuple, p2: Tuple) -> float:
    """3D欧氏距离"""
    return math.sqrt(sum((a - b)**2 for a, b in zip(p1, p2)))
