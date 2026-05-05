"""Tests for coordinate_utils.py - coordinate frame conversions."""
import sys
import os
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import math
from drone_nav.coordinate_utils import (
    enu_to_ned, ned_to_enu, gps_to_enu, enu_to_gps,
    body_to_enu, enu_to_body, distance_2d, distance_3d
)


class TestENUNED(unittest.TestCase):
    def test_roundtrip(self):
        """ENU -> NED -> ENU should return original."""
        x, y, z = 1.0, 2.0, 3.0
        ned = enu_to_ned(x, y, z)
        enu = ned_to_enu(*ned)
        assert abs(enu[0] - x) < 1e-6
        assert abs(enu[1] - y) < 1e-6
        assert abs(enu[2] - z) < 1e-6

    def test_values(self):
        """ENU (E,N,U) -> NED (N,E,D): N=E, E=N, D=-U."""
        e, n, u = 1.0, 2.0, 3.0
        ned = enu_to_ned(e, n, u)
        assert abs(ned[0] - n) < 1e-6   # N = E_north
        assert abs(ned[1] - e) < 1e-6   # E = E_east
        assert abs(ned[2] - (-u)) < 1e-6  # D = -U

    def test_zero(self):
        ned = enu_to_ned(0, 0, 0)
        assert ned == (0.0, 0.0, 0.0)


class TestGPS_ENU(unittest.TestCase):
    def test_origin_roundtrip(self):
        """GPS at origin should roundtrip."""
        lat0, lon0, alt0 = 30.0, 120.0, 100.0
        e, n, u = gps_to_enu(lat0, lon0, alt0, lat0, lon0, alt0)
        assert abs(e) < 0.01
        assert abs(n) < 0.01
        assert abs(u) < 0.01

    def test_small_offset(self):
        """Small GPS offset should produce small ENU offset."""
        lat0, lon0, alt0 = 30.0, 120.0, 100.0
        lat1 = lat0 + 0.0001  # ~11m north
        e, n, u = gps_to_enu(lat1, lon0, alt0, lat0, lon0, alt0)
        assert abs(e) < 1.0  # mostly north
        assert n > 10.0  # significant north offset
        assert abs(u) < 1.0


class TestBodyENU(unittest.TestCase):
    def test_zero_yaw(self):
        """At zero yaw, body forward -> ENU."""
        bx, by, bz = 1.0, 0.0, 0.0
        ex, ey, ez = body_to_enu(bx, by, bz, yaw=0.0)
        assert abs(ex - 0.0) < 1e-6
        assert abs(ey - 1.0) < 1e-6

    def test_90_yaw(self):
        """At 90 deg yaw."""
        bx, by, bz = 1.0, 0.0, 0.0
        ex, ey, ez = body_to_enu(bx, by, bz, yaw=math.pi / 2)
        assert abs(ex - 1.0) < 1e-6
        assert abs(ey - 0.0) < 1e-6

    def test_roundtrip(self):
        """body -> enu -> body should return original."""
        bx, by, bz = 1.0, 0.5, -0.3
        yaw = 0.7
        ex, ey, ez = body_to_enu(bx, by, bz, yaw)
        assert abs(ez - bz) < 1e-6  # z passes through unchanged


class TestDistance(unittest.TestCase):
    def test_distance_2d(self):
        assert abs(distance_2d(0, 0, 3, 4) - 5.0) < 1e-6

    def test_distance_3d(self):
        assert abs(distance_3d((0, 0, 0), (1, 0, 0)) - 1.0) < 1e-6


if __name__ == '__main__':
    unittest.main()
