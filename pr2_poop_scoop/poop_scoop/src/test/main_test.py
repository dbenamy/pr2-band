from math import pi
import os
import sys
import unittest

from os.path import dirname, join, realpath
sys.path.append(realpath(join(dirname(__file__), '..')))
from main import calc_work_x_y_yaw


class TestFindWorkPose(unittest.TestCase):
    def test_poop_ahead(self):
        self.assertItemsAlmostEqual(calc_work_x_y_yaw(10, 10, 15, 10),
                                    (14.2, 10.0, 0.0))
    
    def test_poop_behind(self):
        self.assertItemsAlmostEqual(calc_work_x_y_yaw(10, 10, 5, 10),
                                    (5.8, 10.0, pi))
    
    def test_poop_left(self):
        self.assertItemsAlmostEqual(calc_work_x_y_yaw(10, 10, 10, 20),
                                    (10, 19.2, pi / 2))
    
    def test_poop_right(self):
        self.assertItemsAlmostEqual(calc_work_x_y_yaw(10, 10, 10, -11),
                                    (10, -10.2, -pi / 2))

    def test_no_move(self):
        self.assertItemsAlmostEqual(calc_work_x_y_yaw(10, 10, 10.8, 10),
                                    (10, 10, 0))
    
    def test_only_spin(self):
        self.assertItemsAlmostEqual(calc_work_x_y_yaw(10, 10, 10, 10.8),
                                    (10, 10, pi / 2))
    
    def test_poop_front_right(self):
        self.assertItemsAlmostEqual(calc_work_x_y_yaw(20, 20, 22, 15),
                                    (21.702887458916717, 15.742781352708207,
                                     -1.1902899496825317))
    
    def test_poop_front_left(self):
        self.assertItemsAlmostEqual(calc_work_x_y_yaw(20, 20, 23, 23),
                                    (22.434314575050763, 22.434314575050763,
                                     0.78539816339744828))
    
    def test_poop_back_right(self):
        self.assertItemsAlmostEqual(calc_work_x_y_yaw(20, 20, 3, 10),
                                    (3.6895473721262158, 10.405616101250715,
                                     -2.6098685863309878))
    
    def test_poop_back_left(self):
        self.assertItemsAlmostEqual(calc_work_x_y_yaw(20, 20, 10, 24),
                                    (10.742781352708207, 23.702887458916717,
                                     2.7610862764774282))
    
    
    def assertItemsAlmostEqual(self, a, b):
        self.assertAlmostEqual(len(a), len(b))
        for i in range(len(a)):
            self.assertAlmostEqual(a[i], b[i])


if __name__ == '__main__':
    unittest.main()
