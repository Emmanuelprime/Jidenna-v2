#!/usr/bin/env python3
"""Unit tests for Pure Pursuit algorithm"""

import unittest
import math
from navigation.pure_pursuite import PurePursuit
from navigation.path import Path

class TestPurePursuit(unittest.TestCase):
    
    def setUp(self):
        self.pp = PurePursuit(lookahead_distance=0.5)
    
    def test_curvature_straight_line(self):
        """Test curvature for point directly ahead"""
        pose = (0, 0, 0)  # At origin, facing +X
        target = (1, 0)   # Point straight ahead
        
        curvature = self.pp.compute_curvature(pose, target)
        self.assertAlmostEqual(curvature, 0.0, places=3)
    
    def test_curvature_left_turn(self):
        """Test curvature for point to the left"""
        pose = (0, 0, 0)  # At origin, facing +X
        target = (1, 1)   # Point ahead and left
        
        curvature = self.pp.compute_curvature(pose, target)
        self.assertGreater(curvature, 0)  # Positive curvature = left turn
    
    def test_curvature_right_turn(self):
        """Test curvature for point to the right"""
        pose = (0, 0, 0)  # At origin, facing +X
        target = (1, -1)  # Point ahead and right
        
        curvature = self.pp.compute_curvature(pose, target)
        self.assertLess(curvature, 0)  # Negative curvature = right turn
    
    def test_curvature_behind(self):
        """Test curvature for point behind robot"""
        pose = (0, 0, 0)  # At origin, facing +X
        target = (-1, 0)  # Point behind
        
        curvature = self.pp.compute_curvature(pose, target)
        self.assertAlmostEqual(curvature, 0.0, places=3)
    
    def test_lookahead_point_selection(self):
        """Test lookahead point selection"""
        path = Path([(0, 0), (1, 0), (2, 0), (3, 0)])
        pose = (0, 0, 0)
        
        point, index = self.pp.select_lookahead_point(path, pose, 0)
        
        self.assertIsNotNone(point)
        self.assertGreater(index, 0)
        self.assertAlmostEqual(point[0], 0.5, places=1)  # Should be at 0.5m
        self.assertAlmostEqual(point[1], 0.0, places=2)
    
    def test_goal_reached(self):
        """Test goal reaching detection"""
        pose = (0.05, 0.03, 0.1)  # Close to goal
        goal = (0, 0)
        
        pos_reached, heading_reached = self.pp.is_goal_reached(
            pose, goal, position_tolerance=0.1, heading_tolerance=math.radians(20)
        )
        
        self.assertTrue(pos_reached)
        self.assertTrue(heading_reached)
    
    def test_angle_normalization(self):
        """Test angle normalization"""
        # Test positive angle > pi
        angle1 = self.pp._normalize_angle(3 * math.pi)
        self.assertAlmostEqual(angle1, math.pi)
        
        # Test negative angle < -pi
        angle2 = self.pp._normalize_angle(-3 * math.pi)
        self.assertAlmostEqual(angle2, -math.pi)

if __name__ == '__main__':
    unittest.main()