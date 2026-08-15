#!/usr/bin/env python3
"""Unit tests for path management"""

import unittest
import math
from navigation.path import Path

class TestPath(unittest.TestCase):
    
    def test_path_creation(self):
        """Test basic path creation"""
        points = [(0, 0), (1, 0), (2, 1)]
        path = Path(points)
        self.assertEqual(len(path.points), 3)
    
    def test_path_validation(self):
        """Test path validation"""
        with self.assertRaises(ValueError):
            Path([(0, 0)])  # Too few points
    
    def test_nearest_point(self):
        """Test nearest point finding"""
        path = Path([(0, 0), (1, 0), (2, 0)])
        index, distance = path.get_nearest_point((1.2, 0.1))
        self.assertEqual(index, 1)
        self.assertLess(distance, 0.2)
    
    def test_point_at_distance(self):
        """Test point at distance"""
        path = Path([(0, 0), (1, 0), (2, 0)])
        
        # Point at 0.5m should be (0.5, 0)
        point = path.get_point_at_distance(0, 0.5)
        self.assertAlmostEqual(point[0], 0.5)
        self.assertAlmostEqual(point[1], 0.0)
        
        # Point at 1.5m should be (1.5, 0)
        point = path.get_point_at_distance(0, 1.5)
        self.assertAlmostEqual(point[0], 1.5)
        self.assertAlmostEqual(point[1], 0.0)
    
    def test_remaining_distance(self):
        """Test remaining distance calculation"""
        path = Path([(0, 0), (1, 0), (2, 0)])
        
        remaining = path.get_remaining_distance(0)
        self.assertAlmostEqual(remaining, 2.0)
        
        remaining = path.get_remaining_distance(1)
        self.assertAlmostEqual(remaining, 1.0)

if __name__ == '__main__':
    unittest.main()