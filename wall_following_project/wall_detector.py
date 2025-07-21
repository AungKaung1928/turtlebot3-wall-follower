#!/usr/bin/env python3

import math
import numpy as np

class WallDetector:
    """Simple wall detection for wall following"""
    
    def __init__(self):
        self.wall_detection_range = 1.5  # Maximum range to consider for wall detection
        self.wall_consistency_threshold = 0.3  # Max difference between readings for consistent wall
    
    def get_range_at_angle(self, laser_data, angle_degrees, tolerance=10):
        """Get laser range at specific angle with tolerance"""
        if laser_data is None:
            return float('inf')
        
        angle_rad = math.radians(angle_degrees)
        angle_min = laser_data.angle_min
        angle_increment = laser_data.angle_increment
        
        center_index = int((angle_rad - angle_min) / angle_increment)
        tolerance_indices = int(math.radians(tolerance) / angle_increment)
        
        start_idx = max(0, center_index - tolerance_indices)
        end_idx = min(len(laser_data.ranges), center_index + tolerance_indices + 1)
        
        min_range = float('inf')
        for i in range(start_idx, end_idx):
            if i < len(laser_data.ranges):
                range_val = laser_data.ranges[i]
                if not math.isnan(range_val) and not math.isinf(range_val) and range_val > 0.1:
                    min_range = min(min_range, range_val)
        
        return min_range if min_range != float('inf') else float('inf')
    
    def detect_wall_on_side(self, laser_data, side='right'):
        """Detect if there's a consistent wall on the specified side"""
        if side == 'right':
            angles = [-90, -45, -135]  # right, front-right, back-right
        else:  # left
            angles = [90, 45, 135]     # left, front-left, back-left
        
        # Get distances at different angles
        distances = []
        for angle in angles:
            dist = self.get_range_at_angle(laser_data, angle)
            if dist < self.wall_detection_range:
                distances.append(dist)
        
        # Check if we have consistent readings
        if len(distances) >= 2:
            # Check consistency (similar distances indicate a wall)
            max_diff = max(distances) - min(distances)
            if max_diff < self.wall_consistency_threshold:
                return True, sum(distances) / len(distances)  # Return average distance
        
        return False, float('inf')
    
    def find_best_wall(self, laser_data):
        """Find the best wall to follow (prefer right side)"""
        # Check right side first
        right_wall, right_dist = self.detect_wall_on_side(laser_data, 'right')
        if right_wall:
            return 'right', right_dist
        
        # Check left side
        left_wall, left_dist = self.detect_wall_on_side(laser_data, 'left')
        if left_wall:
            return 'left', left_dist
            
        return None, float('inf')