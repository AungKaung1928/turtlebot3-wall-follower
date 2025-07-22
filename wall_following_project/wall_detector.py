#!/usr/bin/env python3
import math

class WallDetector:
    def __init__(self):
        self.wall_range = 1.5
        self.consistency = 0.3

    def get_range_at_angle(self, laser_data, angle, tol=10):
        if not laser_data: return float('inf')
        
        idx = int((math.radians(angle) - laser_data.angle_min) / laser_data.angle_increment)
        tol_idx = int(math.radians(tol) / laser_data.angle_increment)
        
        start, end = max(0, idx-tol_idx), min(len(laser_data.ranges), idx+tol_idx+1)
        valid = [r for r in laser_data.ranges[start:end] 
                if not (math.isnan(r) or math.isinf(r)) and r > 0.1]
        return min(valid) if valid else float('inf')

    def detect_wall_on_side(self, laser_data, side='right'):
        angles = [-90, -45, -135] if side == 'right' else [90, 45, 135]
        dists = [self.get_range_at_angle(laser_data, a) for a in angles]
        valid = [d for d in dists if d < self.wall_range]
        
        if len(valid) >= 2 and max(valid) - min(valid) < self.consistency:
            return True, sum(valid) / len(valid)
        return False, float('inf')

    def find_best_wall(self, laser_data):
        for side in ['right', 'left']:
            found, dist = self.detect_wall_on_side(laser_data, side)
            if found: return side, dist
        return None, float('inf')