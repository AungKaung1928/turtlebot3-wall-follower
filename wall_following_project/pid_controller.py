#!/usr/bin/env python3

class PIDController:
    def __init__(self, kp=0.5, ki=0.0, kd=0.1, dt=0.1):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.prev_error = self.integral = 0.0

    def update(self, error):
        self.integral += error * self.dt
        self.integral = max(min(self.integral, 1.0), -1.0)  # Limit windup
        
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * (error - self.prev_error) / self.dt)
        
        self.prev_error = error
        return output

    def reset(self):
        self.prev_error = self.integral = 0.0