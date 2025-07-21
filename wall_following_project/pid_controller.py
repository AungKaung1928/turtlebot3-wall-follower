#!/usr/bin/env python3

class PIDController:
    """Simple PID controller for wall following"""
    
    def __init__(self, kp=0.5, ki=0.0, kd=0.1, dt=0.1):
        self.kp = kp
        self.ki = ki  
        self.kd = kd
        self.dt = dt
        
        self.previous_error = 0.0
        self.integral = 0.0
        self.integral_limit = 1.0  # Prevent integral windup
    
    def update(self, error):
        """Update PID controller with current error"""
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * self.dt
        # Limit integral to prevent windup
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / self.dt
        
        # Calculate output
        output = proportional + integral + derivative
        
        # Store error for next iteration
        self.previous_error = error
        
        return output
    
    def reset(self):
        """Reset PID controller state"""
        self.previous_error = 0.0
        self.integral = 0.0