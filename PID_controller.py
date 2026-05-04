import time

# need to add output clamp and integral clamp
class PIDcontroller:
    def __init__(self, kp, kd, ki, initial_val):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.last_updated_time = time.time() #might be problematic in the future
        self.estimated_val_dot = 0.0
        self.current_val = initial_val
        self.val_error = 0.0
        self.val_dot_error = 0.0
        self.integral_error = 0.0

    def calculate_error(self, new_val, target_val):
        now = time.time()
        dt = (now - self.last_updated_time)
        if dt <= 0.0:
            dt = 1e-6
        self.val_error = target_val - new_val
        new_estimated_val_dot = 0.9 * self.estimated_val_dot + 0.1 * (new_val - self.current_val)/dt
        self.estimated_val_dot = new_estimated_val_dot
        self.val_dot_error = new_estimated_val_dot
        self.integral_error += self.val_error * dt
    
        self.last_updated_time = now
        self.current_val = new_val
    
    def calculate_control(self):
        return self.kp * self.val_error - self.kd * self.val_dot_error + self.ki * self.integral_error
        