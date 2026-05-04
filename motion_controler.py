import math
import time
from robot_pose_provider import RobotPoseProvider
from PID_controller import PIDcontroller
from wheel_driver import WheelController

class MotionController:
    def __init__(self, wheel_controller, pose_provider):
        self.wheel_controller = wheel_controller
        self.pose_provider = pose_provider

        self.target_vels = {
            "target_x_vel": 0,
            "target_y_vel": 0,
            "target_yaw_rate": 0,
        }

        self._xvel_pid_control = PIDcontroller(8.0, 1.0, 1.2, 0) #reasonable correction values
        self._yvel_pid_control = PIDcontroller(8.0, 1.0, 1.2, 0) #reasonable correction values
        self._yaw_vel_pid_control = PIDcontroller(6.0, 0.8, 1.0, 0) #reasonable correction values

    def set_target_x_vel(self, x_target):
        self.target_vels["target_x_vel"] = x_target

    def set_target_y_vel(self, y_target):
        self.target_vels["target_y_vel"] = y_target

    def set_target_yaw_rate(self, target_yaw_rate):
        self.target_vels["target_yaw_rate"] = target_yaw_rate
    
    def drive_world_velocity(self, x_ctrl=0.0, y_ctrl=0.0, rot_ctrl=0.0):
        pose = self.pose_provider.pose()
        yaw = pose["yaw"]

        forward_cmd = x_ctrl * math.sin(yaw) - y_ctrl * math.cos(yaw)
        right_cmd = x_ctrl * math.cos(yaw) + y_ctrl * math.sin(yaw)

        self.wheel_controller.drive(
            forward=forward_cmd,
            right=right_cmd,
            rotate=rot_ctrl,
        )

    
    def step(self):
        self._xvel_pid_control.calculate_error(target_val=self.target_vels["target_x_vel"], new_val=self.pose_provider.xvel())
        self._yvel_pid_control.calculate_error(target_val=self.target_vels["target_y_vel"], new_val=self.pose_provider.yvel())
        self._yaw_vel_pid_control.calculate_error(target_val=self.target_vels["target_yaw_rate"], new_val=self.pose_provider.yaw_rate())

        #set control values via world drive function
        self.drive_world_velocity(y_ctrl=self._yvel_pid_control.calculate_control(),
                                  x_ctrl=self._xvel_pid_control.calculate_control(),
                                  rot_ctrl=self._yaw_vel_pid_control.calculate_control())
