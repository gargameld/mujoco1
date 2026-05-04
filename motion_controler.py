import math
from PID_controller import PIDcontroller

VEL_TARGET_STATE = 0
POSE_TARGET_STATE = 1
POSITION_TOLERANCE = 0.03
YAW_TOLERANCE = 0.03
DECAY_PARAMETER = 0.6

class MotionController:
    def __init__(self, wheel_controller, pose_provider):
        self.wheel_controller = wheel_controller
        self.pose_provider = pose_provider
        self.state = VEL_TARGET_STATE
        self.pos_state_translational_speed = None
        self.pos_state_angular_speed = None
        self.pos_state_allow_overshoot = None

        self.target_vels = {
            "target_x_vel": 0,
            "target_y_vel": 0,
            "target_yaw_rate": 0,
        }

        self.dest_pose = {
            "dest_x": None,
            "dest_y": None,
            "dest_yaw": None,
        }

        self._xvel_pid_control = PIDcontroller(8.0, 0.6, 1.2, 0) #reasonable correction values
        self._yvel_pid_control = PIDcontroller(8.0, 0.6, 1.2, 0) #reasonable correction values
        self._yaw_vel_pid_control = PIDcontroller(8.0, 0.6, 1.2, 0) #reasonable correction values

    def set_target_x_vel(self, x_target):
        self.target_vels["target_x_vel"] = x_target

    def set_target_y_vel(self, y_target):
        self.target_vels["target_y_vel"] = y_target

    def set_target_yaw_rate(self, target_yaw_rate):
        self.target_vels["target_yaw_rate"] = target_yaw_rate

    def reset_integral_correction(self):
        self._xvel_pid_control.integral_error = 0
        self._yvel_pid_control.integral_error = 0
        self._yaw_vel_pid_control.integral_error = 0
    
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
        if self.state == VEL_TARGET_STATE:
            self._step_velocity_target()
        else:
            self._step_pose_target()


    def _step_velocity_target(self):
        self._xvel_pid_control.calculate_error(
            target_val=self.target_vels["target_x_vel"],
            new_val=self.pose_provider.xvel(),
        )
        self._yvel_pid_control.calculate_error(
            target_val=self.target_vels["target_y_vel"],
            new_val=self.pose_provider.yvel(),
        )
        self._yaw_vel_pid_control.calculate_error(
            target_val=self.target_vels["target_yaw_rate"],
            new_val=self.pose_provider.yaw_rate(),
        )

        self.drive_world_velocity(
            y_ctrl=self._yvel_pid_control.calculate_control(),
            x_ctrl=self._xvel_pid_control.calculate_control(),
            rot_ctrl=self._yaw_vel_pid_control.calculate_control(),
        )


    def _step_pose_target(self):        
        x_error = self.dest_pose["dest_x"] - self.pose_provider.x()
        y_error = self.dest_pose["dest_y"] - self.pose_provider.y()
        distance_error = math.sqrt(x_error * x_error + y_error * y_error)
        yaw_error = self.dest_pose["dest_yaw"] - self.pose_provider.yaw()

        speed_decay_factor = 1 if self.pos_state_allow_overshoot else MotionController.calculate_decay(distance_error, self.pos_state_translational_speed)
        
        x_vel_target = speed_decay_factor * self.pos_state_translational_speed * x_error / distance_error
        y_vel_target = speed_decay_factor * self.pos_state_translational_speed * y_error / distance_error
        
        
        yaw_rate_target = speed_decay_factor * self.pos_state_angular_speed

        if abs(distance_error < POSITION_TOLERANCE and abs(yaw_error) < YAW_TOLERANCE):
            self.wheel_controller.stop()
            self.state = VEL_TARGET_STATE
            if(self.pos_state_allow_overshoot == False):
                print(self.pose_provider.x())
                print(self.pose_provider.y())
                self.reset_integral_correction()
                self.stop()
                self._step_velocity_target()
        else:
            self.set_target_x_vel(x_target=x_vel_target)
            self.set_target_y_vel(y_target=y_vel_target)
            self.set_target_yaw_rate(target_yaw_rate=yaw_rate_target)

            self._step_velocity_target()

    def stop(self):
        self.set_target_x_vel(0.0)
        self.set_target_y_vel(0.0)
        self.set_target_yaw_rate(0.0)
        self.wheel_controller.stop()
        self.state = VEL_TARGET_STATE


            
            
            

    def move_to_pose(self, dest_x, dest_y, dest_yaw, speed, allow_overshoot=True):
        self.pos_state_allow_overshoot = allow_overshoot
        self.dest_pose["dest_x"] = dest_x
        self.dest_pose["dest_y"] = dest_y
        self.dest_pose["dest_yaw"] = dest_yaw
        self.pos_state_translational_speed = speed
        x_error = self.dest_pose["dest_x"] - self.pose_provider.x()
        y_error = self.dest_pose["dest_y"] - self.pose_provider.y()
        distance_error = math.sqrt(x_error * x_error + y_error * y_error)

        estimated_time = distance_error / self.pos_state_translational_speed
        print(distance_error)
        self.pos_state_angular_speed = (dest_yaw - self.pose_provider.yaw())/estimated_time
        
        self.state = POSE_TARGET_STATE
    
    @staticmethod
    def calculate_decay(distance, speed):
        return (math.exp(DECAY_PARAMETER * distance / speed) - 1) / math.exp(DECAY_PARAMETER * distance / speed )
        
