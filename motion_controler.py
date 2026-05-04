import math
from PID_controller import PIDcontroller

VEL_TARGET_STATE = 0
POSE_TARGET_STATE = 1

POSITION_TOLERANCE = 0.03
YAW_TOLERANCE = 0.03

DECAY_PARAMETER = 0.6

MAX_POSE_YAW_RATE = 1.0


class MotionController:
    def __init__(self, wheel_controller, pose_provider):
        self.wheel_controller = wheel_controller
        self.pose_provider = pose_provider
        self.state = VEL_TARGET_STATE

        self.pos_state_translational_speed = None
        self.pos_state_allow_overshoot = None
        self.printed_target_reached = False

        self.target_vels = {
            "target_x_vel": 0.0,
            "target_y_vel": 0.0,
            "target_yaw_rate": 0.0,
        }

        self.dest_pose = {
            "dest_x": None,
            "dest_y": None,
            "dest_yaw": None,
        }

        # Velocity PID controllers.
        self._xvel_pid_control = PIDcontroller(8.0, 0.6, 1.2, 0.0)
        self._yvel_pid_control = PIDcontroller(8.0, 0.6, 1.2, 0.0)
        self._yaw_vel_pid_control = PIDcontroller(8.0, 0.6, 1.2, 0.0)

        # Yaw-position PID.
        # This outputs the target yaw rate for the velocity layer.
        self._yaw_pose_pid_control = PIDcontroller(2.0, 0.2, 0.0, 0.0)

    def set_target_x_vel(self, x_target):
        self.target_vels["target_x_vel"] = x_target

    def set_target_y_vel(self, y_target):
        self.target_vels["target_y_vel"] = y_target

    def set_target_yaw_rate(self, target_yaw_rate):
        self.target_vels["target_yaw_rate"] = target_yaw_rate

    def reset_integral_correction(self):
        self._xvel_pid_control.integral_error = 0.0
        self._yvel_pid_control.integral_error = 0.0
        self._yaw_vel_pid_control.integral_error = 0.0
        self._yaw_pose_pid_control.integral_error = 0.0

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
            x_ctrl=self._xvel_pid_control.calculate_control(),
            y_ctrl=self._yvel_pid_control.calculate_control(),
            rot_ctrl=self._yaw_vel_pid_control.calculate_control(),
        )

    def _step_pose_target(self):
        pose = self.pose_provider.pose()

        x_error = self.dest_pose["dest_x"] - pose["x"]
        y_error = self.dest_pose["dest_y"] - pose["y"]
        distance_error = math.sqrt(x_error * x_error + y_error * y_error)

        yaw_error = MotionController.angle_difference(
            self.dest_pose["dest_yaw"],
            pose["yaw"],
        )

        if distance_error < POSITION_TOLERANCE and abs(yaw_error) < YAW_TOLERANCE:
            if not self.printed_target_reached:
                print(
                    "Entered target area: "
                    f'x={pose["x"]:.3f}, '
                    f'y={pose["y"]:.3f}, '
                    f'yaw={pose["yaw"]:.3f} | '
                    f'target_x={self.dest_pose["dest_x"]:.3f}, '
                    f'target_y={self.dest_pose["dest_y"]:.3f}, '
                    f'target_yaw={self.dest_pose["dest_yaw"]:.3f} | '
                    f'distance_error={distance_error:.3f}, '
                    f'yaw_error={yaw_error:.3f}'
                )
                self.printed_target_reached = True

            if self.pos_state_allow_overshoot:
                # Important:
                # Do not stop.
                # Do not zero target velocities.
                # Do not reset PID integrals.
                # Just leave pose-target mode and keep the current velocity targets active.
                self.state = VEL_TARGET_STATE
                return

            self.stop()
            self.reset_integral_correction()
            return

        speed_decay_factor = (
            1.0
            if self.pos_state_allow_overshoot
            else MotionController.calculate_decay(
                distance_error,
                self.pos_state_translational_speed,
            )
        )

        if distance_error > 0.0:
            x_vel_target = (
                speed_decay_factor
                * self.pos_state_translational_speed
                * x_error
                / distance_error
            )
            y_vel_target = (
                speed_decay_factor
                * self.pos_state_translational_speed
                * y_error
                / distance_error
            )
        else:
            x_vel_target = 0.0
            y_vel_target = 0.0

        # Yaw-position PID:
        # We want yaw_error -> 0.
        # PIDcontroller computes error = target_val - new_val.
        # So target_val=0 and new_val=-yaw_error gives PID error = yaw_error.
        self._yaw_pose_pid_control.calculate_error(
            target_val=0.0,
            new_val=-yaw_error,
        )

        yaw_rate_target = self._clamp(
            self._yaw_pose_pid_control.calculate_control(),
            -MAX_POSE_YAW_RATE,
            MAX_POSE_YAW_RATE,
        )

        self.set_target_x_vel(x_vel_target)
        self.set_target_y_vel(y_vel_target)
        self.set_target_yaw_rate(yaw_rate_target)

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

        self.printed_target_reached = False
        self.reset_integral_correction()

        self.state = POSE_TARGET_STATE

    @staticmethod
    def calculate_decay(distance, speed):
        if speed <= 0.0:
            return 0.0

        return 1.0 - math.exp(-DECAY_PARAMETER * distance / speed)

    @staticmethod
    def angle_difference(target, current):
        return math.atan2(
            math.sin(target - current),
            math.cos(target - current),
        )

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))