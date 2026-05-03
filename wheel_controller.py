import math

from robot_pose_provider import RobotPoseProvider


class WheelController:
    actuator_names = {
        "front_right": "motor_fr",
        "front_left": "motor_fl",
        "rear_right": "motor_rr",
        "rear_left": "motor_rl",
    }

    def __init__(
        self,
        model,
        data,
        pose_provider=None,
        chassis_body_name="chassis",
        default_torque=0.05,
    ):
        self.model = model
        self.data = data
        self.default_torque = default_torque
        self.pose_provider = pose_provider or RobotPoseProvider(model, data, chassis_body_name)
        self.motor_fr = model.actuator(self.actuator_names["front_right"]).id
        self.motor_fl = model.actuator(self.actuator_names["front_left"]).id
        self.motor_rr = model.actuator(self.actuator_names["rear_right"]).id
        self.motor_rl = model.actuator(self.actuator_names["rear_left"]).id
        self.command = None
        self.position_tolerance = 0.015
        self.yaw_tolerance = 0.03
        self.xy_kp = 4.0
        self.xy_ki = 0.0
        self.xy_kd = 0.0
        self.yaw_kp = 2.0
        self.yaw_ki = 0.0
        self.yaw_kd = 0.0
        self.integral_limit = 0.4

    def set_wheel_torques(self, front_right, front_left, rear_right, rear_left):
        self.data.ctrl[self.motor_fr] = front_right
        self.data.ctrl[self.motor_fl] = front_left
        self.data.ctrl[self.motor_rr] = rear_right
        self.data.ctrl[self.motor_rl] = rear_left

    def stop(self):
        self.command = None
        self.set_wheel_torques(0.0, 0.0, 0.0, 0.0)

    def drive(self, forward=0.0, right=0.0, rotate=0.0, torque=None):
        torque = self.default_torque if torque is None else torque
        front_right = forward - right - rotate
        front_left = forward + right - rotate
        rear_right = forward + right + rotate
        rear_left = forward - right + rotate
        max_command = max(abs(front_right), abs(front_left), abs(rear_right), abs(rear_left), 1.0)
        self.set_wheel_torques(
            torque * front_right / max_command,
            torque * front_left / max_command,
            torque * rear_right / max_command,
            torque * rear_left / max_command,
        )

    def move_forward(self, distance, torque=None):
        self.move_by(distance, forward=1.0, right=0.0, torque=torque)

    def move_backward(self, distance, torque=None):
        self.move_by(distance, forward=-1.0, right=0.0, torque=torque)

    def strafe_right(self, distance, torque=None):
        self.move_by(distance, forward=0.0, right=1.0, torque=torque)

    def strafe_left(self, distance, torque=None):
        self.move_by(distance, forward=0.0, right=-1.0, torque=torque)

    def move_by(self, distance, forward=0.0, right=0.0, torque=None):
        length = math.hypot(forward, right)
        if length == 0.0 or distance == 0.0:
            self.stop()
            return

        sign = 1.0 if distance > 0.0 else -1.0
        pose = self.pose_provider.pose()
        yaw = pose["yaw"]
        forward /= length
        right /= length
        distance = abs(distance)
        target_x = pose["x"] + sign * distance * (
            forward * math.sin(yaw) + right * math.cos(yaw)
        )
        target_y = pose["y"] + sign * distance * (
            -forward * math.cos(yaw) + right * math.sin(yaw)
        )
        self.command = {
            "type": "move",
            "target_x": target_x,
            "target_y": target_y,
            "target_yaw": yaw,
            "torque": self.default_torque if torque is None else torque,
            "integral_forward": 0.0,
            "integral_right": 0.0,
            "integral_yaw": 0.0,
            "previous_forward_error": 0.0,
            "previous_right_error": 0.0,
            "previous_yaw_error": 0.0,
        }

    def rotate(self, angle_radians, torque=None):
        if angle_radians == 0.0:
            self.stop()
            return

        pose = self.pose_provider.pose()
        self.command = {
            "type": "rotate",
            "target_x": pose["x"],
            "target_y": pose["y"],
            "target_yaw": self._wrap_angle(pose["yaw"] + angle_radians),
            "torque": self.default_torque if torque is None else torque,
            "integral_forward": 0.0,
            "integral_right": 0.0,
            "integral_yaw": 0.0,
            "previous_forward_error": 0.0,
            "previous_right_error": 0.0,
            "previous_yaw_error": 0.0,
        }

    def step(self):
        if self.command is None:
            self.stop()
            return

        self._step_pose_pid()

    def _step_pose_pid(self):
        pose = self.pose_provider.pose()
        error_x = self.command["target_x"] - pose["x"]
        error_y = self.command["target_y"] - pose["y"]
        yaw_error = self._angle_difference(self.command["target_yaw"], pose["yaw"])

        forward_axis_x = math.sin(pose["yaw"])
        forward_axis_y = -math.cos(pose["yaw"])
        right_axis_x = math.cos(pose["yaw"])
        right_axis_y = math.sin(pose["yaw"])
        forward_error = error_x * forward_axis_x + error_y * forward_axis_y
        right_error = error_x * right_axis_x + error_y * right_axis_y

        if math.hypot(error_x, error_y) <= self.position_tolerance and abs(yaw_error) <= self.yaw_tolerance:
            self.stop()
            return

        dt = self.model.opt.timestep
        forward_command = self._pid_axis(self.command, "forward", forward_error, dt, self.xy_kp, self.xy_ki, self.xy_kd)
        right_command = self._pid_axis(self.command, "right", right_error, dt, self.xy_kp, self.xy_ki, self.xy_kd)
        yaw_command = self._pid_axis(self.command, "yaw", yaw_error, dt, self.yaw_kp, self.yaw_ki, self.yaw_kd)
        self.drive(
            forward=forward_command,
            right=right_command,
            rotate=yaw_command,
            torque=self.command["torque"],
        )

    def _pid_axis(self, command, axis, error, dt, kp, ki, kd):
        integral_key = f"integral_{axis}"
        previous_key = f"previous_{axis}_error"
        command[integral_key] = min(
            max(command[integral_key] + error * dt, -self.integral_limit),
            self.integral_limit,
        )
        derivative = (error - command[previous_key]) / dt
        command[previous_key] = error
        return kp * error + ki * command[integral_key] + kd * derivative

    @staticmethod
    def _angle_difference(angle, reference):
        return math.atan2(math.sin(angle - reference), math.cos(angle - reference))

    @staticmethod
    def _wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))
