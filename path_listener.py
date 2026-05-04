from motion_controler import (
    MotionController,
    POSITION_TOLERANCE,
    YAW_TOLERANCE,
)


class PathListener:
    def __init__(self, motion_controller, path_planner, default_speed=1.0):
        self.motion_controller = motion_controller
        self.path_planner = path_planner
        self.default_speed = default_speed
        self.current_position_index = 0
        self.is_running = False
        self.is_paused = False

    def start(self, reset=True):
        if reset:
            self.current_position_index = 0

        self.is_running = len(self.path_planner) > 0
        self.is_paused = False

        if self.is_running:
            self._send_current_position_target()

    def pause(self):
        self.is_paused = True
        self.motion_controller.stop()

    def resume(self):
        if self.is_finished():
            return

        self.is_running = True
        self.is_paused = False
        self._send_current_position_target()

    def stop(self):
        self.is_running = False
        self.is_paused = False
        self.current_position_index = 0
        self.motion_controller.stop()

    def step(self):
        if not self.is_running or self.is_paused:
            self.motion_controller.step()
            return

        if self.is_finished():
            self.motion_controller.stop()
            self.is_running = False
            return

        self.motion_controller.step()

        if self._current_position_reached():
            self.current_position_index += 1

            if self.is_finished():
                self.motion_controller.stop()
                self.is_running = False
                return

            self._send_current_position_target()

    def is_finished(self):
        return self.current_position_index >= len(self.path_planner)

    def current_position(self):
        if self.is_finished():
            return None

        return self.path_planner.position(self.current_position_index)

    def _send_current_position_target(self):
        position = self.current_position()
        pose = self.motion_controller.pose_provider.pose()

        yaw = pose["yaw"] if position.yaw is None else position.yaw
        speed = self.default_speed if position.speed is None else position.speed
        allow_overshoot = (
            not self._is_last_position()
            if position.allow_overshoot is None
            else position.allow_overshoot
        )

        self.motion_controller.move_to_pose(
            position.x,
            position.y,
            yaw,
            speed,
            allow_overshoot,
        )

    def _current_position_reached(self):
        position = self.current_position()
        if position is None:
            return True

        pose = self.motion_controller.pose_provider.pose()
        x_error = position.x - pose["x"]
        y_error = position.y - pose["y"]
        yaw = pose["yaw"] if position.yaw is None else position.yaw
        yaw_error = MotionController.angle_difference(yaw, pose["yaw"])

        return (
            x_error * x_error + y_error * y_error
            < POSITION_TOLERANCE * POSITION_TOLERANCE
            and abs(yaw_error) < YAW_TOLERANCE
        )

    def _is_last_position(self):
        return self.current_position_index == len(self.path_planner) - 1
