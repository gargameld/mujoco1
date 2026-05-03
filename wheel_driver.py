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
        max_ctrl=1.0,
    ):
        self.model = model
        self.data = data
        self.max_ctrl = max_ctrl
        self.motor_fr = model.actuator(self.actuator_names["front_right"]).id
        self.motor_fl = model.actuator(self.actuator_names["front_left"]).id
        self.motor_rr = model.actuator(self.actuator_names["rear_right"]).id
        self.motor_rl = model.actuator(self.actuator_names["rear_left"]).id


    def set_wheel_ctrls(self, front_right, front_left, rear_right, rear_left):
        self.data.ctrl[self.motor_fr] = front_right
        self.data.ctrl[self.motor_fl] = front_left
        self.data.ctrl[self.motor_rr] = rear_right
        self.data.ctrl[self.motor_rl] = rear_left

    def stop(self):
        self.set_wheel_ctrls(0.0, 0.0, 0.0, 0.0)

    def drive(self, forward=0.0, right=0.0, rotate=0.0):
        front_right = forward - right - rotate
        front_left = forward + right - rotate
        rear_right = forward + right + rotate
        rear_left = forward - right + rotate
        max_command = max(abs(front_right), abs(front_left), abs(rear_right), abs(rear_left) , self.max_ctrl)

        self.set_wheel_ctrls(
            self.max_ctrl * front_right / max_command,
            self.max_ctrl * front_left / max_command,
            self.max_ctrl * rear_right / max_command,
            self.max_ctrl * rear_left / max_command,
        )
    
