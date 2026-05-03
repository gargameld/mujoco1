import math


class RobotPoseProvider:
    def __init__(self, model, data, chassis_body_name="chassis"):
        self.model = model
        self.data = data
        self.chassis_body_id = model.body(chassis_body_name).id

    def pose(self):
        pos = self.data.xpos[self.chassis_body_id]
        quat = self.data.xquat[self.chassis_body_id]

        yaw = self._yaw_from_quat(quat)

        return {
            "x": float(pos[0]),
            "y": float(pos[1]),
            "z": float(pos[2]),
            "yaw": float(yaw),
        }

    def x(self):
        return self.pose()["x"]

    def y(self):
        return self.pose()["y"]

    def yaw(self):
        return self.pose()["yaw"]

    @staticmethod
    def _yaw_from_quat(q):
        # MuJoCo quaternions are usually [w, x, y, z]
        w, x, y, z = q

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(siny_cosp, cosy_cosp)