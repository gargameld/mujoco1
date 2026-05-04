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

    def linear_velocity(self):
        qvel_addr = self._chassis_qvel_addr()

        # For a free joint:
        # qvel[0:3] = linear velocity x,y,z
        lin = self.data.qvel[qvel_addr:qvel_addr + 3]

        return {
            "xvel": float(lin[0]),
            "yvel": float(lin[1]),
            "zvel": float(lin[2]),
        }

    def xvel(self):
        return self.linear_velocity()["xvel"]

    def yvel(self):
        return self.linear_velocity()["yvel"]

    def angular_velocity(self):
        qvel_addr = self._chassis_qvel_addr()

        # For a free joint:
        # qvel[3:6] = angular velocity x,y,z
        ang = self.data.qvel[qvel_addr + 3:qvel_addr + 6]

        return {
            "roll_rate": float(ang[0]),
            "pitch_rate": float(ang[1]),
            "yaw_rate": float(ang[2]),
        }

    def yaw_rate(self):
        return self.angular_velocity()["yaw_rate"]

    def _chassis_qvel_addr(self):
        body = self.model.body(self.chassis_body_id)

        if body.jntnum == 0:
            raise ValueError("Chassis body has no joint, so qvel cannot be read directly.")

        joint_id = int(body.jntadr[0])
        return int(self.model.jnt_dofadr[joint_id])

    @staticmethod
    def _yaw_from_quat(q):
        # MuJoCo quaternions are usually [w, x, y, z]
        w, x, y, z = q

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(siny_cosp, cosy_cosp)