import math


class RobotPoseProvider:
    def __init__(self, model, data, chassis_body_name="chassis"):
        self.model = model
        self.data = data
        self.chassis_body_id = model.body(chassis_body_name).id
        self.freejoint_qpos_addr = self._find_chassis_freejoint_qpos_addr()

    def pose(self):
        qpos = self.data.qpos
        addr = self.freejoint_qpos_addr
        qw, qx, qy, qz = (float(qpos[addr + i]) for i in range(3, 7))
        return {
            "x": float(qpos[addr]),
            "y": float(qpos[addr + 1]),
            "z": float(qpos[addr + 2]),
            "yaw": self._yaw_from_quaternion(qw, qx, qy, qz),
        }

    def xy(self):
        pose = self.pose()
        return (pose["x"], pose["y"])

    def yaw(self):
        return self.pose()["yaw"]

    def _find_chassis_freejoint_qpos_addr(self):
        body = self.model.body(self.chassis_body_id)
        if int(body.jntnum[0]) < 1:
            raise ValueError("Chassis body does not have a joint")

        joint_id = int(body.jntadr[0])
        return int(self.model.jnt_qposadr[joint_id])

    @staticmethod
    def _yaw_from_quaternion(qw, qx, qy, qz):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
