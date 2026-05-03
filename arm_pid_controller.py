import random

import mujoco


class ArmPidController:
    actuator_names = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow",
        "wrist_1",
        "wrist_2",
        "wrist_3",
        "tray_rotate",
    ]
    joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
        "tray_rotate_joint",
    ]
    home_qpos = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0, 0.0]
    control_direction = [1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    kp = [80.0, 80.0, 70.0, 18.0, 18.0, 12.0, 10.0]
    ki = [1.0, 1.0, 0.8, 0.2, 0.2, 0.1, 0.1]
    kd = [10.0, 10.0, 8.0, 2.5, 2.5, 1.5, 1.0]
    integral_limit = [30.0, 30.0, 25.0, 8.0, 8.0, 6.0, 4.0]

    def __init__(self, model, data, encoder_noise_std=0.0005, step_size=0.05):
        self.model = model
        self.data = data
        self.encoder_noise_std = encoder_noise_std
        self.step_size = step_size

        self.actuators = [model.actuator(name).id for name in self.actuator_names]
        joint_ids = [model.joint(name).id for name in self.joint_names]
        self.qpos_addr = [model.jnt_qposadr[joint_id] for joint_id in joint_ids]

        for qpos_addr, home_qpos in zip(self.qpos_addr, self.home_qpos):
            data.qpos[qpos_addr] = home_qpos
        mujoco.mj_forward(model, data)

        self.targets = [data.qpos[addr] for addr in self.qpos_addr]
        self.integral_errors = [0.0 for _ in self.joint_names]
        self.last_measured_qpos = [data.qpos[addr] for addr in self.qpos_addr]

    def read_noisy_angles(self):
        return [
            self.data.qpos[qpos_addr] + random.gauss(0.0, self.encoder_noise_std)
            for qpos_addr in self.qpos_addr
        ]

    def set_target(self, joint_index, target):
        self.targets[joint_index] = target
        self.integral_errors[joint_index] = 0.0

    def move_joint(self, joint_index, direction):
        self.set_target(
            joint_index,
            self.targets[joint_index]
            + direction * self.step_size * self.control_direction[joint_index],
        )

    def reset_targets_to_measured_pose(self):
        for index, measured_qpos in enumerate(self.read_noisy_angles()):
            self.set_target(index, measured_qpos)

    def step(self):
        measured_qpos = self.read_noisy_angles()
        measured_qvel = [
            (qpos - last_qpos) / self.model.opt.timestep
            for qpos, last_qpos in zip(measured_qpos, self.last_measured_qpos)
        ]
        self.last_measured_qpos[:] = measured_qpos

        for index, (actuator_id, target, qpos, qvel) in enumerate(
            zip(self.actuators, self.targets, measured_qpos, measured_qvel)
        ):
            error = target - qpos
            self.integral_errors[index] = (
                0.99 * self.integral_errors[index] + error * self.model.opt.timestep
            )
            self.integral_errors[index] = min(
                max(self.integral_errors[index], -self.integral_limit[index]),
                self.integral_limit[index],
            )

            torque_command = (
                self.kp[index] * error
                + self.ki[index] * self.integral_errors[index]
                - self.kd[index] * qvel
            )
            low, high = self.model.actuator_ctrlrange[actuator_id]
            self.data.ctrl[actuator_id] = min(max(torque_command, low), high)
