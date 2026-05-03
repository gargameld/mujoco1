import mujoco
import mujoco.viewer
import os
import time

curr_dir_path = os.path.dirname(__file__)
xml_path = os.path.join(curr_dir_path, 'scene.xml')

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Motor indices (based on names in XML)
motor_fr = model.actuator("motor_fr").id
motor_fl = model.actuator("motor_fl").id
motor_rr = model.actuator("motor_rr").id
motor_rl = model.actuator("motor_rl").id

arm_actuator_names = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow",
    "wrist_1",
    "wrist_2",
    "wrist_3",
    "tray_rotate",
]
arm_joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
    "tray_rotate_joint",
]
arm_actuators = [model.actuator(name).id for name in arm_actuator_names]
arm_joint_ids = [model.joint(name).id for name in arm_joint_names]
arm_qpos_addr = [model.jnt_qposadr[joint_id] for joint_id in arm_joint_ids]
arm_dof_addr = [model.jnt_dofadr[joint_id] for joint_id in arm_joint_ids]
arm_home_qpos = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0, 0.0]
for qpos_addr, home_qpos in zip(arm_qpos_addr, arm_home_qpos):
    data.qpos[qpos_addr] = home_qpos
mujoco.mj_forward(model, data)

arm_targets = [data.qpos[addr] for addr in arm_qpos_addr]
arm_integral_errors = [0.0 for _ in arm_joint_names]
selected_arm_joint = 0
arm_step = 0.05
arm_control_direction = [1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
arm_kp = [80.0, 80.0, 70.0, 18.0, 18.0, 12.0, 10.0]
arm_ki = [1.0, 1.0, 0.8, 0.2, 0.2, 0.1, 0.1]
arm_kd = [10.0, 10.0, 8.0, 2.5, 2.5, 1.5, 1.0]
arm_integral_limit = [30.0, 30.0, 25.0, 8.0, 8.0, 6.0, 4.0]


def set_arm_target(joint_index, target):
    arm_targets[joint_index] = target
    arm_integral_errors[joint_index] = 0.0


def move_selected_arm_joint(direction):
    set_arm_target(
        selected_arm_joint,
        arm_targets[selected_arm_joint]
        + direction * arm_step * arm_control_direction[selected_arm_joint],
    )


def key_callback(keycode):
    global selected_arm_joint

    try:
        key = chr(keycode).lower()
    except ValueError:
        return

    if key in "1234567":
        selected_arm_joint = int(key) - 1
        print(f"Selected arm joint {selected_arm_joint + 1}: {arm_actuator_names[selected_arm_joint]}")
    elif key == "f":
        move_selected_arm_joint(-1)
    elif key == "g":
        move_selected_arm_joint(1)
    elif key == "0":
        for index, qpos_addr in enumerate(arm_qpos_addr):
            set_arm_target(index, data.qpos[qpos_addr])
        print("Arm targets reset to current pose")


print("Arm controls: 1-7 select joint, f and g move selected joint, 0 resets targets.")

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # --- SIDEWAYS RIGHT MOVEMENT ---
        torque = 0.00  # adjust strength if needed

        data.ctrl[motor_fr] =  torque
        data.ctrl[motor_fl] = -torque
        data.ctrl[motor_rr] = -torque
        data.ctrl[motor_rl] =  torque
        for index, (actuator_id, target, qpos_addr, dof_addr) in enumerate(
            zip(arm_actuators, arm_targets, arm_qpos_addr, arm_dof_addr)
        ):
            error = target - data.qpos[qpos_addr]
            arm_integral_errors[index] += error * model.opt.timestep
            arm_integral_errors[index] = min(
                max(arm_integral_errors[index], -arm_integral_limit[index]),
                arm_integral_limit[index],
            )

            torque_command = (
                arm_kp[index] * error
                + arm_ki[index] * arm_integral_errors[index]
                - arm_kd[index] * data.qvel[dof_addr]
            )
            low, high = model.actuator_ctrlrange[actuator_id]
            data.ctrl[actuator_id] = min(max(torque_command, low), high)

        # Step physics
        mujoco.mj_step(model, data)

        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
