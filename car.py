import mujoco
import mujoco.viewer
import os
import time

from arm_pid_controller import ArmPidController

curr_dir_path = os.path.dirname(__file__)
xml_path = os.path.join(curr_dir_path, 'scene.xml')

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Motor indices (based on names in XML)
motor_fr = model.actuator("motor_fr").id
motor_fl = model.actuator("motor_fl").id
motor_rr = model.actuator("motor_rr").id
motor_rl = model.actuator("motor_rl").id

arm_controller = ArmPidController(model, data)
selected_arm_joint = 0


def key_callback(keycode):
    global selected_arm_joint

    try:
        key = chr(keycode).lower()
    except ValueError:
        return

    if key in "1234567":
        selected_arm_joint = int(key) - 1
        print(
            f"Selected arm joint {selected_arm_joint + 1}: "
            f"{arm_controller.actuator_names[selected_arm_joint]}"
        )
    elif key == "f":
        arm_controller.move_joint(selected_arm_joint, -1)
    elif key == "g":
        arm_controller.move_joint(selected_arm_joint, 1)
    elif key == "0":
        arm_controller.reset_targets_to_measured_pose()
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
        arm_controller.step()

        # Step physics
        mujoco.mj_step(model, data)

        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
