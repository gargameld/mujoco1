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

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # --- SIDEWAYS RIGHT MOVEMENT ---
        torque = 0.01  # adjust strength if needed

        data.ctrl[motor_fr] =  torque
        data.ctrl[motor_fl] = -torque
        data.ctrl[motor_rr] = -torque
        data.ctrl[motor_rl] =  torque

        # Step physics
        mujoco.mj_step(model, data)

        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)