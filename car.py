import mujoco
import mujoco.viewer
import os
import time

curr_dir_path = os.path.dirname(__file__)
xml_path = os.path.join(curr_dir_path, 'scene.xml')

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Run the simulation loop
    while viewer.is_running():
        step_start = time.time()

        # --- MOVEMENT CONTROLS ---
        
      

        # Step the physics
        mujoco.mj_step(model, data)

        # Sync the viewer with the new physics state
        viewer.sync()

        # Try to run in real-time
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)