import mujoco
import mujoco.viewer
import math
import os
import time

from arm_pid_controller import ArmPidController
from rangefinder import RangefinderReader
from robot_pose_provider import RobotPoseProvider
from wheel_driver import WheelController
from motion_controler import MotionController
from path_planner import PathPlanner
from path_listener import PathListener

curr_dir_path = os.path.dirname(__file__)
xml_path = os.path.join(curr_dir_path, 'model', 'scene.xml')

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

arm_controller = ArmPidController(model, data)
robot_pose_provider = RobotPoseProvider(model, data)
wheel_controller = WheelController(model, data)
rangefinder_reader = RangefinderReader(model, data)
selected_arm_joint = 0
last_rangefinder_print = 0.0
PATH_STEP_DISTANCE = 0.2
PATH_YAW = 0.0


def build_dense_path(anchor_poses, step_distance):
    dense_path = []

    for index, anchor_pose in enumerate(anchor_poses):
        if index == 0:
            dense_path.append(anchor_pose)
            continue

        previous_pose = anchor_poses[index - 1]
        x_distance = anchor_pose["x"] - previous_pose["x"]
        y_distance = anchor_pose["y"] - previous_pose["y"]
        distance = math.sqrt(x_distance * x_distance + y_distance * y_distance)
        steps = max(1, math.ceil(distance / step_distance))

        for step in range(1, steps + 1):
            ratio = step / steps
            dense_pose = {
                "x": previous_pose["x"] + x_distance * ratio,
                "y": previous_pose["y"] + y_distance * ratio,
                "yaw": anchor_pose.get("yaw", PATH_YAW),
                "speed": anchor_pose.get("speed"),
            }

            if index == len(anchor_poses) - 1 and step == steps:
                dense_pose["allow_overshoot"] = False

            dense_path.append(dense_pose)

    return dense_path

def too_close(ranges):
    min_front = 0.18
    min_side = 0.10

    return (
        0.0 < ranges["front_right"] < min_front
        or 0.0 < ranges["front_right"] < min_front
        or 0.0 < ranges["rear_right"] < min_front
        or 0.0 < ranges["rear_left"] < min_front
        or 0.0 < ranges["rear"] < min_front
        or 0.0 < ranges["front"] < min_front
    )

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


def print_rangefinder_distances():
    distances = rangefinder_reader.distances()
    formatted_distances = ", ".join(
        f"{name}: {distance:.3f}m" if distance >= 0.0 else f"{name}: no hit"
        for name, distance in distances.items()
    )
    print(f"Rangefinders: {formatted_distances}")


print(
    "Arm controls: 1-7 select joint, f and g move selected joint, 0 resets targets. "
    "Wheel controls: w/s/a/d move 1m, q/e rotate 90deg, x stop."
)

direction = 1
last_pose_print_time = time.time()
motion_controller = MotionController(wheel_controller=wheel_controller, pose_provider=robot_pose_provider)

parking_anchor_poses = [
    {"x": 0.0, "y": 0.0, "yaw": PATH_YAW, "speed": 1.5},
    {"x": 1.2, "y": -7.0, "yaw": PATH_YAW, "speed": 1.5},
    {"x": 3.5, "y": -7.0, "yaw": PATH_YAW, "speed": 0.8},
    {"x": 1.2, "y": -7.0, "yaw": PATH_YAW, "speed": 0.8},
    {"x": 1.2, "y": -5.0, "yaw": PATH_YAW, "speed": 1.5},
    {"x": 3.5, "y": -5.0, "yaw": PATH_YAW, "speed": 0.8},
    {"x": 1.2, "y": -5.0, "yaw": PATH_YAW, "speed": 0.8},
    {"x": 1.2, "y": -3.0, "yaw": PATH_YAW, "speed": 1.5},
    {"x": 3.5, "y": -3.0, "yaw": PATH_YAW, "speed": 0.8},
    {"x": 1.2, "y": -3.0, "yaw": PATH_YAW, "speed": 0.8},
    {"x": 1.2, "y": -1.0, "yaw": PATH_YAW, "speed": 1.5},
    {"x": 3.5, "y": -1.0, "yaw": PATH_YAW, "speed": 0.8},
]
parking_path = PathPlanner(
    build_dense_path(parking_anchor_poses, PATH_STEP_DISTANCE)
)
path_listener = PathListener(
    motion_controller=motion_controller,
    path_planner=parking_path,
    default_speed=1.0,
)
path_listener.start()

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    while viewer.is_running():
        step_start = time.time()

        

        # Step physics
        mujoco.mj_step(model, data)

        

        viewer.sync()
        arm_controller.step()
        path_listener.step()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
