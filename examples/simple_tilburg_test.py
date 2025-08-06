from time import sleep

import numpy as np
from rcs._core.common import GraspType
from rcs.hand.tilburg_hand import THConfig, TilburgHand


def sim_real_index_flip(q: np.ndarray):
    if len(q) == 18:
        # This is the qpos for the real robot
        q = q[:16]
    assert len(q) == 16, "Expected 16 joint positions for the Tilburg hand"
    q2 = q.copy()
    for i in range(4):
        for j in range(4):
            q2[i * 4 + j] = q[i * 4 + (3 - j)]
    return q2


def sim_real_name_flip(names):
    names2 = names.copy()
    for i in range(4):
        for j in range(4):
            names2[i * 4 + j] = names[i * 4 + (3 - j)]
    return names2


def pad_qpos(q: np.ndarray):
    if len(q) == 16:
        # This is the qpos for the real robot
        q = np.concatenate((q, np.zeros(2)))
    assert len(q) == 18, "Expected 18 joint positions for the Tilburg hand"
    return q


if __name__ == "__main__":
    config = {
        "motor_calib_min_range_deg": [-5, 0, 0, 0, -5, -5, 0, -25, -5, -5, 0, -25, -5, -5, 0, -25, 0, 0],
        "motor_calib_max_range_deg": [95, 90, 100, 90, 95, 95, 95, 25, 95, 95, 95, 25, 95, 95, 95, 25, 1, 1],
    }
    power_grasp_values = [
        0.5,
        0.5,
        0.5,
        1.4,  # THUMB_(IP, MCP, ABD, CMC)
        0.5,
        0.5,
        1.0,
        0.7,  # INDEX_(DIP, PIP, MCP, ABD)
        0.5,
        0.5,
        1.0,
        0.3,
        0.5,
        0.5,
        1.0,
        0.0,
        0,
        0,
    ]
    rads = []
    min_joints_radians = [np.deg2rad(angle) for angle in config["motor_calib_min_range_deg"]]
    max_joints_radians = [np.deg2rad(angle) for angle in config["motor_calib_max_range_deg"]]

    for i in range(len(config["motor_calib_min_range_deg"])):
        joint_angle = (
            power_grasp_values[i] * (config["motor_calib_max_range_deg"][i] - config["motor_calib_min_range_deg"][i])
            + config["motor_calib_min_range_deg"][i]
        )
        rads.append(np.deg2rad(joint_angle))

        print(f"Motor {i}: {joint_angle:.2f} degrees")
    print("power_grasp_radians=[", ", ".join(f"{rad:.2f}" for rad in rads), "]")
    print("min_joints_radians=[", ", ".join(f"{rad:.2f}" for rad in min_joints_radians), "]")
    print("max_joints_radians=[", ", ".join(f"{rad:.2f}" for rad in max_joints_radians), "]")
    config = THConfig(
        calibration_file="/home/sbien/Documents/Development/RCS/robot-control-stack/python/rcs/hand/calibration_og.json",
        grasp_percentage=1,
        hand_orientation="right",
    )
    hand = TilburgHand(cfg=config, verbose=True)
    hand.set_grasp_type(GraspType.POWER_GRASP)
    hand.grasp()
    sleep(2)
    hand.open()
    sleep(5)
    hand.reset()
    hand.close()
    hand.disconnect()
