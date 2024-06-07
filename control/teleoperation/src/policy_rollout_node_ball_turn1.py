#!/usr/bin/env python3

import time
import mujoco
import mujoco.viewer
import numpy as np
import os
import rospy
from std_msgs.msg import Float32MultiArray
from icecream import ic
from collections import deque

SIMULATE_MUJOCO = True
joint_history = deque(maxlen=60)

def smooth_joint_history(current_joint_angles):
    """Smooths the joint history and returns the smoothed joint angles.
    """
    joint_history.append(current_joint_angles)
    
    smoothing_factor = 0.5  # Adjust this value to control the decay rate
    
    nr_smooths = len(joint_history)
    smooth_weights = [smoothing_factor**i for i in range(nr_smooths)]
    smooth_weights = list(reversed(smooth_weights))  # Reverse the order of the list
    smooth_weights = np.array(smooth_weights) / np.sum(smooth_weights)
    smoothed_joint_angles = np.average(joint_history, axis=0, weights=smooth_weights)

    return smoothed_joint_angles

# Function to map range
def map_range(x, X_min, X_max, Y_min, Y_max):
    """
    Linearly maps a value x from one range [X_min, X_max] to another range [Y_min, Y_max].
    """
    X_range = X_max - X_min
    Y_range = Y_max - Y_min
    XY_ratio = Y_range / X_range
    y = (x - X_min) * XY_ratio + Y_min
    return y


def map_to_real_hand(mujoco_angles: np.array, mujoco_range: np.array) -> np.array:
    """Takes angles + ranges from the mujocco file and converts it to angles for the real hand"""
    mapping = [7, 8, 9, 10, 0, 1, 2, 3, 4, 5, 6] # change the order of the angles to match the real hand
    old_max_range = mujoco_range[:,1]#np.array([60, 45, 60, 45, 120, 60, 45 ,90, 90, 45, 45])
    old_min_range = mujoco_range[:,0]#np.array([0, 0, 0, 0, 0, 0, 0, -30, -90, 0, 0])

    ic(mujoco_range)

    mujoco_angles = np.rad2deg(mujoco_angles)
    
    angles_reordered = mujoco_angles[mapping]
    min_range_reordered = old_min_range[mapping]
    max_range_reordered = old_max_range[mapping]

    # Defining the total arrays for minimum and maximum joint angles
    total_min_joint_angles = np.array([110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    total_max_joint_angles = np.array([0, 180, 90, 90, 100, 90, 100, 90, 90, 110, 90])


    new_angles = np.zeros_like(angles_reordered)
    for i, angle in enumerate(angles_reordered):
        new_angles[i] = map_range(angle, min_range_reordered[i], max_range_reordered[i], total_min_joint_angles[i], total_max_joint_angles[i])
    return new_angles


def main():
    # ROS Node initialization
    rospy.init_node('policy_rollout_node', anonymous=True)
    pub = rospy.Publisher('/faive/policy_output', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(20)  # 20 Hz

    # Change the working directory to the script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    
    # skip steps
    slowdown_factor = 1 # skip rolout steps per timestep

    # Load Mujoco model and data
    m = mujoco.MjModel.from_xml_path('mujoco_policy/assets/hand_v3.xml')
    d = mujoco.MjData(m)
    
    ctrl_range = np.zeros((len(d.ctrl), 2))
    for i in range(len(d.ctrl)):
        min_ctrl, max_ctrl = m.actuator_ctrlrange[i]
        min_ctrl_deg, max_ctrl_deg = np.rad2deg(min_ctrl), np.rad2deg(max_ctrl)
        ctrl_range[i] = [min_ctrl_deg, max_ctrl_deg]
        # print(f"{i} range: {min_ctrl_deg, max_ctrl_deg }")
    ctrl_range = np.array(ctrl_range)
    print(np.round(ctrl_range, 2))



    # Load policy
    # file_path_dof = 'policy_recordings/rot_+z_2023-12-12_13-09-16_dof_poses.npy'
    # file_path_dof = 'policy_recordings/rot_+x_2023-12-13_07-19-20_dof_poses.npy'
    # file_path_dof = 'policy_recordings/rot_-z_2023-12-13_12-51-58_dof_poses.npy'
    file_path_dof = 'policy_recordings/rot_sphere_05_axis_-001_0deg_skin_2023-12-14_09-06-04_dof_poses.npy'

    data = np.load(file_path_dof)
    env_nr = 1
    data_env = data[env_nr]

    # Dictionary mapping
    motornr_to_joint_dict = {
        "pointer0": 0,
        "pointer1": 1,
        "middle0": 2,
        "middle1": 3,
        "pinky0": 4,
        "pinky1": 5,
        "pink2": 6,
        "thumb0": 7,
        "thumb1": 8,
        "thumb2": 9,
        "thumb3": 10,
    }
    
    mujoco_angles = np.zeros(len(d.ctrl))

    
    joint_range = []
    for joint_idx in range(len(d.ctrl)):
        joint_range.append(m.actuator_ctrlrange[joint_idx])


    # Mujoco Viewer
    with mujoco.viewer.launch_passive(m, d) as viewer:
        start = time.time()
        step = 0
        while not rospy.is_shutdown() and viewer.is_running() and time.time() - start < 300:
            step_start = time.time()
            
            # Open loop control
            joint_anlges_normalized = data_env[step//slowdown_factor + 100]

            for joint_idx in range(len(d.ctrl)):
                min_ctrl, max_ctrl = m.actuator_ctrlrange[joint_idx]
                
                # modify the offset and multiplier for each joint
                multiplier = 1
                offset = 0
                
                if joint_idx == motornr_to_joint_dict["thumb2"]:
                    multiplier = 1
                    offset = np.deg2rad(6)
                    
                elif joint_idx == motornr_to_joint_dict["thumb0"]:
                    offset = np.deg2rad(0)
                    # pass
                    
                elif joint_idx == motornr_to_joint_dict["pinky1"]:
                    multiplier = 1
                    offset = np.deg2rad(1)
                    
                elif joint_idx == motornr_to_joint_dict["middle0"]:
                    offset = np.deg2rad(0)
                
                # map the normalized joint angles to the mujoco range
                mujoco_angle = map_range(multiplier * joint_anlges_normalized[joint_idx], -1, 1, min_ctrl, max_ctrl)
                mujoco_angle += offset
                d.ctrl[joint_idx] = mujoco_angle
                mujoco_angles[joint_idx] = mujoco_angle
                
                
            
            # Step the Mujoco model
            mujoco.mj_step(m, d)

            # Viewer synchronization
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
            viewer.sync()
            
            
            # map mujoco anlges to real hand angles
            # mujoco_angles*=0
            real_hand_angles = map_to_real_hand(mujoco_angles, ctrl_range)
            smoothed_angles = smooth_joint_history(real_hand_angles)

            ic(smoothed_angles)
            
            smoothed_angles_debug = 0 * smoothed_angles
            i = step//50
            # smoothed_angles_debug[0] = (np.sin(time.time())+1)/2 * 60
            
            smoothed_angles_debug = smoothed_angles
            # ic(smoothed_angles_debug)
            # Publish joint angles
            joint_angles_msg = Float32MultiArray()
            joint_angles_msg.data = list(smoothed_angles_debug)
            pub.publish(joint_angles_msg)

            # Maintain loop rate (mujoco only)
            # time_until_next_step = m.opt.timestep - (time.time() - step_start)
            # if time_until_next_step > 0:
            #     time.sleep(time_until_next_step)
            
            # TODO mapping from mujoco angles to robot 
            
            step += 1
            rate.sleep()
            
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
