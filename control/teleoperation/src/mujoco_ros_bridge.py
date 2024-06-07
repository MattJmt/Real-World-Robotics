#!/usr/bin/env python3

# code copied from https://mujoco.readthedocs.io/en/stable/python.html
# implemented a open loop controller which moves all motors 

import rospy
from std_msgs.msg import Float32MultiArray  # Adjust the message type based on your setup
import time
import mujoco
import mujoco.viewer
import numpy as np
import os
from icecream import ic

# Global variable to store the joint angles
current_joint_angles = np.zeros(11)

def joint_angles_callback(msg):
    global current_joint_angles
    # Assuming the message is a Float32MultiArray with 11 elements
    current_joint_angles = np.array(msg.data)
    # ic(current_joint_angles, "joint angles in mujoco node")

def main():
    global current_joint_angles

    # ROS Node initialization
    rospy.init_node('mujoco_ros_bridge')

    # Subscriber to the joint angles topic
    rospy.Subscriber('/faive/policy_output', Float32MultiArray, joint_angles_callback)

    # Change to the directory containing the XML file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    # Load the MuJoCo model
    m = mujoco.MjModel.from_xml_path('mujoco/hand_v3.xml')
    d = mujoco.MjData(m)

    with mujoco.viewer.launch_passive(m, d) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
        start = time.time()
        while viewer.is_running() and not rospy.is_shutdown():
            step_start = time.time()
            
            # Open loop controll:  
            # actuating all motors with a sin wave        
            for joint_idx in range(len(d.ctrl)):
                # min_ctrl, max_ctrl = m.actuator_ctrlrange[joint_idx]
                d.ctrl[joint_idx] = np.deg2rad(current_joint_angles[joint_idx])
                
                
            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            mujoco.mj_step(m, d)

            # Example modification of a viewer option: toggle contact points every two seconds.
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == '__main__':
    main()

