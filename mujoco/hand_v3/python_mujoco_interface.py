# code copied from https://mujoco.readthedocs.io/en/stable/python.html
# implemented a open loop controller which moves all motors 

import time
import mujoco
import mujoco.viewer

import numpy as np
import os
# change path working directory
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

m = mujoco.MjModel.from_xml_path('hand_v3.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()
        
        # Open loop controll:  
        # actuating all motors with a sin wave           
        for joint_idx in range(len(d.ctrl)):
            min_ctrl, max_ctrl = m.actuator_ctrlrange[joint_idx]
            d.ctrl[joint_idx] =  (np.sin(time.time()) /2 +  0.5) * (max_ctrl - min_ctrl) + min_ctrl
            
            
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
