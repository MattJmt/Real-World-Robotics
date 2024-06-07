#!/usr/bin/env python3
import time
import numpy as np
import torch
from torch.nn.functional import normalize
import os
import pytorch_kinematics as pk
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from typing import List
import tkinter as tk
from tkinter import Scale
import threading

from utils import retarget_utils, gripper_utils
from collections import deque
from icecream import ic

class RetargeterNode:
    def __init__(
        self,
        device: str = "cuda",
        lr: float = 2.5,
        hardcoded_keyvector_scaling: bool = True,
        use_scalar_distance_palm: bool = True,
    ) -> None:
        '''
        RetargeterNode
        Requires urdf file of hand (change urdf_path and urdf_filename)
        retarget_utils and grippear_utils contain functions and hardcoded values for the faive hand, will need to be changed for other hands
        '''
        self.target_angles = None

        self.device = device
        
        self.base_path = os.path.dirname(os.path.realpath(__file__))

        self.joint_map = torch.zeros(25, 11).to(device)

        joint_parameter_names = retarget_utils.JOINT_PARAMETER_NAMES
        gc_tendons = retarget_utils.GC_TENDONS

        
        for i, (name, tendons) in enumerate(gc_tendons.items()):
            self.joint_map[joint_parameter_names.index(name), i] = 1 if len(tendons) == 0 else 0.5
            for tendon, weight in tendons.items():
                self.joint_map[joint_parameter_names.index(
                    tendon), i] = weight * 0.5
        self.urdf_path = self.base_path + "/mujoco/" #"/urdf/"
        self.urdf_filename = self.urdf_path + "hand_v3.xml"#"hand_v3.urdf"
        

        prev_cwd = os.getcwd()
        os.chdir(self.urdf_path)
        # self.chain = pk.build_chain_from_urdf(
        #     open(self.urdf_filename).read()).to(device=self.device)
        self.chain = pk.build_chain_from_mjcf(
            open(self.urdf_filename).read()).to(device=self.device)
        os.chdir(prev_cwd)

        self.gc_joints = torch.ones(11).to(self.device) * 30.0
        self.gc_joints.requires_grad_()

        self.lr = lr
        self.opt = torch.optim.RMSprop([self.gc_joints], lr=self.lr)

        self.root = torch.zeros(1, 3).to(self.device)
        self.palm_offset = torch.tensor([0.0, 0.09, 0.0]).to(self.device)

        
        self.scaling_coeffs = torch.tensor([0.7171, 1.0081, 0.9031, 0.7086, 0.4387, 0.3660, 0.3966, 0.3981, 0.4923,
                                            0.7554, 0.8932, 1.1388, 1.1884, 1.3794, 1.5170]).to(self.device)
        
        self.scaling_factors_set = hardcoded_keyvector_scaling
        
        self.loss_coeffs = torch.tensor([5.0, 5.0, 5.0, 5.0, 5.0, 1.0, 1.0, 1.0, 1.0,
                                            1.0, 1.0, 1.0, 1.0, 1.0, 1.0]).to(self.device)

        if use_scalar_distance_palm:
            self.use_scalar_distance = [False, True, True, True, True, False, False, False, False, False, False, False, False, False, False]
        else:
            self.use_scalar_distance = [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False]



        self.sub = rospy.Subscriber(
            '/ingress/mano', Float32MultiArray, self.callback, queue_size=1, buff_size=2**24)
        self.pub = rospy.Publisher(
            '/faive/policy_output', Float32MultiArray, queue_size=10)
        
        self.pub_tracked_angles = rospy.Publisher(
            '/tracked_angles', Float32MultiArray, queue_size=10)
        
        self.joint_history = deque(maxlen=25)

    def map_range(self, x, in_min, in_max, out_min, out_max):
        # maps x from range [in_min, in_max] to range [out_min, out_max]
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    

    def retarget_finger_mano_joints(self, joints: np.array, warm: bool = True, opt_steps: int = 2, dynamic_keyvector_scaling: bool = False):
        """
        Process the MANO joints and update the finger joint angles
        joints: (21, 3)
        Over the 21 dims:
        0-4: thumb (from hand base)
        5-8: index
        9-12: middle
        13-16: ring
        17-20: pinky
        """

        if not warm:
            self.gc_joints = torch.ones(11).to(self.device) * 30.0
            self.gc_joints.requires_grad_()

        assert joints.shape == (
            21, 3), "The shape of the mano joints array should be (21, 3)"

        joint_positions = joints
        fingers = {
            "thumb": joint_positions[0:5],  # Thumb already starts from the root
            "index": np.concatenate([joint_positions[0:1], joint_positions[5:9]]),  # Add root to index finger
            "middle": np.concatenate([joint_positions[0:1], joint_positions[9:13]]),  # Add root to middle finger
            "ring": np.concatenate([joint_positions[0:1], joint_positions[13:17]]),  # Add root to ring finger
            "pinky": np.concatenate([joint_positions[0:1], joint_positions[17:21]])  # Add root to pinky finger
        }
        finger_joint_angles = np.zeros(11)
                # pointer 1-2, middle 1-2, pinky 1
        pointers_angles = []
        pinky_anlges = []
        middle_angles = []
        thumb_angles = []
        for i in range(1,4):
            pointers_angles.append(self.angle_between_vectors(fingers["index"][i] - fingers["index"][i-1], fingers["index"][i+1] - fingers["index"][i]))
            pinky_anlges.append(self.angle_between_vectors(fingers["pinky"][i] - fingers["pinky"][i-1], fingers["pinky"][i+1] - fingers["pinky"][i]))
            middle_angles.append(self.angle_between_vectors(fingers["middle"][i] - fingers["middle"][i-1], fingers["middle"][i+1] - fingers["middle"][i]))
            
        thumb_angles.append(self.angle_between_vectors(fingers["index"][1] - fingers["index"][0], fingers["thumb"][2] - fingers["thumb"][0]))
        thumb_angles.append(self.angle_between_vectors(fingers["thumb"][2]-fingers["thumb"][1], fingers["thumb"][3] - fingers["thumb"][2]))
        thumb_angles.append(self.angle_between_vectors(fingers["thumb"][2] - fingers["thumb"][3], fingers["thumb"][3] - fingers["thumb"][4]))
        
        # pinky abduction
        pinky_anlges.append(self.angle_between_vectors(fingers["pinky"][-1] - fingers["pinky"][1], fingers["middle"][-1] - fingers["pinky"][1]))

        # robot_map: thumb 0-3, pointer 4-5, middle 6-7, pinky 8-10
        
        # mujoco_map: pointer 0-1, middle 2-3, pinky 4-6, thumb 7-10
        
        dist_thumb_index = np.linalg.norm(fingers["index"][4] - fingers["thumb"][4])
        dist_thumb_middle = np.linalg.norm(fingers["middle"][4] - fingers["thumb"][4])
        dist_thumb_pinky = np.linalg.norm(fingers["pinky"][4] - fingers["thumb"][4])
        
        min_dist = min(dist_thumb_index, dist_thumb_middle, dist_thumb_pinky)
        thumb_pos = 30
        
        if min_dist == dist_thumb_index:
            thumb_pos = -20
        elif min_dist == dist_thumb_middle:
            thumb_pos = 0
        else:
            thumb_pos = 40
        
        max_angle_tracked = np.array([60.56199722, 67.54357833, 65.30999293, 64.62195034,  30, 46.57657829, 74.862677, 30, 40, 40, 72])
        min_angle_tracked = np.array([13.07649493, 23.4057362 ,  6.84405316, 28.2452319 ,  5, 3.5920378, 24.24494585, 30, 28, 30 * 0.6, 27 * 0.6])
        max_angle_robot = np.array([60, 45, 60, 45, 120, 60, 45 ,90, 90, 45, 45])
        min_angle_robot = np.array([0, 0, 0, 0, 0, 0, 0, -30, -90, 0, 0])
        
        #pointer finger
        finger_joint_angles[0] = self.map_range(pointers_angles[0], min_angle_tracked[0], max_angle_tracked[0], min_angle_robot[0], max_angle_robot[0])
        finger_joint_angles[1] = self.map_range((pointers_angles[1] + pointers_angles[2])/2, min_angle_tracked[1], max_angle_tracked[1], min_angle_robot[1], max_angle_robot[1])
        
        #middle finger
        finger_joint_angles[2] = self.map_range(middle_angles[0], min_angle_tracked[2], max_angle_tracked[2], min_angle_robot[2], max_angle_robot[2])
        finger_joint_angles[3] = self.map_range((middle_angles[1] + middle_angles[2])/2, min_angle_tracked[3], max_angle_tracked[3], min_angle_robot[3], max_angle_robot[3])
        
        # pinky abduction
        # finger_joint_angles[4] = self.map_range(pinky_anlges[-1], min_angle_tracked[4], max_angle_tracked[4], min_angle_robot[4], max_angle_robot[4])
        finger_joint_angles[4] = pinky_anlges[-1]
        finger_joint_angles[5] = self.map_range(pinky_anlges[0], min_angle_tracked[5], max_angle_tracked[5], min_angle_robot[5], max_angle_robot[5])
        finger_joint_angles[6] = self.map_range((pinky_anlges[1] + pinky_anlges[2])/2, min_angle_tracked[6], max_angle_tracked[6], min_angle_robot[6], max_angle_robot[6])

        #thumb
        finger_joint_angles[7] = thumb_pos
        finger_joint_angles[8] = 0 #self.map_range(thumb_angles[0], min_angle_tracked[8], max_angle_tracked[8], 60, -30)
        
        finger_joint_angles[9] = self.map_range(-thumb_angles[1], -min_angle_tracked[9], -max_angle_tracked[9], min_angle_robot[9], max_angle_robot[9])
        finger_joint_angles[10] = self.map_range(thumb_angles[2], min_angle_tracked[10], max_angle_tracked[10], min_angle_robot[10], max_angle_robot[10])

        

        # finger_joint_angles = self.gc_joints.detach().cpu().numpy()

        # print(f'Retarget time: {(time.time() - start_time) * 1000} ms')
        
        
        msg = Float32MultiArray()
        msg.data = thumb_angles
        # ic(thumb_angles, len(thumb_angles))
        self.pub_tracked_angles.publish(msg)
        
        
        #return self.smooth_joint_history(finger_joint_angles)
        return self.map_to_real_hand(self.smooth_joint_history(finger_joint_angles), min_angle_robot, max_angle_robot)
        # return finger_joint_angles
        
    def map_to_real_hand(self, angles: np.array, old_min_range: np.array, old_max_range: np.array) -> np.array:
        """Takes angles + ranges from the mujocco file and converts it to angles for the real hand"""

        mapping = [7, 8, 9, 10, 0, 1, 2, 3, 4, 5, 6]

        angles_reordered = angles[mapping]
        min_range_reordered = old_min_range[mapping]
        max_range_reordered = old_max_range[mapping]

        # Defining the total arrays for minimum and maximum joint angles
        total_min_joint_angles = np.array([110, 0, 0, 0, 0, 0, 0, 0, 90, 0, 0])
        total_max_joint_angles = np.array([0, 180, 90, 90, 100, 90, 100, 90, 0, 110, 90])


        new_angles = np.zeros_like(angles_reordered)
        for i, angle in enumerate(angles_reordered):
            new_angles[i] = self.map_range(angle, min_range_reordered[i], max_range_reordered[i], total_min_joint_angles[i], total_max_joint_angles[i])
        return new_angles

    
    def smooth_joint_history(self, current_joint_angles):
        """Smooths the joint history and returns the smoothed joint angles.
        """
        self.joint_history.append(current_joint_angles)
        
        smoothing_factor = 0.5  # Adjust this value to control the decay rate
        
        nr_smooths = len(self.joint_history)
        smooth_weights = [smoothing_factor**i for i in range(nr_smooths)]
        smooth_weights = list(reversed(smooth_weights))  # Reverse the order of the list
        smooth_weights = np.array(smooth_weights) / np.sum(smooth_weights)
        smoothed_joint_angles = np.average(self.joint_history, axis=0, weights=smooth_weights)

        return smoothed_joint_angles
        
    
    def angle_between_vectors(self, v1, v2):
        # Calculate the dot product of the two vectors
        dot_product = np.dot(v1, v2)

        # Calculate the norms (magnitudes) of the vectors
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)

        # Calculate the cosine of the angle
        cos_theta = dot_product / (norm_v1 * norm_v2 + 1e-6)

        # Calculate the angle in radians
        angle = np.arccos(cos_theta)

        # Convert the angle to degrees, if desired
        angle_degrees = np.rad2deg(angle)

        return angle_degrees
    

    def callback(self, msg):
        # Convert the flattened data back to a 2D numpy array
        joints = np.array(msg.data, dtype=np.float32).reshape(
            msg.layout.dim[0].size, msg.layout.dim[1].size)
        
        self.target_angles = self.retarget_finger_mano_joints(joints)

        time = rospy.Time.now()
        assert self.target_angles.shape == (
            11,), "Expected different output format from retargeter"

        msg = Float32MultiArray

        # Create a Float32MultiArray message and set its 'data' field to the flattened array
        arr = self.target_angles
        msg = Float32MultiArray()
        msg.data = arr.flatten().tolist()

        # Set the 'layout' field of the message to describe the shape of the original array
        rows_dim = MultiArrayDimension()
        rows_dim.label = 'rows'
        rows_dim.size = arr.shape[0]
        rows_dim.stride = 1

        cols_dim = MultiArrayDimension()
        cols_dim.label = 'cols'
        cols_dim.size = 1
        cols_dim.stride = 1

        msg.layout.dim = [rows_dim, cols_dim]

        msg.data = self.target_angles
        
        # self.target_angles*=0
        # self.target_angles+=90*np.sin(time.to_sec()/8)**2
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('mano_faive_retargeter', anonymous=True)
    retargeter = RetargeterNode(device="cpu")

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        # ... [rest of your main loop] ...
        r.sleep()