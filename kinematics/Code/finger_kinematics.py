import numpy as np
from typing import List
#from . 
import kinematics_constants as kc

# ------------------- Utility Functions ------------------- #

def rotx(theta):
   '''Input: angle in rad
      Output: rotation matrix around x axis'''
   return np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])

# ------------------- Calculations of Motor Positions ------------------- #

# def motor_pos_joint0(finger_id, theta_joint0):
#    '''Input: joint angle of joint0 in rad
#       Output: motor positions of joint0'''
#    match (finger_id):
#       case "a":
#          return theta_joint0
#       case _:
#          raise ValueError("Invalid finger_id")
      
def motor_pos_joint0(finger_id, theta_joint0):
    '''Input: joint angle of joint0 in rad
       Output: motor positions of joint0'''
    if finger_id == "a":
        return theta_joint0
    else:
        raise ValueError("Invalid finger_id")

      
# def motor_pos_joint1(finger_id, theta_joint1):
#    '''Input: joint angle of joint1 in rad
#       Output: motor positions of joint1'''
#    match (finger_id):
#       case "a":
#          return kc.R_a1/kc.R_spoon*theta_joint1
#       case "d":
#          return kc.R_d1/kc.R_spoon*theta_joint1
#       case _:
#          raise ValueError("Invalid finger_id")

def motor_pos_joint1(finger_id, theta_joint1):
    '''Input: joint angle of joint1 in rad
       Output: motor positions of joint1'''
    if finger_id == "a":
        return kc.R_a1/kc.R_spoon*theta_joint1
    elif finger_id == "d":
        return kc.R_d1/kc.R_spoon*theta_joint1
    else:
        raise ValueError("Invalid finger_id")


# ------------------- Calculations of Tendon Lengths at single joint ------------------- #

# def tendonlength_joint2(finger_id, theta_joint2):
#    '''Input: joint angle of joint2 in rad
#       Output: total normal lengths of flexor tendon through joint2'''
#    match (finger_id):
#       case "a":
#          if theta_joint2 >= 0:
#             return np.linalg.norm(kc.v_a2_c1_T1 + rotx(theta_joint2/2) @ np.array([0, 0, 2*kc.R_a2]) + rotx(theta_joint2) @ kc.v_a2_c2_T2)
#          else:
#             theta_joint2 = -theta_joint2
#             #TODO
#       case ("b"|"c"|"d"):
#          if theta_joint2 >= 0:
#             return np.linalg.norm(kc.v_bcd2_c1_T1 + rotx(theta_joint2/2) @ np.array([0, 0, 2*kc.R_bcd2]) + rotx(theta_joint2) @ kc.v_bcd2_c2_T2)
#          else:
#             theta_joint2 = -theta_joint2
#             #TODO
#       case _:
#          raise ValueError("Invalid finger_id")
      
def tendonlength_joint2(finger_id, theta_joint2):
    '''Input: joint angle of joint2 in rad
       Output: total normal lengths of flexor tendon through joint2'''
    if finger_id == "a":
        if theta_joint2 >= 0:
            return np.linalg.norm(kc.v_a2_c1_T1 + rotx(theta_joint2/2) @ np.array([0, 0, 2*kc.R_a2]) + rotx(theta_joint2) @ kc.v_a2_c2_T2)
        else:
            # Placeholder logic for negative angles for finger "a"
            theta_joint2 = -theta_joint2
            return np.linalg.norm(kc.v_a2_c1_T1 - rotx(theta_joint2/2) @ np.array([0, 0, 2*kc.R_a2]) - rotx(theta_joint2) @ kc.v_a2_c2_T2)

    elif finger_id in ["b", "c", "d"]:
        if theta_joint2 >= 0:
            return np.linalg.norm(kc.v_bcd2_c1_T1 + rotx(theta_joint2/2) @ np.array([0, 0, 2*kc.R_bcd2]) + rotx(theta_joint2) @ kc.v_bcd2_c2_T2)
        else:
            # Placeholder logic for negative angles for fingers "b", "c", "d"
            theta_joint2 = -theta_joint2
            return np.linalg.norm(kc.v_bcd2_c1_T1 - rotx(theta_joint2/2) @ np.array([0, 0, 2*kc.R_bcd2]) - rotx(theta_joint2) @ kc.v_bcd2_c2_T2)

    else:
        raise ValueError("Invalid finger_id")


"""
def tendonlength_extensor_joint2(finger_id, theta_joint2):
   '''Input: joint angle of joint2 in rad
      Output: total normal lengths of extensor tendon through joint2'''
   match (finger_id):
      case "a":
         return np.sqrt(5.7**2 + 4.8**2 - 2*5.7*4.8*np.cos(1.81+theta_joint2))
      case ("b"|"c"|"d"):
         return np.sqrt(5.7**2 + 4.8**2 - 2*5.7*4.8*np.cos(1.81+theta_joint2))
      case _:
         raise ValueError("Invalid finger_id")
"""

# def tendonlength_joint3(finger_id, theta_joint3):
#    '''Input: joint angle of joint3 in rad
#       Output: total normal lengths of flexor tendon through joint3'''
#    match (finger_id):
#       case "a":
#          if theta_joint3 >= 0:
#             return np.linalg.norm(kc.v_a3_c1_T1 + rotx(theta_joint3/2) @ np.array([0, 0, 2*kc.R_a3]) + rotx(theta_joint3) @ kc.v_a3_c2_T2)
#          else:
#             theta_joint3 = -theta_joint3   
#             #TODO
#       case ("b"|"c"|"d"):
#          if theta_joint3 >= 0:
#             return (np.linalg.norm(kc.v_bcd3a_c1_T1 + rotx(theta_joint3/2) @ np.array([0, 0, 2*kc.R_bcd3a]) + rotx(theta_joint3) @ kc.v_bcd3a_c2_T2) +
#                np.linalg.norm(kc.v_bcd3b_c1_T1 + rotx(theta_joint3/2) @ np.array([0, 0, 2*kc.R_bcd3b]) + rotx(theta_joint3) @ kc.v_bcd3b_c2_T2))
#          else:
#             theta_joint3 = -theta_joint3
#             #TODO
#       case _:
#          raise ValueError("Invalid finger_id")
      
      
def tendonlength_joint3(finger_id, theta_joint3):
    '''Input: joint angle of joint3 in rad
       Output: total normal lengths of flexor tendon through joint3'''
    if finger_id == "a":
        if theta_joint3 >= 0:
            return np.linalg.norm(kc.v_a3_c1_T1 + rotx(theta_joint3/2) @ np.array([0, 0, 2*kc.R_a3]) + rotx(theta_joint3) @ kc.v_a3_c2_T2)
        else:
            # Placeholder logic for negative angles for finger "a"
            theta_joint3 = -theta_joint3
            return np.linalg.norm(kc.v_a3_c1_T1 - rotx(theta_joint3/2) @ np.array([0, 0, 2*kc.R_a3]) - rotx(theta_joint3) @ kc.v_a3_c2_T2)

    elif finger_id in ["b", "c", "d"]:
        if theta_joint3 >= 0:
            return (np.linalg.norm(kc.v_bcd3a_c1_T1 + rotx(theta_joint3/2) @ np.array([0, 0, 2*kc.R_bcd3a]) + rotx(theta_joint3) @ kc.v_bcd3a_c2_T2) +
                    np.linalg.norm(kc.v_bcd3b_c1_T1 + rotx(theta_joint3/2) @ np.array([0, 0, 2*kc.R_bcd3b]) + rotx(theta_joint3) @ kc.v_bcd3b_c2_T2))
        else:
            # Placeholder logic for negative angles for fingers "b", "c", "d"
            theta_joint3 = -theta_joint3
            return (np.linalg.norm(kc.v_bcd3a_c1_T1 - rotx(theta_joint3/2) @ np.array([0, 0, 2*kc.R_bcd3a]) - rotx(theta_joint3) @ kc.v_bcd3a_c2_T2) +
                    np.linalg.norm(kc.v_bcd3b_c1_T1 - rotx(theta_joint3/2) @ np.array([0, 0, 2*kc.R_bcd3b]) - rotx(theta_joint3) @ kc.v_bcd3b_c2_T2))

    else:
        raise ValueError("Invalid finger_id")


# ------------------- Calculations of Tendon Lengths for all joints ------------------- #
# TODO: Add your own functions here to calculate the tendon lengths for all joints and for each finger (if needed)

def pose2motor_finger(finger_id, theta_joints: List[float]) -> List[float]:
   if finger_id in ("b", "c"):
      return []
   elif finger_id == "a":
      return [motor_pos_joint0(finger_id, theta_joints[0]), motor_pos_joint1(finger_id, theta_joints[1])]
   elif finger_id == "d":
      return [motor_pos_joint1(finger_id, theta_joints[0])]
   

def pose2tendon_finger(finger_id, theta_Joint2, theta_Joint3):
   '''Input: controllable joint angles (always the same for all fingers, 2nd and 3rd joint)
      Output: array of tendon lengths for given joint angles'''
   return [tendonlength_joint2(finger_id, theta_Joint2),
            tendonlength_joint3(finger_id, theta_Joint3)]
   
def joint_angles2motor_pos(finger_id: int, theta_joints: List[float]):
   '''Input: controllable joint angles
      Output: array of motor positions for given joint angles'''
   return pose2motor_finger(finger_id, theta_joints) + [tendon_length/kc.R_spoon for tendon_length in pose2tendon_finger(finger_id, theta_joints[-2], theta_joints[-1])]