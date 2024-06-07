from re import L
#from .dynamixel_client import *
from dynamixel_client import *
import numpy as np
import time
import yaml
import os
#from . 
import finger_kinematics as fk
from threading import RLock
import keyboard

from typing import List, Tuple


class MuscleGroup:
    """
    An isolated muscle group comprised of joints and tendons, which do not affect the joints and tendons not included in the group.
    """
    attributes = ["joint_ids", "motor_ids", "spool_dirs", "min_joint_angles", "max_joint_angles", "min_currents", "max_currents", "calibration_dirs", "initial_poses"] #tendon_ids, motor_map, spool_rad
    def __init__(self, name, muscle_group_json: dict):
        self.name = name
        for attr_name in MuscleGroup.attributes:
            setattr(self, attr_name, muscle_group_json[attr_name])
        print(f"Created muscle group {name} with joint ids {self.joint_ids}, motor ids {self.motor_ids}, max_current {self.max_currents} and spool direction {self.spool_dirs}")

class GripperController:
    """
    class specialized for the VGripper
    wraps DynamixelClient to make it easier to access hand-related functions, letting the user think with "tendons" and "joints" instead of "motors"
    
    ## about tendon direction
    Signs for the tendon length is modified before sending to the robot so for the user, it is always [positive] = [actual tendon length increases]
    The direction of each tendon is set by the sign of the `spool_rad` variable in each muscle group
    """
    def __init__(self, port: str = '/dev/ttyUSB0', config_yml: str = "gripper_defs.yaml", calibration: bool = False, maxCurrent: int = 150):
        """
        config_yml: path to the config file, relative to this source file
        """
        baudrate = 3000000

        self.motor_lock = RLock() # lock to read / write motor information

        self._load_musclegroup_yaml(os.path.join(os.path.dirname(os.path.abspath(__file__)), config_yml))

        self.command_lock = RLock() # lock to receive and read angle commands
        self._cmd_joint_angles = np.zeros(self.joint_nr)

        # initialize and connect dynamixels
        self._dxc = DynamixelClient(self.motor_ids, port, baudrate)
        self.connect_to_dynamixels()

        # initialize the joint
        self.init_joints(calibrate=calibration, maxCurrent=maxCurrent)

    def terminate(self):
        '''
        disable torque and disconnect from dynamixels
        '''
        self.disable_torque()
        time.sleep(0.1) # wait for disabling torque
        self.disconnect_from_dynamixels()
        

    def _load_musclegroup_yaml(self, filename):
        """
        load muscle group definitions from a yaml file
        Assumed to only run once, i.e. muscle groups are not changed during runtime
        """
        with open(filename, 'r') as f:
            print(f"reading muscle group definitions from {filename} ...")
            data = yaml.load(f, Loader=yaml.FullLoader)

        self.muscle_groups = []
        for muscle_group_name, muscle_group_data in data['muscle_groups'].items():
            self.muscle_groups.append(MuscleGroup(muscle_group_name, muscle_group_data))
        
        # define some useful variables to make it easier to access tendon information
        attrs_to_get = ["joint_ids", "motor_ids", "spool_dirs", "min_joint_angles", "max_joint_angles", "min_currents", "max_currents", "calibration_dirs", "initial_poses"] #"tendon_ids", "spool_rad"
        for attr in attrs_to_get:
            setattr(self, attr, [])
            for muscle_group in self.muscle_groups:
                getattr(self, attr).extend(getattr(muscle_group, attr))
        for attr in attrs_to_get:
            setattr(self, attr, np.array(getattr(self, attr)))

        self.joint_nr = 0
        # run some sanity checks
        for muscle_group in self.muscle_groups:
            self.joint_nr += len(muscle_group.joint_ids)
            assert len(muscle_group.joint_ids) == len(muscle_group.motor_ids), "joint_ids and motor_ids must have the same length"
            assert len(muscle_group.spool_dirs) == len(muscle_group.motor_ids), "spool_dir must be defined for all motors"
            assert len(muscle_group.max_joint_angles) == len(muscle_group.joint_ids), "max_joint_angle must be defined for all joints"
            assert len(muscle_group.min_joint_angles) == len(muscle_group.joint_ids), "min_joint_angle must be defined for all joints"
            assert len(muscle_group.max_currents) == len(muscle_group.motor_ids), "max_current must be defined for all motors"
            assert len(muscle_group.min_currents) == len(muscle_group.motor_ids), "min_current must be defined for all motors"
            assert len(muscle_group.calibration_dirs) == len(muscle_group.motor_ids), "calibration_dir must be defined for all motors"
            assert len(muscle_group.initial_poses) == len(muscle_group.motor_ids), "initial_pos must be defined for all motors"
        assert len(self.motor_ids) == len(set(self.motor_ids)), "duplicate motor ids should not exist"
    '''
    def tendon_pos2motor_pos(self, tendon_lengths):
        """ Input: desired tendon lengths
        Output: desired motor positions """
        motor_pos = np.zeros(len(self.motor_ids))
        m_idx = 0
        t_idx = 0
        for muscle_group in self.muscle_groups:
            m_nr = len(muscle_group.motor_ids)
            t_nr = len(muscle_group.tendon_ids)
            for m_i in range(m_nr):
                m_id = muscle_group.motor_ids[m_i]
                t_i = muscle_group.motor_map.index(m_id)
                motor_pos[m_idx + m_i] = tendon_lengths[t_idx+t_i]/muscle_group.spool_rad[t_i]
            m_idx += m_nr
            t_idx += t_nr
        return motor_pos
    '''

    def motor_pos2tendon_pos(self, motor_pos):
        """ Input: motor positions
        Output: tendon lengths """     
        tendon_lengths = np.zeros(len(self.tendon_ids))
        m_idx = 0
        t_idx = 0
        for muscle_group in self.muscle_groups:
            m_nr = len(muscle_group.motor_ids)
            t_nr = len(muscle_group.tendon_ids)
            for m_i in range(m_nr):
                m_id = muscle_group.motor_ids[m_i]
                t_i = np.where(np.array(muscle_group.motor_map) == m_id)[0]
                for i in t_i:
                    tendon_lengths[t_idx+i] = motor_pos[m_idx+m_i]*muscle_group.spool_rad[i]
            m_idx += m_nr
            t_idx += t_nr
        return tendon_lengths

    def write_desired_motor_pos(self, motor_positions_rad):
        """
        send position command to the motors
        unit is rad, angle of the motor connected to tendon
        """
        with self.motor_lock:
            self._dxc.write_desired_pos(self.motor_ids, motor_positions_rad)


    def write_desired_motor_current(self, motor_currents_mA):
        """
        send current command to the motors
        unit is mA (positive = pull the tendon)
        """
        m_nr = len(motor_currents_mA)
        m_idx = 0
        directions = np.zeros(m_nr)
        for muscle_group in self.muscle_groups:
            for m_id in muscle_group.motor_ids:
                idx = muscle_group.motor_ids.index(m_id)
                directions[m_idx] = np.sign(muscle_group.spool_dirs[idx])
                m_idx += 1
        with self.motor_lock:
            self._dxc.write_desired_current(self.motor_ids, - motor_currents_mA * directions)
    
    def connect_to_dynamixels(self):
        with self.motor_lock:
            self._dxc.connect()

    def disconnect_from_dynamixels(self):
        with self.motor_lock:
            self._dxc.disconnect()
    
    def set_operating_mode(self, mode):
        """
        see dynamixel_client.py for the meaning of the mode
        """
        with self.motor_lock:
            self._dxc.set_operating_mode(self.motor_ids, mode)

    def get_motor_pos(self):
        with self.motor_lock:
            return self._dxc.read_pos_vel_cur()[0]
        
    def get_curr_joint_angles(self) -> List[float]:
        with self.motor_lock:
            motor_pos = self.get_motor_pos()
            return np.array([self.map_value(motor_pos[i], self.motor_id2min_pos[i], self.motor_id2max_pos[i], self.min_joint_angles[i], self.max_joint_angles[i]) for i in range(len(motor_pos))])

    def get_motor_cur(self):
        with self.motor_lock:
            return self._dxc.read_pos_vel_cur()[2]

    def get_motor_vel(self):
        with self.motor_lock:
            return self._dxc.read_pos_vel_cur()[1]

    def wait_for_motion(self):
        while not all(self._dxc.read_status_is_done_moving()) and not np.allclose(self.get_motor_vel(), 0, atol=0.1):
            time.sleep(0.01)

    def enable_torque(self, motor_ids=None):
        if motor_ids is None:
            motor_ids = self.motor_ids
        with self.motor_lock:
            self._dxc.set_torque_enabled(motor_ids, True)        

    def disable_torque(self, motor_ids=None):
        if motor_ids is None:
            motor_ids = self.motor_ids
        with self.motor_lock:
            self._dxc.set_torque_enabled(motor_ids, False)

    def pose2motors(self, joint_angles):
        """ Input: joint angles in rad
        Output: motor positions 
        TODO: Extend the calculation of the tendon lengths for every finger. Tip: A clever design can allow for the same formulas for each finger to reduce complexity.
        """
        motor_pos = np.zeros(len(self.motor_ids))
        bounded_joint_angles = self.bound_joint_angles(joint_angles)
        for muscle_group in self.muscle_groups:
            finger_id = muscle_group.name[-1]
            joint_ids = muscle_group.joint_ids
            motor_pos[joint_ids] = np.array(fk.joint_angles2motor_pos(finger_id, bounded_joint_angles[joint_ids])) * np.array(muscle_group.spool_dirs)

        return motor_pos
    
    def bound_joint_angles(self, joint_angles):
        """
        bound joint angles to the range of [min_joint_angle, max_joint_angle]
        """
        return np.clip(joint_angles, self.min_joint_angles, self.max_joint_angles)

    def init_joints(self, calibrate: bool = False, maxCurrent: int = 150):
        """
        Set the offsets based on the current (initial) motor positions
        :param calibrate: if True, perform calibration and set the offsets else move to the initial position
        TODO: Think of a clever way to perform the calibration. How can you make sure that all the motor are in the corect position?
        """
        
        self.motor_id2init_pos = np.zeros(len(self.motor_ids))
        self.motor_id2min_pos = np.zeros(len(self.motor_ids))
        self.motor_id2max_pos = np.zeros(len(self.motor_ids))

        cal_yaml_fname = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cal.yaml")
        cal_exists = os.path.isfile(cal_yaml_fname)
        
        # Load the calibration file
        with open(cal_yaml_fname, 'r') as cal_file:
            cal_data = yaml.load(cal_file, Loader=yaml.FullLoader)
        
        self.motor_id2init_pos = np.array(cal_data["motor_init_pos"])
        self.motor_id2min_pos = np.array(cal_data["motor_min_pos"])
        self.motor_id2max_pos = np.array(cal_data["motor_max_pos"])

        #self.motor_id2init_pos = self.motor_id2min_pos + self.initial_poses * (self.motor_id2max_pos - self.motor_id2min_pos)
        
        if not calibrate and cal_exists:
            
            #init_pos = self.get_attributes(init_pos=True)[0]
            

            # Set to current based position control mode
            self.set_operating_mode(5)
            self.write_desired_motor_current(maxCurrent * np.ones(len(self.motor_ids)))
            #self.write_desired_motor_pos(self.motor_id2init_pos)
            time.sleep(0.01)
            self.wait_for_motion()

        else: # This will overwrite the current config file with the new offsets and we will lose all comments in the file
            
            #self.given_calibration(maxCurrent)
            self.total_calibration(use_keyboard=True) #, only_calibrate_fingers=[7])
            #self.initial_calibration()
            
            # Save the offsets to a YAML file
            with open(cal_yaml_fname, 'r') as cal_file:
                cal_orig = yaml.load(cal_file, Loader=yaml.FullLoader)

            cal_orig['motor_init_pos'] = self.motor_id2init_pos.tolist()
            cal_orig['motor_min_pos'] = self.motor_id2min_pos.tolist()
            cal_orig['motor_max_pos'] = self.motor_id2max_pos.tolist()

            with open(cal_yaml_fname, 'w') as cal_file:
                yaml.dump(cal_orig, cal_file, default_flow_style=False)
                
            print(f"Motor positions after calibration (0-10): {self.motor_id2init_pos}")
            # Set to current based position control mode
            self.set_operating_mode(5)
            self.write_desired_motor_current(maxCurrent * np.ones(len(self.motor_ids)))
            time.sleep(0.2)

        self.motor_pos_norm = self.pose2motors(np.zeros(len(self.joint_ids)))
        
    def map_value(self, value, x, y, a, b):
        """
        Map a value from range [x, y] to range [a, b].
        """
        return a + (value - x) * (b - a) / (y - x)


    def write_desired_joint_angles(self, joint_angles: np.array, mask = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], calculations: bool = False, protection=False):
        """
        Command joint angles in deg
        :param: joint_angles: [joint 1 angle, joint 2 angle, ...]
        """
        if calculations:
            motor_pos_des = self.pose2motors(np.deg2rad(joint_angles)) - self.motor_pos_norm + self.motor_id2init_pos
        else:
            motor_pos_des = np.array([self.map_value(joint_angle, min_angle, max_angle, min_motor, max_motor) 
                             for joint_angle, min_angle, max_angle, min_motor, max_motor in zip(joint_angles, self.min_joint_angles, self.max_joint_angles, self.motor_id2min_pos, self.motor_id2max_pos)])
        
        if mask is not None:
            motor_pos_des = np.array([motor_pos_des[i] if mask[i] else self.get_motor_pos()[i] for i in range(len(motor_pos_des))])
            
        #if protection:
        #    cur_current = self.get_motor_cur()
        #    motor_pos_des = np.array([motor_pos_des[i] if np.abs(cur_current[i]) < self.max_currents[i] else self.get_motor_pos()[i] for i in range(len(motor_pos_des))])
        self.write_desired_motor_pos(motor_pos_des)
        time.sleep(0.01) # wait for the command to be sent

#################### Calibration ####################

    def given_calibration(self, filename, maxCurrent: int = 150):
        
        # Disable torque to allow the motors to move freely
        self.disable_torque()
        input("Move fingers to init position and press Enter to continue...")

        self.motor_id2init_pos = self.get_motor_pos()
        
    def initial_calibration(self, direction = 1, only_calibrate_fingers : List = None):
        """
        Calibrate the gripper to the initial position (either max or min)
        direction: 1 for max, -1 for min
        """
        # Force motors until spike in current
        
        if direction not in (1, -1):
            raise ValueError("direction must be either 1 or -1")
        
        self.set_operating_mode(5)

        next_motor_pos = self.get_motor_pos()
        calibrated = False * np.ones(len(self.motor_ids))
        
        initial_positions = self.get_motor_pos()
        min_positions = np.zeros(len(self.motor_ids))
        
        for i in range(11):
            if only_calibrate_fingers is not None and i not in only_calibrate_fingers:
                continue
            old_pos, next_motor_pos = initial_positions, initial_positions
            while True:
                if np.abs(self.get_motor_cur()[i]) > self.max_currents[i]:
                    #If the current is too high, we have reached the max position so we save that value
                    min_positions[i] = old_pos[i]
                    initial_positions[i] = min_positions[i]
                    break
                else:
                    next_motor_pos[i] = old_pos[i] - 0.1 * self.calibration_dirs[i] * direction
                print(self.get_motor_cur()[i], self.max_currents[i])
        
                self.write_desired_motor_pos(next_motor_pos)
                self.wait_for_motion()
                time.sleep(0.01)
                old_pos = next_motor_pos
                
            self.write_desired_motor_pos(initial_positions)
            self.wait_for_motion()
            time.sleep(0.1)
                
            print(f"Motor {i} calibrated")
            print(min_positions[i])
            
        self.motor_id2init_pos = self.get_motor_pos()
        
        
        
    def total_calibration(self, use_keyboard=False, only_calibrate_fingers : List = None) -> Tuple[List[float], List[float], List[float]]:
        """
        Calibrate the gripper so that it knows max and min joint angles as well as the initial position
        """
        self.set_operating_mode(5)
        
        initial_positions = self.get_motor_pos()
        min_positions, max_positions = np.zeros(len(self.motor_ids)), np.zeros(len(self.motor_ids))
        
        for i in range(11):
            if only_calibrate_fingers is not None and i not in only_calibrate_fingers:
                min_positions[i] = self.motor_id2min_pos[i]
                max_positions[i] = self.motor_id2max_pos[i]
                continue
            old_pos, next_motor_pos = initial_positions, initial_positions
            current_direction = 1
            while True:
                if ((((current_direction == 1 and (np.abs(self.get_motor_cur()[i]) >= self.max_currents[i])) or
                    (current_direction == -1 and (np.abs(self.get_motor_cur()[i]) >= self.min_currents[i]))) and
                    (np.sign(self.get_motor_cur()[i] * self.calibration_dirs[i]) == current_direction)) or
                    (use_keyboard and keyboard.is_pressed('space'))):
                    if current_direction == 1:
                        min_positions[i] = old_pos[i]
                        next_motor_pos = initial_positions
                        current_direction = -1
                        print("got min position")
                    else:
                        max_positions[i] = old_pos[i]
                        initial_positions[i] = min_positions[i] + self.initial_poses[i] * (max_positions[i]-min_positions[i])
                        print("got min and max positions")
                        break
                else:
                    next_motor_pos[i] = old_pos[i] + 0.1 * self.calibration_dirs[i] * current_direction
                if current_direction == 1:
                    print(self.get_motor_cur()[i], self.max_currents[i])
                else:
                    print(self.get_motor_cur()[i], self.min_currents[i])
        
                self.write_desired_motor_pos(next_motor_pos)
                self.wait_for_motion()
                time.sleep(0.001)
                old_pos = next_motor_pos
                
            self.write_desired_motor_pos(initial_positions)
            self.wait_for_motion()
            time.sleep(0.1)
                
            print(f"Motor {i} calibrated")
            print(min_positions[i], max_positions[i])
            
        self.motor_id2init_pos = initial_positions
        self.motor_id2min_pos = min_positions
        self.motor_id2max_pos = max_positions
        
        
#################### Example usage ####################
        

if __name__ == "__main__" :
    gc = GripperController("/dev/ttyUSB0")
    gc.connect_to_dynamixels()

    gc.init_joints(calibrate=True)

    time.sleep(3.0)
