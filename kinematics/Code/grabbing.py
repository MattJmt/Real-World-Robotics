from gripper_controller import GripperController
import time
import numpy as np


"""
Script to grab an object
"""


def main():
    global gc
    gc = GripperController(port="/dev/tty.usbserial-FT89FE3P",calibration=False)
    
    time.sleep(0.1)
    starting_position = gc.get_curr_joint_angles()
    #print(starting_position)
    
    step = [0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1]
    next_position = starting_position
    direction = 1

    while True:
        next_position = next_position + direction * step
        gc.write_desired_joint_angles(next_position)
        gc.wait_for_motion()
        time.sleep(0.1)
        if np.sum(np.abs(gc.get_curr_joint_angles() - next_position)) < 1:
            if direction == 1:
                direction = -1
            else:
                break

    gc.terminate()


if __name__ == "__main__":
    main()