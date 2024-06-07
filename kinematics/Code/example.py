from gripper_controller import GripperController
import time


"""
Example script to control the finger joint angles
"""

homepos = [45, 45, 45, 45, 0, 45, 45, 45, 45, 45, 45]
goalpos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


def main():
    global gc
    gc = GripperController(port="/dev/tty.usbserial-FT89FE3P",calibration=False)
    
    time.sleep(5)
    
    print("Starting gripper controller")
    
    mask = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
    gc.write_desired_joint_angles(homepos, mask=mask, calculations=True)
    
    print("Waiting for motion to finish")
    
    while True:
        pass
    time.sleep(3)
    gc.wait_for_motion()
    
    #while True:
    #    pass
    
    print("Motion finished")
    
    gc.write_desired_joint_angles(goalpos, calculations=True)

    gc.wait_for_motion()
    
    print("Motion 2 finished")

    time.sleep(3)

    gc.terminate()


if __name__ == "__main__":
    main()