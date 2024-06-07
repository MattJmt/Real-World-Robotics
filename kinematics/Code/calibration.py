from gripper_controller import GripperController
import time
import numpy as np


"""
Example script to control the finger joint angles
"""


def main():
    global gc
    gc = GripperController(port="/dev/tty.usbserial-FT89FE3P",calibration=True)
    
    time.sleep(5)

    gc.terminate()


if __name__ == "__main__":
    main()