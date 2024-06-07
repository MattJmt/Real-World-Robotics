import sys
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt
import numpy as np
from gripper_controller import GripperController

class GripperGUI(QWidget):
    def __init__(self, gripper_controller):
        super().__init__()
        self.gc = gripper_controller
        self.initUI()

    def initUI(self):
        vbox = QVBoxLayout()

        self.sliders = []
        self.labels = []
        # Assuming muscle_groups is a list of your finger definitions
        for min_angle, max_angle, initial_pose in zip(self.gc.min_joint_angles, self.gc.max_joint_angles, self.gc.initial_poses):
            # Slider label
            label = QLabel(self)
            initial_value = int(initial_pose * (max_angle - min_angle) + min_angle)
            label.setText(f"Min: {min_angle}°, Max: {max_angle}°, Current: {initial_value}°")
            vbox.addWidget(label)
                
             # Slider
            slider = QSlider(Qt.Horizontal, self)
            slider.setMinimum(min_angle)
            slider.setMaximum(max_angle)
            slider.setValue(initial_value)
            slider.valueChanged.connect(lambda value, s=slider, l=label: self.updateGripperAndLabel(value, s, l))
            self.sliders.append(slider)
            self.labels.append(label)
            vbox.addWidget(slider)

        self.setLayout(vbox)
        self.setWindowTitle('Gripper Controller')

    def updateGripperAndLabel(self, value, slider, label):
        joint_angles = np.array([s.value() for s in self.sliders])
        self.gc.write_desired_joint_angles(joint_angles)
        min_angle = slider.minimum()
        max_angle = slider.maximum()
        label.setText(f"Min: {min_angle}°, Max: {max_angle}°, Current: {value}°")

def main():
    gc = GripperController(port="/dev/ttyUSB0", calibration=True)

    app = QApplication(sys.argv)
    ex = GripperGUI(gc)
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
