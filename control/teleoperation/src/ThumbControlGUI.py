from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt

class JointControlGUI(QWidget):
    def __init__(self, update_function):
        super().__init__()
        self.update_function = update_function
        self.initUI()
        
    def initUI(self):
        layout = QVBoxLayout()

        # Joint 7
        self.joint7_value_label = QLabel('0')
        layout.addLayout(self.create_joint_control("Joint 7", 0, 180, self.update_joint7, self.joint7_value_label))

        # Joint 8
        self.joint8_value_label = QLabel('0')
        layout.addLayout(self.create_joint_control("Joint 8", 0, 180, self.update_joint8, self.joint8_value_label))

        self.setLayout(layout)
        self.setWindowTitle('Joint Control')

    def create_joint_control(self, joint_name, min_value, max_value, update_callback, value_label):
        hbox = QHBoxLayout()

        label = QLabel(joint_name)
        hbox.addWidget(label)

        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(min_value)
        slider.setMaximum(max_value)
        slider.valueChanged.connect(lambda value: self.slider_changed(value, update_callback, value_label))
        hbox.addWidget(slider)

        min_label = QLabel(f'Min: {min_value}')
        hbox.addWidget(min_label)

        value_label.setText(f'Val: {slider.value()}')
        hbox.addWidget(value_label)

        max_label = QLabel(f'Max: {max_value}')
        hbox.addWidget(max_label)

        return hbox

    def slider_changed(self, value, update_callback, value_label):
        value_label.setText(f'Val: {value}')
        update_callback(value)

    def update_joint7(self, value):
        self.update_function(7, value)

    def update_joint8(self, value):
        self.update_function(8, value)
        
def update_joint_angles(joint_id, angle):
    print(f"Updated {joint_id}: {angle}")

if __name__ == '__main__':
    app = QApplication([])
    gui = JointControlGUI(update_joint_angles)
    gui.show()
    app.exec_()
