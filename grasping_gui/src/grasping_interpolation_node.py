#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import threading
import yaml
import os
import glob

class InterpolationGUI:
    def __init__(self, publisher):
        self.publisher = publisher
        self.yaml_files = self.get_yaml_files()
        self.vectors = {file: self.read_yaml_file(file) for file in self.yaml_files}

        self.root = tk.Tk()
        self.root.title("Interpolation GUI")

        self.create_widgets()
        self.rate = rospy.Rate(25)  # Set publish rate to 25 Hz

        self.publish_thread = threading.Thread(target=self.publish_data)
        self.publish_thread.daemon = True

    def get_yaml_files(self):
        # Get all YAML files in the "joint_configs" directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        yaml_dir = os.path.join(script_dir, "joint_configs")
        os.chdir(yaml_dir)
        return glob.glob("*.yaml")
    
    def read_yaml_file(self, filepath):
        # Read YAML file and return the vector
        with open(filepath, 'r') as file:
            data = yaml.safe_load(file)
        return np.array(data['joint_angles'])

    def create_widgets(self):
        # Dropdown menus for YAML file selection
        self.file1_var = tk.StringVar()
        self.file2_var = tk.StringVar()
        self.file1_menu = ttk.Combobox(self.root, textvariable=self.file1_var, values=self.yaml_files)
        self.file2_menu = ttk.Combobox(self.root, textvariable=self.file2_var, values=self.yaml_files)
        self.file1_menu.pack(pady=5)
        self.file2_menu.pack(pady=5)

        # Slider for interpolation
        self.interpolation_slider = ttk.Scale(self.root, from_=0.0, to=1.0, orient='horizontal')
        self.interpolation_slider.set(0.5)
        self.interpolation_slider.pack(fill='x', expand=True)

    def publish_data(self):
        while not rospy.is_shutdown():
            vector1 = self.vectors.get(self.file1_var.get(), np.zeros(11))
            vector2 = self.vectors.get(self.file2_var.get(), np.zeros(11))
            interpolation_factor = self.interpolation_slider.get()
            interpolated_vector = (1 - interpolation_factor) * vector1 + interpolation_factor * vector2

            msg = Float32MultiArray(data=interpolated_vector)
            self.publisher.publish(msg)
            self.rate.sleep()

    def run(self):
        self.publish_thread.start()
        self.root.mainloop()

def main():
    rospy.init_node('interpolation_publisher_node', anonymous=True)
    publisher = rospy.Publisher('/faive/policy_output', Float32MultiArray, queue_size=10)

    gui = InterpolationGUI(publisher)
    gui.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
