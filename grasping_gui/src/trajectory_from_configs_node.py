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
from icecream import ic

class InterpolationGUI:
    def __init__(self, publisher):
        self.publisher = publisher
        self.yaml_files = self.get_yaml_files()
        self.vectors = {file: self.read_yaml_file(file) for file in self.yaml_files}
        self.config_menus = []  # List to keep track of all dropdown menus

        self.root = tk.Tk()  # Initialize the Tkinter window first
        self.root.title("Interpolation GUI")

        self.create_widgets()
        
        self.rate = rospy.Rate(25)  # Set publish rate to 25 Hz

        self.publish_thread = threading.Thread(target=self.publish_data)
        self.publish_thread.daemon = True
        
        self.iterator = 0
        self.interpolation_iterator = 0
        self.loop_iters = 100
        
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
        # Button to add new configuration dropdown menu
        self.add_config_button = tk.Button(self.root, text="Add Config", command=self.add_config_menu)
        self.add_config_button.pack(pady=5)

        # Initial configuration dropdown menus
        self.add_config_menu()
        self.add_config_menu()
        
    def add_config_menu(self):
        # Frame to hold each combobox and its delete button
        frame = tk.Frame(self.root)
        frame.pack(padx=5, pady=5)

        # Create a new dropdown menu for configuration selection
        var = tk.StringVar()
        menu = ttk.Combobox(frame, textvariable=var, values=self.yaml_files, width=50)
        menu.pack(side=tk.LEFT, padx=5, pady=5)

        # Create a delete button for this dropdown menu
        delete_button = tk.Button(frame, text="Delete", command=lambda: self.delete_config_menu(frame, menu, delete_button))
        delete_button.pack(side=tk.LEFT, padx=5, pady=5)

        # Add the new dropdown menu, its variable, delete button, and frame to the list
        self.config_menus.append((var, menu, delete_button, frame))

    def delete_config_menu(self, frame, menu, delete_button):
        # Remove the frame (and all its children) from the GUI
        frame.destroy()

        # Update the config_menus list to remove the deleted config
        self.config_menus = [(var, m, btn, f) for var, m, btn, f in self.config_menus if f != frame]

    def publish_data(self):
        while not rospy.is_shutdown():

            config_vectors = [self.vectors.get(var.get(), np.zeros(11)) for var, menu, delete_button, frame in self.config_menus]                        
            
            start_vec = self.access_list_loop(config_vectors, self.interpolation_iterator)
            end_vec = self.access_list_loop(config_vectors, self.interpolation_iterator+1)
            
            interpolation_factor = (self.iterator%self.loop_iters) / self.loop_iters
            interpolated_vector = (1 - interpolation_factor) * start_vec + interpolation_factor * end_vec
            

            msg = Float32MultiArray(data=interpolated_vector)
            self.publisher.publish(msg)
            self.iterator += 1
            
            if self.iterator%self.loop_iters == 0:
                self.interpolation_iterator += 1
                
            ic(self.interpolation_iterator)
            ic(self.iterator)
            ic(interpolation_factor)
            
            self.rate.sleep()
            
    def access_list_loop(self, list: list, index: int):
        loop_idx = index%len(list)
        return list[loop_idx]
            
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
