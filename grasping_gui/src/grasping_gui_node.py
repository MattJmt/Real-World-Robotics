#!/usr/bin/env python3


import tkinter as tk
from tkinter import ttk
from tkinter import simpledialog
from std_msgs.msg import Float32MultiArray
import rospy
import numpy as np
import time
import threading
import yaml
import os

class SliderGUI:
    def __init__(self, publisher):
        self.publisher = publisher

        self.root = tk.Tk()
        self.root.title("Slider GUI")

        self.sliders = []
        self.labels = []
        
        save_button = ttk.Button(self.root, text="Save Values", command=self.save_to_yaml)
        save_button.pack(pady=10)
        
        joint_range = [
            (0.0, 110),
            (0.0, 180),
            (0.0, 90),
            (0.0, 90),
            (0.0, 100),
            (0.0, 90),
            (0.0, 100),
            (0.0, 90),
            (0.0, 90),
            (0.0, 110),
            (0.0, 90),
        ]
                
        for i in range(11):
            frame = ttk.Frame(self.root)
            frame.pack(padx=10, pady=5)

            label = ttk.Label(frame, text=f"Value {i+1}: 0.5")
            label.pack(side=tk.LEFT)

            # Use a default value in lambda to capture the current value of i
            slider = ttk.Scale(frame, from_=joint_range[i][0], to=joint_range[i][1], length=300, orient=tk.HORIZONTAL, 
                               command=lambda val, idx=i: self.update_label(idx, val))
            slider.set(10)  # Set initial value
            slider.pack(side=tk.RIGHT, fill=tk.X, expand=True)

            self.sliders.append(slider)
            self.labels.append(label)
            
        self.rate = rospy.Rate(25)  # Set publish rate to 25 Hz

        self.publish_thread = threading.Thread(target=self.publish_data)
        self.publish_thread.daemon = True  # Daemonize thread

    def update_label(self, index, value):
        # Ensure index is within the range of labels
        # print(len(self.labels))
        if 0 <= index < len(self.labels):
            self.labels[index].config(text=f"Value {index+1}: {float(value):.2f}")
        else:
            pass #print(f"Index out of range: {index}")

    def publish_data(self):
        while not rospy.is_shutdown():
            slider_values = [slider.get() for slider in self.sliders]
            msg = Float32MultiArray(data=slider_values)
            self.publisher.publish(msg)
            self.rate.sleep()

    def run(self):
        self.publish_thread.start()  # Start the publisher thread
        self.root.mainloop()  # Start the Tkinter main loop
        

    def save_to_yaml(self):
        while True:
            # Prompt for filename
            filename = simpledialog.askstring("Save File", "Enter filename:", parent=self.root)
            if not filename:  # Check if the user cancelled the input
                print("Save cancelled.")
                return

            # Append .yaml if not present
            if not filename.endswith('.yaml'):
                filename += '.yaml'

            # Check if file exists
            if os.path.exists(os.path.join('joint_configs', filename)):
                response = tk.messagebox.askyesno("File Exists", f"{filename} already exists. Do you want to enter a different name?")
                if response:  # User wants to enter a different name
                    continue
                else:  # User does not want to enter a different name
                    print("Save cancelled.")
                    return
            else:
                break

        # Save the YAML file in the joint_configs directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        save_dir = os.path.join(script_dir, 'joint_configs')
        os.makedirs(save_dir, exist_ok=True)
        os.chdir(save_dir)

        slider_values = [slider.get() for slider in self.sliders]
        data = {'joint_angles': slider_values}

        print(f"saving to: {filename}")
        print(f"filepaths: {os.getcwd()}")

        with open(filename, 'w') as file:
            yaml.dump(data, file)

        print(f"Values saved to {filename}")



def main():
    rospy.init_node('policy_publisher_node', anonymous=True)
    publisher = rospy.Publisher('/faive/policy_output', Float32MultiArray, queue_size=10)

    gui = SliderGUI(publisher)
    gui.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass