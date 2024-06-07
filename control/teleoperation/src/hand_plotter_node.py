#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import Float32MultiArray
import numpy as np
import threading

class HandPlotter:
    def __init__(self):
        rospy.init_node('hand_plotter_node')
        rospy.Subscriber('/ingress/mano', Float32MultiArray, self.callback, queue_size=1, buff_size=2**24)

        self.joint_positions = None
        self.lock = threading.Lock()
        
        self.plot_thread = threading.Thread(target=self.plot_loop)
        self.plot_thread.daemon = True
        self.plot_thread.start()

    def plot_loop(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.lines = [self.ax.plot([], [], [], 'o-')[0] for _ in range(5)]
        
        self.ax.set_xlim([-0.2, 0.2])
        self.ax.set_ylim([-0.2, 0.2])
        self.ax.set_zlim([-0.2, 0.2])

        
        plt.ion()
        plt.show()

        while not rospy.is_shutdown():
            if self.joint_positions is not None:
                with self.lock:
                    joint_positions = self.joint_positions
                    fingers = {
                        "thumb": joint_positions[0:5],  # Thumb already starts from the root
                        "index": np.concatenate([joint_positions[0:1], joint_positions[5:9]]),  # Add root to index finger
                        "middle": np.concatenate([joint_positions[0:1], joint_positions[9:13]]),  # Add root to middle finger
                        "ring": np.concatenate([joint_positions[0:1], joint_positions[13:17]]),  # Add root to ring finger
                        "pinky": np.concatenate([joint_positions[0:1], joint_positions[17:21]])  # Add root to pinky finger
                    }



                    for i, finger in enumerate(fingers.values()):
                        xs, ys, zs = zip(*finger)
                        self.lines[i].set_data(xs, ys)
                        self.lines[i].set_3d_properties(zs)

                    self.ax.relim()
                    self.ax.autoscale_view()
                    self.fig.canvas.draw()
                    self.fig.canvas.flush_events()

            rospy.sleep(0.1)

    def callback(self, msg):
        joints = np.array(msg.data, dtype=np.float32).reshape(msg.layout.dim[0].size, msg.layout.dim[1].size)
        
        with self.lock:
            self.joint_positions = joints

if __name__ == '__main__':
    plotter = HandPlotter()
    rospy.spin()















# #!/usr/bin/env python3

# import rospy
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from std_msgs.msg import Float32MultiArray
# import numpy as np

# class HandPlotter:
#     def __init__(self):
#         # Initialize ROS node
#         rospy.init_node('hand_plotter_node')

#         # Subscribe to the hand tracking topic
#         rospy.Subscriber('/ingress/mano', Float32MultiArray, self.callback, queue_size=1, buff_size=2**24)


#         # Setup matplotlib 3D plot
#         self.fig = plt.figure()
#         self.ax = self.fig.add_subplot(111, projection='3d')

#         # Initialize plot
#         self.lines = [self.ax.plot([], [], [], 'o-')[0] for _ in range(5)]

#         # Show the plot in non-blocking mode
#         plt.ion()
#         plt.show()

#     def callback(self, msg):
#         # Extract joint positions from the message
#         # Assuming data.joints is a flat list of 21*3=63 elements
#         # joints = data.joints
#         joints = np.array(msg.data, dtype=np.float32).reshape(
#             msg.layout.dim[0].size, msg.layout.dim[1].size)
#         joint_positions =  joints#[(joints[i], joints[i+1], joints[i+2]) for i in range(0, len(joints), 3)]

#         # Indices for each finger
#         fingers = {
#             "thumb": joint_positions[0:5],
#             "index": joint_positions[5:9],
#             "middle": joint_positions[9:13],
#             "ring": joint_positions[13:17],
#             "pinky": joint_positions[17:21]
#         }

#         # Update the plot for each finger
#         for i, finger in enumerate(fingers.values()):
#             xs, ys, zs = zip(*finger)
#             self.lines[i].set_data(xs, ys)
#             self.lines[i].set_3d_properties(zs)

#         # Redraw the plot
#         self.ax.relim()
#         self.ax.autoscale_view()
#         self.fig.canvas.draw()
#         self.fig.canvas.flush_events()

# if __name__ == '__main__':
#     plotter = HandPlotter()
#     rospy.spin()
