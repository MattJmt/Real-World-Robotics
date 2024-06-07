#
import rospy
from geometry_msgs.msg import Twist, Pose
import math

pos_x = 0.0
pos_y = 0.0
pos_z = 0.0
ori_x = 0.0
ori_y = 0.0
ori_z = 0.0

# PID controller class
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def compute(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        if abs(output) > 0.05:
            if output < 0:
                output = -0.05
            else:
                output = 0.05
        return output

def pose_callback(msg):
    global pos_x, pos_y, pos_z, ori_x, ori_y, ori_z
    pos_x = msg.position.x
    pos_y = msg.position.y
    pos_z = msg.position.z
    ori_x = msg.orientation.x
    ori_y = msg.orientation.y
    ori_z = msg.orientation.z


def main_function():
    pub = rospy.Publisher('franka_twist', Twist, queue_size=10)
    sub = rospy.Subscriber('PID_input', Pose, pose_callback)
    rospy.init_node('franka_script', anonymous=True)
    rate = rospy.Rate(100) # 100 Hz

    # PID controllers for position
    pid_x = PIDController(1, 0.0, 0.1)
    pid_y = PIDController(1, 0.0, 0.1)
    pid_z = PIDController(1, 0.0, 0.1)

    # PID controllers for orientation
    pid_roll = PIDController(1.0, 0.0, 0.1)
    pid_pitch = PIDController(1.0, 0.0, 0.1)
    pid_yaw = PIDController(1.0, 0.0, 0.1)

    # Target position and orientation
    target_x, target_y, target_z = 0.1, 0.3, 0.5  # Example target position
    target_roll, target_pitch, target_yaw = 0.0, 0.0, 0.0  # Example target orientation

    while not rospy.is_shutdown():
        # Assume a function to get the current position and orientation (to be implemented)
        current_x = pos_x
        current_y = pos_y
        current_z = pos_z
        current_roll = ori_x
        current_pitch = ori_y
        current_yaw = ori_z

        # Calculate error for position
        error_x = target_x - current_x
        error_y = target_y - current_y
        error_z = target_z - current_z

        # Calculate error for orientation
        error_roll = target_roll - current_roll
        error_pitch = target_pitch - current_pitch
        error_yaw = target_yaw - current_yaw

        # Compute PID output for position
        control_x = pid_x.compute(error_x, 1/100.0)
        control_y = pid_y.compute(error_y, 1/100.0)
        control_z = pid_z.compute(error_z, 1/100.0)

        # Compute PID output for orientation
        control_roll = pid_roll.compute(error_roll, 1/100.0)
        control_pitch = pid_pitch.compute(error_pitch, 1/100.0)
        control_yaw = pid_yaw.compute(error_yaw, 1/100.0)

        # Create and publish twist message
        twist_msg = Twist()
        twist_msg.linear.x = control_x
        twist_msg.linear.y = control_y
        twist_msg.linear.z = control_z
        twist_msg.angular.x = control_roll
        twist_msg.angular.y = control_pitch
        twist_msg.angular.z = control_yaw

        #rospy.loginfo(twist_msg)
        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass


