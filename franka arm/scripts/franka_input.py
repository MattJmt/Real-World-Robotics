import rospy
from geometry_msgs.msg import Twist, Pose
import math

lin_x = 0.0
lin_y = 0.0
lin_z = 0.0

ang_x = 0.0
ang_y = 0.0
ang_z = 0.0

def twist_callback(msg):
    global lin_x, lin_y, lin_z, ang_x, ang_y, ang_z
    lin_x = msg.linear.x
    lin_y = msg.linear.y
    lin_z = msg.linear.z
    ang_x = msg.angular.x
    ang_y = msg.angular.y
    ang_z = msg.angular.z

def main_function():
    rospy.init_node('franka_input_node', anonymous=True)
    pub = rospy.Publisher('PID_input', Pose, queue_size=10)
    sub = rospy.Subscriber('franka_twist', Twist, twist_callback)

    rate = rospy.Rate(100)  # 100Hz

    old_pose = Pose()
    old_pose.position.x = 0
    old_pose.position.y = 0
    old_pose.position.z = 0
    old_pose.orientation.x = 0
    old_pose.orientation.y = 0
    old_pose.orientation.z = 0

    while not rospy.is_shutdown():
        # Update pose based on velocities
        new_x = old_pose.position.x + lin_x * rate.sleep_dur.to_sec()
        new_y = old_pose.position.y + lin_y * rate.sleep_dur.to_sec()
        new_z = old_pose.position.z + lin_z * rate.sleep_dur.to_sec()
        new_roll = old_pose.orientation.x + ang_x * rate.sleep_dur.to_sec()
        new_pitch = old_pose.orientation.y + ang_y * rate.sleep_dur.to_sec()
        new_yaw = old_pose.orientation.z + ang_z * rate.sleep_dur.to_sec()

        # Create and publish pose message
        pose_msg = Pose()
        pose_msg.position.x = new_x
        pose_msg.position.y = new_y
        pose_msg.position.z = new_z
        pose_msg.orientation.x = new_roll
        pose_msg.orientation.y = new_pitch
        pose_msg.orientation.z = new_yaw

        pub.publish(pose_msg)

        # Update old_pose for next iteration
        old_pose = pose_msg

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
