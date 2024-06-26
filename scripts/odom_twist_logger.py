#!/usr/bin/env python3
import csv
import os
import rospkg
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from tf.transformations import euler_from_quaternion

# Get path to package directory
rospack = rospkg.RosPack()
package_dir = rospack.get_path('visualizers')
csv_path = os.path.join(package_dir, 'scripts', 'data.csv')

# delete file if it already exists
if os.path.exists(csv_path):
    os.remove(csv_path)

# Initialize variables to store data

# pose and orientation
odom_x_pose = 0
odom_y_pose = 0
odom_yaw = 0

# velocities
odom_lin_x = 0
odom_ang_z = 0
twist_lin_x = 0
twist_ang_z = 0

def odometry_callback(data):
    global odom_x_pose, odom_y_pose, odom_lin_x, odom_ang_z

    # Get current time in seconds
    t = rospy.get_time()

    # Get pose
    odom_x_pose = data.pose.pose.position.x
    odom_y_pose = data.pose.pose.position.y

    # Convert orientation from quaternion to euler
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    odom_yaw = yaw

    # Get linear and angular velocities
    odom_lin_x = data.twist.twist.linear.x
    odom_ang_z = data.twist.twist.angular.z


    # Write data to CSV file
    with open(csv_path, 'a') as f:
        writer = csv.writer(f)
        writer.writerow([t, odom_x_pose, odom_y_pose, odom_yaw, odom_lin_x, odom_ang_z, twist_lin_x, twist_ang_z])

def cmd_vel_callback(data):
    global twist_lin_x, twist_ang_z

    # Get current time in seconds
    t = rospy.get_time()
    twist_lin_x = data.linear.x
    twist_ang_z = data.angular.z

    # Write data to CSV file
    with open(csv_path, 'a') as f:
        writer = csv.writer(f)
        writer.writerow([t, odom_x_pose, odom_y_pose, odom_yaw, odom_lin_x, odom_ang_z, twist_lin_x, twist_ang_z])

rospy.init_node('data_logger')
rospy.Subscriber('/odometry/filtered', Odometry, odometry_callback)
rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
rospy.spin()
