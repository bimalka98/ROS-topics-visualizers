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
csv_path = os.path.join(package_dir, 'scripts', 'velocity_data.csv')

# delete file if it already exists
if os.path.exists(csv_path):
    os.remove(csv_path)

# Initialize variables to store data
cmd_vel_lin_x = 0
cmd_vel_ang_z = 0
smoothed_cmd_vel_lin_x = 0
smoothed_cmd_vel_ang_z = 0


def cmd_vel_callback(data):
    global cmd_vel_lin_x, cmd_vel_ang_z

    # Get current time in seconds
    t = rospy.get_time()
    cmd_vel_lin_x = data.linear.x
    cmd_vel_ang_z = data.angular.z

    # Write data to CSV file
    with open(csv_path, 'a') as f:
        writer = csv.writer(f)
        writer.writerow([t, cmd_vel_lin_x, cmd_vel_ang_z, smoothed_cmd_vel_lin_x, smoothed_cmd_vel_ang_z])

def smoothed_cmd_vel_callback(data):
    global smoothed_cmd_vel_lin_x, smoothed_cmd_vel_ang_z

    # Get current time in seconds
    t = rospy.get_time()
    smoothed_cmd_vel_lin_x = data.linear.x
    smoothed_cmd_vel_ang_z = data.angular.z

    # Write data to CSV file
    with open(csv_path, 'a') as f:
        writer = csv.writer(f)
        writer.writerow([t, cmd_vel_lin_x, cmd_vel_ang_z, smoothed_cmd_vel_lin_x, smoothed_cmd_vel_ang_z])

rospy.init_node('data_logger')
rospy.Subscriber('/smoothed_cmd_vel', Twist, smoothed_cmd_vel_callback)
rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
rospy.spin()
