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
csv_path = os.path.join(package_dir, 'scripts', 'odom_fusion_data.csv')

# delete file if it already exists
if os.path.exists(csv_path):
    os.remove(csv_path)

# Initialize variables to store data

# pose and orientation
odom_x_pose_lidar = 0
odom_y_pose_lidar = 0
odom_x_pose_filtered = 0
odom_y_pose_filtered = 0

# velocities
odom_lin_x_lidar = 0
odom_ang_z_lidar = 0
odom_lin_x_filtered = 0
odom_ang_z_filtered = 0


def lidar_odometry_callback(data):

    global odom_x_pose_lidar, odom_y_pose_lidar, odom_lin_x_lidar, odom_ang_z_lidar
    global odom_x_pose_filtered, odom_y_pose_filtered, odom_lin_x_filtered, odom_ang_z_filtered

    # Get current time in seconds
    t = rospy.get_time()

    # Get pose
    odom_x_pose_lidar = data.pose.pose.position.x
    odom_y_pose_lidar = data.pose.pose.position.y

    # Get linear and angular velocities
    odom_lin_x_lidar = data.twist.twist.linear.x
    odom_ang_z_lidar = data.twist.twist.angular.z


    # Write data to CSV file
    with open(csv_path, 'a') as f:
        writer = csv.writer(f)
        writer.writerow([t, odom_x_pose_lidar, odom_y_pose_lidar, odom_lin_x_lidar, odom_ang_z_lidar, odom_x_pose_filtered, odom_y_pose_filtered, odom_lin_x_filtered, odom_ang_z_filtered])

def fused_odometry_callback(data):
    
    global odom_x_pose_lidar, odom_y_pose_lidar, odom_lin_x_lidar, odom_ang_z_lidar
    global odom_x_pose_filtered, odom_y_pose_filtered, odom_lin_x_filtered, odom_ang_z_filtered

    # Get current time in seconds
    t = rospy.get_time()
    
    # Get pose
    odom_x_pose_filtered = data.pose.pose.position.x
    odom_y_pose_filtered = data.pose.pose.position.y

    # Get linear and angular velocities
    odom_lin_x_filtered = data.twist.twist.linear.x
    odom_ang_z_filtered = data.twist.twist.angular.z

    # Write data to CSV file
    with open(csv_path, 'a') as f:
        writer = csv.writer(f)
        writer.writerow([t, odom_x_pose_lidar, odom_y_pose_lidar, odom_lin_x_lidar, odom_ang_z_lidar, odom_x_pose_filtered, odom_y_pose_filtered, odom_lin_x_filtered, odom_ang_z_filtered])

rospy.init_node('data_logger')
rospy.Subscriber('/odometry/filtered', Odometry, fused_odometry_callback)
rospy.Subscriber('/odom/lidar', Odometry, lidar_odometry_callback)
rospy.spin()
