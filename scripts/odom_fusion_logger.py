#!/usr/bin/env python3
import csv
import os
import rospkg
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
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

# lidar odometry: 
# /odom/lidar
# Type: nav_msgs/Odometry
odom_x_pose_lidar = 0; odom_y_pose_lidar = 0; odom_lin_x_lidar = 0; odom_ang_z_lidar = 0

# wheel odometry: 
# topic: /odom/wheel
# Type: nav_msgs/Odometry
odom_x_pose_wheel = 0; odom_y_pose_wheel = 0; odom_lin_x_wheel = 0; odom_ang_z_wheel = 0

# imu odometry
# topic: /imu/data
# Type: sensor_msgs/Imu
odom_x_pose_imu = 0; odom_y_pose_imu = 0; odom_lin_x_imu = 0; odom_ang_z_imu = 0

# gps odometry
# topic: /robot_rtk_xyz
# Type: geometry_msgs/PointStamped
odom_x_pose_gps = 0; odom_y_pose_gps = 0; odom_lin_x_gps = 0; odom_ang_z_gps = 0

# fused odometry
# topic: /odometry/filtered
# Type: nav_msgs/Odometry
odom_x_pose_filtered = 0; odom_y_pose_filtered = 0; odom_lin_x_filtered = 0; odom_ang_z_filtered = 0

def write_to_file():
    # Get current time in seconds
    t = rospy.get_time()

    # Write data to CSV file
    with open(csv_path, 'a') as f:
        writer = csv.writer(f)
        writer.writerow([t, 
                            odom_x_pose_lidar, odom_y_pose_lidar, odom_lin_x_lidar, odom_ang_z_lidar, 
                            odom_x_pose_wheel, odom_y_pose_wheel, odom_lin_x_wheel, odom_ang_z_wheel,
                            odom_x_pose_imu, odom_y_pose_imu, odom_lin_x_imu, odom_ang_z_imu,
                            odom_x_pose_gps, odom_y_pose_gps, odom_lin_x_gps, odom_ang_z_gps,
                            odom_x_pose_filtered, odom_y_pose_filtered, odom_lin_x_filtered, odom_ang_z_filtered])


def lidar_odometry_callback(data):
    global odom_x_pose_lidar, odom_y_pose_lidar, odom_lin_x_lidar, odom_ang_z_lidar

    # Get pose
    odom_x_pose_lidar = data.pose.pose.position.x
    odom_y_pose_lidar = data.pose.pose.position.y

    # Get linear and angular velocities
    odom_lin_x_lidar = data.twist.twist.linear.x
    odom_ang_z_lidar = data.twist.twist.angular.z

    # Write data to CSV file
    write_to_file()
    
def wheel_odometry_callback(data):        
    global odom_x_pose_wheel, odom_y_pose_wheel, odom_lin_x_wheel, odom_ang_z_wheel
    
    # Get pose
    odom_x_pose_wheel = data.pose.pose.position.x
    odom_y_pose_wheel = data.pose.pose.position.y

    # Get linear and angular velocities
    odom_lin_x_wheel = data.twist.twist.linear.x
    odom_ang_z_wheel = data.twist.twist.angular.z

    # Write data to CSV file
    write_to_file()


def fused_odometry_callback(data):
    global odom_x_pose_filtered, odom_y_pose_filtered, odom_lin_x_filtered, odom_ang_z_filtered

    # Get pose
    odom_x_pose_filtered = data.pose.pose.position.x
    odom_y_pose_filtered = data.pose.pose.position.y

    # Get linear and angular velocities
    odom_lin_x_filtered = data.twist.twist.linear.x
    odom_ang_z_filtered = data.twist.twist.angular.z

    # Write data to CSV file
    write_to_file()

def imu_callback(data):
    global odom_x_pose_imu, odom_y_pose_imu, odom_lin_x_imu, odom_ang_z_imu

    # Get pose
    # Note: IMU data typically does not provide position information
    # You may need to integrate the linear acceleration data to get position
    # Or use some other method to estimate position from the IMU data
    odom_x_pose_imu = 0
    odom_y_pose_imu = 0

    # Get linear and angular velocities
    # Note: The IMU message provides angular velocity and linear acceleration data
    # You may need to convert the linear acceleration to linear velocity
    # Or use some other method to estimate linear velocity from the IMU data
    odom_lin_x_imu = 0
    odom_ang_z_imu = data.angular_velocity.z

    # Write data to CSV file
    write_to_file()

def gps_callback(data):
    global odom_x_pose_gps, odom_y_pose_gps, odom_lin_x_gps, odom_ang_z_gps

    # Get pose
    odom_x_pose_gps = data.point.x
    odom_y_pose_gps = data.point.y

    # Get linear and angular velocities
    # Note: GPS data typically does not provide velocity information
    # You may need to differentiate the position data to get velocity
    # Or use some other method to estimate velocity from the GPS data
    odom_lin_x_gps = 0
    odom_ang_z_gps = 0

    # Write data to CSV file
    write_to_file()


rospy.init_node('data_logger')
# Subscribe to topics
rospy.Subscriber('/odom/lidar', Odometry, lidar_odometry_callback)
rospy.Subscriber('/odom/wheel', Odometry, wheel_odometry_callback)
rospy.Subscriber('/imu/data', Imu, imu_callback)
rospy.Subscriber('/robot_rtk_xyz', PointStamped, gps_callback)
rospy.Subscriber('/odometry/filtered', Odometry, fused_odometry_callback)
rospy.spin()
