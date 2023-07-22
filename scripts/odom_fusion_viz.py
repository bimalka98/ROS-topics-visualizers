#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
import numpy as np

# Get path to package directory
csv_path = 'odom_fusion_data.csv'

# Initialize lists to store data

# time
time = []

# lidar odometry: 
# /odom/lidar
# Type: nav_msgs/Odometry
odom_x_pose_lidar = []; odom_y_pose_lidar = []; odom_lin_x_lidar = []; odom_ang_z_lidar = []

# wheel odometry: 
# topic: /odom/wheel
# Type: nav_msgs/Odometry
odom_x_pose_wheel = []; odom_y_pose_wheel = []; odom_lin_x_wheel = []; odom_ang_z_wheel = []

# imu odometry
# topic: /imu/data
# Type: sensor_msgs/Imu
odom_x_pose_imu = []; odom_y_pose_imu = []; odom_lin_x_imu = []; odom_ang_z_imu = []

# gps odometry
# topic: /robot_rtk_xyz
# Type: geometry_msgs/PointStamped
odom_x_pose_gps = []; odom_y_pose_gps = []; odom_lin_x_gps = []; odom_ang_z_gps = []

# fused odometry
# topic: /odometry/filtered
# Type: nav_msgs/Odometry
odom_x_pose_filtered = []; odom_y_pose_filtered = []; odom_lin_x_filtered = []; odom_ang_z_filtered = []



# Read data from data.csv
with open(csv_path) as f:
    reader = csv.reader(f)
    for row in reader:        
        t, x_pose_lidar, y_pose_lidar, lin_x_lidar, ang_z_lidar, x_pose_wheel, y_pose_wheel, lin_x_wheel, ang_z_wheel,x_pose_imu, y_pose_imu, lin_x_imu, ang_z_imu,x_pose_gps, y_pose_gps, lin_x_gps, ang_z_gps,x_pose_filtered, y_pose_filtered, lin_x_filtered, ang_z_filtered = map(float, row)

        time.append(t)

        odom_x_pose_lidar.append(x_pose_lidar)
        odom_y_pose_lidar.append(y_pose_lidar)
        odom_lin_x_lidar.append(lin_x_lidar)
        odom_ang_z_lidar.append(ang_z_lidar)

        odom_x_pose_wheel.append(x_pose_wheel)
        odom_y_pose_wheel.append(y_pose_wheel)
        odom_lin_x_wheel.append(lin_x_wheel)
        odom_ang_z_wheel.append(ang_z_wheel)

        odom_x_pose_imu.append(x_pose_imu)
        odom_y_pose_imu.append(y_pose_imu)
        odom_lin_x_imu.append(lin_x_imu)
        odom_ang_z_imu.append(ang_z_imu)

        odom_x_pose_gps.append(x_pose_gps)
        odom_y_pose_gps.append(y_pose_gps)
        odom_lin_x_gps.append(lin_x_gps)
        odom_ang_z_gps.append(ang_z_gps)

        odom_x_pose_filtered.append(x_pose_filtered)
        odom_y_pose_filtered.append(y_pose_filtered)
        odom_lin_x_filtered.append(lin_x_filtered)
        odom_ang_z_filtered.append(ang_z_filtered)


# Convert ROS time to seconds and start from first available ROS time as zero
start = time[0]
time = [t - start for t in time]

# conver list to numpy array
time = np.array(time)

odom_x_pose_lidar = np.array(odom_x_pose_lidar)
odom_y_pose_lidar = np.array(odom_y_pose_lidar)
odom_lin_x_lidar = np.array(odom_lin_x_lidar)
odom_ang_z_lidar = np.array(odom_ang_z_lidar)

odom_x_pose_wheel = np.array(odom_x_pose_wheel)
odom_y_pose_wheel = np.array(odom_y_pose_wheel)
odom_lin_x_wheel = np.array(odom_lin_x_wheel)
odom_ang_z_wheel = np.array(odom_ang_z_wheel)

odom_x_pose_imu = np.array(odom_x_pose_imu)
odom_y_pose_imu = np.array(odom_y_pose_imu)
odom_lin_x_imu = np.array(odom_lin_x_imu)
odom_ang_z_imu = np.array(odom_ang_z_imu)

odom_x_pose_gps = np.array(odom_x_pose_gps)
odom_y_pose_gps = np.array(odom_y_pose_gps)
odom_lin_x_gps = np.array(odom_lin_x_gps)
odom_ang_z_gps = np.array(odom_ang_z_gps)

odom_x_pose_filtered = np.array(odom_x_pose_filtered)
odom_y_pose_filtered = np.array(odom_y_pose_filtered)
odom_lin_x_filtered = np.array(odom_lin_x_filtered)
odom_ang_z_filtered = np.array(odom_ang_z_filtered)

# plots the (x, y) trajectory of the robot for both odometeries
fig = plt.figure(figsize=(10, 10))

ax0 = fig.add_subplot(111)
ax0.scatter(odom_x_pose_lidar, odom_y_pose_lidar, label='lidar', marker='x', alpha=0.5)
ax0.scatter(odom_x_pose_wheel, odom_y_pose_wheel, label='wheel + imu', marker='x', alpha=0.5)
# ax0.scatter(odom_x_pose_imu, odom_y_pose_imu, label='imu', marker='x', alpha=0.5)
ax0.scatter(odom_x_pose_gps, odom_y_pose_gps, label='gps', marker='x', alpha=0.5)
ax0.scatter(odom_x_pose_filtered, odom_y_pose_filtered, label='filtered', marker='o', alpha=0.5)
ax0.grid(True)
ax0.set_xlabel('$x$')
ax0.set_ylabel('$y$')
ax0.legend()

#save plot a png file with max dpi
plt.savefig('xy_traj.png', dpi=300)
plt.show()