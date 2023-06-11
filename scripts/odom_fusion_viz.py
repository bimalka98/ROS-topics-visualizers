#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
import numpy as np

# Get path to package directory
csv_path = 'odom_fusion_data.csv'

# Initialize lists to store data

# time
time = []

# pose and orientation
odom_x_pose_lidar = []
odom_y_pose_lidar = []
odom_x_pose_filtered = []
odom_y_pose_filtered = []

# velocities
odom_lin_x_lidar = []
odom_ang_z_lidar = []
odom_lin_x_filtered = []
odom_ang_z_filtered = []


# Read data from data.csv
with open(csv_path) as f:
    reader = csv.reader(f)
    for row in reader:
        t, x_lidar, y_lidar, lin_x_lidar, ang_z_lidar, x_filtered, y_filtered, lin_x_filtered, ang_z_filtered = map(float, row)
        
        time.append(t)

        odom_x_pose_lidar.append(x_lidar)
        odom_y_pose_lidar.append(y_lidar)
        odom_lin_x_lidar.append(lin_x_lidar)
        odom_ang_z_lidar.append(ang_z_lidar)

        odom_x_pose_filtered.append(x_filtered)
        odom_y_pose_filtered.append(y_filtered)
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
odom_x_pose_filtered = np.array(odom_x_pose_filtered)
odom_y_pose_filtered = np.array(odom_y_pose_filtered)
odom_lin_x_filtered = np.array(odom_lin_x_filtered)
odom_ang_z_filtered = np.array(odom_ang_z_filtered)

# get the difference between the two odometeries
odom_x_pose_diff = odom_x_pose_filtered - odom_x_pose_lidar
odom_y_pose_diff = odom_y_pose_filtered - odom_y_pose_lidar
odom_lin_x_diff = odom_lin_x_filtered - odom_lin_x_lidar
odom_ang_z_diff = odom_ang_z_filtered - odom_ang_z_lidar

# get the mean of the difference
odom_x_pose_diff_mean = np.mean(odom_x_pose_diff)
odom_y_pose_diff_mean = np.mean(odom_y_pose_diff)
odom_lin_x_diff_mean = np.mean(odom_lin_x_diff)
odom_ang_z_diff_mean = np.mean(odom_ang_z_diff)

# get the standard deviation of the difference
odom_x_pose_diff_std = np.std(odom_x_pose_diff)
odom_y_pose_diff_std = np.std(odom_y_pose_diff)
odom_lin_x_diff_std = np.std(odom_lin_x_diff)
odom_ang_z_diff_std = np.std(odom_ang_z_diff)

# print the mean and standard deviation of the difference
# use this format: print('cmd_vel_lin_x: min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(cmd_vel_lin_x), max(cmd_vel_lin_x), mean(cmd_vel_lin_x)))
print('odom_x_pose_diff: mean = {:<10.2f} std = {:<10.2f}'.format(odom_x_pose_diff_mean, odom_x_pose_diff_std))
print('odom_y_pose_diff: mean = {:<10.2f} std = {:<10.2f}'.format(odom_y_pose_diff_mean, odom_y_pose_diff_std))
print('odom_lin_x_diff:  mean = {:<10.2f} std = {:<10.2f}'.format(odom_lin_x_diff_mean, odom_lin_x_diff_std))
print('odom_ang_z_diff:  mean = {:<10.2f} std = {:<10.2f}'.format(odom_ang_z_diff_mean, odom_ang_z_diff_std))

# plot the data in a figure with 4 subplots and shared time axis
fig = plt.figure(figsize=(20, 10))

ax0 = fig.add_subplot(411)
ax0.plot(time, odom_x_pose_diff)
ax0.grid(True)
ax0.set_ylabel('$\Delta x$')

ax1 = fig.add_subplot(413, sharex=ax0)
ax1.plot(time, odom_y_pose_diff)
ax1.grid(True)
ax1.set_ylabel('$\Delta y$')

ax2 = fig.add_subplot(412, sharex=ax0)
ax2.plot(time, odom_lin_x_diff)
ax2.grid(True)
ax2.set_ylabel('$\Delta v_{x}$')

ax3 = fig.add_subplot(414, sharex=ax0)
ax3.plot(time, odom_ang_z_diff)
ax3.grid(True)
ax3.set_ylabel('$\Delta \omega_{z}$')

#save plot a png file with max dpi
plt.savefig('odom_fusion.png', dpi=300)
plt.show()

# plots the (x, y) trajectory of the robot for both odometeries
fig = plt.figure(figsize=(10, 10))

ax0 = fig.add_subplot(111)
ax0.plot(odom_x_pose_lidar, odom_y_pose_lidar, label='lidar')
ax0.plot(odom_x_pose_filtered, odom_y_pose_filtered, label='filtered')
ax0.grid(True)
ax0.set_xlabel('$x$')
ax0.set_ylabel('$y$')
ax0.legend()

#save plot a png file with max dpi
plt.savefig('odom_fusion_xy_plot.png', dpi=300)
plt.show()

# plots the (x, y) trajectory of the robot for both odometeries
fig = plt.figure(figsize=(10, 10))

ax0 = fig.add_subplot(111)
ax0.scatter(odom_x_pose_lidar, odom_y_pose_lidar, label='lidar', marker='x', alpha=0.5)
ax0.scatter(odom_x_pose_filtered, odom_y_pose_filtered, label='filtered', marker='o', alpha=0.5)
ax0.grid(True)
ax0.set_xlabel('$x$')
ax0.set_ylabel('$y$')
ax0.legend()

#save plot a png file with max dpi
plt.savefig('odom_fusion_xy_scatter.png', dpi=300)
plt.show()