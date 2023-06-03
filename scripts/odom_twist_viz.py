#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
from statistics import mean

# Get path to package directory
csv_path = 'data.csv'

# Initialize lists to store data
time = []

# robot pose
x = []
y = []
odom_yaw = []

# robot velocities
odom_linear_x = []
odom_angular_z = []

# twist velocities from ros nav stack
twist_linear_x = []
twist_angular_z = []

# Read data from data.csv
with open(csv_path) as f:
    reader = csv.reader(f)
    for row in reader:
        t, x_, y_, odom_yaw_, odom_linear_x_, odom_angular_z_, twist_linear_x_, twist_angular_z_ = map(float, row)
        time.append(t)
        
        x.append(x_)
        y.append(y_)
        odom_yaw.append(odom_yaw_)

        odom_linear_x.append(odom_linear_x_)
        odom_angular_z.append(odom_angular_z_)
        twist_linear_x.append(twist_linear_x_)
        twist_angular_z.append(twist_angular_z_)

# Convert ROS time to seconds and start from first available ROS time as zero
start = time[0]
time = [t - start for t in time]

# limit number of data points accoridng to to given start time and end time
start_time = 250
end_time = 360

# get the index of the element in time list that is closest to start_time and end_time
start_index = min(range(len(time)), key=lambda i: abs(time[i]-start_time))
end_index = min(range(len(time)), key=lambda i: abs(time[i]-end_time))

# update lists
time = time[start_index:end_index]

x = x[start_index:end_index]
y = y[start_index:end_index]
odom_yaw = odom_yaw[start_index:end_index]

odom_linear_x = odom_linear_x[start_index:end_index]
odom_angular_z = odom_angular_z[start_index:end_index]
twist_linear_x = twist_linear_x[start_index:end_index]
twist_angular_z = twist_angular_z[start_index:end_index]

# Print min and max of each list
# print('time:           min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(time), max(time), mean(time)))
# print('x:              min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(x), max(x), mean(x)))
# print('y:              min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(y), max(y), mean(y)))
# print('odom_yaw:       min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(odom_yaw), max(odom_yaw), mean(odom_yaw)))
print('odom_linear_x:  min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(odom_linear_x), max(odom_linear_x), mean(odom_linear_x)))
print('odom_angular_z: min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(odom_angular_z), max(odom_angular_z), mean(odom_angular_z)))
print('twist_linear_x: min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(twist_linear_x), max(twist_linear_x), mean(twist_linear_x)))
print('twist_angular_z:min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(twist_angular_z), max(twist_angular_z), mean(twist_angular_z)))

# Create plots
# odometry pose x vs y in a separate plot
fig = plt.figure(figsize=(20, 10)) # width, height
ax1 = fig.add_subplot(111)
ax1.scatter(x, y, s=10, c='b', marker="s", label='odometry', alpha=0.5)
ax1.grid(True)
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_title('Odometry: x vs y')

# save plot a png file with max dpi
plt.savefig('odom_pose_viz.png', dpi=300)
plt.show()

# rest of the plots in a single plot. share time axis
fig = plt.figure(figsize=(20, 10))

ax0 = fig.add_subplot(511)
ax0.plot(time, odom_yaw)
ax0.grid(True)
ax0.set_ylabel('$Odom: \\theta_{yaw}$')

ax1 = fig.add_subplot(512)
ax1.plot(time, odom_linear_x)
ax1.grid(True)
ax1.set_ylabel('$Odom: V_{x}$')

ax2 = fig.add_subplot(513)
ax2.plot(time, odom_angular_z)
ax2.grid(True)
ax2.set_ylabel('$Odom: \\omega_{z}$')

ax3 = fig.add_subplot(514)
ax3.plot(time, twist_linear_x)
ax3.grid(True)
ax3.set_ylabel('$Twist: V_{x}$')

ax4 = fig.add_subplot(515)
ax4.plot(time, twist_angular_z)
ax4.grid(True)
ax4.set_ylabel('$Twist: \\omega_{z}$')

#save plot a png file with max dpi
plt.savefig('odom_twist_viz.png', dpi=300)
plt.show()

