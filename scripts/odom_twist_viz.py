#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
from statistics import mean

# Get path to package directory
csv_path = 'data.csv'

# Initialize lists to store data
time = []
x = []
y = []
odom_linear_x = []
odom_angular_z = []
twist_linear_x = []
twist_angular_z = []

# Read data from data.csv
with open(csv_path) as f:
    reader = csv.reader(f)
    for row in reader:
        t, x_, y_, odom_linear_x_, odom_angular_z_, twist_linear_x_, twist_angular_z_ = map(float, row)
        time.append(t)
        x.append(x_)
        y.append(y_)
        odom_linear_x.append(odom_linear_x_)
        odom_angular_z.append(odom_angular_z_)
        twist_linear_x.append(twist_linear_x_)
        twist_angular_z.append(twist_angular_z_)

# Convert ROS time to seconds and start from first available ROS time as zero
start_time = time[0]
time = [t - start_time for t in time]

# Print min and max of each list
# print('time:           min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(time), max(time), mean(time)))
# print('x:              min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(x), max(x), mean(x)))
# print('y:              min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(y), max(y), mean(y)))
print('odom_linear_x:  min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(odom_linear_x), max(odom_linear_x), mean(odom_linear_x)))
print('odom_angular_z: min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(odom_angular_z), max(odom_angular_z), mean(odom_angular_z)))
print('twist_linear_x: min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(twist_linear_x), max(twist_linear_x), mean(twist_linear_x)))
print('twist_angular_z:min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(twist_angular_z), max(twist_angular_z), mean(twist_angular_z)))

# Create plots
# odometry pose x vs y in a separate plot
fig = plt.figure(figsize=(10, 10))
ax1 = fig.add_subplot(111)
ax1.scatter(x, y, s=10, c='b', marker="s", label='odometry')
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_title('Odometry: x vs y')

# save plot a png file with max dpi
plt.savefig('odom_pose_viz.png', dpi=300)
plt.show()

# rest of the plots in a single plot. share time axis
fig = plt.figure(figsize=(10, 10))

ax1 = fig.add_subplot(411)
ax1.plot(time, odom_linear_x)
ax1.set_ylabel('OLX')

ax2 = fig.add_subplot(412)
ax2.plot(time, odom_angular_z)
ax2.set_ylabel('OAZ')


ax3 = fig.add_subplot(413)
ax3.plot(time, twist_linear_x)
ax3.set_ylabel('TLX')

ax4 = fig.add_subplot(414)
ax4.plot(time, twist_angular_z)
ax4.set_ylabel('TAZ')

#save plot a png file with max dpi
plt.savefig('odom_twist_viz.png', dpi=300)

plt.show()

