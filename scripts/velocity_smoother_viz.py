#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
from statistics import mean

# Get path to package directory
csv_path = 'velocity_data.csv'

# Initialize lists to store data
time = []
cmd_vel_lin_x = []
cmd_vel_ang_z = []
smoothed_cmd_vel_lin_x = []
smoothed_cmd_vel_ang_z = []


# Read data from data.csv
with open(csv_path) as f:
    reader = csv.reader(f)
    for row in reader:
        t, cmd_vel_lin_x_, cmd_vel_ang_z_, smoothed_cmd_vel_lin_x_, smoothed_cmd_vel_ang_z_ = map(float, row)
        time.append(t)
        cmd_vel_lin_x.append(cmd_vel_lin_x_)
        cmd_vel_ang_z.append(cmd_vel_ang_z_)
        smoothed_cmd_vel_lin_x.append(smoothed_cmd_vel_lin_x_)
        smoothed_cmd_vel_ang_z.append(smoothed_cmd_vel_ang_z_)

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

cmd_vel_lin_x = cmd_vel_lin_x[start_index:end_index]
cmd_vel_ang_z = cmd_vel_ang_z[start_index:end_index]
smoothed_cmd_vel_lin_x = smoothed_cmd_vel_lin_x[start_index:end_index]
smoothed_cmd_vel_ang_z = smoothed_cmd_vel_ang_z[start_index:end_index]


# Print min and max of each list
print('cmd_vel_lin_x: min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(cmd_vel_lin_x), max(cmd_vel_lin_x), mean(cmd_vel_lin_x)))
print('cmd_vel_ang_z: min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(cmd_vel_ang_z), max(cmd_vel_ang_z), mean(cmd_vel_ang_z)))
print('smoothed_cmd_vel_lin_x: min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(smoothed_cmd_vel_lin_x), max(smoothed_cmd_vel_lin_x), mean(smoothed_cmd_vel_lin_x)))
print('smoothed_cmd_vel_ang_z: min = {:<10.2f} max = {:<10.2f} avg = {:<10.2f}'.format(min(smoothed_cmd_vel_ang_z), max(smoothed_cmd_vel_ang_z), mean(smoothed_cmd_vel_ang_z)))

# Create plots
# rest of the plots in a single plot. share time axis
fig = plt.figure(figsize=(20, 10))

ax0 = fig.add_subplot(411)
ax0.plot(time, cmd_vel_lin_x)
ax0.grid(True)
ax0.set_ylabel('$Cmd: V_{x}$')

ax1 = fig.add_subplot(412, sharex=ax0)
ax1.plot(time, cmd_vel_ang_z)
ax1.grid(True)
ax1.set_ylabel('$Cmd: \omega_{z}$')

ax2 = fig.add_subplot(413, sharex=ax0)
ax2.plot(time, smoothed_cmd_vel_lin_x)
ax2.grid(True)
ax2.set_ylabel('$Smoothed: V_{x}$')

ax3 = fig.add_subplot(414, sharex=ax0)
ax3.plot(time, smoothed_cmd_vel_ang_z)
ax3.grid(True)
ax3.set_ylabel('$Smoothed: \omega_{z}$')

#save plot a png file with max dpi
plt.savefig('cmd_vel.png', dpi=300)
plt.show()

