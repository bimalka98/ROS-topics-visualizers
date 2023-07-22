#!/usr/bin/env python3
import numpy as np

# Open the file for reading
with open('path.txt', 'r') as file:
    # Read all lines of the file
    lines = file.readlines()

# Initialize lists to store the x, y, and z coordinates
x_coords = []
y_coords = []
z_coords = []

# Iterate over the lines of the file
for line in lines:
    # Split the line into its components
    parts = line.split()
    # Check if the line contains an x, y, or z coordinate
    if 'x:' in parts:
        x_coords.append(float(parts[1]))
    elif 'y:' in parts:
        y_coords.append(float(parts[1]))
    elif 'z:' in parts:
        z_coords.append(float(parts[1]))

# Convert the lists to numpy arrays
x_coords = np.array(x_coords)
y_coords = np.array(y_coords)
z_coords = np.array(z_coords)

# plot the path in 2D
import matplotlib.pyplot as plt

# plots the (x, y) trajectory of the robot for both odometeries
fig = plt.figure(figsize=(10, 10))

ax0 = fig.add_subplot(111)
ax0.scatter(x_coords, y_coords, label='path', s=1)
ax0.grid(True)
ax0.set_xlabel('$x$')
ax0.set_ylabel('$y$')
ax0.legend()

#save plot a png file with max dpi
plt.savefig('init_path.png', dpi=300)
plt.show()