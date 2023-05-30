#!/usr/bin/env python3
import matplotlib.pyplot as plt

# Read data from file
with open('velocities.txt', 'r') as f:
    data = f.readlines()

# Extract x and y values
x_values = []
y_values = []
for line in data:
    odom_linear_x, twist_linear_x = line.strip().split(' | ')
    x_values.append(float(odom_linear_x))
    y_values.append(float(twist_linear_x))

# Create plot
plt.plot(x_values, y_values)
plt.xlabel('Odometry Linear X Velocity')
plt.ylabel('Twist Linear X Velocity')
plt.title('Odometry vs Twist Linear X Velocities')
plt.show()
