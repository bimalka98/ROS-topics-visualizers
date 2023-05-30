#!/usr/bin/env python3
import rospy
import rospkg 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import os

class VelocityLogger:
    def __init__(self):
        rospy.init_node('velocity_logger')
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.twist_callback)
        self.odom_linear_x = 0.0
        self.twist_linear_x = 0.0
        # get the package path
        rospack = rospkg.RosPack()
        filep =  rospack.get_path('visualizers')
        filep = os.path.join(filep, 'scripts/velocities.txt')
        self.file = open(filep, 'w')

    def odom_callback(self, msg):
        self.odom_linear_x = msg.twist.twist.linear.x
        self.write_to_file()

    def twist_callback(self, msg):
        self.twist_linear_x = msg.linear.x
        self.write_to_file()

    def write_to_file(self):
        self.file.write(f'{self.odom_linear_x} | {self.twist_linear_x}\n')

    def run(self):
        rospy.spin()
        self.file.close()

if __name__ == '__main__':
    logger = VelocityLogger()
    logger.run()
