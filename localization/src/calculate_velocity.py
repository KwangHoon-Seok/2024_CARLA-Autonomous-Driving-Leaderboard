#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from carla_msgs.msg import CarlaSpeedometer  # Import the correct message type
import math

class VelocityCalculator:
    def __init__(self):
        self.speed = 0.0
        self.yaw = 0.0

        # Subscribe to speedometer and IMU topics
        rospy.Subscriber("/carla/ego_vehicle/speedometer", CarlaSpeedometer, self.speed_callback)
        rospy.Subscriber("/carla/ego_vehicle/imu", Imu, self.imu_callback)

    def speed_callback(self, msg):
        # Speed from speedometer (m/s)
        self.speed = msg.speed  # Adjust according to the correct field

    def imu_callback(self, msg):
        # Yaw from IMU orientation (assumed to be in radians)
        # Converting quaternion to yaw
        quaternion = msg.orientation
        self.yaw = self.quaternion_to_yaw(quaternion)

        # Calculate linear velocities
        vx, vy, vz = self.calculate_linear_velocity()

        # Print the velocities to the terminal
        rospy.loginfo("Linear Velocity: vx=%.2f, vy=%.2f, vz=%.2f", vx, vy, vz)

    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw (in radians)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def calculate_linear_velocity(self):
        # Assuming that the speed is along the x-axis of the vehicle frame
        vx = self.speed * math.cos(self.yaw)
        vy = self.speed * math.sin(self.yaw)
        vz = 0.0  # Assuming the vehicle is moving in a 2D plane
        return vx, vy, vz

if __name__ == '__main__':
    rospy.init_node('velocity_calculator')
    VelocityCalculator()
    rospy.spin()
