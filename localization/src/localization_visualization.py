#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt

# Constants for WGS84 to ENU conversion
EARTH_RADIUS_EQUA = 6378137.0  # Earth's radius in meters

def deg2rad(degrees):
    return degrees * np.pi / 180.0

def lla_to_enu(lat, lon, alt, lat0, lon0, alt0, x_offset=0, y_offset=0):
    scale = np.cos(deg2rad(lat0))

    mx = scale * deg2rad(lon) * EARTH_RADIUS_EQUA
    my = scale * EARTH_RADIUS_EQUA * np.log(np.tan(np.pi / 4.0 + deg2rad(lat) / 2.0))

    mx_ref = scale * deg2rad(lon0) * EARTH_RADIUS_EQUA
    my_ref = scale * EARTH_RADIUS_EQUA * np.log(np.tan(np.pi / 4.0 + deg2rad(lat0) / 2.0))

    e = mx - mx_ref + x_offset
    n = my - my_ref + y_offset
    u = alt - alt0
    return e, n, u

class LocalizationEvaluator:
    def __init__(self):
        self.gnss_data = []
        self.imu_data = []
        self.odom_data = []
        self.kf_data = []

        rospy.init_node('localization_evaluator', anonymous=True)

        rospy.Subscriber("/carla/ego_vehicle/gnss", NavSatFix, self.gnss_callback)
        rospy.Subscriber("/carla/ego_vehicle/imu", Imu, self.imu_callback)
        rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.odom_callback)
        rospy.Subscriber("/enu_coordinates", Point, self.kf_callback)

        self.reference_lat = 0
        self.reference_lon = 0
        self.reference_alt = 0

    def gnss_callback(self, data):
        if self.reference_lat is None:
            self.reference_lat = 0
            self.reference_lon = 0
            self.reference_alt = 0

        e, n, u = lla_to_enu(data.latitude, data.longitude, data.altitude,
                             self.reference_lat, self.reference_lon, self.reference_alt)
        gnss_point = np.array([e, n, u])
        self.gnss_data.append(gnss_point)

    def imu_callback(self, data):
        imu_point = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.angular_velocity.z])
        self.imu_data.append(imu_point)

    def odom_callback(self, data):
        odom_point = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self.odom_data.append(odom_point)

    def kf_callback(self, data):
        kf_point = np.array([data.x, data.y, data.z])
        self.kf_data.append(kf_point)

    def plot_data(self):
        if not self.gnss_data or not self.odom_data or not self.kf_data:
            print("Insufficient data for plotting.")
            return

        gnss_data = np.array(self.gnss_data)
        odom_data = np.array(self.odom_data)
        kf_data = np.array(self.kf_data)

        # Adjust lengths to be the same
        min_length = min(len(odom_data), len(kf_data), len(gnss_data))
        gnss_data = gnss_data[:min_length]
        odom_data = odom_data[:min_length]
        kf_data = kf_data[:min_length]
        
        # Adjust imu_data length if necessary
        if self.imu_data:
            imu_data = np.array(self.imu_data)
            imu_data = imu_data[:min_length]
        else:
            imu_data = None

        fig, axs = plt.subplots(2, 2, figsize=(12, 8))

        # Plot 1: Euclidean Position Error
        gnss_error = np.linalg.norm(odom_data[:, :2] - gnss_data[:, :2], axis=1)
        kf_error = np.linalg.norm(odom_data[:, :2] - kf_data[:, :2], axis=1)
        axs[0, 0].plot(kf_error, label='Kalman filter error')
        axs[0, 0].scatter(range(len(gnss_error)), gnss_error, s=1, color='g', label='GNSS error')
        axs[0, 0].set_xlabel('Time step')
        axs[0, 0].set_ylabel('Error (m)')
        axs[0, 0].legend()
        axs[0, 0].set_title('Euclidean Position Error')

        # Plot 2: Odometry vs Kalman filter path
        axs[0, 1].plot(odom_data[:, 0], odom_data[:, 1], label='Odometry')
        axs[0, 1].plot(kf_data[:, 0], kf_data[:, 1], label='Kalman filter')
        axs[0, 1].set_xlabel('X (m)')
        axs[0, 1].set_ylabel('Y (m)')
        axs[0, 1].legend()
        axs[0, 1].set_title('Odometry vs Kalman filter path')

        # Plot 3: Odom X,Y vs GNSS E,N difference
        diff_x = odom_data[:, 0] - gnss_data[:, 0]
        diff_y = odom_data[:, 1] - gnss_data[:, 1]
        axs[1, 0].plot(diff_x, label='Difference in X')
        axs[1, 0].plot(diff_y, label='Difference in Y')
        axs[1, 0].set_xlabel('Time step')
        axs[1, 0].set_ylabel('Difference (m)')
        axs[1, 0].legend()
        axs[1, 0].set_title('Difference between Odometry and GNSS')

        # Plot 4: Placeholder for future use
        axs[1, 1].text(0.5, 0.5, 'Future plot here', ha='center', va='center')
        axs[1, 1].set_axis_off()

        plt.tight_layout()
        plt.show()



if __name__ == '__main__':
    evaluator = LocalizationEvaluator()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        evaluator.plot_data()
