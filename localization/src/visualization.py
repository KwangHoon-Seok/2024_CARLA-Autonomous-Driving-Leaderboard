#!/usr/bin/env python
#######################################33 Static Plot After Program End ####################################33

import rospy
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt

# Constants for WGS84 to ENU conversion
EARTH_RADIUS_EQUA = 6378137.0  # Earth's radius in meters

# Global variables to store the latest coordinates
enu_coords = []
odom_coords = []
gnss_coords = []
distances = []
gnss_distances = []
rmse_values = []

# Initial GPS reference point (lat0, lon0, alt0)
lat0 = 0.0
lon0 = 0.0
alt0 = 0.0

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

def enu_callback(data):
    global enu_coords
    enu_coords.append((data.x, data.y))

def odom_callback(data):
    global odom_coords
    odom_coords.append((data.pose.pose.position.x, data.pose.pose.position.y))

def gnss_callback(data):
    global gnss_coords
    gnss_enu_x, gnss_enu_y, _ = lla_to_enu(data.latitude, data.longitude, data.altitude, lat0, lon0, alt0)
    gnss_coords.append((gnss_enu_x, gnss_enu_y))

def calculate_distance(x1, y1, x2, y2):
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def calculate_rmse(errors):
    return np.sqrt(np.mean(np.square(errors)))

def plot_results():
    # First plot: ENU coordinates, Odom coordinates, and GNSS coordinates
    plt.figure(figsize=(14, 8))
    plt.subplot(2, 2, 1)
    enu_x, enu_y = zip(*enu_coords)
    odom_x, odom_y = zip(*odom_coords)
    gnss_x, gnss_y = zip(*gnss_coords)

    plt.plot(enu_x, enu_y, label='Kalman Filter ENU', color='red', marker='o', markersize=3)
    plt.plot(odom_x, odom_y, label='Odometry', color='blue', marker='x', markersize=3)
    plt.scatter(gnss_x, gnss_y, label='GNSS', color='green', s=5)  # Smaller point size

    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('ENU, Odom, and GNSS Coordinates')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')  # Ensure equal scaling of X, Y axes

    # Second plot: Distance errors
    plt.subplot(2, 2, 2)

    for enu, odom in zip(enu_coords, odom_coords):
        distances.append(calculate_distance(enu[0], enu[1], odom[0], odom[1]))

    for gnss, odom in zip(gnss_coords, odom_coords):
        gnss_distances.append(calculate_distance(gnss[0], gnss[1], odom[0], odom[1]))

    plt.plot(distances, label='ENU vs Odometry Distance Error', color='red')
    plt.scatter(range(len(gnss_distances)), gnss_distances, label='GNSS vs Odometry Distance Error', color='green', s=5)

    plt.xlabel('Time (frames)')
    plt.ylabel('Distance Error (meters)')
    plt.title('Euclidean Error')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.ylim(0, max(max(distances, default=1), max(gnss_distances, default=1)) + 1)

    # Third plot: RMSE over time
    plt.subplot(2, 2, 4)

    rmse_values.extend([calculate_rmse(distances[:i+1]) for i in range(len(distances))])

    plt.plot(rmse_values, label='RMSE Over Time', color='blue')
    plt.xlabel('Time (frames)')
    plt.ylabel('RMSE (meters)')
    plt.title('Root Mean Squared Error (RMSE)')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.ylim(0, max(rmse_values) + 1)

    # Adjust spacing between plots
    plt.subplots_adjust(hspace=0.4, wspace=0.3)  # Increase space between the subplots

    plt.show()

def main():
    rospy.init_node('coordinate_plotter_node')

    rospy.Subscriber("/enu_coordinates", Point, enu_callback)
    rospy.Subscriber("/carla/hero/Odom", Odometry, odom_callback)
    rospy.Subscriber("/carla/hero/GPS", NavSatFix, gnss_callback)

    rospy.spin()
    plot_results()

if __name__ == '__main__':
    main()
