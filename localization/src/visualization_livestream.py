#!/usr/bin/env python
#######################################33 Live Stream ####################################33

import rospy
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants for WGS84 to ENU conversion
EARTH_RADIUS_EQUA = 6378137.0  # Earth's radius in meters

# Global variables to store the latest coordinates
enu_coords = []
odom_coords = []
distances = []
gnss_distances = []
rmse_values = []  # To store RMSE values
legend_shown = [False, False, False]  # Variables to control legend display for each subplot

# Initial GPS reference point (lat0, lon0, alt0)
lat0 = 0.0
lon0 = 0.0
alt0 = 0.0

def deg2rad(degrees):
    return degrees * np.pi / 180.0

def lla_to_enu(lat, lon, alt, lat0, lon0, alt0, x_offset=0, y_offset=0):
    scale = np.cos(deg2rad(lat0))

    mx = scale * deg2rad(lon) * EARTH_RADIUS_EQUA
    my = EARTH_RADIUS_EQUA * np.log(np.tan(np.pi / 4.0 + deg2rad(lat) / 2.0))

    mx_ref = scale * deg2rad(lon0) * EARTH_RADIUS_EQUA
    my_ref = EARTH_RADIUS_EQUA * np.log(np.tan(np.pi / 4.0 + deg2rad(lat0) / 2.0))

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
    global gnss_enu_x, gnss_enu_y
    gnss_enu_x, gnss_enu_y, _ = lla_to_enu(data.latitude, data.longitude, data.altitude, lat0, lon0, alt0)

def calculate_distance(x1, y1, x2, y2):
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def calculate_rmse(distances):
    return np.sqrt(np.mean(np.array(distances) ** 2))

def plot_data(i):
    global legend_shown
    
    plt.cla()
    
    # Unpack coordinates
    if enu_coords and odom_coords:
        enu_x, enu_y = zip(*enu_coords)
        odom_x, odom_y = zip(*odom_coords)
        
        # Calculate distance errors
        distance = calculate_distance(enu_x[-1], enu_y[-1], odom_x[-1], odom_y[-1])
        gnss_distance = calculate_distance(gnss_enu_x, gnss_enu_y, odom_x[-1], odom_y[-1])
        
        distances.append(distance)
        gnss_distances.append(gnss_distance)
        
        # Calculate RMSE
        rmse = calculate_rmse(distances)
        rmse_values.append(rmse)
        
        # Plot ENU and Odom coordinates
        plt.subplot(3, 1, 1)
        plt.plot(enu_x, enu_y, 'r-', label='ENU Coordinates')
        plt.plot(odom_x, odom_y, 'b-', label='Odometry Coordinates')
        plt.scatter(enu_x[-1], enu_y[-1], color='red', s=5)
        plt.scatter(odom_x[-1], odom_y[-1], color='blue', s=5)
        
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.title('ENU vs Odom Coordinates')
        if not legend_shown[0]:
            plt.legend(loc='upper right')
            legend_shown[0] = True
        plt.grid(True)
        plt.axis('equal')
        
        # Adjust the plot limits based on the current data
        plt.xlim(min(min(enu_x), min(odom_x)) - 5, max(max(enu_x), max(odom_x)) + 5)
        plt.ylim(min(min(enu_y), min(odom_y)) - 5, max(max(enu_y), max(odom_y)) + 5)

        # Plot distance errors
        plt.subplot(3, 1, 2)
        plt.plot(distances, 'r-', label='ENU vs Odom Distance Error')
        plt.scatter(range(len(gnss_distances)), gnss_distances, color='green', label='GNSS vs Odom Distance Error', s=5)
        
        plt.xlabel('Time (frames)')
        plt.ylabel('Distance Error (meters)')
        plt.title('Euclidean Error')
        if not legend_shown[1]:
            plt.legend(loc='upper right')
            legend_shown[1] = True
        plt.grid(True)
        plt.ylim(0, max(max(distances, default=1), max(gnss_distances, default=1)) + 0.1)
        plt.yticks(np.arange(0, max(max(distances, default=1), max(gnss_distances, default=1)) + 0.1, 0.1))

        # Plot RMSE
        plt.subplot(3, 1, 3)
        plt.plot(rmse_values, 'b-', label='RMSE of ENU vs Odom')
        
        plt.xlabel('Time (frames)')
        plt.ylabel('RMSE (meters)')
        plt.title('Root Mean Square Error (RMSE)')
        if not legend_shown[2]:
            plt.legend(loc='upper right')
            legend_shown[2] = True
        plt.grid(True)
        plt.ylim(0, max(rmse_values, default=1) + 0.1)
        plt.yticks(np.arange(0, max(rmse_values, default=1) + 0.1, 0.1))

def main():
    rospy.init_node('coordinate_plotter_node')

    # rospy.Subscriber("/carla/hero/position", Point, enu_callback)
    rospy.Subscriber("/enu_coordinates", Point, enu_callback)
    rospy.Subscriber("/carla/hero/Odom", Odometry, odom_callback)
    rospy.Subscriber("/carla/hero/GPS", NavSatFix, gnss_callback)

    # Setup the plot for live update
    plt.figure(figsize=(10, 12))
    ani = FuncAnimation(plt.gcf(), plot_data, interval=100)

    # Keep the plot open
    plt.show()

    # ROS spin
    rospy.spin()

    # After program termination, plot the final data
    plt.figure(figsize=(10, 8))
    plot_data(None)
    plt.show()

if __name__ == '__main__':
    main()
