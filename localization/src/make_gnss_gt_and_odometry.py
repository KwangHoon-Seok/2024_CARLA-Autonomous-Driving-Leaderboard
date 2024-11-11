#!/usr/bin/env python
import rospy
import csv
import numpy as np
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import os

# Constants
EARTH_RADIUS_EQUA = 6378137.0  # Earth's equatorial radius in meters

def deg2rad(deg):
    return deg * np.pi / 180.0

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

class DataSaver:
    def __init__(self):
        # 초기 위치 (lat0, lon0, alt0)는 0, 0, 0으로 설정
        self.lat0 = 0.0
        self.lon0 = 0.0
        self.alt0 = 0.0
        
        # 파일 경로 설정
        self.gnss_csv_file = os.path.expanduser('~/gnss_data.csv')
        self.odom_csv_file = os.path.expanduser('~/odom_data.csv')

        # CSV 파일 생성 및 헤더 작성
        with open(self.gnss_csv_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['E', 'N'])
        
        with open(self.odom_csv_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['X', 'Y'])

        # ROS Subscriber 설정
        rospy.Subscriber("/carla/ego_vehicle/gnss", NavSatFix, self.gnss_callback)
        rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.odom_callback)

    def gnss_callback(self, data):
        # GNSS 데이터를 ENU 좌표계로 변환
        e, n, _ = lla_to_enu(data.latitude, data.longitude, data.altitude, self.lat0, self.lon0, self.alt0)
        
        # CSV 파일에 기록
        with open(self.gnss_csv_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([e, n])

    def odom_callback(self, data):
        # Odometry 데이터에서 x, y 좌표 추출
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        
        # CSV 파일에 기록
        with open(self.odom_csv_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([x, y])

if __name__ == '__main__':
    rospy.init_node('data_saver_node', anonymous=True)
    data_saver = DataSaver()
    rospy.spin()
