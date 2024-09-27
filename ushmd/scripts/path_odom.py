#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import os
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from pyproj import Proj
from tf.transformations import euler_from_quaternion


class TFNode:
    def __init__(self):
        rospy.init_node('tf_node', anonymous=True)
        rospy.Subscriber("/ublox_gps/rtcm", NavSatFix, self.navsat_callback)
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        
        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)

        # 초기화
        self.x, self.y = None, None
        self.is_imu=False
        self.is_gps=False
        self.proj_UTM = Proj(proj='utm',zone=52, ellps='WGS84', preserve_units=False)
        self.current_yaw = 0
        self.odom_msg=Odometry()

        self.odom_msg.header.frame_id='/odom'
        self.odom_msg.child_frame_id='/base_link'

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            os.system('clear')
            if  self.is_imu==True and self.is_gps == True:
                self.convertLL2UTM()
                self.odom_pub.publish(self.odom_msg)
                print(f"odom_msg is now being published at '/odom' topic!\n")
                print('-----------------[ odom_msg ]---------------------')
                print(self.odom_msg.pose)
            if not self.is_imu:
                print("[1] can't subscribe '/imu' topic... \n    please check your IMU sensor connection")
            if not self.is_gps:
                print("[2] can't subscribe '/gps' topic... \n    please check your GPS sensor connection")
            self.is_gps = self.is_imu = False
            rate.sleep()

    def navsat_callback(self, gps_msg):
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude            
        self.is_gps=True

    def convertLL2UTM(self):    
        xy_zone = self.proj_UTM(self.lon, self.lat)
        self.x = xy_zone[0]
        self.y = xy_zone[1]

        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y

    def imu_callback(self, data):
        self.odom_msg.pose.pose.orientation.x = data.orientation.x
        self.odom_msg.pose.pose.orientation.y = data.orientation.y
        self.odom_msg.pose.pose.orientation.z = data.orientation.z
        self.odom_msg.pose.pose.orientation.w = data.orientation.w

        # 쿼터니언을 Euler 각도로 변환
        orientation = [self.odom_msg.pose.pose.orientation.x,
                    self.odom_msg.pose.pose.orientation.y,
                    self.odom_msg.pose.pose.orientation.z,
                    self.odom_msg.pose.pose.orientation.w]

        # 쿼터니언에서 Euler 각도 (roll, pitch, yaw)로 변환
        _, _, self.current_yaw = euler_from_quaternion(orientation)
        self.is_imu=True
    
if __name__ == '__main__':
    try:
        tf_node = TFNode()
    except rospy.ROSInterruptException:
        pass
