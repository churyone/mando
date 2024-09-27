#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from pyproj import Proj
from tf.transformations import euler_from_quaternion

class ODOM:
    def __init__(self):
        rospy.init_node('odom', anonymous=True)
        rospy.Subscriber("/ublox_gps/rtcm", NavSatFix, self.navsat_callback)
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)

        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)

        self.x, self.y = None, None
        self.is_imu=False
        self.is_gps=False

        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id='/odom'
        self.odom_msg.child_frame_id='/base_link'

        self.proj_UTM = Proj(proj='utm',zone=52, ellps='WGS84', preserve_units=False)

        self.encoder_speed = 0.0

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_imu==True and self.is_gps == True:
                self.convertLL2UTM()
                self.odom_pub.publish(self.odom_msg)
                self.is_gps = self.is_imu = False
                rospy.loginfo("is publishing odom")
            else:
                rospy.loginfo("gps or imu error")
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
        self.is_imu=True

    def mission_callback(self, msg):
        self.mission = msg    

    def get_heading(self):    
        if self.odom_msg is not None:
            orientation = [self.odom_msg.pose.pose.orientation.x,
                        self.odom_msg.pose.pose.orientation.y,
                        self.odom_msg.pose.pose.orientation.z,
                        self.odom_msg.pose.pose.orientation.w]
            _, _, heading = euler_from_quaternion(orientation)
            return heading

if __name__ == '__main__':
    try:
        odom = ODOM()
    except rospy.ROSInterruptException:
        pass
