#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, sqrt, pow, atan2, pi
from geometry_msgs.msg import Point
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Int16, Float32
import numpy as np
from tf.transformations import euler_from_quaternion
from ushmd.msg import mission, obstacle  # 사용자 정의 메시지 임포트 (CtrlCmd 삭제)
import rospy
import time

class pure_pursuit:
    def __init__(self):
        rospy.init_node('control', anonymous=True)

        self.speed_cmd_pub = rospy.Publisher('/target_speed', Float32, queue_size=10)
        self.angle_cmd_pub = rospy.Publisher('/target_angle', Float32, queue_size=10)

        self.speed_cmd_msg = Float32()
        self.angle_cmd_msg = Float32()

        # S자 패턴 실행
        self.move_in_s_pattern()

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            # 퍼블리시
            self.speed_cmd_msg = -1
            self.angle_cmd_msg = 0
            self.speed_cmd_pub.publish(self.speed_cmd_msg)
            self.angle_cmd_pub.publish(self.angle_cmd_msg)
            rate.sleep()

    # 10초 동안 속도 5로 S자 패턴으로 움직이는 함수
    def move_in_s_pattern(self):
        # 속도와 각도 설정
        speed = 1
        angle_amplitude = 20  # S자 궤적의 좌우 조향 각도
        duration = 20  # 움직일 시간 (초)
        interval = 2  # 각도를 변경할 간격 (초)
        iterations = int(duration / interval)  # 총 반복 횟수

        start_time = rospy.Time.now().to_sec()  # 시작 시간

        for i in range(iterations):
            # S자 패턴을 위해 각도를 반복적으로 변화
            # 좌우로 각도 조절
            if i % 2 == 0:
                angle = angle_amplitude  # 오른쪽으로 조향
            else:
                angle = -angle_amplitude  # 왼쪽으로 조향

            # 속도와 각도 메시지 설정
            self.speed_cmd_msg.data = speed
            self.angle_cmd_msg.data = angle

            # 퍼블리시
            self.speed_cmd_pub.publish(self.speed_cmd_msg)
            self.angle_cmd_pub.publish(self.angle_cmd_msg)

            # 각도 변경 후 일정 시간 대기
            rospy.sleep(interval)

        # 10초 후 멈추기
        self.stop_vehicle()

    def stop_vehicle(self):
        self.speed_cmd_msg.data = 0.0
        self.angle_cmd_msg.data = 0.0  # 조향 각도를 0으로 설정
        self.speed_cmd_pub.publish(self.speed_cmd_msg)
        self.angle_cmd_pub.publish(self.angle_cmd_msg)
        rospy.sleep(2)
        rospy.loginfo("Vehicle stopped")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        control = pure_pursuit()
    except rospy.ROSInterruptException:
        pass
