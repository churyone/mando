#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from ushmd.msg import mission  # 사용자 정의 메시지 임포트
import math

class MissionNode:
    def __init__(self):
        rospy.init_node('mission_node', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.mission_pub = rospy.Publisher('/mission', mission, queue_size=1)

        self.current_position = Point()
        self.mission_info = mission()

        self.proximity_threshold = 2

        self.previous_mission_num = -1  # 이전 미션 번호 저장
        self.count_timer = None  # 카운트를 증가시키는 타이머
        self.count_duration = 0  # 현재 카운트 시간 저장

        # 미션 0 START (시작지점)
        # 미션 1 돌발상황 급정지 (출발하고거의 바로 직후)
        # 미션 2 경사로 (경사로 시작선 1미터 후 지점)
        # 미션 3 신호등1 (신호등 정지선 지점)
        # 미션 4 신호등2 (신호등 정지선 지점)
        # 미션 5 주차1 (T자 주차 미션구간중에 처음 직진해서 안까지 쭉들어갔을때, 그 끝 지점(후진 하기 바로 직전))
        # 미션 51 주차1 (T자 주차에서 후진주차 끝난 지점(이제 직진하기 시작하는 포인트))
        # 미션 6 신호등3 (신호등 정지선 지점)
        # 미션 7 주차2 (평행주차 시작 지점(후진 하기 바로 직전))
        # 미션 71 주차2 (평행주차에서 후진 끝난지점(이제 직진하기 시작하는 포인트))
        # 미션 8 END (종료지점)

        # Define missions with coordinates
        self.missions = [
            {'mission_number': 0, 'x': -93.40589900134364, 'y': 17.37121018813923},
            {'mission_number': 1, 'x': -118.53937525855144, 'y': 4.976131527218968},
            {'mission_number': 2, 'x': -149.3136748817633, 'y': 29.46185883367434},
            {'mission_number': 3, 'x': -147.42292780935531, 'y': 75.13428315287456},
            {'mission_number': 4, 'x': -72.11734004126629, 'y': 111.0132733262144},
            {'mission_number': 5, 'x': 12.72007946803933, 'y': 110.19885071832687},
            {'mission_number': 51, 'x': -0.06720089790178463, 'y': 94.99975404236466},
            {'mission_number': 6, 'x': -73.75782771245576, 'y': 75.91509827692062},
            {'mission_number': 7, 'x': -69.90093989501474, 'y': -53.7754229628481},
            {'mission_number': 71, 'x': -69.86719508410897, 'y': -79.85414305655286},
            {'mission_number': 8, 'x': -46.3847617636784, 'y': -104.03170958580449}
        ]

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            self.update_mission()
            self.mission_pub.publish(self.mission_info)
            rate.sleep()

    def odom_callback(self, msg):
        self.is_odom = True
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

    def distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def count_timer_callback(self, event):
        """Callback function for the timer to increase the count."""
        if self.count_duration < 5:
            self.mission_info.count += 1
            self.count_duration += 1
        else:
            self.mission_info.count = 0
            self.count_timer.shutdown()  # Stop the timer after 5 seconds
            self.count_duration = 0  # Reset count duration

    def update_mission(self):
        """Update the current mission based on the current position."""
        # Iterate through missions to find the one we're currently targeting
        for mission in self.missions:
            dist = self.distance(self.current_position.x, self.current_position.y, mission['x'], mission['y'])
            
            if dist < self.proximity_threshold:
                # Mission reached, update mission info
                if mission['mission_number'] != self.previous_mission_num:
                    self.mission_info.num = mission['mission_number']
                    self.mission_info.count = 1  # Reset and start count at 1
                    self.previous_mission_num = mission['mission_number']

                    if self.count_timer is not None:
                        self.count_timer.shutdown()  # Stop the previous timer if it exists
                    
                    # Start a new repeating timer to increment the count every second for 5 seconds
                    self.count_duration = 1  # Initialize count duration
                    self.count_timer = rospy.Timer(rospy.Duration(1.0), self.count_timer_callback, oneshot=False)


if __name__ == '__main__':
    try:
        mission_node = MissionNode()
    except rospy.ROSInterruptException:
        pass
