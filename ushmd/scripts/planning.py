#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
from ushmd.msg import mission

class planning:

    def __init__(self):
        rospy.init_node('planning', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/mission", mission, self.mission_callback)

        # 런치 파일에서 시작 미션 번호를 파라미터로 가져옴 (기본값: 0)
        self.start_mission_num = rospy.get_param('start_mission_num', 0)

        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        self.is_odom = False
        self.local_path_size = 20

        self.current_mission_num = self.start_mission_num  # 미션 번호가 변경되었는지 확인하기 위한 변수

        # 최초 경로 로드 (첫 번째 경로는 기본으로 로드)
        self.load_path(self.start_mission_num)

        rate = rospy.Rate(20)  # 20hz
        while not rospy.is_shutdown():
            if self.is_odom:
                local_path_msg = Path()
                local_path_msg.header.frame_id = '/map'

                x = self.x
                y = self.y
                min_dis = float('inf')
                current_waypoint = -1
                for i, waypoint in enumerate(self.global_path_msg.poses):

                    distance = sqrt(pow(x - waypoint.pose.position.x, 2) + pow(y - waypoint.pose.position.y, 2))
                    if distance < min_dis:
                        min_dis = distance
                        current_waypoint = i

                if current_waypoint != -1:
                    if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                        for num in range(current_waypoint, current_waypoint + self.local_path_size):
                            tmp_pose = PoseStamped()
                            tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                            tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                            tmp_pose.pose.orientation.w = 1
                            local_path_msg.poses.append(tmp_pose)
                    else:
                        for num in range(current_waypoint, len(self.global_path_msg.poses)):
                            tmp_pose = PoseStamped()
                            tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                            tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                            tmp_pose.pose.orientation.w = 1
                            local_path_msg.poses.append(tmp_pose)

                self.local_path_pub.publish(local_path_msg)

            rate.sleep()

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)

    def mission_callback(self, msg):
        self.mission = msg
        # print(self.mission)
        # 미션 번호가 변경된 경우에만 load_path 호출
        if self.mission.num != self.current_mission_num:
            self.current_mission_num = self.mission.num
            self.load_path(self.current_mission_num)

    def load_path(self, mission_num):
        """경로 파일을 로드하는 함수. 미션 번호에 따라 경로 파일을 동적으로 로드"""
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('ushmd')
        file_path = f'{pkg_path}/path/test_path{mission_num}.txt'

        try:
            with open(file_path, 'r') as f:
                self.global_path_msg.poses.clear()  # 이전 경로 지우기
                lines = f.readlines()
                for line in lines:
                    tmp = line.split()
                    read_pose = PoseStamped()
                    read_pose.pose.position.x = float(tmp[0])
                    read_pose.pose.position.y = float(tmp[1])
                    read_pose.pose.orientation.w = float(tmp[2])  # w에 헤딩값 넣어둠
                    self.global_path_msg.poses.append(read_pose)
            rospy.loginfo(f"Loaded path for mission {mission_num}")
        except FileNotFoundError:
            rospy.logerr(f"Path file for mission {mission_num} not found: {file_path}")

if __name__ == '__main__':
    try:
        test_track = planning()
    except rospy.ROSInterruptException:
        pass
