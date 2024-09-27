#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, sqrt, pow, atan2, pi
from geometry_msgs.msg import Point
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Int16, Float32
import numpy as np
from tf.transformations import euler_from_quaternion
from ushmd.msg import mission, obstacle  # 사용자 정의 메시지 임포트

class pure_pursuit:
    def __init__(self):
        rospy.init_node('control', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/obstacle_info", obstacle, self.yolo_callback)
        rospy.Subscriber("/lidar_detected", obstacle, self.lidar_callback)
        rospy.Subscriber("/mission", mission, self.mission_callback)
        rospy.Subscriber("traffic_light_color", Int16, self.traffic_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback)

        self.speed_cmd_pub = rospy.Publisher('/target_speed', Float32, queue_size=10)
        self.angle_cmd_pub = rospy.Publisher('/target_angle', Float32, queue_size=10)

        self.speed_cmd_msg = Float32()
        self.angle_cmd_msg = Float32()

        self.is_path = False
        self.is_odom = False
        self.is_yolo = False

        self.traffic_color = ""
        self.obstacle = obstacle()
        self.forward_point = Point()
        self.current_postion = Point()
        self.is_look_forward_point = False


        self.is_lidar_obstacle = False

        # 최대 조향각 설정
        self.max_steering_angle = 22.5

        # 노드별 파라미터 가져오기
        self.vehicle_length = rospy.get_param('~vehicle_length', 0.7)  # 기본값 1.1
        self.lfd = rospy.get_param('~lfd', 5)  # 기본값 2.5
        self.default_speed = rospy.get_param('~default_speed', 5)  # 기본값 5

        rospy.loginfo(f"Vehicle length: {self.vehicle_length}, LFD: {self.lfd}, Default speed: {self.default_speed}")

        self.mission = mission()

        # 미션별 상태 변수 초기화
        self.mission5_active = False
        self.mission5_phase = 0
        self.mission_start_time_5 = None

        self.mission7_active = False
        self.mission7_phase = 0
        self.mission_start_time_7 = None

        self.missions_active = False  # 미션 수행 중 여부를 나타내는 플래그

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom:
                vehicle_position = self.current_postion
                self.is_look_forward_point = False

                # 좌표 변환 행렬 계산
                translation = [vehicle_position.x, vehicle_position.y]
                t = np.array([
                    [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
                    [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
                    [0, 0, 1]
                ])
                det_t = np.array([
                    [t[0][0], t[1][0], -(t[0][0]*translation[0]+t[1][0]*translation[1])],
                    [t[0][1], t[1][1], -(t[0][1]*translation[0]+t[1][1]*translation[1])],
                    [0, 0, 1]
                ])

                # 경로에서 전방 주시 포인트 찾기
                for num, i in enumerate(self.path.poses):
                    path_point = i.pose.position

                    global_path_point = [path_point.x, path_point.y, 1]
                    local_path_point = det_t.dot(global_path_point)
                    if local_path_point[0] > 0:
                        dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                        if dis >= self.lfd:
                            self.forward_point = path_point
                            self.is_look_forward_point = True
                            break

                theta = atan2(local_path_point[1], local_path_point[0])

                # 현재 진행 중인 미션이 있는지 확인
                if self.missions_active:
                    # 미션 수행 로직
                    if self.mission5_active or self.mission.num == 5:
                        self.perform_mission_5()  # 후진 T자 주차 및 전진 T자 주차
                    elif self.mission7_active or self.mission.num == 7:
                        self.perform_mission_7()  # 후진 평행 주차 및 전진 평행 주차
                else:
                    # 미션이 끝나면 다시 경로를 따라가는 주행 로직 실행
                    if self.is_look_forward_point:
                        self.steering_angle_control(theta)
                        self.velocity_control()
                    else:
                        rospy.loginfo_once("No forward point found")
                        self.stop_vehicle()

                # 퍼블리시
                self.speed_cmd_pub.publish(self.speed_cmd_msg)
                self.angle_cmd_pub.publish(self.angle_cmd_msg)

            self.is_path = self.is_odom = False
            rate.sleep()

    def steering_angle_control(self, theta):
        # 조향각 계산 및 제한
        self.angle_cmd_msg.data = atan2(2.0 * self.vehicle_length * sin(theta), self.lfd) * 180 / pi  # degree로 변환

        if abs(self.angle_cmd_msg.data) > self.max_steering_angle:
            self.angle_cmd_msg.data = self.max_steering_angle * (self.angle_cmd_msg.data / abs(self.angle_cmd_msg.data))

    def velocity_control(self):
        # 속도 계산 및 퍼블리시
        normalized_steer = abs(self.angle_cmd_msg.data) / self.max_steering_angle

        if self.mission.num == 0:
            self.speed_cmd_msg.data = self.default_speed * (1 - 0.6 * normalized_steer) * (1 - (self.obstacle.collision_probability/100)) * (1-int(self.is_lidar_obstacle))
        else:
            self.speed_cmd_msg.data = self.default_speed * (1 - 0.6 * normalized_steer)

    def perform_mission_5(self): 
        # 후진 및 전진 T자 주차를 모두 처리하는 미션
        if not self.mission5_active:
            self.mission_start_time_5 = rospy.Time.now()
            self.mission5_phase = 1
            self.mission5_active = True
            self.missions_active = True  # 미션이 활성화됨
            rospy.loginfo("Mission 5 started.")

        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.mission_start_time_5).to_sec()

        if self.mission5_phase == 1:
            if elapsed_time < 10.0:
                # 첫 번째 단계: 10초 동안 후진, 조향각 0도
                self.speed_cmd_msg.data = -5.0
                self.angle_cmd_msg.data = 0.0
            else:
                self.stop_vehicle()
                self.mission5_phase = 2
                self.mission_start_time_5 = current_time
                rospy.loginfo("10초 동안 일직선으로 후진 완료")
        elif self.mission5_phase == 2:
            if elapsed_time < 2.0: 
                self.stop_vehicle()
            else:
                self.mission5_phase = 3
                self.mission_start_time_5 = current_time
                rospy.loginfo("2초간 잠시 정지")        
        elif self.mission5_phase == 3:
            if elapsed_time < 5.0:
                # 두 번째 단계: 5초 동안 후진, 조향각 22.5도
                self.speed_cmd_msg.data = -5.0
                self.angle_cmd_msg.data = 22.5
            else:
                self.stop_vehicle()
                self.mission5_phase = 4
                self.mission_start_time_5 = current_time
                rospy.loginfo("5초동안 뒤로 좌회전 완료")
        elif self.mission5_phase == 4:
            if elapsed_time < 2.0:
                self.stop_vehicle()
            else:
                self.mission5_phase = 5
                self.mission_start_time_5 = current_time
                rospy.loginfo("2초간 잠시 정지")
        elif self.mission5_phase == 5:
            if elapsed_time < 5.0:
                # 세 번째 단계: 5초 동안 후진, 조향각 0도
                self.speed_cmd_msg.data = -5.0
                self.angle_cmd_msg.data = 0.0
            else:
                self.stop_vehicle()
                self.mission5_phase = 6
                self.mission_start_time_5 = current_time
                rospy.loginfo("5초 동안 일직선으로 후진 완료")
        elif self.mission5_phase == 6:
            if elapsed_time < 2.0:
                self.stop_vehicle()
            else:
                self.mission5_phase = 7
                self.mission_start_time_5 = current_time
                rospy.loginfo("2초간 잠시 정지")
        elif self.mission5_phase == 7:
            if elapsed_time < 5.0:
                # 네 번쩨 단계 : 5초동안 전진, 조향각 0도
                self.speed_cmd_msg.data = 5.0
                self.angle_cmd_msg.data = 0.0
            else:
                self.stop_vehicle()
                self.mission5_phase = 8
                self.mission_start_time_5 = current_time
                rospy.loginfo("5초 동안 일직선으로 전진 완료")
        elif self.mission5_phase == 8:
            if elapsed_time < 2.0:
                self.stop_vehicle()
            else:
                self.mission5_phase = 9
                self.mission_start_time_5 = current_time
                rospy.loginfo("2초간 잠시 정지")
        elif self.mission5_phase == 9:
            if elapsed_time < 5.0:
                # 다섯 번째 단계 : 5초 동안 전진, 조향각 -22.5도
                self.speed_cmd_msg.data = 5.0
                self.angle_cmd_msg.data = -22.5
            else:
                self.stop_vehicle()
                self.mission5_phase = 10
                self.mission_start_time_5 = current_time
                rospy.loginfo("5초동안 앞으로 우회전 완료")
        elif self.mission5_phase == 10:
            if elapsed_time < 2.0:
                self.stop_vehicle()
            else:
                self.mission5_phase = 11
                self.mission_start_time_5 = current_time
                rospy.loginfo("2초간 잠시 정지")
        elif self.mission5_phase == 11:
            if elapsed_time < 5.0:
                # 여섯 번 째 단계 : 5초 동안 전진, 조향각 0도
                self.speed_cmd_msg.data = 5.0
                self.angle_cmd_msg.data = 0.0
            else:
                self.stop_vehicle()
                self.mission5_phase = 0
                self.mission5_active = False
                self.missions_active = False 
                rospy.loginfo("주차 완료")
        else:
            self.stop_vehicle()
            self.mission5_phase = 0
            self.mission5_active = False
            self.missions_active = False
            rospy.logwarn("예외")

    def perform_mission_7(self):  # 아직 구현 안ㅇ함
        # 후진 평행 주차 및 전진 평행 주차를 모두 처리하는 미션
        if not self.mission7_active:
            self.mission_start_time_7 = rospy.Time.now()
            self.mission7_phase = 1
            self.mission7_active = True
            self.missions_active = True  # 미션이 활성화됨
            rospy.loginfo("Mission 7 started.")

        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.mission_start_time_7).to_sec()

        if self.mission7_phase == 1:
            if elapsed_time < 5.0:
                # 첫 번째 단계: 5초 동안 후진, 조향각 -22.5도 (우회전)
                self.speed_cmd_msg.data = -5.0
                self.angle_cmd_msg.data = -22.5
            else:
                self.mission7_phase = 2
                self.mission_start_time_7 = current_time
                rospy.loginfo("Mission 7 Phase 1 complete.")
        elif self.mission7_phase == 2:
            if elapsed_time < 5.0:
                # 두 번째 단계: 5초 동안 후진, 조향각 22.5도 (좌회전)
                self.speed_cmd_msg.data = -5.0
                self.angle_cmd_msg.data = 22.5
            else:
                self.stop_vehicle()
                self.mission7_phase = 0
                self.mission7_active = False
                self.missions_active = False  # 미션 완료 후 pure pursuit 로직으로 복귀 가능
                rospy.loginfo("Mission 7 completed.")
        else:
            self.stop_vehicle()
            self.mission7_phase = 0
            self.mission7_active = False
            self.missions_active = False
            rospy.logwarn("Mission 7 interrupted.")

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def yolo_callback(self, msg):
        self.is_yolo = True
        self.obstacle = msg

    def mission_callback(self, msg):
        self.mission = msg

    def lidar_callback(self, msg):
        self.is_lidar_obstacle = msg.data

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y
        self.odom_msg = msg

    def traffic_callback(self, msg):
        self.traffic_color = msg.data

    def stop_vehicle(self):
        self.speed_cmd_msg.data = 0.0
        self.angle_cmd_msg.data = 0.0
        self.speed_cmd_pub.publish(self.speed_cmd_msg)
        self.angle_cmd_pub.publish(self.angle_cmd_msg)
        rospy.loginfo("Vehicle stopped")

if __name__ == '__main__':
    try:
        control = pure_pursuit()
    except rospy.ROSInterruptException:
        pass
