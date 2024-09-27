#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32

class pure_pursuit:
    def __init__(self):
        rospy.init_node('control', anonymous=True)

        self.speed_cmd_pub = rospy.Publisher('/target_speed', Float32, queue_size=10)
        self.angle_cmd_pub = rospy.Publisher('/target_angle', Float32, queue_size=10)

        self.speed_cmd_msg = Float32()
        self.angle_cmd_msg = Float32()

        # 미션 5 하드 코딩 테스트 바로 실행
        self.run_mission_5()

    def run_mission_5(self):
        # 미션 5 하드 코딩된 동작 수행
        mission_phase = 1
        mission_start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            elapsed_time = current_time - mission_start_time.to_sec()

            if mission_phase == 1:
                if elapsed_time < 10.0:
                    # 첫 번째 단계: 10초 동안 후진, 조향각 0도
                    self.speed_cmd_msg.data = -5.0
                    self.angle_cmd_msg.data = 0.0
                else:
                    self.stop_vehicle()
                    mission_phase = 2
                    mission_start_time = rospy.Time.now()
                    rospy.loginfo("10초 동안 후진 완료")
            elif mission_phase == 2:
                if elapsed_time < 2.0:
                    self.stop_vehicle()  # 2초 정지
                else:
                    mission_phase = 3
                    mission_start_time = rospy.Time.now()
                    rospy.loginfo("2초 정지 완료")
            elif mission_phase == 3:
                if elapsed_time < 5.0:
                    # 두 번째 단계: 5초 동안 후진, 조향각 22.5도
                    self.speed_cmd_msg.data = -5.0
                    self.angle_cmd_msg.data = 22.5
                else:
                    self.stop_vehicle()
                    mission_phase = 4
                    mission_start_time = rospy.Time.now()
                    rospy.loginfo("5초 동안 후진 완료")
            elif mission_phase == 4:
                if elapsed_time < 2.0:
                    self.stop_vehicle()  # 2초 정지
                else:
                    mission_phase = 5
                    mission_start_time = rospy.Time.now()
                    rospy.loginfo("2초 정지 완료")
            elif mission_phase == 5:
                if elapsed_time < 5.0:
                    # 세 번째 단계: 5초 동안 후진, 조향각 0도
                    self.speed_cmd_msg.data = -5.0
                    self.angle_cmd_msg.data = 0.0
                else:
                    self.stop_vehicle()
                    mission_phase = 6
                    mission_start_time = rospy.Time.now()
                    rospy.loginfo("5초 동안 후진 완료")
            elif mission_phase == 6:
                if elapsed_time < 2.0:
                    self.stop_vehicle()  # 2초 정지
                else:
                    mission_phase = 7
                    mission_start_time = rospy.Time.now()
                    rospy.loginfo("2초 정지 완료")
            elif mission_phase == 7:
                if elapsed_time < 5.0:
                    # 네 번째 단계: 5초 동안 전진, 조향각 0도
                    self.speed_cmd_msg.data = 5.0
                    self.angle_cmd_msg.data = 0.0
                else:
                    self.stop_vehicle()
                    mission_phase = 8
                    mission_start_time = rospy.Time.now()
                    rospy.loginfo("5초 동안 전진 완료")
            elif mission_phase == 8:
                if elapsed_time < 2.0:
                    self.stop_vehicle()  # 2초 정지
                else:
                    mission_phase = 9
                    mission_start_time = rospy.Time.now()
                    rospy.loginfo("2초 정지 완료")
            elif mission_phase == 9:
                if elapsed_time < 5.0:
                    # 다섯 번째 단계: 5초 동안 전진, 조향각 -22.5도
                    self.speed_cmd_msg.data = 5.0
                    self.angle_cmd_msg.data = -22.5
                else:
                    self.stop_vehicle()
                    mission_phase = 10
                    mission_start_time = rospy.Time.now()
                    rospy.loginfo("5초 동안 전진 완료")
            elif mission_phase == 10:
                if elapsed_time < 2.0:
                    self.stop_vehicle()  # 2초 정지
                else:
                    mission_phase = 11
                    mission_start_time = rospy.Time.now()
                    rospy.loginfo("2초 정지 완료")
            elif mission_phase == 11:
                if elapsed_time < 5.0:
                    # 여섯 번째 단계: 5초 동안 전진, 조향각 0도
                    self.speed_cmd_msg.data = 5.0
                    self.angle_cmd_msg.data = 0.0
                else:
                    self.stop_vehicle()
                    rospy.loginfo("Mission 5 완료")
                    break
            else:
                self.stop_vehicle()
                rospy.logwarn("Mission 5 중단됨.")
                break

            # 퍼블리시: 루프마다 퍼블리시
            self.speed_cmd_pub.publish(self.speed_cmd_msg)
            self.angle_cmd_pub.publish(self.angle_cmd_msg)

    # 차량 정지 함수
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
