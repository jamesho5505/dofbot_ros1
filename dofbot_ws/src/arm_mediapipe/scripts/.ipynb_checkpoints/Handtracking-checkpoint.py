#!/usr/bin/env python3
# encoding: utf-8
'''
這份程式是一個基於 PID 控制器 的手部追蹤系統，
能讓機械手臂根據畫面中手的位置自動調整 joint1~joint4，
把手保持在視野正中心。
'''
import threading
import numpy as np
import cv2 as cv
from media_library import *
from time import sleep, time
import rospy

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.prev_error = 0
        self.last_time = None
    def calculate(self, error):
        current_time = time()
        delta_time = current_time - self.last_time if self.last_time else 0.01
        self.last_time = current_time
        
        self.integral += error * delta_time
        derivative = (error - self.prev_error)/delta_time if delta_time > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error
        return output
        
class HandCtrlArmCenterPID:
    def __init__(self):
        self.media_ros = Media_ROS()
        self.hand_detector = HandDetector()
        self.pTime = 0
        self.Joy_active = True
        self.event = threading.Event()
        self.event.set()

        self.centerX = 320
        self.centerY = 240
        self.dead_zone = 20 # 偏差小於 20 pixel 不動作

        self.joint_angles = [90, 135, 0, 45, 90, 90]  # joint1~6 初始值
        self.pid_x = PIDController(0.05, 0.0001, 0.05)  # joint1 控制器
        self.pid_y = PIDController(0.03, 0.00005, 0.01)  # joint2~4 控制器

        self.media_ros.pub_arm(self.joint_angles) # 初始化手臂姿勢

    def process(self, frame):
        # 不鏡像畫面（方向正確）
        # frame = cv.flip(frame, 1)
        if self.Joy_active:
            frame, lmList, bbox = self.hand_detector.findHands(frame)
            if len(lmList) != 0:
                threading.Thread(target=self.pid_control_thread, args=(bbox,)).start()

        # FPS 顯示
        self.cTime = time()
        fps = 1 / (self.cTime - self.pTime) if self.pTime != 0 else 0
        self.pTime = self.cTime
        cv.putText(frame, f"FPS: {int(fps)}", (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv.putText(frame, f"errorX: {int(errorX)}", (20, 60), ...)

        return frame

    def pid_control_thread(self, bbox):
        self.event.clear()
        try:
            # 計算手掌中心座標
            indexX = (bbox[0] + bbox[2]) / 2
            indexY = (bbox[1] + bbox[3]) / 2

            errorX = indexX - self.centerX
            errorY = indexY - self.centerY

            outputX = self.pid_x.calculate(errorX) if abs(errorX) > self.dead_zone else 0
            outputY = self.pid_y.calculate(errorY) if abs(errorY) > self.dead_zone else 0

            # 控制 joint1（水平旋轉）
            self.joint_angles[0] -= outputX  # 左右反向調整
            # 控制 joint2~4（抬高/降低）
            self.joint_angles[1] -= outputY * 0.6
            self.joint_angles[2] += outputY * 0.3
            self.joint_angles[3] -= outputY * 0.3

            print("joint:", self.joint_angles)

            # 限制 joint 範圍
            for i in range(4):
                self.joint_angles[i] = max(0, min(180, self.joint_angles[i]))

            self.media_ros.pub_arm(self.joint_angles)
            sleep(0.05)
        finally:
            self.event.set()

if __name__ == '__main__':
    rospy.init_node('HandCtrlArm_center_pid_node', anonymous=True)
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))

    ctrl_arm = HandCtrlArmCenterPID()
    while capture.isOpened():
        ret, frame = capture.read()
        if not ret:
            break
        frame = ctrl_arm.process(frame)
        cv.imshow('frame', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    capture.release()
    cv.destroyAllWindows()