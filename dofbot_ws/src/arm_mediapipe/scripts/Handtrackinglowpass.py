#!/usr/bin/env python3
# encoding: utf-8
import threading
import numpy as np
import cv2 as cv
from media_library import *
from time import sleep, time
import rospy
from arm_mediapipe.msg import PIDParams

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.prev_error = 0
        self.last_time = None

    def update_param(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def calculate(self, error):
        current_time = time()
        delta_time = current_time - self.last_time if self.last_time else 0.01
        self.last_time = current_time

        self.integral += error * delta_time
        derivative = (error - self.prev_error) / delta_time if delta_time > 0 else 0
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
        self.dead_zone = 20

        self.joint_angles = [90, 135, 0, 45, 90, 90]
        self.pid_x = PIDController(0.05, 0.0001, 0.02)
        self.pid_y = PIDController(0.03, 0.00005, 0.01)

        self.alpha = 0.5  # Low-pass filter alpha
        self.prev_indexX = None
        self.prev_indexY = None

        rospy.Subscriber("/pid_param", PIDParams, self.update_pid)
        self.media_ros.pub_arm(self.joint_angles)

    def update_pid(self, msg):
        self.pid_x.update_param(msg.Kp_x, msg.Ki_x, msg.Kd_x)
        self.pid_y.update_param(msg.Kp_y, msg.Ki_y, msg.Kd_y)
        rospy.loginfo("Updated PID parameters.")

    def process(self, frame):
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        if len(lmList) != 0 and self.event.is_set():
            threading.Thread(target=self.pid_control_thread, args=(bbox,)).start()

        self.cTime = time()
        fps = 1 / (self.cTime - self.pTime) if self.pTime != 0 else 0
        self.pTime = self.cTime
        cv.putText(frame, f"FPS: {int(fps)}", (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return frame

    def pid_control_thread(self, bbox):
        self.event.clear()
        try:
            indexX = (bbox[0] + bbox[2]) / 2
            indexY = (bbox[1] + bbox[3]) / 2

            # Low-pass filter
            if self.prev_indexX is None:
                self.prev_indexX = indexX
                self.prev_indexY = indexY
            indexX = self.alpha * indexX + (1 - self.alpha) * self.prev_indexX
            indexY = self.alpha * indexY + (1 - self.alpha) * self.prev_indexY
            self.prev_indexX = indexX
            self.prev_indexY = indexY

            errorX = indexX - self.centerX
            errorY = indexY - self.centerY

            outputX = self.pid_x.calculate(errorX) if abs(errorX) > self.dead_zone else 0
            outputY = self.pid_y.calculate(errorY) if abs(errorY) > self.dead_zone else 0

            self.joint_angles[0] -= outputX
            self.joint_angles[1] -= outputY * 0.6
            self.joint_angles[2] += outputY * 0.3
            self.joint_angles[3] -= outputY * 0.3

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