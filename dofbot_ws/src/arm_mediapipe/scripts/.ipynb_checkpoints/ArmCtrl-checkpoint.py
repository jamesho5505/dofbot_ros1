#!/usr/bin/env python3
# encoding: utf-8
import threading
import numpy as np
import cv2 as cv
from media_library import *
from time import sleep, time
import rospy

class HandCtrlArm:
    def __init__(self):
        self.media_ros = Media_ROS()
        self.hand_detector = HandDetector()
        self.pTime = 0
        self.Joy_active = True
        self.event = threading.Event()
        self.event.set()

        # 初始化位置與濾波參數
        self.prev_indexX = None
        self.prev_indexY = None
        self.alpha = 0.4  # 平滑參數

        # 初始化手臂姿勢
        self.media_ros.pub_arm([90, 135, 0, 45, 90, 90])

    def process(self, frame):
        # 不鏡像畫面（方向正確）
        # frame = cv.flip(frame, 1)

        if self.Joy_active:
            frame, lmList, bbox = self.hand_detector.findHands(frame)
            if len(lmList) != 0:
                threading.Thread(target=self.arm_ctrl_threading, args=(lmList, bbox)).start()

        # FPS 顯示
        self.cTime = time()
        fps = 1 / (self.cTime - self.pTime) if self.pTime != 0 else 0
        self.pTime = self.cTime
        cv.putText(frame, f"FPS: {int(fps)}", (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        return frame

    def arm_ctrl_threading(self, lmList, bbox):
        if not self.event.is_set():
            return

        self.event.clear()
        try:
            fingers = self.hand_detector.fingersUp(lmList)
            self.hand_detector.draw = True

            # 計算手指角度（控制夾爪）
            angle = self.hand_detector.ThumbTOforefinger(lmList)
            gripper = np.interp(angle, [0, 70], [185, 20])

            # 計算手掌中心座標
            indexX = (bbox[0] + bbox[2]) / 2
            indexY = (bbox[1] + bbox[3]) / 2

            # 平滑處理
            if self.prev_indexX is None:
                self.prev_indexX = indexX
                self.prev_indexY = indexY
            else:
                indexX = self.alpha * indexX + (1 - self.alpha) * self.prev_indexX
                indexY = self.alpha * indexY + (1 - self.alpha) * self.prev_indexY
                self.prev_indexX = indexX
                self.prev_indexY = indexY

            # 限制 indexY 範圍
            indexY = max(200, min(400, indexY))

            # 映射成 joint 角度
            joint2 = -0.35 * indexY + 160
            joint3 = 0.05 * indexY + 25
            joint4 = -0.125 * indexY + 85

            if 300 < indexX < 340:
                joint1 = 90
            else:
                joint1 = -0.2 * indexX + 180

            # 限制 joint 安全範圍
            joint1 = max(10, min(170, joint1))
            joint2 = max(0, min(180, joint2))
            joint3 = max(0, min(180, joint3))
            joint4 = max(0, min(180, joint4))
            gripper = max(20, min(185, gripper))

            # 若移動明顯才下指令
            if abs(indexX - self.prev_indexX) > 5 or abs(indexY - self.prev_indexY) > 5:
                print(f"indexX: {int(indexX)}, joint1: {int(joint1)}")
                self.media_ros.pub_arm([joint1, joint2, joint3, joint4, 90, gripper])

            sleep(0.1)
        finally:
            self.event.set()

if __name__ == '__main__':
    rospy.init_node('HandCtrlArm_node', anonymous=True)
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))

    ctrl_arm = HandCtrlArm()
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
