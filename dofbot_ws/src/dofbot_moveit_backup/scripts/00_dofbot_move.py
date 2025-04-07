#!/usr/bin/env python3
# coding: utf-8
'''
執行此段代碼-->訂閱發佈話題為 "joint_states" 的各關節角度，驅動真機移動
Execute this code --> Subscribe and publish the joint angles of the topic "joint_states" to drive the real machine to move
'''
import rospy
import Arm_Lib
from math import pi
from sensor_msgs.msg import JointState

# 弧度轉換成角度
RA2DE = 180 / pi

def topic(msg):
    # 如果不是該話題的數據直接返回
    if not isinstance(msg, JointState):
        return

    # 定義關節角度容器，最後一個是夾爪的角度，預設夾爪不動為 90
    joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    print("Received msg.position:", msg.position)
    print("Length of joints:", len(joints))
    print("Length of msg.position:", len(msg.position))

    # 將接收到的弧度轉換成角度
    for i in range(min(len(joints), len(msg.position))):
        if i == 5:
            joints[i] = (msg.position[i] * 116) + 180
        else:
            joints[i] = (msg.position[i] * RA2DE) + 90

    # 調用驅動函式
    sbus.Arm_serial_servo_write6_array(joints, 100)

if __name__ == '__main__':
    sbus = Arm_Lib.Arm_Device()
    rospy.init_node("ros_dofbot")
    subscriber = rospy.Subscriber("/joint_states", JointState, topic)
    rate = rospy.Rate(2)
    rospy.spin()
