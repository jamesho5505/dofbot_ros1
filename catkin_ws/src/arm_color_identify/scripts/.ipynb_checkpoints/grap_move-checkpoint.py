#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
from time import sleep


class grap_move:
    def __init__(self):
        # 设置移动状态
        self.move_status = True
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device()
        # 夹爪加紧角度
        self.grap_joint = 135

    def move(self, joints, joints_down):
        '''
        移动过程
        :param joints: 移动到物体位置的各关节角度
        :param joints_up: 机械臂抬起各关节角度
        :param color_angle: 移动到对应垃圾桶的角度
        '''
        joints_uu = [90, 80, 50, 50, 265, self.grap_joint]
        # 抬起
        joints_up = [joints_down[0], 80, 50, 50, 265, 30]
        # 架起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1500)
        sleep(1.5)
        # 松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(joints, 500)
        sleep(0.5)
        # 进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)
        # 架起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
        # 抬起至对应位置上方
        self.arm.Arm_serial_servo_write(1, joints_down[0], 500)
        sleep(0.5)
        # 抬起至对应位置
        self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        sleep(1)
        # 释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # 抬起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)


    def arm_run(self, name, joints):
        '''
        机械臂移动函数
        :param name:识别的颜色
        :param joints: 反解求得的各关节角度
        '''
        if name == "red" and self.move_status == True:
            # 此处设置,需执行完本次操作,才能向下运行
            self.move_status = False
            print ("red")
            # print (joints[0], joints[1], joints[2], joints[3], joints[4])
            # 获得目标关节角
            joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
            # 移动到垃圾桶上方对应姿态
#             joints_down = [45, 80, 35, 40, 265, self.grap_joint]
            # 移动到垃圾桶位置放下对应姿态
            joints_down = [45, 50, 20, 60, 265, self.grap_joint]
            # 移动
            self.move(joints, joints_down)
            # 移动完毕
            self.move_status = True
        if name == "blue" and self.move_status == True:
            self.move_status = False
            # print ("blue")
            # print (joints[0], joints[1], joints[2], joints[3], joints[4])
            joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
#             joints_down = [27, 110, 0, 40, 265, self.grap_joint]
            joints_down = [27, 75, 0, 50, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
        if name == "green" and self.move_status == True:
            self.move_status = False
            # print ("green")
            # print (joints[0], joints[1], joints[2], joints[3], joints[4])
            joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
#             joints_down = [152, 110, 0, 40, 265, self.grap_joint]
            joints_down = [147, 75, 0, 50, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
        if name == "yellow" and self.move_status == True:
            self.move_status = False
            # print ("yellow")
            # print (joints[0], joints[1], joints[2], joints[3], joints[4])
            joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
#             joints_down = [137, 80, 35, 40, 265, self.grap_joint]
            joints_down = [133, 50, 20, 60, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
