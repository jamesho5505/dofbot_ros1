# !/usr/bin/env python
# coding: utf-8
import random
import Arm_Lib
import cv2 as cv
import numpy as np
import tkinter as tk

class Arm_Calibration:
    def __init__(self):
        self.image = None
        self.threshold_num = 130
        self.xy=[90,135]

        self.arm = Arm_Lib.Arm_Device()

    def calibration_map(self, image,xy=None, threshold_num=130):
        if xy!=None: self.xy=xy

        joints_init = [self.xy[0], self.xy[1], 0, 0, 90, 0]

        self.arm.Arm_serial_servo_write6_array(joints_init, 1500)
        self.image = image
        self.threshold_num = threshold_num

        dp = []
        h, w = self.image.shape[:2]

        contours = self.Morphological_processing()

        for i, c in enumerate(contours):

            area = cv.contourArea(c)

            if h * w / 2 < area < h * w:

                mm = cv.moments(c)
                if mm['m00'] == 0:
                    continue
                cx = mm['m10'] / mm['m00']
                cy = mm['m01'] / mm['m00']

                cv.drawContours(self.image, contours, i, (255, 255, 0), 2)

                dp = np.squeeze(cv.approxPolyDP(c, 100, True))

                cv.circle(self.image, (np.int(cx), np.int(cy)), 5, (0, 0, 255), -1)
        return dp, self.image

    def Morphological_processing(self):
        gray = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)
        gray = cv.GaussianBlur(gray, (5, 5), 1)
        ref, threshold = cv.threshold(gray, self.threshold_num, 255, cv.THRESH_BINARY)
        kernel = np.ones((3, 3), np.uint8)
        blur = cv.morphologyEx(threshold, cv.MORPH_OPEN, kernel, iterations=4)
        mode = cv.RETR_EXTERNAL
        method = cv.CHAIN_APPROX_NONE
        contours, hierarchy = cv.findContours(blur, mode, method)
        return contours

    def Perspective_transform(self, dp, image):

        if len(dp)!=4: return
        upper_left = []
        lower_left = []
        lower_right = []
        upper_right = []
        for i in range(len(dp)):
            if dp[i][0]<320 and dp[i][1]<240:
                upper_left=dp[i]
            if dp[i][0]<320 and dp[i][1]>240:
                lower_left=dp[i]
            if dp[i][0]>320 and dp[i][1]>240:
                lower_right=dp[i]
            if dp[i][0]>320 and dp[i][1]<240:
                upper_right=dp[i]

        pts1 = np.float32([upper_left, lower_left, lower_right, upper_right])

        pts2 = np.float32([[0, 0], [0, 480], [640, 480], [640, 0]])

        M = cv.getPerspectiveTransform(pts1, pts2)

        Transform_img = cv.warpPerspective(image, M, (640, 480))
        return Transform_img

    def get_hsv(self, img):
        H = [];S = [];V = []
        img = cv.resize(img, (640, 480), )

        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        cv.rectangle(img, (290, 280), (350, 340), (0, 255, 0), 2)

        for i in range(290, 350):
            for j in range(280, 340):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])

        H_min = min(H);H_max = max(H)
        S_min = min(S);S_max = max(S)
        V_min = min(V);V_max = max(V)

        if H_max + 2 > 255:H_max = 255
        else:H_max += 2
        if H_min - 2 < 0:H_min = 0
        else:H_min -= 2
        if S_min-10<0:S_min=0
        else:S_min -= 15
        if V_min-10<0:V_min=0
        else:V_min -= 15
#         S_max = 255; V_max = 255
        lowerb = 'lowerb : (' + str(H_min) + ' ,' + str(S_min) + ' ,' + str(V_min)+ ')'
        upperb = 'upperb : (' + str(H_max) + ' ,' + str(S_max) + ' ,' + str(V_max)+ ')'
        txt1 = 'Learning ...'
        txt2 = 'OK !!!'
        color = [[random.randint(0, 255) for _ in range(3)] for _ in range(255)]
        if S_min<5 or V_min <5: cv.putText(img, txt1, (230, 270), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        else: cv.putText(img, txt2, (270, 270), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv.putText(img, lowerb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv.putText(img, upperb, (150, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        hsv_range = ((int(H_min), int(S_min), int(V_min)),(int(H_max), int(S_max), int(V_max)))
        return img, hsv_range


class update_hsv:
    def __init__(self):
        self.image = None
        self.binary = None

    def Image_Processing(self, hsv_range):

        (lowerb, upperb) = hsv_range

        color_mask = self.image.copy()

        hsv_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)

        color = cv.inRange(hsv_img, lowerb, upperb)

        color_mask[color == 0] = [0, 0, 0]

        gray_img = cv.cvtColor(color_mask, cv.COLOR_RGB2GRAY)

        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))

        dst_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)

        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)

        # _, contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) #python2
        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)  # python3
        return contours, binary

    def draw_contours(self, hsv_name, contours):

        for i, cnt in enumerate(contours):

            mm = cv.moments(cnt)
            if mm['m00'] == 0:
                continue
            cx = mm['m10'] / mm['m00']
            cy = mm['m01'] / mm['m00']

            area = cv.contourArea(cnt)

            if area > 800:

                (x, y) = (np.int32(cx), np.int32(cy))

                cv.circle(self.image, (x, y), 5, (0, 0, 255), -1)

                rect = cv.minAreaRect(cnt)

                box = cv.boxPoints(rect)

                box = np.int64(box)

                cv.drawContours(self.image, [box], 0, (255, 0, 0), 2)
                cv.putText(self.image, hsv_name, (int(x - 15), int(y - 15)),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

    def get_contours(self, img, color_name, hsv_msg, color_hsv):
        binary = None

        self.image = cv.resize(img, (640, 480), )
        for key, value in color_hsv.items():

            if color_name == key:
                color_contours, binary = self.Image_Processing(hsv_msg)
            else:
                color_contours, _ = self.Image_Processing(color_hsv[key])

            self.draw_contours(key, color_contours)
        return self.image, binary

def write_HSV(wf_path, dict):
    with open(wf_path, "w") as wf:
        for key, value in dict.items():
            wf_str = '"' + key + '": [' + str(value[0][0]) + ', ' + str(
                value[0][1]) + ', ' + str(value[0][2]) + ', ' + str(
                value[1][0]) + ', ' + str(value[1][1]) + ', ' + str(
                value[1][2]) + '], ' + '\n'
            wf.write(wf_str)
        wf.flush()


def read_HSV(rf_path, dict):
    rf = open(rf_path, "r+")
    for line in rf.readlines():
        list = []
        name = None
        aa = line.find('"')
        bb = line.rfind('"')
        cc = line.find('[')
        dd = line.rfind(']')
        if aa >= 0 and bb >= 0: name = line[aa + 1:bb]
        if cc >= 0 and dd >= 0:
            rf_str = line[cc + 1:dd].split(',')
            for index, i in enumerate(rf_str): list.append(int(i))
            if name != None: dict[name] = ((list[0], list[1], list[2]), (list[3], list[4], list[5]))
    rf.flush()


def write_XYT(wf_path, xy, thresh):
    with open(wf_path, "w") as wf:
        str1 = 'x' + '=' + str(xy[0])
        str2 = 'y' + '=' + str(xy[1])
        str3 = 'thresh' + '=' + str(thresh)
        wf_str = str1 + '\n' + str2 + '\n' + str3
        wf.write(wf_str)
        wf.flush()


def read_XYT(rf_path):
    dict = {}
    rf = open(rf_path, "r+")
    for line in rf.readlines():
        index = line.find('=')
        dict[line[:index]] = line[index + 1:]
    xy = [int(dict['x']), int(dict['y'])]
    thresh = int(dict['thresh'])
    rf.flush()
    return xy, thresh


def write_PIDT(wf_path, PID, time_config):
    with open(wf_path, "w") as wf:
        str1 = 'P' + '=' + str(PID[0])
        str2 = 'I' + '=' + str(PID[1])
        str3 = 'D' + '=' + str(PID[2])
        str4 = 'T1' + '=' + str(time_config[1])
        str5 = 'T2' + '=' + str(time_config[1])
        str6 = 'T3' + '=' + str(time_config[2])
        wf_str = str1 + '\n' + str2 + '\n' + str3 + '\n' + str4 + '\n' + str5 + '\n' + str6
        wf.write(wf_str)
        wf.flush()


def read_PIDT(rf_path):
    dict = {}
    rf = open(rf_path, "r+")
    for line in rf.readlines():
        index = line.find('=')
        dict[line[:index]] = line[index + 1:]
    PID = [int(dict['P']), int(dict['I']), int(dict['D'])]
    time_config = [int(dict['T1']), int(dict['T2']), int(dict['T3'])]
    rf.flush()
    return PID, time_config
