#!/usr/bin/env python3
# coding: utf-8
import os
import time
import rospy
import Arm_Lib
import cv2 as cv
import numpy as np
import tensorflow as tf
from numpy import random
from timeit import default_timer as timer
from tensorflow.compat.v1.keras import backend as K
from tensorflow.keras.layers import Input
from tensorflow.keras.layers import Lambda
from tensorflow.keras.models import Model
from PIL import ImageFont, ImageDraw, Image
from nets.yolo4_tiny import yolo_body, yolo_eval
from utils.utils import letterbox_image
from garbage_grap_move import garbage_grap_move
from arm_info.srv import kinemarics, kinemaricsRequest, kinemaricsResponse

gpus = tf.config.experimental.list_physical_devices(device_type='GPU')
for gpu in gpus: tf.config.experimental.set_memory_growth(gpu, True)

class garbage_identify:
    def __init__(self):
        self.score = 0.5
        self.iou = 0.3
        self.eager = False
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device()
        self.anchors_path = '/root/catkin_ws/src/arm_garbage_identify/model_data/yolo_anchors.txt'
        self.classes_path = '/root/catkin_ws/src/arm_garbage_identify/model_data/garbage.txt'
        self.model_path = '/root/catkin_ws/src/arm_garbage_identify/model_data/garbage.h5'
        self.font_path = '/root/catkin_ws/src/arm_garbage_identify/font/Block_Simplified.TTF'
        self.class_names = self._get_class()
        self.anchors = self._get_anchors()
        self.model_image_size = (416, 416)
        if not self.eager:
            tf.compat.v1.disable_eager_execution()
            self.sess = K.get_session()
        self.generate()
        # 画框设置不同的颜色
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self.class_names))]
        # 机械臂识别位置调节
        self.xy = [90, 135]
        # 创建垃圾识别抓取实例
        self.grap_move = garbage_grap_move()
        self.garbage_index = 0
        # 创建节点句柄
#         self.n = rospy.init_node('ros_arm', anonymous=True)
        # 创建用于调用的ROS服务的句柄。
        self.client = rospy.ServiceProxy("get_kinemarics", kinemarics)

    # 获得所有的分类
    def _get_class(self):
        # 展开文件路径
        classes_path = os.path.expanduser(self.classes_path)
        # 打开文件,逐行读取.
        with open(classes_path) as f: class_names = f.readlines()
        # 将label放入列表中
        class_names = [c.strip() for c in class_names]
        return class_names

    # 获得所有的先验框
    def _get_anchors(self):
        anchors_path = os.path.expanduser(self.anchors_path)
        with open(anchors_path) as f: anchors = f.readline()
        anchors = [float(x) for x in anchors.split(',')]
        return np.array(anchors).reshape(-1, 2)

    # 获得所有的分类
    def generate(self):
        model_path = os.path.expanduser(self.model_path)
        assert model_path.endswith('.h5'), 'Keras model or weights must be a .h5 file.'
        # 计算anchor数量
        num_anchors = len(self.anchors)
        num_classes = len(self.class_names)
        # 载入模型，如果原来的模型里已经包括了模型结构则直接载入。
        # 否则先构建模型再载入
        self.yolo_model = yolo_body(Input(shape=(None, None, 3)), num_anchors // 2, num_classes)
        self.yolo_model.load_weights(self.model_path)
        print('{} model, anchors, and classes loaded.'.format(model_path))
        if self.eager:
            self.input_image_shape = Input([2, ], batch_size=1)
            inputs = [*self.yolo_model.output, self.input_image_shape]
            outputs = Lambda(yolo_eval, output_shape=(1,), name='yolo_eval',
                             arguments={'anchors': self.anchors, 'num_classes': len(self.class_names),
                                        'frame_shape': self.model_image_size,
                                        'score_threshold': self.score, 'eager': True})(inputs)
            self.yolo_model = Model([self.yolo_model.input, self.input_image_shape], outputs)
        else:
            self.input_image_shape = K.placeholder(shape=(2,))
            self.boxes, self.scores, self.classes = yolo_eval(self.yolo_model.output, self.anchors,
                                                              num_classes, self.input_image_shape,
                                                              score_threshold=self.score, iou_threshold=self.iou)

    # 检测图片
    def detect_image(self, image):
        # 格式转变，BGRtoRGB
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        # 转变成frame
        image = Image.fromarray(np.uint8(image))
        # 调整图片使其符合输入要求
        new_image_size = self.model_image_size
        boxed_image = letterbox_image(image, new_image_size)
        image_data = np.array(boxed_image, dtype='float32')
        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.
        if self.eager:
            # 预测结果
            input_image_shape = np.expand_dims(np.array([image.size[1], image.size[0]], dtype='float32'), 0)
            out_boxes, out_scores, out_classes = self.yolo_model.predict([image_data, input_image_shape])
        else:
            # 预测结果
            out_boxes, out_scores, out_classes = self.sess.run(
                [self.boxes, self.scores, self.classes],
                feed_dict={
                    self.yolo_model.input: image_data,
                    self.input_image_shape: [image.size[1], image.size[0]],
#                     K.learning_phase(): 0
                })
        msg = {}
        for i, c in list(enumerate(out_classes)):
            predicted_class = self.class_names[c]
            box = out_boxes[i]
            score = out_scores[i]
            top, left, bottom, right = box
            top = top - 5
            left = left - 5
            bottom = bottom + 5
            right = right + 5
            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(image.size[1], np.floor(bottom + 0.5).astype('int32'))
            right = min(image.size[0], np.floor(right + 0.5).astype('int32'))
            label = '{}: {:.2f}'.format(predicted_class, score)
            label = label.encode('utf-8')
            # return box,label
            x = (right + left) / 2
            y = (bottom + top) / 2
            r = 5
            # 创建画笔
            draw = ImageDraw.Draw(image)
            # 画圆心
            draw.ellipse((x - r, y - r, x + r, y + r), fill=tuple(self.colors[int(-i)]))
            # 画边框
            draw.rectangle((left, top, right, bottom), outline=tuple(self.colors[int(i)]), width=3)
            # 设置字体
            fontStyle = ImageFont.truetype(self.font_path, size=20, encoding="utf-8")
            # 写字
            draw.text((left, top - 25), str(label, 'UTF-8'), fill=(255, 0, 0), font=fontStyle)
            # 计算方块在图像中的位置
#                 (a, b) = (round(((x - 320) / 4000), 5), round(((240 - y) / 3000 + 0.265) * 0.95, 5))
            (a, b) = (round(((x - 320) / 4000), 5), round(((480 - y) / 3000) * 0.8+0.19, 5))
            msg[predicted_class] = (a, b)
            del draw
        end = timer()
        # print(end - start)
        image = cv.cvtColor(np.array(image), cv.COLOR_RGB2BGR)
        return image, msg

    def garbage_run(self, img):
        '''
        执行垃圾识别函数
        :param image: 原始图像
        :return: 识别后的图像,识别信息(name, pos)
        '''
        # 规范输入图像大小
        img = cv.resize(img, (640, 480))
        txt0 = 'Model-Loading...'
        msg = {}
        if self.garbage_index < 3:
            cv.putText(img, txt0, (190, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            self.garbage_index += 1
            return img, msg
        if self.garbage_index >= 3:
            # 创建消息容器
            try: img, msg = self.detect_image(img)  # 获取识别消息
            except Exception: print("get_pos NoneType")
            return img, msg

    def garbage_grap(self, msg, xy=None):
        '''
        执行抓取函数
        :param msg: {name:pos,...}
        '''
        if xy != None: self.xy = xy
        if len(msg) != 0:
            self.arm.Arm_Buzzer_On(1)
            time.sleep(0.5)
        # print("msg : ",msg)
        for index, name in enumerate(msg):
            try:
                # 此处ROS反解通讯,获取各关节旋转角度
                joints = self.server_joint(msg[name])
                # print(joints)
                # 调取移动函数
                self.grap_move.arm_run(str(name), joints)
            except Exception:
                print("sqaure_pos empty")
        # 初始位置
        joints_0 = [self.xy[0], self.xy[1], 0, 0, 90, 30]
        # 移动至初始位置
        self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
        time.sleep(1)

    def server_joint(self, posxy):
        '''
        发布位置请求,获取关节旋转角度
        :param posxy: 位置点x,y坐标
        :return: 每个关节旋转角度
        '''
        # 等待server端启动
        self.client.wait_for_service()
        # 创建消息包
        request = kinemaricsRequest()
        request.tar_x = posxy[0]
        request.tar_y = posxy[1]
        request.kin_name = "ik"
        try:
            response = self.client.call(request)
            if isinstance(response, kinemaricsResponse):
                # 获取反解的响应结果
                joints = [0, 0, 0, 0, 0]
                joints[0] = response.joint1
                joints[1] = response.joint2
                joints[2] = response.joint3
                joints[3] = response.joint4
                joints[4] = response.joint5
                # 角度调整
                if joints[2] < 0:
                    joints[1] += joints[2] / 2
                    joints[3] += joints[2] * 3 / 4
                    joints[2] = 0
                # print joints
                return joints
        except Exception:
            rospy.loginfo("arg error")


