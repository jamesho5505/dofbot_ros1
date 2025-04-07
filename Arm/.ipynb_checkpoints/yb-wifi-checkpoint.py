#bgr8 转 jpeg 格式
import enum
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
# from PIL import Image
# import ipywidgets.widgets as widgets
import threading
import RPi.GPIO as GPIO
import os, sys, time

# Pin Definitions
KEY1_PIN = 20
LED_PIN = 21

key1_pressed = False
config_Mode = False
count = 0
SSID = ''
PASSWD = ''

def getwlanip():
    ip = os.popen("/sbin/ifconfig wlan0 | grep 'inet' | awk '{print $2}'").read()
    ip = ip[0 : ip.find('\n')]
    if(ip == ''):
        print("no connect any!")
    return ip

def key_scan(ip):
    global key1_pressed, count, config_Mode, thread1
    if GPIO.input(KEY1_PIN) == GPIO.LOW:
        time.sleep(0.05)
        if GPIO.input(KEY1_PIN) == GPIO.LOW:
            #print("low")
            #print(key1_pressed)
            if key1_pressed == False:
                key1_pressed = True
                count = 0
            else:
                count += 1
                if ip == '': #time too long
                    if count == 4:
                        count = 0
                        key1_pressed = False
                        print("reset_wifi")
                        thread1.start()
                        thread1.join()
                        config_Mode = True
                else:
                    if count == 30:
                        count = 0
                        key1_pressed = False
                        print("reset_wifi")
                        thread1.start()
                        thread1.join()
                        config_Mode = True
                        #reset_wifi()
        else:
            count = 0
            key1_pressed = False
    else:
        count = 0
        key1_pressed = False
        time.sleep(0.05)

def bgr8_to_jpeg(value, quality=75):
    return bytes(cv2.imencode('.jpg', value)[1])


# 导入库并显示摄像头显示组件
# image_widget = widgets.Image(format='jpeg', width=1280, height=720)
# display(image_widget) #显示摄像头组件

# 定义解析二维码接口
def decodeDisplay(image):
    global config_Mode, SSID, PASSWD
    barcodes = pyzbar.decode(image)
    for barcode in barcodes:
        # 提取二维码的边界框的位置
        # 画出图像中条形码的边界框
        (x, y, w, h) = barcode.rect
        cv2.rectangle(image, (x, y), (x + w, y + h), (225, 225, 225), 2)
        # 提取二维码数据为字节对象，所以如果我们想在输出图像上
        # 画出来，就需要先将它转换成字符串
        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type
        # 绘出图像上条形码的数据和条形码类型
        text = "{} ({})".format(barcodeData, barcodeType)
        cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 0),
        2)
        a = barcodeData.find('SSID')
        b = barcodeData.find('|')
        SSID = barcodeData[6:b-1]
        PASSWD = barcodeData[b+2:-1]
        print(SSID)
        print(PASSWD)
        # 向终端打印条形码数据和条形码类型
        print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
    return image

#线程
class myThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
    def run(self):
        print ("开始线程：" + self.name)
        if(self.threadID == 1):
            detect(self.name)
        elif(self.threadID == 2):
            while True:
                ip = getwlanip()
                key_scan(ip)
                if(ip == ''):
                    GPIO.output(LED_PIN, GPIO.LOW)
                    time.sleep(1)
                    GPIO.output(LED_PIN, GPIO.HIGH)
                    time.sleep(1)
                elif(ip != ''): # 
                    GPIO.output(LED_PIN, GPIO.LOW)
        print ("退出线程：" + self.name)

def detect(threadName):
    global config_Mode, SSID, PASSWD, thread1
    camera = cv2.VideoCapture(0)
    ret, frame = camera.read()
#     if(ret == True):
#         image_widget.value = bgr8_to_jpeg(frame)
    while True:
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(0.02)
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(0.02)
        # 读取当前帧
        ret, frame = camera.read()
        if(ret == False):
            #print("read error")
            continue
        # 转为灰度图像
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        im = decodeDisplay(gray)
#         image_widget.value = bgr8_to_jpeg(im)
        if(SSID != '' and PASSWD != ''):
            config_Mode = False
            cmd = "sudo nmcli dev wifi connect \'" + SSID + "\' password \'" + PASSWD + "\'"
            os.system(cmd)
            os.system("sudo systemctl restart yb-wifi.service")
            print("退出进程")

            break
    camera.release()

thread1 = myThread(1, "Thread-1")
thread2 = myThread(2, "Thread-2")

def main():
    global config_Mode
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(KEY1_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(LED_PIN, GPIO.OUT, initial = GPIO.HIGH)
    thread2.start()
    thread2.join()
#     while True:
#         ip = getwlanip()
#         key_scan()
#         if(config_Mode == True):
#             GPIO.output(LED_PIN, GPIO.LOW)
#             time.sleep(0.1)
#             GPIO.output(LED_PIN, GPIO.HIGH)
#             time.sleep(0.1)
#         else:
#             if(ip == ''):
#                 GPIO.output(LED_PIN, GPIO.LOW)
#                 time.sleep(1)
#                 GPIO.output(LED_PIN, GPIO.HIGH)
#                 time.sleep(1)
#             elif(ip != ''): # 
#                 GPIO.output(LED_PIN, GPIO.LOW)

if __name__ == '__main__':
    main()
