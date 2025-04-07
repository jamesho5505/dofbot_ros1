#bgr8 to jpeg format
import enum
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import threading
import RPi.GPIO as GPIO
import os, sys, time
from Arm_Lib import Arm_Device

class Arm_WIFI:
    # Init function
    def __init__(self):
        self.KEY1_PIN = 5
        self.LED_PIN = 13

        self.key1_pressed = False
        self.config_Mode = False
        self.count = 0
        self.SSID = ''
        self.PASSWD = ''
        self.ip = 'x.x.x.x'
        #self.conn = 0

        self.key_scan_state = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.KEY1_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
        GPIO.setup(self.LED_PIN, GPIO.OUT, initial = GPIO.LOW)

        self.Arm = Arm_Device()
        closeTid = threading.Thread(target = self.key_scan)
        closeTid.setDaemon(True)
        closeTid.start()


    # get loacal IP
    def getLocalip(self):
        ip = os.popen("/sbin/ifconfig eth0 | grep 'inet' | awk '{print $2}'").read()
        ip = ip[0 : ip.find('\n')]
        if(ip == ''):
            #Read WLAN IP
            ip = os.popen("/sbin/ifconfig wlan0 | grep 'inet' | awk '{print $2}'").read()
            ip = ip[0 : ip.find('\n')]
            if(ip == ''):
                ip = 'x.x.x.x'
        return ip

    #Key scan
    def key_scan(self):
        while True:
            if GPIO.input(self.KEY1_PIN) == GPIO.LOW:
                time.sleep(0.05)
                if GPIO.input(self.KEY1_PIN) == GPIO.LOW:
                    if self.key1_pressed == False:
                        self.key1_pressed = True
                        self.count = 0
                    else:
                        self.count += 1
                        if self.count == 20:
                            self.count = 0
                            self.key1_pressed = False
                            print("key1_pressed")
                            self.config_Mode = True
                                
                else:
                    self.count = 0
                    self.key1_pressed = False
            else:
                self.count = 0
                self.key1_pressed = False
                time.sleep(0.05)
    
    def read_mode(self):
        return self.config_Mode

    def set_mode(self, mode=False):
        self.config_Mode = mode
        if self.config_Mode == False:
            self.ip = self.getLocalip()
            if self.ip == 'x.x.x.x':
                self.wifi_LED_OFF()
            else:
                # if self.conn == 0:
                #self.conn = 1
                #Arm.Arm_Buzzer_On(1) 
                self.wifi_LED_ON()

    def wifi_LED_ON(self):
        GPIO.output(self.LED_PIN, GPIO.HIGH)

    def wifi_LED_OFF(self):
        GPIO.output(self.LED_PIN, GPIO.LOW)

    # Define the QR code interface
    def decodeDisplay(self, image):
        barcodes = pyzbar.decode(image)
        SSID = ''
        PASSWD = ''
        for barcode in barcodes:
            # Extract the position of the bounding box of the QR code
            # Draw the bounding box of the QR code in the image
            (x, y, w, h) = barcode.rect
            cv2.rectangle(image, (x, y), (x + w, y + h), (225, 225, 225), 2)
            # Extract the QR code data as a byte object
            # Convert to string
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            # Plot the barcode data and barcode type on the image
            text = "{} ({})".format(barcodeData, barcodeType)
            cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 0), 2)
            a = barcodeData.find('SSID')
            b = barcodeData.find('|')
            SSID = barcodeData[6:b-1]
            PASSWD = barcodeData[b+2:-1]
            print(SSID)
            print(PASSWD)
            # Print barcode data and barcode type to the terminal
            print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        return image, SSID, PASSWD

    #Camera connect wifi
    def connect(self, frame):
        print("connect--------------")
        if self.config_Mode == True:
            self.wifi_LED_ON()
            time.sleep(0.2)
            self.wifi_LED_OFF()
            time.sleep(0.2)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img, self.SSID, self.PASSWD = self.decodeDisplay(gray)
                
            if(self.SSID != '' and self.PASSWD != ''):
                passwd = "yahboom"
                cmd_scan = "sudo iw dev wlan0 scan"
                os.system('echo %s | sudo -S %s' % (passwd, cmd_scan))
                cmd = "sudo nmcli dev wifi connect \'" + self.SSID + "\' password \'" + self.PASSWD + "\'"
                os.system('echo %s | sudo -S %s' % (passwd, cmd))
                self.config_Mode = False
                self.ip = '0.0.0.0'
                for i in range(3):
                    self.Arm.Arm_Buzzer_On(1) 
                    time.sleep(0.2)
        else:
            img = frame
        return img, self.ip

    #Clear GPIO
    def destroy(self):
        GPIO.cleanup()
        



if __name__ == '__main__':
    wifi = Arm_WIFI()
    camera = cv2.VideoCapture(0)
    try:
        GPIO.output(13, GPIO.LOW)
        while True:
            ret, frame = camera.read()
            if(ret == False):
                continue
            img, ip = wifi.connect(frame)
            if ip == 'x.x.x.x':
                print('network disconnection')
            else:
                print("IP:"+ip)
            
    except KeyboardInterrupt:
        camera.release()
        GPIO.cleanup()
        print('yb-wifi.py killed')
