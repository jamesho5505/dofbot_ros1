#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
import threading
import cv2 as cv
import time
from time import sleep
from garbage_identify import garbage_identify
import smbus
bus = smbus.SMBus(1)

i2c_addr = 0x0f   #Speech recognition module address
asr_add_word_addr  = 0x01   #Entry add address
asr_mode_addr  = 0x02   #Recognition mode setting address, the value is 0-2, 0: cyclic recognition mode 1: password mode, 2: button mode, the default is cyclic detection
asr_rgb_addr = 0x03   #RGB lamp setting address, need to send two bits, the first one is directly the lamp number 1: blue 2: red 3: green
                      #The second byte is brightness 0-255, the larger the value, the higher the brightness
asr_rec_gain_addr  = 0x04    #Identification sensitivity setting address, sensitivity can be set to 0x00-0x7f, the higher the value, the easier it is to detect but the easier it is to misjudge
                             #It is recommended to set the value to 0x40-0x55, the default value is 0x40
asr_clear_addr = 0x05   #Clear the operation address of the power-off cache, clear the cache area information before entering the information
asr_key_flag = 0x06  #Used in key mode, set the startup recognition mode
asr_voice_flag = 0x07   #Used to set whether to turn on the recognition result prompt sound
asr_result = 0x08  #Recognition result storage address
asr_buzzer = 0x09  #Buzzer control register, 1 bit is on, 0 bit is off
asr_num_cleck = 0x0a #Check the number of entries
asr_vession = 0x0b #firmware version number
asr_busy = 0x0c #Busy and busy flag

i2c_speech_addr = 0x30   #语音播报模块地址
speech_date_head = 0xfd


EncodingFormat_Type = {
                        'GB2312':0x00,
                        'GBK':0X01,
                        'BIG5':0x02,
                        'UNICODE':0x03
                        }

ChipStatus_Type = {
                    'ChipStatus_InitSuccessful':0x4A,#初始化成功回传
                    'ChipStatus_CorrectCommand':0x41,#收到正确的命令帧回传
                    'ChipStatus_ErrorCommand':0x45,#收到不能识别命令帧回传
                    'ChipStatus_Busy':0x4E,#芯片忙碌状态回传
                    'ChipStatus_Idle':0x4F #芯片空闲状态回传                  
                }

Style_Type = {
                'Style_Single':0,#为 0，一字一顿的风格
                'Style_Continue':1#为 1，正常合成
                }#合成风格设置 [f?]

Language_Type = {
                'Language_Auto':0,#为 0，自动判断语种
                'Language_Chinese':1,#为 1，阿拉伯数字、度量单位、特殊符号等合成为中文
                'Language_English':2#为 1，阿拉伯数字、度量单位、特殊符号等合成为中文
                }#合成语种设置 [g?]

Articulation_Type = {
                'Articulation_Auto':0,#为 0，自动判断单词发音方式
                'Articulation_Letter':1,#为 1，字母发音方式
                'Articulation_Word':2#为 2，单词发音方式
                }#设置单词的发音方式 [h?]

Spell_Type = {
                'Spell_Disable':0,#为 0，不识别汉语拼音
                'Spell_Enable':1#为 1，将“拼音＋1 位数字（声调）”识别为汉语拼音，例如： hao3
                }#设置对汉语拼音的识别 [i?]

Reader_Type = {
                'Reader_XiaoYan':3,#为 3，设置发音人为小燕(女声, 推荐发音人)
                'Reader_XuJiu':51,#为 51，设置发音人为许久(男声, 推荐发音人)
                'Reader_XuDuo':52,#为 52，设置发音人为许多(男声)
                'Reader_XiaoPing':53,#为 53，设置发音人为小萍(女声
                'Reader_DonaldDuck':54,#为 54，设置发音人为唐老鸭(效果器)
                'Reader_XuXiaoBao':55#为 55，设置发音人为许小宝(女童声)                
                }#选择发音人 [m?]

NumberHandle_Type = {
                'NumberHandle_Auto':0,#为 0，自动判断
                'NumberHandle_Number':1,#为 1，数字作号码处理
                'NumberHandle_Value':2#为 2，数字作数值处理
                }#设置数字处理策略 [n?]

ZeroPronunciation_Type = {
                'ZeroPronunciation_Zero':0,#为 0，读成“zero
                'ZeroPronunciation_O':1#为 1，读成“欧”音
                }#数字“0”在读 作英文、号码时 的读法 [o?]

NamePronunciation_Type = {
                'NamePronunciation_Auto':0,#为 0，自动判断姓氏读音
                'NamePronunciation_Constraint':1#为 1，强制使用姓氏读音规则
                }#设置姓名读音 策略 [r?]

OnePronunciation_Type = {
                'OnePronunciation_Yao':0,#为 0，合成号码“1”时读成幺
                'OnePronunciation_Yi':1#为 1，合成号码“1”时读成一
                }#设置号码中“1”的读法 [y?]

Rhythm_Type = {
                'Rhythm_Diasble':0,#为 0，“ *”和“#”读出符号
                'Rhythm_Enable':1#为 1，处理成韵律，“*”用于断词，“#”用于停顿
                }#是否使用韵律 标记“*”和“#” [z?]    


class speech_garbage:
    def __init__(self):
        # 中间变量
        self.name_tmp = ' '
        # 初始化垃圾名称
        self.garbage_num = 'None'
        # 初始化垃圾类别
        self.garbage_class = 'None'
        # 初始化计数器
        self.num = 0
        # 初始化运动状态
        self.status = 'waiting'
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device()
        # 夹爪加紧角度
        self.grap_joint = 135
        # 初始化垃圾识别实例
        self.garbage_identify = garbage_identify()
        '''
        The mode and phrase have the function of power-down save, if there is no modification after the first entry, you can change 1 to 0
        '''
        cleck = 1

        if 1:
            bus.write_byte_data(i2c_addr, asr_clear_addr, 0x40)#Clear the power-down buffer area
            self.Busy_Wait()#Wait for the module to be free
            print("Cache cleared")
            bus.write_byte_data(i2c_addr, asr_mode_addr, 1)
            self.Busy_Wait()
            print("The mode is set")
            self.AsrAddWords(0,"xiao ya")
            self.Busy_Wait()
            self.AsrAddWords(1,"zhe shi shen me la ji")
            self.Busy_Wait()
            while cleck != 2:
                cleck = self.I2CReadByte(asr_num_cleck)
                print(cleck)

        bus.write_byte_data(i2c_addr, asr_rec_gain_addr, 0x40)#Set the sensitivity, the recommended value is 0x40-0x55
        bus.write_byte_data(i2c_addr, asr_voice_flag, 1)#Set switch sound
        bus.write_byte_data(i2c_addr, asr_buzzer, 1)#buzzer
        self.RGBSet(255,255,255)
        time.sleep(1)
        self.RGBSet(0,0,0)
        bus.write_byte_data(i2c_addr, asr_buzzer, 0)#buzzer

        self.SetReader(Reader_Type["Reader_XiaoPing"])#选择播音人晓萍
        self.SetVolume(8)
        self.Speech_text("语音初始化完成",EncodingFormat_Type["GB2312"])
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
            time.sleep(0.1)  

    def single_garbage_run(self, image):
        '''
        执行垃圾识别函数
        :param image: 原始图像
        :return: 识别后的图像
        '''
        # 规范输入图像大小
        self.frame = cv.resize(image, (640, 480))
        try: self.garbage_getName()
        except Exception: print("sqaure_pos empty")
        return self.frame
    
    def garbage_getName(self):
        name = "None"
        if self.status == 'waiting':
            self.frame, msg = self.garbage_identify.garbage_run(self.frame)
            for key, pos in msg.items(): name = key
            if name == "Zip_top_can":              (self.garbage_num, self.garbage_class) = ('00', '01')
            if name == "Old_school_bag":           (self.garbage_num, self.garbage_class) = ('01', '01')
            if name == "Newspaper":                (self.garbage_num, self.garbage_class) = ('02', '01')
            if name == "Book":                     (self.garbage_num, self.garbage_class) = ('03', '01')
            if name == "Toilet_paper":             (self.garbage_num, self.garbage_class) = ('04', '02')
            if name == "Peach_pit":                (self.garbage_num, self.garbage_class) = ('05', '02')
            if name == "Cigarette_butts":          (self.garbage_num, self.garbage_class) = ('06', '02')
            if name == "Disposable_chopsticks":    (self.garbage_num, self.garbage_class) = ('07', '02')
            if name == "Egg_shell":                (self.garbage_num, self.garbage_class) = ('08', '03')
            if name == "Apple_core":               (self.garbage_num, self.garbage_class) = ('09', '03')
            if name == "Watermelon_rind":          (self.garbage_num, self.garbage_class) = ('10', '03')
            if name == "Fish_bone":                (self.garbage_num, self.garbage_class) = ('11', '03')
            if name == "Expired_tablets":          (self.garbage_num, self.garbage_class) = ('12', '04')
            if name == "Expired_cosmetics":        (self.garbage_num, self.garbage_class) = ('13', '04')
            if name == "Used_batteries":           (self.garbage_num, self.garbage_class) = ('14', '04')
            if name == "Syringe":                  (self.garbage_num, self.garbage_class) = ('15', '04')
            if name == "None":                     (self.garbage_num, self.garbage_class) = ('None', 'None')
            if self.name_tmp == name and self.name_tmp != "None":
                self.num += 1
                # 每当连续识别3次并且运动状态为waiting的情况下,执行抓取任务
                if self.num % 3 == 0 and self.status == 'waiting':
                    self.status = 'speech'
                    self.num = 0 
                    self.Speech_text("识别完成",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
#                 else: self.num = 0
                print(self.num)
            else:
                self.name_tmp = name
        elif self.status == 'speech':
            result = self.I2CReadByte(asr_result)
            if result == 1:
                if self.garbage_num == '00':
                    self.Speech_text("这是易拉罐，属于可回收垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '01':
                    self.Speech_text("这是旧书包，属于可回收垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '02':
                    self.Speech_text("这是报纸，属于可回收垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '03':
                    self.Speech_text("这是书本，属于可回收垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '04':
                    self.Speech_text("这是卫生纸，属于干垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '05':
                    self.Speech_text("这是桃核，属于干垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '06':
                    self.Speech_text("这是烟蒂，属于干垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '07':
                    self.Speech_text("这是一次性筷子，属于干垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '08':
                    self.Speech_text("这是鸡蛋壳，属于湿垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '09':
                    self.Speech_text("这是苹果核，属于湿垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '10':
                    self.Speech_text("这是西瓜皮，属于湿垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '11':
                    self.Speech_text("这是鱼骨，属于湿垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '12':
                    self.Speech_text("这是过期药品，属于有毒垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '13':
                    self.Speech_text("这是过期化妆品，属于有毒垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '14':
                    self.Speech_text("这是废旧电池，属于有毒垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                elif self.garbage_num == '15':
                    self.Speech_text("这是注射器，属于有毒垃圾",EncodingFormat_Type["GB2312"])
                    while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                        time.sleep(0.1) 
                self.status = 'waiting'
                # 开启抓取线程
                #threading.Thread(target=self.single_garbage_grap, args=(self.garbage_class,)).start()

    def move(self, joints_down):
        '''
        移动过程
        :param joints_down: 机械臂抬起各关节角度
        :param color_angle: 移动到对应垃圾桶的角度
        '''
        # 初始位置
        joints_0 = [90, 90, 20, 15, 90, 30]
        # 抓取角度
        joints = [90, 40, 30, 67, 265, 30]
        joints_uu = [90, 80, 50, 50, 265, self.grap_joint]
        # 抬起
        joints_up = [joints_down[0], 80, 50, 50, 265, 30]
        # 移动至物体位置上方
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
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
        # 移动至初始位置
        self.arm.Arm_serial_servo_write6_array(joints_0, 1000)

    def single_garbage_grap(self, name):
        '''
        机械臂移动函数
        :param name:识别的垃圾类别
        '''
        self.arm.Arm_Buzzer_On(1)
        sleep(0.5)
        # 有害垃圾--红色 04
        if name == "04":
            # print("有害垃圾")
            # 移动到垃圾桶前对应姿态
            joints_down = [45, 80, 35, 40, 265, self.grap_joint]
            # 移动到垃圾桶位置放下对应姿态
            # joints_down = [45, 50, 20, 60, 265, self.grap_joint]
            # 移动
            self.move(joints_down)
            # 移动完毕
            self.status = 'waiting'
        # 可回收垃圾--蓝色 01
        if name == "01":
            # print("可回收垃圾")
            joints_down = [27, 110, 0, 40, 265, self.grap_joint]
            # joints_down = [27, 75, 0, 50, 265, self.grap_joint]
            self.move(joints_down)
            self.status = 'waiting'
        # 厨余垃圾--绿色 03
        if name == "03":
            # print("厨余垃圾")
            joints_down = [152, 110, 0, 40, 265, self.grap_joint]
            # joints_down = [147, 75, 0, 50, 265, self.grap_joint]
            self.move(joints_down)
            self.status = 'waiting'
        # 其他垃圾--灰色 02
        if name == "02":
            # print("其他垃圾")
            joints_down = [137, 80, 35, 40, 265, self.grap_joint]
            # joints_down = [133, 50, 20, 60, 265, self.grap_joint]
            self.move(joints_down)
            self.status = 'waiting'
    
    def I2C_WriteBytes(self, str_):
        global i2c_speech_addr
        for ch in str_:
            try:
                bus.write_byte(i2c_speech_addr,ch)
                time.sleep(0.01)
            except:
                print("write I2C error")


    def Speech_text(self, str_,encoding_format):
        str_ = str_.encode('gb2312')   
        size = len(str_)+2
        DataHead = speech_date_head
        Length_HH = size>>8
        Length_LL = size & 0x00ff
        Commond = 0x01
        EncodingFormat = encoding_format

        Date_Pack = [DataHead,Length_HH,Length_LL,Commond,EncodingFormat]

        self.I2C_WriteBytes(Date_Pack)

        self.I2C_WriteBytes(str_)

    def SetBase(self, str_):
        str_ = str_.encode('gb2312')   
        size = len(str_)+2

        DataHead = speech_date_head
        Length_HH = size>>8
        Length_LL = size & 0x00ff
        Commond = 0x01
        EncodingFormat = 0x00

        Date_Pack = [DataHead,Length_HH,Length_LL,Commond,EncodingFormat]

        self.I2C_WriteBytes(Date_Pack)

        self.I2C_WriteBytes(str_)

    def TextCtrl(self, ch,num):
        if num != -1:
            str_T = '[' + ch + str(num) + ']'
            self.SetBase(str_T)
        else:
            str_T = '[' + ch + ']'
            self.SetBase(str_T)


    def GetChipStatus(self):
        global i2c_speech_addr
        AskState = [0xfd,0x00,0x01,0x21]
        try:
            self.I2C_WriteBytes(AskState)
            time.sleep(0.05)
        except:
            print("I2CRead_Write error")


        try:
            Read_result = bus.read_byte(i2c_speech_addr)
            return Read_result
        except:
            print("I2CRead error")

    def SetStyle(self, num):
        self.TextCtrl('f',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)   


    def SetLanguage(self, num):
        self.TextCtrl('g',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)

    def SetArticulation(self, num):
        self.TextCtrl('h',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)


    def SetSpell(self, num):
        self.TextCtrl('i',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)


    def SetReader(self, num):
        self.TextCtrl('m',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)


    def SetNumberHandle(self, num):
        self.TextCtrl('n',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)



    def SetZeroPronunciation(self, num):
        self.TextCtrl('o',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)




    def SetNamePronunciation(self, num):
        self.TextCtrl('r',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)

    #设置语速 [s?] ? 为语速值，取值：0～10
    def SetSpeed(self, speed):
        self.TextCtrl('s',speed)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)


    #设置语调 [t?] ? 为语调值，取值：0～10
    def SetIntonation(self, intonation):
        self.TextCtrl('t',intonation)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)

    #设置音量 [v?] ? 为音量值，取值：0～10
    def SetVolume(self, volume):
        self.TextCtrl('v',volume)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)



    def SetOnePronunciation(self, num):
        self.TextCtrl('y',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)


    def SetRhythm(self, num):
        self.TextCtrl('z',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)

    #恢复默认的合成参数 [d] 所有设置（除发音人设置、语种设置外）恢复为默认值
    def SetRestoreDefault(self):
        self.TextCtrl('d',-1)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)

    #Write entry
    def AsrAddWords(self, idnum,str):
        global i2c_addr
        global asr_add_word_addr
        words = []
        words.append(asr_add_word_addr)
        words.append(len(str) + 2)
        words.append(idnum)
        for  alond_word in str:
            words.append(ord(alond_word))
        words.append(0)
        print(words)
        for date in words:
            bus.write_byte (i2c_addr, date)
            time.sleep(0.03)

    #Set RGB
    def RGBSet(self, R,G,B):
        global i2c_addr
        global asr_rgb_addr
        date = []
        date.append(R)
        date.append(G)
        date.append(B)
        print(date)
        bus.write_i2c_block_data (i2c_addr,asr_rgb_addr,date)

    #Read result
    def I2CReadByte(self, reg):
        global i2c_addr
        bus.write_byte (i2c_addr, reg)
        #time.sleep(0.05)
        Read_result = bus.read_byte (i2c_addr)
        return Read_result

    #Wait busy
    def Busy_Wait(self):
        busy = 255
        while busy != 0:
            busy = self.I2CReadByte(asr_busy)
            print(asr_busy)