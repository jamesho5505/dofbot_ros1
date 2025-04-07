#!/usr/bin/env python3
import rospy
from arm_mediapipe.msg import PIDParams

if __name__ == '__main__':
    rospy.init_node("pid_tuner")
    pub = rospy.Publisher("/pid_param", PIDParams, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        print("\n--- PID 調整 ---")
        try:
            Kp_x = float(input("Kp_x: "))
            Ki_x = float(input("Ki_x: "))
            Kd_x = float(input("Kd_x: "))
            Kp_y = float(input("Kp_y: "))
            Ki_y = float(input("Ki_y: "))
            Kd_y = float(input("Kd_y: "))

            msg = PIDParams(
                Kp_x=Kp_x,
                Ki_x=Ki_x,
                Kd_x=Kd_x,
                Kp_y=Kp_y,
                Ki_y=Ki_y,
                Kd_y=Kd_y,
            )
            pub.publish(msg)
            print("已發送 PID 參數")

        except ValueError:
            print("請輸入正確的數值格式！")

        rate.sleep()
