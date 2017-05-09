#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib;roslib.load_manifest('balance')
import rospy
from  balance.msg import Num, carOdom #自定义的消息
from geometry_msgs.msg import Twist

import glob
from math import sqrt, atan2, pow
sumspeed=0
pub = rospy.Publisher('car_speed', carOdom,queue_size=1000)
pub2 = rospy.Publisher('control_speed', Num,queue_size=1000)
class balance_cmd():
    def __init__(self):
        rospy.init_node('link2balance')
    
    def callback(self,msg ):
        control = Num()
        cmd_twist_rotation =  msg.angular.z #
        cmd_twist_x  = msg.linear.x * 10.0
        cmd_twist_y =  msg.linear.y * 10.0      
        wheelspeed = self.odom_to_speed(cmd_twist_x, cmd_twist_y,cmd_twist_rotation)
        print 'msg:', msg
        print wheelspeed
        control.leftspeed=wheelspeed[0]
        control.rightspeed=wheelspeed[1]
        pub2.publish(control)   
    def odom_to_speed(self, cmd_twist_x =0, cmd_twist_y=0,cmd_twist_rotation=0):
        
        cent_speed = sqrt(pow(cmd_twist_x, 2) + pow(cmd_twist_y, 2))
        yawrate2 = self.yawrate_to_speed(cmd_twist_rotation)
        
        Lwheelspeed = cent_speed - yawrate2/2
        Rwheelspeed = cent_speed + yawrate2/2
        
        return Lwheelspeed, Rwheelspeed
        
    def yawrate_to_speed(self, yawrate):
        if yawrate > 0:
            theta_to_speed = 0.0328 #右转系数
        else:
            theta_to_speed = 0.0328  #左转系数
            
        x = (yawrate * 0.05) / theta_to_speed #yawrate ：rad/s  *0.02表示 20ms内应该转多少弧度，/0.0076是把 要转的弧度转化为左右轮速度差
        return   x
    def wheelcallback(self,data ):
        car_speed = carOdom()
        resluts = self.speed_to_odom(data.leftspeed, data.rightspeed )
        car_speed.x = resluts[0]
        car_speed.y = resluts[1]
        car_speed.vth = resluts[2]
        pub.publish(car_speed)

    def talker(self):
        rospy.Subscriber("/cmd_vel", Twist, self.callback)#订阅move_base发出的控制指令
        rospy.Subscriber("/wheel_speed", Num, self.wheelcallback)#订阅move_base发出的控制指令
        r = rospy.Rate(100) # 100hzs
        while not rospy.is_shutdown():
            r.sleep()

    def speed_to_odom(self, Lspeed = 0, Rspeed = 0):
        global sumspeed
        delta_speed = Rspeed - Lspeed
        if delta_speed < 0:
            theta_to_speed = 0.0328 #右转系数
        else:
            theta_to_speed = 0.0328  #左转系数
        sumspeed=sumspeed+delta_speed
        print "delta:",delta_speed
        print "sum:",sumspeed
        v_th = delta_speed  * theta_to_speed / 0.02    # first : transform delta_speed to  delta_theta .   second: dived by delta_t (20ms), get the yawrate
        v_x = (Rspeed + Lspeed)*20/100.0/2.0    # Lspeed : dm/s   -- > m/s  so need to /10.0
        v_y = 0.0
        return v_x, v_y, v_th
        
#        print data
            
if __name__ == '__main__':
    try:
        car_cmd = balance_cmd()
        car_cmd.talker()
    except rospy.ROSInterruptException: pass

