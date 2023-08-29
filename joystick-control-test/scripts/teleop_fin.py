#!/usr/bin/env python
#-- coding: utf-8 --

import rospy
from serial import Serial
from std_msgs.msg import String
from sensor_msgs.msg import Joy

ser = Serial('/dev/ttyACM0', 9600)

def callback():
    pub_x = rospy.Publisher('/linear_x/joystick_teleop', Joy, queue_size=10)
    pub_y = rospy.Publisher('/linear_y/joystick_teleop', Joy, queue_size=10)
    
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        if ser.readable():
            try:
                res = ser.readline()
                res = res.decode()
                res_forward = res.split("*")[0]
                res_backward = res.split("*")[1]
                
                buttons = list(map(int, res_backward.split(":")))
                axes = list(map(int, res_forward.split(":")))
            
            except:
                print("Pass")

            else:
                joy = Joy()
                joy.axes = axes
                joy.buttons = buttons
                rate.sleep()
                
                pub_x.publish(joy)
                pub_y.publish(joy)

def on_shutdown():
    ser.close()
    rospy.loginfo('Stopping Reading')

def init_node():
    rospy.init_node('joystick_node', anonymous=True)
    callback()
    rospy.on_shutdown(on_shutdown)
    rospy.spin()

if __name__ == '__main__':
    init_node()
