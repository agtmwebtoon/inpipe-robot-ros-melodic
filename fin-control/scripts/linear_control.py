#!/usr/bin/env python

import rospy, time
import RPi.GPIO as GPIO
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Joy

SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3


rospy.init_node('servo_node', anonymous=True)

G = int(rospy.get_param('~pin_num'))
f = int(rospy.get_param('~frequency'))
mx = int(rospy.get_param('~max_pulse_width_us'))
mn = int(rospy.get_param('~min_pulse_width_us'))

GPIO.setmode(GPIO.BCM)
GPIO.setup(G, GPIO.OUT)

servo = GPIO.PWM(G, 50)
servo.start(3)


def callback(data):
    print(data)
    transformed_x = data.data
    
    if transformed_x > 180:
        transformed_x = 180
    
    if transformed_x < 0:
        transformed_x = 0
    
    duty = SERVO_MIN_DUTY + (transformed_x *  (SERVO_MAX_DUTY - SERVO_MIN_DUTY)/180.0)

    rospy.loginfo(rospy.get_caller_id() + 'Setting %s', duty)
    servo.ChangeDutyCycle(duty)


def on_shutdown():
    rospy.loginfo('Stopping servo')
    servo.stop()
    GPIO.cleanup()

def listener():

    rospy.Subscriber('pid_controller', Int32, callback)
    rospy.on_shutdown(on_shutdown)
    rospy.spin()

if __name__ == '__main__':
    listener()

