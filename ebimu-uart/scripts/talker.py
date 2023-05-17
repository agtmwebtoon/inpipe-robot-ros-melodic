#!/usr/bin/env python

import rospy
from serial import Serial
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import geometry_msgs
import tf

roll = pitch = yaw = 0.0

ser = Serial('/dev/ttyUSB0', 9600)

def talker():
    pub_x = rospy.Publisher('/x_control/chatter', Imu, queue_size=10)
    pub_y = rospy.Publisher('/y_control/chatter', Imu, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    
    while not rospy.is_shutdown():

        if ser.readable():
            res = ser.readline()
            
            try:
                received_str = res.decode()[1:len(res)-1]
                print(received_str)
                x, y, z, w = list(map(float, received_str.split(",")))
            
            except:
                print("Pass")

            else:
                p = Imu()
            
                p.orientation.x = x 
                p.orientation.y = y
                p.orientation.z = z
                p.orientation.w = w
                rate.sleep()
                pub_x.publish(p)
                pub_y.publish(p)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
