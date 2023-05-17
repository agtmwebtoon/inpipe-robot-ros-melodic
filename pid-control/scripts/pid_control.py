#!/usr/bin/env python

import rospy
import message_filters
from pid import PID

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

pub = rospy.Publisher('pid_controller', Int32, queue_size=10)

def calc_pose(pose_data, setpoint_data):
 
    quaternions = pose_data.orientation

    print(quaternions)
    roll, pitch, yaw = euler_from_quaternion([quaternions.x,  quaternions.y,  quaternions.z, quaternions.w])

    pid_control_pitch.setpoint = setpoint_data.axes[0] * (180.0 / 970) - 90
    print(setpoint_data.axes[0] * (180.0 / 970) - 90)
    control = pid_control_pitch(pitch * 57.2958)
    print(pitch* 57.2958)
    pub.publish(90 + control)
    rate.sleep()

def listener():

    joy_sub = message_filters.Subscriber('joystick_teleop', Joy)
    imu_sub = message_filters.Subscriber('chatter', Imu)
    
    print(imu_sub)
    time_sync = message_filters.TimeSynchronizer([imu_sub, joy_sub], 100)
    time_sync.registerCallback(calc_pose)
    
    rospy.spin()

if __name__ == '__main__':
    
    rospy.init_node('pid')
    rate = rospy.Rate(100) # 10hz

    pid_control_pitch = PID(1, 0.1, 0.05, setpoint=45) 
    pid_control_pitch.output_limits = (-30, 30)
    listener()
