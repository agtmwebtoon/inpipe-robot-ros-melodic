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

pub_x = rospy.Publisher('/x_control/pid_controller', Int32, queue_size=10)
pub_y = rospy.Publisher('/y_control/pid_controller', Int32, queue_size=10)

init_angle = []
first_flag = True

def calc_pose(pose_data, setpoint_data=[0, 0]):
    global first_flag, init_angle

    quaternions = pose_data.orientation

    yaw, roll, pitch = euler_from_quaternion([quaternions.x,  quaternions.y,  quaternions.z, quaternions.w])
    

    roll *= 57.2958
    pitch *= 57.2958
    yaw *= 57.2958
    
    
    if yaw > 0:
        yaw -= 180
    elif yaw < 0:
        yaw += 180

    
    if first_flag:
        init_angle.append(roll)
        init_angle.append(pitch)
        init_angle.append(yaw)
        first_flag = False
    
    if init_angle[0] > 0: roll += init_angle[0]
    else: roll -= init_angle[0]

    if init_angle[1] > 0: pitch += init_angle[1]
    else: pitch -= init_angle[1]

    if init_angle[2] > 0: yaw += init_angle[2]
    else: yaw -= init_angle[2]
    
    print(str(roll) + " " + str(pitch) + " " + str(yaw))
    
    print(init_angle)
    pid_control_pitch.setpoint = 0
    pid_control_yaw.setpoint = 0
    
    control_pitch = pid_control_pitch(pitch)
    control_yaw = pid_control_yaw(yaw)
    
    pub_x.publish(45 + control_yaw)
    pub_y.publish(45 + control_pitch)
    rate.sleep()

def listener():

    #joy_sub = message_filters.Subscriber('joystick_teleop', Joy)
    #imu_sub = message_filters.Subscriber('chatter', Imu)
    
    #time_sync = message_filters.TimeSynchronizer([imu_sub, joy_sub], 100)
    #time_sync.registerCallback(calc_pose)
    
    sub = rospy.Subscriber("chatter", Imu, calc_pose)
    rospy.spin()

if __name__ == '__main__':
    
    rospy.init_node('pid')
    rate = rospy.Rate(100) # 10hz

    pid_control_pitch = PID(1, 0.1, 0.05, setpoint=0)
    pid_control_yaw = PID(1, 0.1, 0.05, setpoint=0)
    pid_control_pitch.output_limits = (-30, 30)
    pid_control_yaw.output_limits = (-30, 30)

    listener()
