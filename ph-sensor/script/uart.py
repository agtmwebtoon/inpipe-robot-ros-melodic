#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import sys
import time
import string 
from serial import SerialException
from std_msgs.msg import Float32

def read_line():
   """
   taken from the ftdi library and modified to 
   use the ezo line separator "\r"
   """
   lsl = len(b'\r')
   line_buffer = []
   while True:
      next_char = ser.read(1)
      if next_char == b'':
         break
      line_buffer.append(next_char)
      if (len(line_buffer) >= lsl and
            line_buffer[-lsl:] == [b'\r']):
         break
   return b''.join(line_buffer)
   
def read_lines():
   """
   also taken from ftdi lib to work with modified readline function
   """
   lines = []
   try:
      while True:
         line = read_line()
         if not line:
            break
            ser.flush_input()
         lines.append(line)
      return lines
   
   except SerialException as e:
      print( "Error, ", e)
      return None    

def send_cmd(cmd):
   """
   Send command to the Atlas Sensor.
   Before sending, add Carriage Return at the end of the command.
   :param cmd:
   :return:
   """
   buf = cmd + "\r"       # add carriage return
   try:
      ser.write(buf.encode('utf-8'))
      return True
   except SerialException as e:
      print ("Error, ", e)
      return None
         
if __name__ == "__main__":
   
   usbport = '/dev/ttyAMA1' # change to match your pi's setup
   rospy.init_node('pH_data_publisher', anonymous=True)
   pH_pub = rospy.Publisher('pH_data', Float32, queue_size=10)

   print( "Opening serial port now...")

   try:
      ser = serial.Serial(usbport, 9600, timeout=0)
   except serial.SerialException as e:
      print("Error, ", e)
      sys.exit(0)

   while True and not rospy.is_shutdown():
      send_cmd("R")
      lines = read_lines()
      try:
        for i in range(len(lines)):
            if lines[i][0] != b'*'[0]:   
                pH = Float32()
                pH.data = float(lines[i].decode('utf-8'))
                print(lines[i].decode('utf-8'))
                pH_pub.publish(pH)

      except:
          pass
