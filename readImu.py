import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import serial
import sys
import numpy as np


def readImu():
  imu = Imu()
  rospy.init_node('imu')
  imu_pub = rospy.Publisher('imu', Imu, queue_size = 1)
  ser = serial.Serial(
    port='/dev/ttyS0',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=1)

  print("connected to: " + ser.portstr)
  i = 0
  while True:
   buff = ser.readline() #(1000)
   if len(buff) > 0:
      sp = buff.split(' ')
      if (sp[0] == 'Orientation:'):
        x = float(sp[1])
        y = float(sp[2])
        z = float(sp[3])
        imu.orientation.x = z * np.pi/180.0
        imu.orientation.y = y * np.pi/180.0
        imu.orientation.z = x * np.pi/180.0
        imu_pub.publish(imu)
        if (i >= 100):
      	  print 'x = ',x,'  y = ', y,'  z = ', z
          i = 0
        else:
          i = i+1

  ser.close()
while(True):
  try:
    readImu()
  except:
    print "read IMU Error"
