#!/usr/bin/env python
#!/usr/bin/python

START_ANGLE = -90
END_ANGLE = 90
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_srvs.srv import Empty

#
def scanDistance(deg):
  return scanData.ranges[len(scanData.ranges) / 360 * ((deg + 360) % 360)]

#
def calcPoint():
  minDistance = scanDistance(0)
  minAngle = 0
  for i in range(START_ANGLE, END_ANGLE):
    if (minDistance > scanDistance(i)):
      minDistance = scanDistance(i)
      minAngle = i
  print("minAngle:", minAngle, ", minDistance:", minDistance)

#
def laserScan(data):
  global scanData
  scanData = data
  # scanNumber = len(scanData.ranges)
  if (scanFlag == True):
    calcPoint()
  else:
    print   "0:", scanDistance(  0), \
           "90:", scanDistance( 90), \
          "180:", scanDistance(180), \
          "270:", scanDistance(-90)

#
def btrScanStart(self):
  global scanFlag
  scanFlag = True
  return

#
def btrScanStop(self):
  global scanFlag
  scanFlag = False
  return

# main
#
if __name__ == '__main__':
  # args = sys.argv
  # if (len(args) == 1):
  #   challenge = args[1]

  scanFlag = False
  centerPoint = Point()
  leftPoint = Point()
  rightPoint = Point()

  rospy.init_node('btr_scan')
  sub01 = rospy.Subscriber("/scan", LaserScan, laserScan)
  srv01 = rospy.Service("/btr/scan_start", Empty, btrScanStart)
  srv02 = rospy.Service("/btr/scan_stop", Empty, btrScanStop)
  pub01 = rospy.Publisher("/btr/centerPoint", Point, queue_size = 10)
  pub02 = rospy.Publisher("/btr/leftPoint", Point, queue_size = 10)
  pub03 = rospy.Publisher("/btr/rightPoint", Point, queue_size = 10)
  rate = rospy.Rate(10)

  rospy.spin()

  scanData = LaserScan
  
  while not rospy.is_shutdown():
    if (scanFlag == True):
      pub01.publish(centerPoint)
      pub02.publish(leftPoint)
      pub03.publish(rightPoint)
      rate.sleep()

