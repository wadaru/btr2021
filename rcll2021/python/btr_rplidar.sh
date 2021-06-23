#!/usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan

def laserScan(msg):
  scanNumber = len(msg.ranges)
  print   "0:", msg.ranges[0], \
	 "90:", msg.ranges[scanNumber / 4], \
	"180:", msg.ranges[scanNumber / 2], \
	"270:", msg.ranges[scanNumber / 4 * 3] 


# main
#
if __name__ == '__main__':
  # args = sys.argv
  # if (len(args) == 1):
  #   challenge = args[1]

  rospy.init_node('btr_scan')
  sub = rospy.Subscriber("/scan", LaserScan, laserScan)
  rospy.spin()


