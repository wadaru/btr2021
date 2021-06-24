#!/usr/bin/python

import struct
import time
import math
import sys
import rospy
from std_srvs.srv import Empty, EmptyResponse

def goToMPSCenter():
    rospy.wait_for_service('/rvw2/goToMPSCenter')
    goToMPSCenter = rospy.ServiceProxy('/rvw2/goToMPSCenter', Empty)
    print("goToMPSCenter")
    resp = goToMPSCenter()
    print("reached")
    
# main
#
if __name__ == '__main__':

  rospy.init_node('btr2021')
  rate = rospy.Rate(10)

  # while True:
  while not rospy.is_shutdown():
    goToMPSCenter()
    rate.sleep()


