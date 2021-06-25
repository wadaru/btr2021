#!/usr/bin/python
TEAMNAME = "BabyTigers"

import struct
import time
import math
import sys
import rospy
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Point, Quaternion
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Int8, UInt32, String, Float32, Float32MultiArray, \
                         Bool, Header
from std_srvs.srv import SetBool, SetBoolResponse, Empty, EmptyResponse
from nav_msgs.msg import Odometry
import rcll_ros_msgs
import rcll_btr_msgs
from rcll_btr_msgs.srv import SetOdometry, SetPosition, SetVelocity
from rcll_ros_msgs.msg import BeaconSignal, ExplorationInfo, \
                              ExplorationSignal, ExplorationZone, GameState, \
                              LightSpec, MachineInfo, Machine, \
                              MachineReportEntry, MachineReportEntryBTR, \
                              MachineReportInfo, OrderInfo, Order, \
                              ProductColor, RingInfo, Ring, Team, Time, \
                              NavigationRoutes, Route
from rcll_ros_msgs.srv import SendBeaconSignal, SendMachineReport, \
                              SendMachineReportBTR, SendPrepareMachine

def gameState(data):
    global refboxTime, refboxGameState, refboxGamePhase, \
           refboxPointsMagenta, refboxTeamMagenta, \
           refboxPointCyan, refboxTeamCyan
    refboxTime = data.game_time
    refboxGameState = data.state
    refboxGamePhase = data.phase
    refboxPointsMagenta = data.points_magenta
    refboxTeamMagenta = data.team_magenta
    refboxPointsCyan = data.points_cyan
    refboxTeamCyan = data.team_cyan
    # print("GameState: ", data)

def machineInfo(data):
    global refboxMachineInfo, refboxFlagGetMachineInfo
    refboxMachineInfo = data
    refboxFlagGetMachineInfo = True
    # print("MachineInfo: ", data)

def navigationRoutes(data):
   global refboxNavigationRoutes, refboxFlagGetNaviInfo
   refboxNavigationRoutes = data
   refboxFlagGetNaviInfo = True
   # print("NavigaionRoutes: ", data)

# main
#
if __name__ == '__main__':
  args = sys.argv
  if (len(args) >= 2):
    challenge = args[1]

  # valiables for refbox
  refboxGameState = Int8()
  refboxGamePhase = Int8()
  refboxPointsMagenta = UInt32()
  refboxTeamMagenta = String()
  refboxPointsCyan = UInt32()
  refboxTeamCyan = String()
  refboxMachineInfo = MachineInfo()
  refboxMachine = Machine()
  refboxTeam = Team()
  refboxTime = Time()
  refboxNavigationRoutes = NavigationRoutes()
  refboxFlagGetMachineInfo = False
  refboxFlagGetNaviInfo = False

  rospy.init_node('btr2021')
  rospy.Subscriber("rcll/game_state", GameState, gameState)
  rospy.Subscriber("rcll/machine_info", MachineInfo, machineInfo)
  rospy.Subscriber("rcll/routes_info", NavigationRoutes, navigationRoutes)
  rate = rospy.Rate(10)

  print(challenge)
  # while True:
  while not rospy.is_shutdown():

    if (refboxGamePhase == 30 and challenge == "navigation"):
        if (refboxFlagGetNaviInfo and refboxFlagGetMachineInfo):
          print(refboxNavigationRoutes)
          print(refboxMachineInfo)

    rate.sleep()


