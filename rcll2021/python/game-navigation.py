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

#nav_alg
import numpy as np
import robot1_route_re as ro1
#import robot3_route_re as ro3
import robot3_route_re as ro3


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

def initField():
    global btrField
    btrField = [[0 for y in range(5)] for x in range(5)]
    #
    # this field is [y][x]
    # but game field is from -5 to -1
    # so when you use this variable, please shift argment + 5.
    #   (-5, 5) => (0, 4)
    #   (-5 ,1) => (0, 0)

def setField(x, y, number):
    btrField[y - 1][x + 5] = number

def getField(x, y):
    return btrField[y - 1][x + 5]

def setMPStoField():
    global btrField
    obstacles = np.array([[0,0]] * len(refboxMachineInfo.machines))
    i = 0
    for machine in refboxMachineInfo.machines:
        zone = machine.zone
        if zone < 0:
            zone = zone + 255
        y = zone % 10
        x = (zone % 100) // 10
        xx = x
        if (zone > 100):
            x = -x
        # print(machine.name, x, y)
        # print(obstacles, machine, xx, y)
        obstacles[i] = [xx,y]
        i += 1
    if (x !=0 and y !=0) :setField(x, y, 999)
    return obstacles

def getNextPoint():
    point = Pose2D()
    route = refboxNavigationRoutes.route
    targets = np.array([[0,0]]*12)
    #zone = route[0].zone
    for i in range(12):
        zone = route[i].zone
        if zone < 0:
            zone = zone + 255
        y = zone % 10
        x = (zone % 100) // 10
        xx = x
        if (zone > 100):
            x = -x
        targets[i] = [xx, y]
    #print(zone)
    return targets


def startNavigation():
    global btrField
    initField()
    global obstacles,targets
    obstacles = setMPStoField()
    targets = getNextPoint()
    # print(refboxNavigationRoutes)
    # print(refboxMachineInfo)
    return()

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

  rospy.init_node('btr2021Navigation')
  rospy.Subscriber("rcll/game_state", GameState, gameState)
  rospy.Subscriber("rcll/machine_info", MachineInfo, machineInfo)
  rospy.Subscriber("rcll/routes_info", NavigationRoutes, navigationRoutes)
  
  pub1 = rospy.Publisher("btr/route1", Float32MultiArray, queue_size = 10)
  pub2 = rospy.Publisher("btr/route2", Float32MultiArray, queue_size = 10)
  pub3 = rospy.Publisher("btr/route3", Float32MultiArray, queue_size = 10)
  
  rate = rospy.Rate(10)
  RR1 = [0, 0]
  print(challenge)
  # while True:
  while not rospy.is_shutdown():

    if (challenge == "navigation"):
        if (refboxFlagGetNaviInfo and refboxFlagGetMachineInfo):
          print(refboxNavigationRoutes)
          print(refboxMachineInfo)

          startNavigation()
          print(targets)

          if (int(args[2]) == 3):
              RR1,RR2,RR3 = ro3.plan(targets,obstacles)
              pub1.publish(RR1)
              pub2.publish(RR2)
              pub3.publish(RR3)
          else:
              RR1 = ro1.plan(targets, obstacles)
              pub1.publish(RR1)

    rate.sleep()


