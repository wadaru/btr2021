#!/usr/bin/python
TEAMNAME = "BabyTigers"

FIELDMINX = -5
FIELDMAXX = -1
FIELDMINY =  1
FIELDMAXY =  5
FIELDSIZEX = (FIELDMAXX - FIELDMINX) + 1
FIELDSIZEY = (FIELDMAXY - FIELDMINY) + 1
FIELDSIZE = FIELDSIZEX * FIELDSIZEY
MAXSTEP = 999

import struct
import time
import math
import sys
import rospy
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Point, Quaternion
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Int8, Int16, UInt32, String, \
                         Float32, Float32MultiArray, \
                         Bool, Header
from std_srvs.srv import SetBool, SetBoolResponse, Empty, EmptyResponse
from nav_msgs.msg import Odometry
import rcll_ros_msgs
import rcll_btr_msgs
from rcll_btr_msgs.srv import SetOdometry, SetPosition, SetVelocity, \
                              SetDistance
from rcll_ros_msgs.msg import BeaconSignal, ExplorationInfo, \
                              ExplorationSignal, ExplorationZone, GameState, \
                              LightSpec, MachineInfo, Machine, \
                              MachineReportEntry, MachineReportEntryBTR, \
                              MachineReportInfo, OrderInfo, Order, \
                              ProductColor, RingInfo, Ring, Team, Time, \
                              NavigationRoutes, Route
from rcll_ros_msgs.srv import SendBeaconSignal, SendMachineReport, \
                              SendMachineReportBTR, SendPrepareMachine

def setOdometry(data):
    odometry = SetOdometry()
    pose = Pose2D()
    rospy.wait_for_service('/rvw2/setOdometry')
    setOdometry = rospy.ServiceProxy('/rvw2/setOdometry', SetOdometry)
    odometry.header = Header()
    pose = data
    odometry.pose = pose
    resp = setOdometry(odometry.header, odometry.pose)

def goToPoint(x, y, theta):
    position = SetPosition()
    pose = Pose2D()
    rospy.wait_for_service('/rvw2/positionDriver')
    setPosition = rospy.ServiceProxy('/rvw2/positionDriver', SetPosition)
    position.header = Header()
    pose.x = x
    pose.y = y
    pose.theta  = theta
    position.pose = pose
    print("send")
    resp = setPosition(position.header, position.pose)
    print("goToPoint")

def moveRobotino(x, y, theta):
    position = SetPosition()
    pose = Pose2D()
    rospy.wait_for_service('/rvw2/move')
    setPosition = rospy.ServiceProxy('/rvw2/move', SetPosition)
    position.header = Header()
    pose.x = x
    pose.y = y
    pose.theta  = theta
    position.pose = pose
    print("send")
    resp = setPosition(position.header, position.pose)
    print("goToPoint")

def goToInputVelt():
    goToMPSCenter(350)

def goToOutputVelt():
    goToMPSCenter(310)

def goToMPSCenter(distance):
    setDistance = SetDistance()
    rospy.wait_for_service('/rvw2/goToMPSCenter')
    goToMPSCenter = rospy.ServiceProxy('/rvw2/goToMPSCenter', SetDistance)
    setDistance.header = Header()
    setDistance.distance = Int16(distance)
    print("goToMPSCenter", distance)
    resp = goToMPSCenter(setDistance.header, setDistance.distance)
    print("reached")

def turnClockwise():
    rospy.wait_for_service('/rvw2/turnClockwiseMPS')
    turnClockwise = rospy.ServiceProxy('/rvw2/turnClockwiseMPS', Empty)
    print("turnClockwiseMPS")
    resp = turnClockwise()

def turnCounterClockwise():
    rospy.wait_for_service('/rvw2/turnCouinterClockwiseMPS')
    turnCounterClockwise = rospy.ServiceProxy('/rvw2/turnCounterClockwiseMPS', Empty)
    print("turnCounterClockwiseMPS")
    resp = turnCounterClockwise()
    
def getWork():
    rospy.wait_for_service('/btr/move_g')
    getWork = rospy.ServiceProxy('/btr/move_g', Empty)
    print("getWork")
    resp = getWork()

def putWork():
    rospy.wait_for_service('btr/move_r')
    putWork = rospy.SericeProxy('/btr/move_r', Empty)
    print("putWOrk")
    resp = putWork()

def beaconSignal(data):
    global refboxBeaconSignal
    refboxBeaconSignal = data
    # print("BeaconSignal: ", data)

def explorationInfo(data):
    global refboxExplorationInfo
    refboxExplorationInfo = data
    # print("ExplorationInfo: ", data)

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
    global refboxMachineInfo, refboxMachineInfoFlag
    refboxMachineInfo = data
    refboxMachineInfoFlag = True
    # print("MachineInfo: ", data)

def machineReportInfo(data):
    global refboxMachineReportInfo
    refboxMachineReportInfo = data
    # print("MachineReportInfo: ", data)

def orderInfo(data):
    global refboxOrderInfo
    refboxOrderInfo = data
    # print("OrderInfo: ", data)

def ringInfo(data):
    global refboxRingInfo
    refboxRingInfo = data
    # print("RingInfo: ", data)

def navigationRoutes(data):
   global refboxNavigationRoutes, refboxNavigationRoutesFlag
   refboxNavigationRoutes = data
   refboxNavigationRoutesFlag = True
   # print("NavigaionRoutes: ", data)

#
# send information to RefBox
#
def sendBeacon():
    beacon = SendBeaconSignal()
    header1 = Header()
    header2 = Header()
    poseStamped = PoseStamped()
    pose = Pose()
    
    # pose.position = point
    # set Pose
    pose.position.x = btrOdometry.pose.pose.position.x
    pose.position.y = btrOdometry.pose.pose.position.y
    pose.position.z = 0
    # set quaternion
    # theta = math.radians(float(udp.view3Recv[4]) / 10)
    pose.orientation.x = btrOdometry.pose.pose.orientation.x # math.cos(theta / 2.0)
    pose.orientation.y = btrOdometry.pose.pose.orientation.y # math.sin(theta / 2.0)
    pose.orientation.z = btrOdometry.pose.pose.orientation.z # math.sin(theta / 2.0)
    pose.orientation.w = btrOdometry.pose.pose.orientation.w # 0
    pose = btrOdometry.pose.pose
    header1.seq = 1
    header1.stamp = rospy.Time.now()
    header1.frame_id = TEAMNAME
    header2.seq = 1
    header2.stamp = rospy.Time.now()
    header2.frame_id = "robot1"
    poseStamped.header = header2
    poseStamped.pose = pose
    beacon.header = header1
    beacon.pose  = poseStamped

    rospy.wait_for_service('/rcll/send_beacon')
    try:
        refboxSendBeacon = rospy.ServiceProxy('/rcll/send_beacon', SendBeaconSignal)
        resp1 = refboxSendBeacon(beacon.header, beacon.pose)
        # print("sendBeacon: ", beacon.header, beacon.pose)
        # print("resp: ", resp1)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def sendMachineReport(report):
    sendReport = SendMachineReport()
    machineReport = MachineReportEntryBTR()
    machineReport.name = report.name
    machineReport.type = report.type
    machineReport.zone = report.zone
    machineReport.rotation = report.rotation
    if (refboxTeamCyan == TEAMNAME):
        sendReport.team_color = 1
    else:
        sendReport.team_color = 2
    machineReportEntryBTR = [machineReport]
    sendReport.machines = machineReportEntryBTR
    print("machineReport: ", machineReport)

    rospy.wait_for_service('/rcll/send_machine_report')
    try:
        refboxMachineReport = rospy.ServiceProxy('/rcll/send_machine_report', SendMachineReportBTR)
        resp1 = refboxMachineReport(sendReport.team_color, sendReport.machines)
        # print("resp: ", resp1)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def sendPrepareMachine(data):
    prepare = SendPrepareMachine()
    # prepare.machine = data.machine
    prepare.machine = data.machine
    prepare.bs_side = 0
    prepare.bs_base_color = 0
    prepare.ds_order_id = 0
    prepare.cs_operation = 0
    prepare.rs_ring_color =0
    
    machineType = prepare.machine[2:4]
    print(machineType)
    if (machineType == "BS"):
        prepare.bs_side = data.bs_side
        prepare.bs_base_color =data.bs_base_color
    if (machineType == "DS"):
        prepare.ds_order_id = data.ds_order_id
    if (machineType == "CS"):
        prepare.cs_operation = data.cs_operation
    if (machineType == "RS"):
        prepare.rs_ring_color = data.rs_ring_color
    prepare.wait = data.wait
    rospy.wait_for_service('/rcll/send_prepare_machine')
    try:
        refboxPrepareMachine = rospy.ServiceProxy('/rcll/send_prepare_machine', SendPrepareMachine)
        resp1 = refboxPrepareMachine(prepare.machine, prepare.wait, prepare.bs_side, prepare.bs_base_color, prepare.ds_order_id, prepare.rs_ring_color, prepare.cs_operation)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def robotinoOdometry(data):
    global btrBeaconCounter
    btrOdometry = data
    btrBeaconCounter +=1
    if (btrBeaconCounter > 5):
      sendBeacon()

def robotinoVelocity(data):
    btrVelocity = data

# 
# challenge program
#

def startGrasping():
    for i in range(3):
        goToInputVelt()
        #
        # please write here
        #   to cobotta get the work
        #
        if (robotNum != 2):
            turnClockwise()
        else:
            turnCounterClockwise()
        goToMPSOutputVelt()
        #
        # please write here
        #   to cobotta put the work
        #
        if (RobotNum != 2):
            turnCounterClockwise()
        else:
            turnClockwise()
        # finish?

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
    btrField[y - FIELDMINY][x - FIELDMINX] = number

def getField(x, y):
    return btrField[y - FIELDMINY][x - FILEDMINX]

def zoneToPose2D(zone):
    point = Pose2D()
    if zone < 0:        
        zone = zone + 255
    point.x = zone % 1
    point.y = (zone % 100) // 10
    if (zone > 100):
        point.x = -point.x
    return point

def setMPStoField():
    global btrField
    point = Pose2D()
    for machine in refboxMachineInfo.machines:
        point = zoneToPose2D(machine.zone)
        print(machine.name, point.x, point.y)
    setField(point.x, point.y, MAXSTEP)

def getStep(x, y):
    step = getField(x, y)
    if (step == 0):
        step = MAXSTEP
    return step

def makeNextPoint(destination):
    global btrField
    tmpField = btrField
    setField(destination.x, destination.y, 1)
    for i in range(FILEDSIZE):
        for x in range(FIELDMINX, FIELDMAXX + 1):
            for y in range(FIELDMINY, FIELDMAXY + 1):
                if (x == destination.x and y == destination.y):
                    setField(destination.x, destination.y, 1)
                else:
                    setField(x, y, min(getStep(x - 1, y), getStep(x, y - 1), \
                                       getSTep(x + 1, y), getStep(x, y + 1)) \
                                   + 1)

    # get optimized route
    for y in range(FIELDMINY, FIELDMAXY + 1):
        for x in range(FIELDMINX, FIELDMAXX + 1):
            print getField(x, y), 
        print()
    #




def getNextPoint():
    point = Pose2D()
    route = refboxNavigationRoutes.route
    zone = route[0].zone

    print(zone)
    makeNextPoint(zone)


def startNavigation():
    global btrField
    initField()
    setMPStoField()
    getNextPoint()
    # print(refboxNavigationRoutes)
    # print(refboxMachineInfo)


# main
#
if __name__ == '__main__':
  args = sys.argv
  if (len(args) >= 2):
    challenge = args[1]
    if (len(args) >= 3):
      robotNum = int(args[2])
    else:
      robotNum = 1

  # valiables for refbox
  refboxBeaconSignal = BeaconSignal()
  refboxExplorationInfo = ExplorationInfo()
  refboxExplorationSignal = ExplorationSignal()
  refboxExplorationZone = ExplorationZone()
  refboxGameState = Int8()
  refboxGamePhase = Int8()
  refboxPointsMagenta = UInt32()
  refboxTeamMagenta = String()
  refboxPointsCyan = UInt32()
  refboxTeamCyan = String()
  refboxLightSpec = LightSpec()
  refboxMachineInfo = MachineInfo()
  refboxMachine = Machine()
  refboxMachineReportEntry = MachineReportEntryBTR()
  refboxMachineReportInfo = MachineReportInfo()
  refboxOrderInfo = OrderInfo()
  refboxOrder = Order()
  refboxProductColor = ProductColor()
  refboxRingInfo = RingInfo()
  refboxRing = Ring()
  refboxTeam = Team()
  refboxTime = Time()
  refboxNavigationRoutes = NavigationRoutes()
  refboxMachineInfoFlag = False
  refboxNavigationRoutesFlag = False

  btrOdometry = Odometry()
  btrBeaconCounter = 0
  btrVelocity = Float32MultiArray()

  btrField = [[0 for y in range(5)] for x in range(5)]

  rospy.init_node('btr2021')
  rospy.Subscriber("rcll/beacon", BeaconSignal, beaconSignal)
  rospy.Subscriber("rcll/exploration_info", ExplorationInfo, explorationInfo)
  rospy.Subscriber("rcll/game_state", GameState, gameState)
  rospy.Subscriber("rcll/machine_info", MachineInfo, machineInfo)
  rospy.Subscriber("rcll/machine_report_info", MachineReportInfo, machineReportInfo)
  rospy.Subscriber("rcll/order_info", OrderInfo, orderInfo)
  rospy.Subscriber("rcll/ring_info", RingInfo, ringInfo)
  rospy.Subscriber("robotino/odometry", Odometry, robotinoOdometry)
  rospy.Subscriber("robotino/getVelocity", Float32MultiArray, robotinoVelocity)
  rospy.Subscriber("rcll/routes_info", NavigationRoutes, navigationRoutes)
  rate = rospy.Rate(10)

  machineReport = MachineReportEntryBTR()
  prepareMachine = SendPrepareMachine() 

  pose = Pose2D()
  pose.x = -1000 * robotNum - 1500
  pose.y = 500
  pose.theta = 90
  if (challenge == "grasping"):
      startX =     [ -500, -4500, -500]
      startY =     [  500,  1500, 4500]
      startTheta = [   90,    90,  180]
      pose.x = startX[robotNum - 1]
      pose.y = startY[robotNum - 1]
      pose.theta = startTheta[robotNum - 1]
      print(pose.x, pose.y, pose.theta)
  setOdometry(pose)

  print(challenge)
  challengeFlag = True
  # while True:
  while not rospy.is_shutdown():
    # sendBeacon()
    # print("sendBeacon")
    if (refboxGamePhase == 30 and challenge == "grasping"):
        startGrasping()

    if (refboxGamePhase == 30 and challenge == "navigation"):
        if (refboxMachineInfoFlag and refboxNavigationRoutesFlag):
            startNavigation()
        

    if (challenge == "test" and challengeFlag):
        # moveRobotino(-100, 0, 0)
        # print("goToPoint")
        # goToPoint(1, 0, 0)
        # goToMPSCenter()
        goToOutputVelt()
        getWork()
        turnClockwise()
        goToIuputVelt()
        putWork()
        turnCounterClockwise()
        challengeFlag = False

    # send machine report for Exploration Phase
    if (refboxGamePhase == 20):
        if (refboxTime.sec == 10):
            machineReport.name = "C-CS1"
            machineReport.type = "CS"
            machineReport.zone = -53 
            machineReport.rotation = 210
            sendMachineReport(machineReport)

    # send machine prepare command
    if (refboxGamePhase == 30):
        # make C0
        # which requires get base with cap from shelf at C-CS1, 
        #                Retrieve cap at C-CS1,
        #                bring base without cap to C-RS1,
        #                get base at C-BS,
        #                bring base to C-CS1,
        #                Mount cap at C-CS1,
        #                bring it to C-DS corresponded by order it.

        if (refboxTime.sec ==   5):
            prepareMachine.machine = "C-CS1"
            prepareMachine.cs_operation = 1 # CS_OP_RETRIEVE_CAP
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)
        if (refboxTime.sec ==  30):
            prepareMachine.machine = "C-BS"
            prepareMachine.bs_side = 1  # INPUT or OUTPUT side
            prepareMachine.bs_base_color = 1 # BASE COLOR
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)
        if (refboxTime.sec ==  60):
            prepareMachine.machine = "C-CS1"
            prepareMachine.cs_operation = 0 # CS_OP_MOUNT_CAP
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)
        if (refboxTime.sec ==  90):
            prepareMachine.machine = "C-DS"
            prepareMachine.ds_order_id = 1 # ORDER ID
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)


    rate.sleep()


