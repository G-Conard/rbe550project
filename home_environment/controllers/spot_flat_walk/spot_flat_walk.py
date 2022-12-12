"""spot_flat_walk controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor, Motor, Camera, CameraRecognitionObject, LED, Lidar
import math, copy
import numpy as np
import csv

from SpotKinematics import SpotModel
from Bezier import BezierGait

from Vector2D import Vector2D
from APF import APF


import sys
sys.path.insert(1, '../../scripts')
from XboxController import XboxController
xbox = XboxController()

class Sim:
  def __init__(self):
    # CREATE ROBOT INSTANCE
    self.robot = Supervisor()
    self.spot_node = self.robot.getFromDef("Spot")

    # Initialize tool to get position of robot
    self.trans_field = self.spot_node.getField("translation")

    # get the time step of the current world.
    self.TIME_STEP = int(self.robot.getBasicTimeStep())

    # INIT MOTORS
    self.MOTOR_INITIAL_POS = [
      0.001, -0.5185, 1.21,
      0.001, -0.5185, 1.21,
      0.001, -0.5185, 1.21,
      0.001, -0.5185, 1.21,
    ]
    self.motors = []
    self.MOTOR_NAMES = [
      'front left shoulder abduction motor',  'front left shoulder rotation motor',  'front left elbow motor',
      'front right shoulder abduction motor', 'front right shoulder rotation motor', 'front right elbow motor',
      'rear left shoulder abduction motor',   'rear left shoulder rotation motor',   'rear left elbow motor',
      'rear right shoulder abduction motor',  'rear right shoulder rotation motor',  'rear right elbow motor'];
    for i in range(len(self.MOTOR_NAMES)):
      self.motors.append(self.robot.getDevice(self.MOTOR_NAMES[i]))
      self.motors[i].setPosition(self.MOTOR_INITIAL_POS[i])
      self.motors[i].setPosition(0)

    # INIT CAMERAS
    self.cameras = []
    self.CAMERA_NAMES = ['left head camera', 'right head camera', 'left flank camera', 'right flank camera', 'rear camera']
    for i in range(len(self.CAMERA_NAMES)):
      self.cameras.append(self.robot.getDevice(self.CAMERA_NAMES[i]))
      self.cameras[i].enable(2 * self.TIME_STEP)
    
    self.objectCams = []
    self.OBJECT_CAMS_NAMES = ['FloorCamera']
    self.objectCams.append(self.robot.getDevice(self.OBJECT_CAMS_NAMES[0]))
    self.objectCams[0].enable(2*self.TIME_STEP)
    self.objectCams[0].recognitionEnable(2*self.TIME_STEP)
    
    # INIT LEDs
    self.leds = []
    self.LED_NAMES = ['left top led', 'left middle up led', 'left middle down led',
                'left bottom led', 'right top led', 'right middle up led',
                'right middle down led', 'right bottom led']
    for i in range(len(self.LED_NAMES)):
      self.leds.append(self.robot.getDevice(self.LED_NAMES[i]))
      self.leds[i].set(1)

    # INIT LIDAR
    self.lidar = []
    # self.LIDAR_NAME =['Velodyne Puck']
    # self.lidar.append(self.robot.getDevice(self.LIDAR_NAME[0]))
    # self.lidar[0].enable(self.TIME_STEP)
    # self.lidar[0].enablePointCloud()
    # self.LIDAR_RESOLUTION = self.lidar[0].getHorizontalResolution()
    # self.layers = self.lidar[0].getNumberOfLayers()
    self.lidar.append(self.robot.getDevice('front top lidar'))
    self.lidar[0].enable(self.TIME_STEP)
    self.lidar[0].enablePointCloud()
    self.lidar.append(self.robot.getDevice('rear lidar'))
    self.lidar[1].enable(self.TIME_STEP)
    self.lidar[1].enablePointCloud()
    self.lidar.append(self.robot.getDevice('front floor lidar'))
    self.lidar[2].enable(self.TIME_STEP)
    self.lidar[2].enablePointCloud()

    # INIT CONTACT SENSORS
    self.touchSensors = []
    self.TOUCH_SENSOR_NAMES = ['front left touch sensor','front right touch sensor',
                              'rear left touch sensor','rear right touch sensor']
    for i in range(len(self.TOUCH_SENSOR_NAMES)):
      self.touchSensors.append(self.robot.getDevice(self.TOUCH_SENSOR_NAMES[i]))
      self.touchSensors[i].enable(self.TIME_STEP)

    ## Spot Control
    self.spotModel = SpotModel()
    self.T_bf0 = self.spotModel.WorldToFoot
    self.T_bf = copy.deepcopy(self.T_bf0)
    self.bzg = BezierGait(dt=self.TIME_STEP/1000)

    # ------------------ Inputs for Bezier Gait control ----------------
    self.yaw_d = -1.0
    self.SPEED=2.0
    self.STEP_DIRECTION=1 #fwd:1; left/right:0.1
    self.TURN=0.0
    self.STEP_LENGTH = 0.024
    self.LATERAL_FRACTION = 0.0
    self.YAW_RATE = 1.0
    self.STEP_VELOCITY = 0.01
    self.CLEARANCE_HEIGHT = 0.024
    self.PENETRATION_DEPTH = 0.003
    self.SWING_PERIOD = 0.2
    self.YAW_CONTROL = 0.0
    self.YAW_CONTROL_ON = False

    # ------------------ Outputs of Contact sensors ----------------
    self.legContacts = [1, 1, 1, 1]
    self.chatteringLegContacts = [0, 0, 0, 0]
    self.CHATTERING_LIM = 4

  def lie_down(self, duration):
    self.MOTORS_TARGET_POS= [-0.40, -0.99, 1.59,   # Front left leg
                        0.40,  -0.99, 1.59,   # Front right leg
                        -0.40, -0.99, 1.59,   # Rear left leg
                        0.40,  -0.99, 1.59]  # Rear right leg
    self.movement_decomposition(self.MOTORS_TARGET_POS, duration)

  def stand_up(self, duration):
    self.MOTORS_TARGET_POS = [-0.1, 0.0, 0.0,   # Front left leg
                        0.1,  0.0, 0.0,   # Front right leg
                        -0.1, 0.0, 0.0,   # Rear left leg
                        0.1,  0.0, 0.0]   # Rear right leg

    self.movement_decomposition(self.MOTORS_TARGET_POS, duration)

  def ground_contacts(self):
    for i in range(len(self.TOUCH_SENSOR_NAMES)):
      if bool(self.touchSensors[i].getValue()) == 0:
        self.chatteringLegContacts[i] += 1
        if self.chatteringLegContacts[i] > self.CHATTERING_LIM:
          self.legContacts[i] = 0
      else:
        self.legContacts[i] = 1
        self.chatteringLegContacts[i] = 0

  def step(self):
    if self.robot.step(self.TIME_STEP) == -1:
      pass

    spot_rot = self.spot_node.getField("rotation")
    spot_rot_val = spot_rot.getSFRotation()
    self.yaw_dot = spot_rot_val[2]

    # UPDATE GROUND CONTACTS
    self.ground_contacts()

  def setMotorPositions(self, motors_target_pos):
    for i, motor in enumerate(self.motors):
      motor.setPosition(motors_target_pos[i] - self.MOTOR_INITIAL_POS[i])

  def yaw_control(self):
    """ Yaw body controller"""
    yaw_target = self.YAW_CONTROL
    thr = np.pi / 2
    if (yaw_target > thr and self.yaw_dot < -thr) or (self.yaw_dot > thr and yaw_target < -thr):
      residual = (yaw_target - self.yaw_dot) * np.sign(yaw_target - self.yaw_dot) - 2 * np.pi
      yaw_rate_d = 2.0 * np.sqrt(abs(residual)) * np.sign(residual)
    else:
      residual = yaw_target - self.yaw_dot
      yaw_rate_d = 4.0 * np.sqrt(abs(residual)) * np.sign(residual)
    return yaw_rate_d

  def inverse_control(self, pos, orn):
    # yaw controller
    if self.YAW_CONTROL_ON == 1.0:
      YAW_RATE_d = self.yaw_control()
    else:
      YAW_RATE_d = self.YAW_RATE

    # Update Swing Period
    self.bzg.Tswing = self.SWING_PERIOD

    # Get Desired Foot Poses
    step_length=self.STEP_LENGTH * self.STEP_DIRECTION * self.SPEED
    yaw_rate=YAW_RATE_d * self.TURN
    T_bf = self.bzg.GenerateTrajectory(step_length, self.LATERAL_FRACTION, yaw_rate,
                                        self.STEP_VELOCITY, self.T_bf0, self.T_bf,
                                        self.CLEARANCE_HEIGHT, self.PENETRATION_DEPTH,
                                        self.legContacts)

    joint_angles = -self.spotModel.IK(orn, pos, T_bf)

    motors_target_pos = [
      joint_angles[0][0], joint_angles[0][1], joint_angles[0][2],
      joint_angles[1][0], joint_angles[1][1], joint_angles[1][2],
      joint_angles[2][0], joint_angles[2][1], joint_angles[2][2],
      joint_angles[3][0], joint_angles[3][1], joint_angles[3][2],
    ]

    self.setMotorPositions(motors_target_pos)

def mapInput(speedInput, turnInput):
  maxSpeed = 1.9
  minSpeed = -1.9
  diffScaleSpeed = maxSpeed - minSpeed
  maxTurn = np.pi/3
  minTurn = -np.pi/3
  diffScaleTurn = maxTurn - minTurn

  maxInputSpeed = 1.0
  minInputSpeed = -1.0
  maxInputTurn = 1.0
  minInputTurn = -1.0
  diffInputSpeed = maxInputSpeed - minInputSpeed
  diffInputTurn = maxInputTurn - minInputTurn

  if abs(speedInput) >= 0.09:
    mapSpeed = ((speedInput - minInputSpeed)*(diffScaleSpeed/diffInputSpeed)+minSpeed)
  else:
    mapSpeed = 0.0
    
  if abs(turnInput) >= 0.09:
    mapTurn = ((-turnInput - minInputTurn)*(diffScaleTurn/diffInputTurn)+minTurn)
  else:
    mapTurn = 0.0
  # print(mapSpeed, mapTurn)

  return mapSpeed, mapTurn


def unitVectorToScale(apfTurn):
  maxTurn = 1.0
  minTurn = -1.0
  diffScaleTurn = maxTurn - minTurn

  maxInputTurn = np.pi/3
  minInputTurn = -np.pi/3
  diffInputTurn = maxInputTurn - minInputTurn

  psi = np.arcsin(apfTurn[1])
    
  
  mapTurn = ((psi - minInputTurn)*(diffScaleTurn/diffInputTurn)+minTurn)
  
  # print(mapTurn)

  return mapTurn

def printInput():
  print("Speed Input: ", xbox.LeftY, "Turn Input: ", -xbox.LeftX)

# MAIN METHOD
def main():
  sim = Sim()
  apf = APF(1000, 6000, 0.75)
  angle = apf.fview(30,30)  # magnitudes of max to min angles (left to right)
  
  #define simulation time variable (seconds)
  t = 0
  startTime = t
  state = 0
  
  dataArray = []

  # SIM LOOP
  while True:
    t+=float(sim.TIME_STEP)*1.0/1000

    now = t - startTime
    
    if state == 1:
      print("State 1")
      if not xbox.Start:
        state = 2
    elif state == 2:
      print("State 2")
      if xbox.Start:
        timeStamp = now
        state = 3
    elif state == 3:
      print("State 3")
      if not xbox.Start and now >= timeStamp + 1:
        timeStamp = now
        runTime = 0
        startTime = t
        state = 4
    elif state == 4:
      # print("State 4")
      # stepDuration = 1
      # self.walk(xbox = xbox, phase = (now - timeStamp) / stepDuration)
      runTime = now
      print(runTime)
      
      imagen0 = sim.lidar[0].getRangeImage()
      imagen1 = sim.lidar[1].getRangeImage()
      imagen2 = sim.lidar[2].getRangeImage()
      frontTop = apf.obstacles(imagen0,angle,2)
      frontFloor = apf.obstacles(imagen2,angle,2)
      rearTop = apf.obstacles(imagen1,angle,2)
      frontTopDist = apf.averageReading(frontTop)
      # print('Front obstacles:','{}' .format(fronttop[0:8])) 
      # print('Front obstacles:','{}' .format(fronttop[250:258]))  
      # print('Front obstacles:','{}' .format(fronttop[503:511])) 
      # print(angle[0],angle[250],angle[511]) 
      # print('Floor obstacles:','{}' .format(frontfloor[250:258]))  
      # print('Rear obstacles:','{}' .format(reartop[0:8]))  
      
      
      # SET DESIRED POSE
      localPos = np.array([0, 0.0, 0.0])
      localOrn = np.array([0, 0, 0.0])
      # # print(sim.trans_field.getSFVec3f())
      # globalPos = sim.spot_node.getPosition()
      # globalOrn = sim.spot_node.getOrientation()
      # zAngle = math.atan2(globalOrn[3],globalOrn[0])
      # print(zAngle)

      # CALL INVERSE CONTROL ON DESIRED POSE
      sim.inverse_control(localPos,localOrn)

      # Adjust speed and turn rate based on joystick input and APF
      v, psi = mapInput(xbox.LeftY, xbox.LeftX)
      apfDirection = apf.plan(psi,frontTop)
      print("APF Unit Vector:", apfDirection)
      sim.TURN = unitVectorToScale(apfDirection)
      if frontTopDist < 0.5 and xbox.LeftY > 0:
        sim.SPEED = 0
      else:
        sim.SPEED = v
      printInput()
      print("Speed Total:", sim.SPEED, "Turn Total:", sim.TURN)

      # print(sim.objectCams[0].getRecognitionNumberOfObjects())
      
      # Save data
      # World position (x,y,z), mapped input speed (scalar), mapped input turn (scalar), adjusted heading from APF (radians), APF direction vector (x, y), adjusted speed (scalar), adjusted turn (scalar)
      dataArray.append([*sim.spot_node.getPosition(), v, -xbox.LeftX, psi, *apfDirection, sim.SPEED, sim.TURN, runTime])
      
      if xbox.Start:
        state = 0
    else:
      print("\nPress START to continue.")
      state = 1

    if xbox.Select:
      print("\nQuadruped Test Complete (Terminated by Xbox Controller).\n")
      print("Saving data...\n")
      with open('dataFile','w') as f:
        write = csv.writer(f)
        write.writerows(dataArray)
      print("Data saved! Terminating\n")  
      break
      
    

    # STEP THE SIMULATION FRWD
    sim.step()

if __name__=='__main__':
  main()
