"""spot_flat_walk controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor, Motor, Camera, LED, Lidar
import math, copy
import numpy as np

from SpotKinematics import SpotModel
from Bezier import BezierGait

class Sim:
  def __init__(self):
    # CREATE ROBOT INSTANCE
    self.robot = Supervisor()
    self.spot_node = self.robot.getFromDef("Spot")

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
    self.LIDAR_NAME =['Velodyne Puck']
    self.lidar.append(self.robot.getDevice(self.LIDAR_NAME[0]))
    self.lidar[0].enable(self.TIME_STEP)
    self.lidar[0].enablePointCloud()
    self.LIDAR_RESOLUTION = self.lidar[0].getHorizontalResolution()
    self.layers = self.lidar[0].getNumberOfLayers()

    # INIT CONTACT SENSORS
    self.touchSensors = []
    self.TOUCH_SENSOR_NAMES = ['front left touch sensor','front right touch sensor',
                              'rear left touch sensor','rear right touch sensor']
    for i in range(len(self.TOUCH_SENSOR_NAMES)):
      self.touchSensors.append(self.robot.getDevice(self.TOUCH_SENSOR_NAMES[i]))
      self.touchSensors[i].enable(self.TIME_STEP)

    ## Spot Control
    self.spotModel = SpotModel()
    # spot_node=robot.getFromDef("Spot")
    self.T_bf0 = self.spotModel.WorldToFoot
    self.T_bf = copy.deepcopy(self.T_bf0)
    self.bzg = BezierGait(dt=self.TIME_STEP/1000)

    # ------------------ Inputs for Bezier Gait control ----------------
    self.yaw_d = 0.0
    self.Step_Length = 0.0
    self.Lateral_Fraction = 0.0
    self.Yaw_Rate = 0.0
    self.Step_Velocity = 0.0
    self.Clearance_Height = 0.0
    self.Penetration_Depth = 0.0
    self.Swing_Period = 0.0
    self.Yaw_Control = 0.0
    self.Yaw_Control_On = False

    # ------------------ Outputs of Contact sensors ----------------
    self.legContacts = [1, 1, 1, 1]
    self.chatteringLegContacts = [0, 0, 0, 0]
    self.CHATTERING_LIM = 4

  # JOINT MOVEMENT DECOMPOSITION
  def movement_decomposition(self, target, duration):
    STEP_NUM = duration * 1000 / self.TIME_STEP
    step_difference = []
    current_position = []

    for i in range(len(self.MOTOR_NAMES)):
      current_position.append(self.motors[i].getTargetPosition())
      step_difference.append((target[i] - current_position[i]) / STEP_NUM)
    

    for i in range(math.floor(STEP_NUM)):
      for j in range(len(self.MOTOR_NAMES)):
        current_position[j] += step_difference[j]
        self.motors[j].setPosition(current_position[j])
        
      self.step()

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
    yaw_target = self.Yaw_Control
    thr = np.pi / 2
    if (yaw_target > thr and self.yaw_dot < -thr) or (self.yaw_dot > thr and yaw_target < -thr):
      residual = (yaw_target - self.yaw_dot) * np.sign(yaw_target - self.yaw_dot) - 2 * np.pi
      yawrate_d = 2.0 * np.sqrt(abs(residual)) * np.sign(residual)
    else:
      residual = yaw_target - self.yaw_dot
      yawrate_d = 4.0 * np.sqrt(abs(residual)) * np.sign(residual)
    return yawrate_d

  def inverse_control(self, pos, orn):
    # yaw controller
    if self.Yaw_Control_On == 1.0:
      YawRate_desired = self.yaw_control()
    else:
      YawRate_desired = self.Yaw_Rate

    # Update Swing Period
    self.bzg.Tswing = self.Swing_Period

    # Get Desired Foot Poses
    T_bf = self.bzg.GenerateTrajectory(self.Step_Length, self.Lateral_Fraction, YawRate_desired,
                                        self.Step_Velocity, self.T_bf0, self.T_bf,
                                        self.Clearance_Height, self.Penetration_Depth,
                                        self.legContacts)

    joint_angles = -self.spotModel.IK(orn, pos, T_bf)

    motors_target_pos = [
      joint_angles[0][0], joint_angles[0][1], joint_angles[0][2],
      joint_angles[1][0], joint_angles[1][1], joint_angles[1][2],
      joint_angles[2][0], joint_angles[2][1], joint_angles[2][2],
      joint_angles[3][0], joint_angles[3][1], joint_angles[3][2],
    ]

    self.setMotorPositions(motors_target_pos)

# MAIN METHOD
def main():
  sim = Sim()

  # SIM LOOP
  while True:
    # SET DESIRED POSE
    pos = np.array([0, 0, 0.3])
    orn = np.array([0, 0, 0])

    # CALL INVERSE CONTROL ON DESIRED POSE
    sim.inverse_control(pos,orn)

    # STEP THE SIMULATION FRWD
    sim.step()

if __name__=='__main__':
  main()