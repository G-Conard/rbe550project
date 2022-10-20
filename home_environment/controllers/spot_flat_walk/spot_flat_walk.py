"""spot_flat_walk controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Camera, LED, Lidar
import math, copy
import numpy as np

from SpotKinematics import SpotModel
from Bezier import BezierGait

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Initialize the robot's information
NUMBER_OF_JOINTS = 12

# INIT MOTORS
motors_initial_pos = [
  0.001, -0.5185, 1.21,
  0.001, -0.5185, 1.21,
  0.001, -0.5185, 1.21,
  0.001, -0.5185, 1.21,
]
motors = [];
motorNames = [
  'front left shoulder abduction motor',  'front left shoulder rotation motor',  'front left elbow motor',
  'front right shoulder abduction motor', 'front right shoulder rotation motor', 'front right elbow motor',
  'rear left shoulder abduction motor',   'rear left shoulder rotation motor',   'rear left elbow motor',
  'rear right shoulder abduction motor',  'rear right shoulder rotation motor',  'rear right elbow motor'];
for i in range(12):
    motors.append(robot.getDevice(motorNames[i]))
    motors[i].setPosition(motors_initial_pos[i])
    motors[i].setPosition(0)

# INIT CAMERAS
cameras = []
cameraNames = ['left head camera', 'right head camera', 'left flank camera', 'right flank camera', 'rear camera'];
for i in range(5):
    cameras.append(robot.getDevice(cameraNames[i]))
    cameras[i].enable(2 * timestep)

# INIT LEDs
leds = []
ledNames = ['left top led', 'left middle up led', 'left middle down led',
            'left bottom led', 'right top led', 'right middle up led',
            'right middle down led', 'right bottom led'];
for i in range(8):
    leds.append(robot.getDevice(ledNames[i]))
    leds[i].set(1)

# INIT LIDAR
lidar = []
lidarName =['Velodyne Puck']
lidar.append(robot.getDevice(lidarName[0]))
lidar[0].enable(timestep)
lidar[0].enablePointCloud()
resolution = lidar[0].getHorizontalResolution()
layers = lidar[0].getNumberOfLayers()

# INIT CONTACT SENSORS
touch_fl = robot.getDevice("front left touch sensor")
touch_fr = robot.getDevice("front right touch sensor")
touch_rl = robot.getDevice("rear left touch sensor")
touch_rr = robot.getDevice("rear right touch sensor")

touch_fl.enable(timestep)
touch_fr.enable(timestep)
touch_rl.enable(timestep)
touch_rr.enable(timestep)

## Spot Control
spot = SpotModel()
spot_node=robot.getFromDef("Spot")
T_bf0 = spot.WorldToFoot
T_bf = copy.deepcopy(T_bf0)
bzg = BezierGait(dt=robot.timestep/1000)

# ------------------ Inputs for Bezier Gait control ----------------
xd = 0.0
yd = 0.0
zd = 0.0
rolld = 0.0
pitchd = 0.0
yawd = 0.0
StepLength = 0.0
LateralFraction = 0.0
YawRate = 0.0
StepVelocity = 0.0
ClearanceHeight = 0.0
PenetrationDepth = 0.0
SwingPeriod = 0.0
YawControl = 0.0
YawControlOn = False

# ------------------ Spot states ----------------
x_inst = 0.
y_inst = 0.
z_inst = 0.
roll_inst = 0.
pitch_inst = 0.
yaw_inst = 0.
search_index = -1

# ------------------ Outputs of Contact sensors ----------------
front_left_lower_leg_contact = 1
front_right_lower_leg_contact = 1
rear_left_lower_leg_contact = 1
rear_right_lower_leg_contact = 1
chattering_front_left_lower_leg_contact = 0
chattering_front_right_lower_leg_contact = 0
chattering_rear_left_lower_leg_contact = 0
chattering_rear_right_lower_leg_contact = 0
lim_chattering = 4

# JOINT MOVEMENT DECOMPOSITION
def movement_decomposition(target, duration):
  timestep = int(robot.getBasicTimeStep())
  # print(timestep)
  n_steps_to_achieve_target = duration * 1000 / timestep
  step_difference = []
  current_position = []

  for i in range(0,NUMBER_OF_JOINTS):
    current_position.append(motors[i].getTargetPosition())
    step_difference.append((target[i] - current_position[i]) / n_steps_to_achieve_target)
  

  for i in range(0, math.floor(n_steps_to_achieve_target)):
    for j in range(0, NUMBER_OF_JOINTS):
      current_position[j] += step_difference[j]
      motors[j].setPosition(current_position[j])
      # print(j,current_position[j])
    step()

def lie_down(duration):
  motors_target_pos= [-0.40, -0.99, 1.59,   # Front left leg
                      0.40,  -0.99, 1.59,   # Front right leg
                      -0.40, -0.99, 1.59,   # Rear left leg
                      0.40,  -0.99, 1.59]  # Rear right leg
  movement_decomposition(motors_target_pos, duration)

def stand_up(duration):
  motors_target_pos = [-0.1, 0.0, 0.0,   # Front left leg
                       0.1,  0.0, 0.0,   # Front right leg
                       -0.1, 0.0, 0.0,   # Rear left leg
                       0.1,  0.0, 0.0]   # Rear right leg

  movement_decomposition(motors_target_pos, duration)

def fl_ground_contact(data):
  if data == 0:
      chattering_front_left_lower_leg_contact += 1
      if chattering_front_left_lower_leg_contact > lim_chattering:
          front_left_lower_leg_contact = 0
  else:
      front_left_lower_leg_contact = 1
      chattering_front_left_lower_leg_contact = 0

def fr_ground_contact(data):
  if data == 0:
      chattering_front_right_lower_leg_contact += 1
      if chattering_front_right_lower_leg_contact > lim_chattering:
          front_right_lower_leg_contact = 0
  else:
      front_right_lower_leg_contact = 1
      chattering_front_right_lower_leg_contact = 0

def rl_ground_contact(data):
  if data == 0:
      chattering_rear_left_lower_leg_contact += 1
      if chattering_rear_left_lower_leg_contact > lim_chattering:
          rear_left_lower_leg_contact = 0
  else:
      rear_left_lower_leg_contact = 1
      chattering_rear_left_lower_leg_contact = 0

def rr_ground_contact(data):
  if data == 0:
      chattering_rear_right_lower_leg_contact += 1
      if chattering_rear_right_lower_leg_contact > lim_chattering:
          rear_right_lower_leg_contact = 0
  else:
      rear_right_lower_leg_contact = 1
      chattering_rear_right_lower_leg_contact = 0

def step():
  # timestep = wb_robot_get_basic_time_step();
  if robot.step(timestep) == -1:
    # wb_robot_cleanup();
    # exit(0);
    pass

  spot_rot = spot_node.getField("rotation")
  spot_rot_val = spot_rot.getSFRotation()
  yaw_inst = spot_rot_val[2]

def setMotorPositions(motors_target_pos):
  for i, motor in enumerate(motors):
    motor.setPosition(motors_target_pos[i] - motors_initial_pos[i])

def yaw_control():
  """ Yaw body controller"""
  yaw_target = YawControl
  thr = np.pi / 2
  if (yaw_target > thr and yaw_inst < -thr) or (yaw_inst > thr and yaw_target < -thr):
    residual = (yaw_target - yaw_inst) * np.sign(yaw_target - yaw_inst) - 2 * np.pi
    yawrate_d = 2.0 * np.sqrt(abs(residual)) * np.sign(residual)
  else:
    residual = yaw_target - yaw_inst
    yawrate_d = 4.0 * np.sqrt(abs(residual)) * np.sign(residual)
  return yawrate_d

def inverse_control(pos, orn):
  # yaw controller
  if YawControlOn == 1.0:
    YawRate_desired = yaw_control()
  else:
    YawRate_desired = YawRate

  # Update Swing Period
  bzg.Tswing = SwingPeriod
  contacts = [
    front_left_lower_leg_contact,
    front_right_lower_leg_contact,
    rear_left_lower_leg_contact,
    rear_right_lower_leg_contact
  ]

  # Get Desired Foot Poses
  T_bf = bzg.GenerateTrajectory(StepLength, LateralFraction, YawRate_desired,
                                      StepVelocity, T_bf0, T_bf,
                                      ClearanceHeight, PenetrationDepth,
                                      contacts)

  joint_angles = -spot.IK(orn, pos, T_bf)

  target = [
    joint_angles[0][0], joint_angles[0][1], joint_angles[0][2],
    joint_angles[1][0], joint_angles[1][1], joint_angles[1][2],
    joint_angles[2][0], joint_angles[2][1], joint_angles[2][2],
    joint_angles[3][0], joint_angles[3][1], joint_angles[3][2],
  ]

  setMotorPositions(target)

# MAIN METHOD
def main():
  # SIM LOOP
  while True:
    # SET DESIRED POSE
    xd=0
    yd=0
    zd=0
    rolld=0
    pitchd=0
    yawd=0
    pos = np.array([xd, yd, zd])
    orn = np.array([rolld, pitchd, yawd])

    # CHECK GROUND CONTACTS
    fl_ground_contact(bool(touch_fl.getValue()))
    fr_ground_contact(bool(touch_fr.getValue()))
    rl_ground_contact(bool(touch_rl.getValue()))
    rr_ground_contact(bool(touch_rr.getValue()))

    # CALL INVERSE CONTROL ON DESIRED POSE
    inverse_control(pos,orn)

    # STEP THE SIMULATION FRWD
    step()

if __name__=='__main__':
  main()