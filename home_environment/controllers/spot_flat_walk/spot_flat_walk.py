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
time_step = int(robot.getBasictime_step())

# Initialize the robot's information
NUMBER_OF_JOINTS = 12

# INIT MOTORS
motors = [];
motorNames = [
  'front left shoulder abduction motor',  'front left shoulder rotation motor',  'front left elbow motor',
  'front right shoulder abduction motor', 'front right shoulder rotation motor', 'front right elbow motor',
  'rear left shoulder abduction motor',   'rear left shoulder rotation motor',   'rear left elbow motor',
  'rear right shoulder abduction motor',  'rear right shoulder rotation motor',  'rear right elbow motor'];
for i in range(12):
    motors.append(robot.getDevice(motorNames[i]))
    motors[i].setPosition(0)
  
# INIT CAMERAS
cameras = []
cameraNames = ['left head camera', 'right head camera', 'left flank camera', 'right flank camera', 'rear camera'];
for i in range(5):
    cameras.append(robot.getDevice(cameraNames[i]))
    cameras[i].enable(2 * time_step)

# INIT LEDs
leds = []
ledNames = ['left top led', 'left middle up led', 'left middle down led',
            'left bottom led', 'right top led', 'right middle up led',
            'right middle down led', 'right bottom led'];
for i in range(8):
    leds.append(robot.getDevice(ledNames[i]))
    leds[i].set(1)

#TODO: ENABLE TOUCH SENSORS

## Spot Control
# rate = __node.create_rate(100)
# time_step = __robot.time_step

spot = SpotModel()
T_bf0 = spot.WorldToFoot
T_bf = copy.deepcopy(T_bf0)
bzg = BezierGait(dt=time_step/1000)
motors_initial_pos = []

# ------------------ Inputs for Bezier Gait control ----------------
xd = 0.0
yd = 0.0
zd = 0.1
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
fl_ground_contact = 1
fr_ground_contact = 1
rl_ground_contact = 1
rr_ground_contact = 1
chattering_fl_ground_contact = 0
chattering_fr_ground_contact = 0
chattering_rl_ground_contact = 0
chattering_rr_ground_contact = 0
lim_chattering = 4

lidar = []
lidarName =['Velodyne Puck']
lidar.append(robot.getDevice(lidarName[0]))
lidar[0].enable(time_step)
lidar[0].enablePointCloud()
resolution = lidar[0].getHorizontalResolution()
layers = lidar[0].getNumberOfLayers()

# Movement decomposition
def movement_decomposition(target, duration):
  n_steps_to_achieve_target = duration * 1000 / time_step
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

#TODO: COMBINE MOVEMENTDECOMPOSITION() AND SETMOTORPOSITION()
def setMotorPositions(motors_target_pos):
    for idx, motor in enumerate(motors):
        motor.setPosition(motors_target_pos[idx] - motors_initial_pos[idx])

def spot_inverse_control(pos, orn):
    # yaw controller
    if YawControlOn == 1.0:
        YawRate_desired = yaw_control()
    else:
        YawRate_desired = YawRate

    # Update Swing Period
    bzg.Tswing = SwingPeriod
    contacts = [fl_ground_contact, fr_ground_contact,
                rl_ground_contact,rr_ground_contact]

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

    if not motors_initial_pos:
        motors_initial_pos = target

    setMotorPositions(target)

# STEP THE SIMULATION
def step():
  if robot.step(time_step) == -1:
    pass
  
  # TODO: INIT TOUCH SENSORS
  # callback_fl_ground_contact(bool(touch_fl.getValue()))
  # callback_fr_ground_contact(bool(touch_fr.getValue()))
  # callback_rl_ground_contact(bool(touch_rl.getValue()))
  # callback_rr_ground_contact(bool(touch_rr.getValue()))

  #TODO: MOVE THIS TO THE MAIN METHOD AND ADD STEP TO ACTIONS
  pos = np.array([xd, yd, zd])
  orn = np.array([rolld, pitchd, yawd])
  spot_inverse_control(pos,orn)

def fl_ground_contact(self, data):
    if data == 0:
        chattering_fl_ground_contact += 1
        if chattering_fl_ground_contact > lim_chattering:
            fl_ground_contact = 0
    else:
        fl_ground_contact = 1
        chattering_fl_ground_contact = 0

def fr_ground_contact(self, data):
    if data == 0:
        chattering_fr_ground_contact += 1
        if chattering_fr_ground_contact > lim_chattering:
            fr_ground_contact = 0
    else:
        fr_ground_contact = 1
        chattering_fr_ground_contact = 0

def rl_ground_contact(self, data):
    if data == 0:
        chattering_rl_ground_contact += 1
        if chattering_rl_ground_contact > lim_chattering:
            rl_ground_contact = 0
    else:
        rl_ground_contact = 1
        chattering_rl_ground_contact = 0

def rr_ground_contact(self, data):
    if data == 0:
        chattering_rr_ground_contact += 1
        if chattering_rr_ground_contact > lim_chattering:
            rr_ground_contact = 0
    else:
        rr_ground_contact = 1
        chattering_rr_ground_contact = 0

# Main method
if __name__=="__main__":
  # - perform simulation steps until Webots is stopping the controller
  while robot.step(time_step) != -1:
      # Read the sensors:
      # Enter here functions to read sensor data, like:
      #  val = ds.getValue()

      # Process sensor data here.

      # Enter here functions to send actuator commands, like:
      #  motor.setPosition(10.0)
        
      # lie_down(4.0)
      stand_up(4.0)
      # lie_down(4.0)