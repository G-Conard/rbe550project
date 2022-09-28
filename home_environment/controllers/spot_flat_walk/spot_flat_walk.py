"""spot_flat_walk controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Camera, LED
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Initialize the robot's information
NUMBER_OF_JOINTS = 12

motors = [];
motorNames = [
  'front left shoulder abduction motor',  'front left shoulder rotation motor',  'front left elbow motor',
  'front right shoulder abduction motor', 'front right shoulder rotation motor', 'front right elbow motor',
  'rear left shoulder abduction motor',   'rear left shoulder rotation motor',   'rear left elbow motor',
  'rear right shoulder abduction motor',  'rear right shoulder rotation motor',  'rear right elbow motor'];
for i in range(12):
    motors.append(robot.getDevice(motorNames[i]))

    
cameras = []
cameraNames = ['left head camera', 'right head camera', 'left flank camera', 'right flank camera', 'rear camera'];
for i in range(5):
    cameras.append(robot.getDevice(cameraNames[i]))
    cameras[i].enable(2 * timestep)


leds = []
ledNames = ['left top led', 'left middle up led', 'left middle down led',
            'left bottom led', 'right top led', 'right middle up led',
            'right middle down led', 'right bottom led'];
for i in range(8):
    leds.append(robot.getDevice(ledNames[i]))
    leds[i].set(1)



# Movement decomposition
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
    
    # step();

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


def step():
  # timestep = wb_robot_get_basic_time_step();
  if robot.step(timestep) == -1:
    # wb_robot_cleanup();
    # exit(0);
    pass
 

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
    
    
    lie_down(4.0)
    stand_up(4.0)
    lie_down(4.0)

# Enter here exit cleanup code.
