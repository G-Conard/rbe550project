"""xbox_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import sys

# tell python to look for libraries in our python libraries folder
# sys.path.insert(1, '../../../scripts')
from XboxController import XboxController
xbox = XboxController()

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

#define simulation time variable (seconds)
t = 0
startTime = t
state = 0

def printInput():
    print(xbox.RightTrigger,xbox.LeftTrigger,xbox.LeftX,xbox.LeftY)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    #update current elapsed time in simulation
    t+=float(timestep)*1.0/1000

    now = t - startTime
    
    if state == 1:
        print("State 1")
        if not xbox.Start:
            state = 2
    elif state == 2:
        print("State 2")
        if xbox.Start:
            # self.stand()
            timeStamp = now
            state = 3
    elif state == 3:
        print("State 3")
        if not xbox.Start and now >= timeStamp + 1:
            timeStamp = now
            state = 4
    elif state == 4:
        print("State 4")
        # stepDuration = 1
        # self.walk(xbox = xbox, phase = (now - timeStamp) / stepDuration)
        printInput()
        if xbox.Start:
            state = 0
    else:
        print("\nPress START to continue.")
        # self.home()
        state = 1

    if xbox.Select:
        print("\nQuadruped Test Complete (Terminated by Xbox Controller).\n")
        break

# Enter here exit cleanup code.
