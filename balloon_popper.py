
from Robot_Controllers import TwoWheelRobot
import time

if __name__ == "__main__":

    myRobot=TwoWheelRobot()
    
    myRobot.Drive("FORWARD", 100)
    time.sleep(2)
    myRobot.Drive("REVERSE", 100)
    time.sleep(2)
    myRobot.Drive("LEFT", 100)
    time.sleep(2)
    myRobot.Drive("RIGHT", 100)
    
    #myRobot = tBController(LEFT_MOTOR, RIGHT_MOTOR, 4)
    
