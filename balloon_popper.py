
from tbMotorController import TwoWheelRobot
from tbVision import BallTracker
import threading
import logging
import time as Time
import atexit

myRobot = TwoWheelRobot()

ImgWidth = 640
ImgHeight = 480
ballFound = False
ballCentered = False

Px = 0
Py = 0
Radius=0
Px_old = 0
doingSomething = False;

def BallDataCallBack(x, y, radius):

    Px = x
    Radius = radius
    #TODO Is this an artifact? - well thats for BallTracker to find out

    if Radius is not None:
        logging.warning("Ball is at:%f", Px)
        if not -0.1 <= Px <= 0.1 and doingSomething == False:
            t = threading.Thread(None, CenterBall, "Motor")
            t.start()
    Px_old = Px

def CenterBall():

    for i in range(5):
        doingSomething = True;
        if -0.1 <= Px <= 0.1:
            break
        #if Px - Px_old > 0:
        #    logging.warning("Ball was last moved right")
            myRobot.Drive("RIGHT", 100)
            Time.sleep(2)
        #elif Px - Px_old < 0:
        #    logging.warning("Ball was last moved left")
            myRobot.Drive("LEFT", 100)
            Time.sleep(2)
        #else:
        #   continue
        #myRobot.Stop()
    doingSomething=False

if __name__ == "__main__":
    bt = BallTracker(BallDataCallBack, ImgWidth, ImgHeight, "RED")
    bt.startBallTracker()
