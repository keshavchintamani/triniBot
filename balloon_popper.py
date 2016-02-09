
from tbMotorController import TwoWheelRobot
from tbVision import BallTracker
import threading
import logging
import time as Time
import atexit

myRobot = TwoWheelRobot()

ImgWidth = 640
ImgHeight = 480

counter = 0
Direction = False

def CenterBall():
    global counter, Direction
    if(counter % 3 == 0):
        Direction = not Direction

    if Direction == True:
        logging.warning("Turning RIGHT")
        myRobot.Drive("RIGHT", 50)
    else:
        logging.warning("Turning LEFT")
        myRobot.Drive("LEFT", 50)
    Time.sleep(0.5)
    counter = counter + 1

def onExit():
    bt.stopBallTracker()
    myRobot.Stop()
    
def BallDataCallBack(x, y, radius):
    global Attempts
    #TODO Is this an artifact? - well thats for BallTracker to find out
    try:
        logging.warning("Received from BallTracker: %f", float(x))
        if not (-0.1 <= x <= 0.1):
            CenterBall()
            #t = threading.Thread(None, CenterBall, "Motor")
            #t.start()
            #t.join()
            Attempts = Attempts +1
            if Attempts > 500:
                print "Tired! no ball in sight, shutting down.."
                onExit()
            return
        else:
            myRobot.Stop()
    except TypeError:
        print "Received none"


if __name__ == "__main__":
    try:
        Attempts = 0
        bt = BallTracker(BallDataCallBack, ImgWidth, ImgHeight, "RED")
        bt.startBallTracker()
    except(KeyboardInterrupt, SystemExit):
        onExit()
        print "Exiting"
        

