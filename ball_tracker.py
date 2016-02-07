
from tbMotorController import TwoWheelRobot
import time
from pyfirmata import Arduino, util
import time as Time
import atexit
import sense_hat as SenseHat

board = Arduino('/dev/ttyUSB0')


it = util.Iterator(board)
it.start()

board.analog[0].enable_reporting()

myRobot = TwoWheelRobot()
myLoop=True;
def doSomething():
    myRobot.Drive("REVERSE", 100)
    Time.sleep(3)
    myRobot.Stop()
    myRobot.Drive("LEFT", 100)
    Time.sleep(3)

def onExit():
    print "Exiting..."
    myLoop=False;
    board.exit()
    myRobot.Stop()

atexit.register(onExit)

if __name__ == "__main__":
    
    try:
        while myLoop:   
            try:
                voltage = float(board.analog[0].read())*5;
                print "Pin: %f" % voltage
            except TypeError:
                print("Invalid");
            if voltage > 1.5:       
                myRobot.Stop()
                doSomething()
            else:
                myRobot.Drive("FORWARD", 100)
    except(KeyboardInterrupt, SystemExit):
        print "Exiting"
        onExit()
