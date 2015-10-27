import sys
import zmq
import time
import threading
import atexit

ip="localhost"
port = "5556"
if len(sys.argv) > 1:
    port =  sys.argv[1]
    int(port)
    
if len(sys.argv) > 2:
    port1 =  sys.argv[2]
    int(port1)

#Class to subscribe topics
class zmqSub():
    def __init__(self, subName, ip, port, topicfilter):
        self.name = subName
        self.port = port
        self.ip = ip
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
        self.socket.connect("tcp://%s:%s" % (self.ip, self.port))

    def subscribe(self):
      string = self.socket.recv()
      string = string.split()
      return(string[0],string[1:len(string)])

    def teardown(self):
      self.socket.close()  

class robot():

    def __init__(self):
        self.stat=False;
        #Do some cool stuff with the PiSense LEDs like a countdown

    def foo1(self):
        while (self.stat==True):
            print("In foo1") 
            time.sleep(0.1)
        print("Exiting foo1")

    def foo2(self):
        while (self.stat==True):
            print("In foo2") 
            time.sleep(0.1)
        print("Exiting foo2")
         
    def stopLoop(self):
        self.stat=False
        time.sleep(0.1)
        self.stat=True
        
if __name__ == "__main__":

    print "Collecting updates from weather server..."
    topicfilter = "tB_TOPIC_COMMAND"
    sub = zmqSub("command_subscriber", "127.0.0.1", "5556", topicfilter)
    atexit.register(sub.teardown)
    r=robot()
    while(True):
        topic, message = sub.subscribe()
        print "Received %s %s" % (topic, message)
        if(message[0] =="A"):
            r.stopLoop()
            t = threading.Thread(target=r.foo1)
            t.start()
        elif(message[0] =="B"):
            r.stopLoop()
            t = threading.Thread(target=r.foo2)
            t.start()
    sub.teardown()
