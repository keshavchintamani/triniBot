import zmq
print zmq.pyzmq_version()
import time
import sys
import atexit

port = "5556"
if len(sys.argv) > 1:
   port = sys.argv[1]
   int(port)


class zmqPub():
   def __init__(self,pubName, ip, port ):
      self.name = pubName
      self.port = port
      self.ip = ip
      self.context = zmq.Context()   
      self.socket = self.context.socket(zmq.PUB)
      self.socket.bind("tcp://*:%s" % (self.port))

   def publish(self, topic, message):
      print "Publishing %s %s" % (topic, message)
      self.socket.send("%s %s" % (topic, message))

   def teardown(self):
      self.socket.close()
       
if __name__ == "__main__":

   pub = zmqPub("GUI_pub", "localhost", port)
   atexit.register(pub.teardown)
   while(True):
      pub.publish("tB_TOPIC_COMMAND","B");
      time.sleep(1)
      pub.publish("tB_TOPIC_COMMAND","A");
      time.sleep(1)
   pub.teardown()
