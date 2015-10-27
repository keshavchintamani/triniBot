import socket
import threading
import SocketServer
import time
import zmq

class robot():

    def init(self):
        self.stat=False;
        #Do some cool stuff with the PiSense LEDs like a countdown

    def foo1(self):
        self.stat=True
        
        while (self.stat==True):
            print("In foo1 1") 
            time.sleep(1)

    def foo2(self):
        self.stat=True
        
        while (self.stat==True):
            print("In foo1 2") 
            time.sleep(1)

    def stopLoop(self):
        self.stat=False
    

class ThreadedTCPRequestHandler(SocketServer.BaseRequestHandler):

    def handle(self):

        self.data = self.request.recv(1024).strip()
        print "{}".format(self.data)
        cur_thread = threading.current_thread()
        message = self.data.split()
        print "{} requested: processing on {}".format(message[0],cur_thread)
        if(len(message) < 1):
             self.request.send("Sorry cant process empty commands!")
        #Handles incoming requests
        self.processRequest(message)

    def processRequest(self, message ):
        
        command = message[0]
        if(command == "TB_INIT"):
            #Initialize the app
            r.init()
            self.request.send("TB_STATUS OK")
        elif(command == "TB_DRIVE_FORWARD"):
            self.request.send("TB_STATUS OK")
            r.foo1()
        elif(command == "TB_TURN_LEFT"):
            self.request.send("TB_STATUS OK")
            r.foo2()
        elif(command == "TB_DRIVE_RIGHT"):
            self.request.send("TB_STATUS OK")
        elif(command == "TB_DRIVE_BACK"):
            self.request.send("TB_STATUS OK")
        elif(command == "TB_DRIVE_STOP"):
            self.request.send("TB_STATUS OK")
        else:
            self.request.send("TB_ERROR 0")

class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    pass


r = robot()

if __name__ == "__main__":
    # Port 0 means to select an arbitrary unused port
    HOST, PORT = "localhost", 9999

    server = ThreadedTCPServer((HOST, PORT), ThreadedTCPRequestHandler)
    ip, port = server.server_address

    # Start a thread with the server -- that thread will then start one
    # more thread for each request
    server_thread = threading.Thread(target=server.serve_forever)
    # Exit the server thread when the main thread terminates
    server_thread.daemon = True
    server_thread.start()
    print "Server loop running in thread:", server_thread.name
    r.init()

