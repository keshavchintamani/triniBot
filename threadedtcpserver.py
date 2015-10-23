import socket
import threading
import SocketServer
import time


def initTriniBot():
    #Do some cool stuff with the PiSense LEDs like a countdown
    for i in range(0, 2):
        print("Showing something wow %s") % i
        time.sleep(1)

class ThreadedTCPRequestHandler(SocketServer.BaseRequestHandler):

    def handle(self):

        #clientRequest = self.request.recv(1024)
        #print "{}".format(clientRequest)
        #while(clientRequest != ''):
        #print(clientRequest)
        #self.request.sendall("WAITING")
        
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
            initTriniBot()
            self.request.send("TB_STATUS OK")
        elif(command == "TB_DRIVE_FORWARD"):
            self.request.send("TB_STATUS OK")
        elif(command == "TB_DRIVE_LEFT"):
            self.request.send("TB_STATUS OK")
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

