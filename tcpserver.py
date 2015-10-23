import SocketServer
#import picamera
import time

#camera = picamera.PiCamera()

#list of error codes
# 0 command not supported


def initTriniBot():
    #Do some cool stuff with the PiSense LEDs like a countdown
    for i in range(0, 3):
        print("Showing something wow %s") % i
        time.sleep(1)
              

class MyTCPHandler(SocketServer.BaseRequestHandler):
    """
    The RequestHandler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        # self.request is the TCP socket connected to the client
        self.data = self.request.recv(1024).strip()
        print "{} wrote:".format(self.client_address[0])
        print self.data

        req = self.data.split()
        if(len(req) < 1):
             self.request.sendall("Sorry cant process empty commands!")
        #Handles incoming requests
        self.processRequest(req)

    def processRequest(self, request):

        command = request[0]
        if(command == "TB_INIT"):
            #Initialize the app
            initTriniBot()
            self.request.sendall("TB_STATUS OK")
        elif(command == "TB_DRIVE_FORWARD"):
             self.request.sendall("TB_STATUS OK")
        elif(command == "TB_DRIVE_LEFT"):
             self.request.sendall("TB_STATUS OK")
        elif(command == "TB_DRIVE_RIGHT"):
             self.request.sendall("TB_STATUS OK")
        elif(command == "TB_DRIVE_BACK"):
             self.request.sendall("TB_STATUS OK")
        elif(command == "TB_DRIVE_STOP"):
             self.request.sendall("TB_STATUS OK")
        else:
             self.request.sendall("TB_ERROR 0")
        

if __name__ == "__main__":
    HOST, PORT = "localhost", 9998

    # Create the server, binding to localhost on port 9999
    server = SocketServer.TCPServer((HOST, PORT), MyTCPHandler)

    # Activate the server; this will keep running until you
    # interrupt the program with Ctrl-C
    print("Listening on %s") % {HOST, PORT}
    server.serve_forever()
