import socket

class tbClient():
    def __init__(self, ip, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((ip, port))

    def sendMessage(self, msg):
        self.sock.send(msg)
       

    def processResponse(self):
        response = self.sock.recv(1024)
        print "Received: {}".format(response)
        

if __name__ == "__main__":

    ip, port = "localhost", 9999
    c1 = tbClient(ip,port)
    c2 = tbClient(ip,port)

    c1.sendMessage("TB_INIT")
    c1.processResponse()
    for i in range(0,1):
        print"{}".format(i)
        c2.sendMessage("TB_DRIVE_FORWARD")
        c2.processResponse()
