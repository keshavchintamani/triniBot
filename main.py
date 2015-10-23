__author__ = 'kc'

from tB_platform_handler import tBTCPMotorHandler
from tB_platform_handler import tBTCPSensorHandler
import SocketServer
import threading
import atexit
import socket

def cleanClose():
    print('Attempting to shutdown server');

IP = "192.168.1.9"
PORT_MOTOR=9999
PORT_SENSOR=9998

# init the Servers
if __name__ == "__main__":

    print "{}".format(socket.gethostbyname(socket.gethostname()))
    #Start the motor server
    HOST, PORT = IP, PORT_MOTOR
    # Create the server, binding to localhost on port 9999
    motor_server = SocketServer.TCPServer((HOST, PORT), tBTCPMotorHandler)

    motor_server_thread = threading.Thread(target=motor_server.serve_forever, name= 'motor-thread')
    motor_server_thread.daemon = True
    motor_server_thread.start()
    print("Listening on %s") % {HOST, PORT}

    #start the sensor_server
    #HOST, PORT = IP, PORT_SENSOR
    #Create the server, binding to localhost on port 9998
    #sensor_server = SocketServer.TCPServer((HOST, PORT), tBTCPSensorHandler)
    #sensor_server_thread = threading.Thread(target=sensor_server.serve_forever, name= 'sensor-thread')
    #sensor_server_thread.daemon = True
    #sensor_server_thread.start()
    #print("Listening on %s") % {HOST, PORT}

atexit.register(cleanClose)
