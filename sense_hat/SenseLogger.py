from datetime import datetime
from sense_hat import SenseHat
from evdev import InputDevice, categorize, ecodes,list_devices
from select import select
import threading

sense = SenseHat()

## Logging Settings
TEMPERATURE=True
HUMIDITY=True
PRESSURE=True
ORIENTATION=True
ACCELERATION=True
MAG=True
GYRO=True
DELAY = 1
BASENAME = ""
## Main Program

class SenseLogger():

    def __init__(self, senseHat):
        print("Initializing logger!")
        self.senseHat = senseHat
        self.logging = True
        self.dev = self.get_joystick()
        self.batch_data=[]
        self.sense_data=[]
        if BASENAME == "":
            self.filename = "SenseLog-"+str(datetime.now())+".csv"
        else:
            self.filename = BASENAME+"-"+str(datetime.now())+".csv"
        self.file_setup(self.filename)
        if DELAY >= 1:
            self.timed_log()

    def file_setup(self, filename):
        self.header =[]
        if TEMPERATURE:
            self.header.extend(["temp_h","temp_p"])
        if HUMIDITY:
            self.header.append("humidity")
        if PRESSURE:
            self.header.append("pressure")
        if ORIENTATION:
            self.header.extend(["pitch","roll","yaw"])
        if ACCELERATION:
            self.header.extend(["mag_x","mag_y","mag_z"])
        if MAG:
            self.header.extend(["accel_x","accel_y","accel_z"])
        if GYRO:
            self.header.extend(["gyro_x","gyro_y","gyro_z"])
        self.header.append("timestamp")

        with open(filename,"w") as f:
            f.write(",".join(str(value) for value in self.header)+ "\n")

    ## Function to capture input from the Sense HAT Joystick
    def get_joystick(self):
        devices = [InputDevice(fn) for fn in list_devices()]
        for dev in devices: 
            if dev.name == "Raspberry Pi Sense HAT Joystick":
                return dev

    ## Function to collect data from the Sense HAT and build a string
    def get_sense_data(self):
        self.sense_data=[]
        
        if TEMPERATURE:
            self.sense_data.extend([self.senseHat.get_temperature_from_humidity(),sense.get_temperature_from_pressure()])
            
        if HUMIDITY:
            self.sense_data.append(self.senseHat.get_humidity())
         
        if PRESSURE:
            self.sense_data.append(self.senseHat.get_pressure())
            
        if ORIENTATION:
            yaw,pitch,roll = self.senseHat.get_orientation().values()        
            self.sense_data.extend([pitch,roll,yaw])

        if MAG:
            mag_x,mag_y,mag_z = self.senseHat.get_compass_raw().values()
            self.sense_data.append([mag_x,mag_y,mag_z])

        if ACCELERATION:
            x,y,z = self.senseHat.get_accelerometer_raw().values()
            self.sense_data.extend([x,y,z])

        if GYRO:
            gyro_x,gyro_y,gyro_z = self.senseHat.get_gyroscope_raw().values()
            self.sense_data.extend([gyro_x,gyro_y,gyro_z])
        
        self.sense_data.append(datetime.now())
         
        return self.sense_data

    #def show_state(logging):
        #if logging:
            #sense.show_letter("!",text_colour=[0,255,0])
        #else:
            #sense.show_letter("!",text_colour=[255,0,0])

    def check_input(self):
        r, w, x = select([self.dev.fd], [], [],0.01)
        for fd in r:
            for event in self.dev.read():
                if event.type == ecodes.EV_KEY and event.value == 1:
                    print(event.code)
                    if event.code == ecodes.KEY_UP:
                        print("quiting")
                        return True,False
                    else:
                        return True,True
        return False,True

    def log_data(self):
        self.get_sense_data()
        output_string = ",".join(str(value) for value in self.sense_data)
        self.batch_data.append(output_string)
        with open(self.filename,"a") as f:
            for line in self.batch_data:
                f.write(str(line) + "\n")
            self.batch_data= []

    def timed_log(self):
      threading.Timer(DELAY, self.timed_log).start()
      if self.logging == True:
            print("logged")
            self.log_data()

    def on_key_press(self):
        
        if self.logging==True:
            self.log_data()
        self.logging = not(self.logging)


