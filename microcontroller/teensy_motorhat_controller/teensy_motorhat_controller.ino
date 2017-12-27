


/*
  Generic High frequency closed loop PID controller applicable for a range of precise two axis, brushed dc motor with quadrature encoder feedback
  for a two axis tracked robot based on the pololu micrometal covering:
    - displacement (mm) control
    - angular displacement (degrees turned)
    - wheel turn rate (deg/sec) control
    - platform (yaw) turn rate (deg/sec) control

  Keshav Chintamani
  keshav.chintamani@gmail.com

  Code extended from fdavies
  https://forum.pjrc.com/threads/26803-Hardware-Quadrature-Code-for-Teensy-3-x

  This code is in the public domain.
*/
#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"

//Defines
#define MOTORS 2
#define CS_PIN 10
#define CLOCK_PIN 13
#define MOSI_PIN 12
#define MISO_PIN 11
#define ledPin 13

#define POSITION 0
#define VELOCITY 1
#define TURNRATE 1
#define dT 20 //Interrupt timer in microseconds1

#define VELOCITY_SAMPLING_RATE 100 //Instantaneous velocity capture in hz


// definition of how the pins are set up
#define xQ1Pin 3 // pin A for x axis quadrature encoder    J1 pin 4
#define xQ2Pin 4  // pin B for x axis quadrature encoder    J1 pin 3
#define yQ1Pin 16  // pin A for y axis quadrature encoder   J2 pin 4
#define yQ2Pin 17  // pin B for y axis quadrature encoder   J2 pin 3

//For encoders
int gear_ratio = 100;
int ppr=12;
float pulse_deg=3.33;//Number of encoder pulses per degree
float pulse_deg_base= 7.59; //Round off - should be 12.38 insted
int max_pwm=250;

//For tracks
float radius = 19.5; //mm
float base_radius = 44; //mm
float base_length = 88; //mm
float circumference = 122.5221135; //mm
int encoders_wheelrotation = 1200; //pulses
int encoders_baserotation = 2769; //pulse
#define D2R 0.0174533
#define R2D 57.295
//Constants used to convert speed and spin targets into pulses/sec @ specified velocity sampling rate period

//Converts speed in m/s into pulses/sec @ specified velocity sampling rate period
//Note: the speed constant is the required wheel speed (ref xSpeed, ySpeed) at the VELOCITY SAMPLING RATE, i.e. (setpoint in pulses per second) *(1/VELOCITY SAMPLING RATE)
//That is: it is
float speed_constant = (float) pulse_deg*(R2D/(radius*pow(10,-3)))/VELOCITY_SAMPLING_RATE;

//Converts speed in deg/s into pulses/sec @ specified velocity sampling rate period
float spin_constant = (float) (radius/base_radius)*pulse_deg_base/VELOCITY_SAMPLING_RATE;

// thiss the decode table for the optical encoder
signed long enc_TAB[]={0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

// Create an IntervalTimer object
IntervalTimer encoder;

//All things to do with serial
char* type;
String serialcommand, header, values;
String inputString = "";
char msg[48];

boolean stringComplete = false;

//float w_w2w_ratio=0.4;
signed int xoutMin = 0;
signed int xoutMax = 0;
signed int youtMin = 0;
signed int youtMax = 0;
volatile unsigned int mode = VELOCITY; // by default set it to velocity tracking
volatile int modeAngular = 0;
unsigned int actuationTimePeriod=0;
volatile unsigned long encoderState = 0; // use volatile for shared variables
volatile float error = 0;
volatile signed long xEncoder = 0; // This is the position based on counting encoder pulses
volatile signed long yEncoder = 0; // This is the position based on counting encoder pulses
volatile float xSpeed = 0; // This is the position based on counting encoder pulses
volatile float ySpeed = 0;
volatile signed int elapsedtime = 0;
volatile float xSetPoint = 0; // this is where the PID loop will try to move the axis
volatile float ySetPoint = 0;
unsigned int count1 = 0;  // so that the encoder counting is faster than the PID updating
unsigned int channelCount = 0;  //

volatile float xITerm=0;
volatile float yITerm=0;
volatile float xLastInput=0;
volatile float yLastInput=0;

float V_left =0; 
float V_right=0;

volatile signed long lastxEncoder = 0;
volatile signed long lastyEncoder = 0;

volatile signed long xki=1;  // PID w we integration constant
volatile signed long xkp=10;  // PID proportional constant
volatile signed long xkd=1;  // PID derivative constant

volatile signed long yki=1;  // PID integration constant
volatile signed long ykp=10;  // PID proportional constant
volatile signed long ykd=1;  // PID derivative constant

volatile float pwm_output=0; // for PID loops

//All odometry
int time_last = millis();
int time_now = millis();
int dt =0;
float vx_now=0;
float vy_now=0;
float w_now=0;
float x=0,y=0, th=0;
float v_now = 0;
boolean fbswitch = false;
int odometry_counter = 0;

//All things to send data over Serial
#define ST 0xaa
#define ET 0xbb
typedef union
{
 float value;
 uint8_t bytes[4];
} FLOATUNION_t;

//All things MotorHat Related
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);

void set_parameters(int _gr, int _ppr, float wheel_dia, float base_dia, int max_pwm = 150 ){

  radius = wheel_dia/2;
  base_radius = base_dia/2;
  circumference = PI*wheel_dia;
  encoders_wheelrotation =  _gr*_ppr;
  encoders_baserotation  =  encoders_wheelrotation*(PI*base_dia)/circumference;
  pulse_deg = encoders_wheelrotation/360;
  pulse_deg_base = encoders_baserotation/360;
  speed_constant = (float) pulse_deg*(R2D/(radius*pow(10,-3)))/VELOCITY_SAMPLING_RATE;
  //Converts speed in deg/s into pulses/sec @ specified velocity sampling rate period
  spin_constant = (float) (radius/base_radius)*pulse_deg_base/VELOCITY_SAMPLING_RATE;

}

void get_parameters(){

  Serial.println("Parameters:");
  Serial.print("Wheel radius:");
  Serial.println(radius);
  Serial.print("Base radius:");
  Serial.println(base_radius);
  Serial.print("Wheel circumference:");
  Serial.println(circumference);
  Serial.print("Encoders/wheel:");
  Serial.println(encoders_wheelrotation);
  Serial.print("Encoders/base:");
  Serial.println(encoders_baserotation);
  Serial.print("pulse_deg:");
  Serial.println(pulse_deg);
  Serial.print("pulse_deg_base:");
  Serial.println(pulse_deg_base);
}

void motorsStop(){
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}


void setup() {

  feedbackSwitch(false);
  Serial.begin(115200);
  serialcommand.reserve(200);
  AFMS.begin();  // create with the default frequency 1.6KHz
  pinMode(xQ1Pin, INPUT);
  pinMode(xQ2Pin, INPUT);
  pinMode(yQ1Pin, INPUT);
  pinMode(yQ2Pin, INPUT);
  pinMode(ledPin, OUTPUT);

  /* move all the motors weakly negative for 2 seconds to home them.  Do this better later */
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  for (int i=0; i<150; i++) {
    leftMotor->setSpeed(i);
    rightMotor->setSpeed(i);
    delay(5);
  }
  //delay(1000);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
  for (int i=0; i<150; i++) {
    leftMotor->setSpeed(i);
    rightMotor->setSpeed(i);
    delay(5);
  }
  
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);

  //Figure out the frequency to read the error
  setactuationTimePeriod(mode);
}

void loop() {

    //Calculate the elapsed time
    time_now = millis();
    dt  = time_now - time_last;
  
    float value = 0;
    cli(); // disable interrupt
      float copy_xSpeed = xSpeed;
      float copy_ySpeed = ySpeed; 
    sei(); // reenable the interrupt

    float v_left_now =  copy_xSpeed/speed_constant;
    float v_right_now = copy_ySpeed/speed_constant;
    //Compare
    if (V_right < 0)
      v_right_now = -1 * abs(v_right_now);
    else
      v_right_now = abs(v_right_now);
 
    if (V_left < 0)
      v_left_now = -1 * abs(v_left_now);   
    else
      v_left_now = abs(v_left_now);
      
    odometry_counter++;
    //Calculate the change in angle since last reading
    w_now = (v_right_now-v_left_now)/(base_length*pow(10,-3));
    th += w_now*dt*pow(10,-3);
    vx_now = cos(th)*(v_right_now + v_left_now)*0.5;
    vy_now = sin(th)*(v_right_now + v_left_now)*0.5;
    x += (vx_now)*dt*pow(10,-3);
    y += (vy_now)*dt*pow(10,-3);

    time_last = time_now;
    SendValue(73, x);
    SendValue(74, y);
    SendValue(75, th);
    SendValue(76, vx_now);
    SendValue(77, vy_now);
    SendValue(78, w_now);

    /*if(fbswitch==true){
        char logg[128];
        //sprintf(logg ,"%f %f ", v_left_now, v_right_now);
        sprintf(logg ,"%d %f %f %f %f %f %f %d", odometry_counter, x, y, th, vx_now, vy_now, w_now, dt);
        Serial.println(logg);
    }*/
  
    if (stringComplete) {
        header = serialcommand.substring(0, serialcommand.indexOf("_"));
        values = serialcommand.substring(serialcommand.indexOf("_")+1 , serialcommand.length());
        value = values.toFloat();

        if (header == "stop" ){
          //encoder.end();
          feedbackSwitch(false);
          motorsStop();
          delay(100);
          vx_now = vy_now = w_now = 0;
        }
        else if (header == "goto"){
          //in meters
          setPoint(POSITION,(value*pow(10,3)/circumference)*encoders_wheelrotation, -(value*pow(10,3)/circumference)  * encoders_wheelrotation);
          encoder.end();
          //setDirection();
          encoder.begin(encoderUpdate, dT);
        }
        else if (header == "turn"){
          setPoint(POSITION, -(value/360)  * encoders_baserotation, -(value/360)  * encoders_baserotation);
          encoder.end();
          //setDirection();
          encoder.begin(encoderUpdate, dT);
          feedbackSwitch(true);
        }
        else if (header == "twist"){
          odometry_counter = 0;
          feedbackSwitch(false);
          float _vx, _vy, _w;
          sscanf(values.c_str(),"%f %f %f", &_vx, &_vy, &_w);
          float linear_V = sqrt(pow(_vx,2) + pow(_vy,2));
          V_left = (2*linear_V - base_length*pow(10,-3)*_w)*0.5;
          V_right = (2*linear_V + base_length*pow(10,-3)*_w)*0.5;
          encoder.end();
            setPoint(VELOCITY, V_left, V_right);
          encoder.begin(encoderUpdate, dT);  // blinkLED to run every 166 microsecond ±6 KHz
          feedbackSwitch(true);
          
        }
        else if(header == "gains"){
          char logg[16];
          signed long kpi,kii, kdi;
          sscanf(values.c_str(),"%ld %ld %ld", &kpi,&kii,&kdi);
          sprintf(logg ,"Setting PID to %ld %ld %ld", kpi,kii, kdi);
          Serial.println(logg);
          setGains(kpi,kii, kdi);
        }
        else if(header == "odoreset"){
          feedbackSwitch(false);
          x = y = th = vx_now = vy_now = w_now = 0;
          feedbackSwitch(true);
        }
        else if(header == "configure"){
          int _gr, _ppr, max_pwm;
          float wheel_dia, base_dia;
          sscanf(values.c_str(),"%d %d %f %f %d", &_gr, &_ppr, &wheel_dia, &base_dia, &max_pwm );
          Serial.println(wheel_dia);
          set_parameters(_gr, _ppr, wheel_dia, base_dia, max_pwm);
        }
        else if(header == "configure?"){
          get_parameters();
        }
        // clear all strings
        serialcommand = "";
        stringComplete = false;
        header ="";
    }
   
    delay(10);
    

}

void feedbackSwitch(boolean state){
    fbswitch = state;
}

// functions called by IntervalTimer should be short, run as quickly as
// possible, and should avoid calling other functions if possible.
void encoderUpdate(void) {

  unsigned long nowState = 0 ;
  unsigned long encIndex =0;

  float dInput = 0;

  nowState = (digitalReadFast(xQ2Pin)<<1) | digitalReadFast(xQ1Pin) | (digitalReadFast(yQ2Pin)<<5) | (digitalReadFast(yQ1Pin)<<4);
  encIndex= nowState | (encoderState << 2);
  xEncoder = xEncoder + enc_TAB[ (encIndex >> 0) & 0x0f];  // these 4 lines take about 0.7 microseconds, maybe can be optimized
  yEncoder = yEncoder + enc_TAB[ (encIndex >> 4) & 0x0f];
  encoderState = nowState;

  elapsedtime++;
  if (elapsedtime*dT % (1000000/VELOCITY_SAMPLING_RATE) == 0){
    xSpeed = (xEncoder-lastxEncoder);
    ySpeed = (yEncoder-lastyEncoder);
    lastxEncoder=xEncoder;
    lastyEncoder=yEncoder;
    elapsedtime=0;
  }

  count1++;
  if (count1*dT >= actuationTimePeriod) {  // aiming for 1 khz update rate
      count1 = 0; // restart the count
      channelCount++;
      if (channelCount >=2) {
      channelCount = 0;
      }
      switch (channelCount)
      {  // actually each channel has its (actuationTimePeriod/2) Hz proportionate to its requirements

              case 0: // x channel

                    switch(mode){
                      case POSITION:
                        error = xSetPoint - xEncoder;
                        dInput = xEncoder - xLastInput;
                      break;
                      case VELOCITY:
                        error = xSetPoint - xSpeed;
                        dInput = xSpeed - xLastInput;
                      break;
                    }

                    xITerm+= (xki * error*1024);  // scaling for fixed point

                    if(xITerm > xoutMax*1024) xITerm= xoutMax*1024;
                    else if(xITerm < xoutMin*1024) xITerm= xoutMin*1024;

                    pwm_output = (xkp * error + xITerm- xkd * (dInput))/1024;

                    if(pwm_output > xoutMax) pwm_output = xoutMax;
                    else if (pwm_output < xoutMin) pwm_output = xoutMin;

                    rightMotor->setSpeed(abs(pwm_output));

                    switch(mode){
                      case POSITION:
                        xLastInput = xEncoder;
                      break;
                      case VELOCITY:
                        xLastInput = xSpeed;
                      break;
                    }
              break;


              case 1: // y channel

                    switch(mode){
                      case POSITION:
                        error = ySetPoint - yEncoder;
                        dInput = yEncoder - yLastInput;
                      break;
                      case VELOCITY:
                        error = ySetPoint - ySpeed;
                        dInput = ySpeed - yLastInput;
                    }

                    yITerm+= (yki * error*1024);  // scaling for fixed point
                    if(yITerm > youtMax*1024) yITerm= youtMax*1024;
                    else if(yITerm < youtMin*1024) yITerm= youtMin*1024;

                    pwm_output = (ykp * error + yITerm- ykd * (dInput))/1024;

                    if(pwm_output > youtMax) pwm_output = youtMax;
                    else if (pwm_output < youtMin) pwm_output = youtMin;

                    leftMotor->setSpeed(abs(pwm_output));

                    switch(mode){
                      case POSITION:
                        yLastInput = yEncoder;
                      break;
                      case VELOCITY:
                        yLastInput = ySpeed;
                      break;
                    }
              break;
      }
  }

}

void setactuationTimePeriod(int mode){
  //Varies the feedforward rate - assumes the motors will get (actuationTimePeriod/2) Hz e.g. if 50Hz is desired Feedforward rate, estimate for 100Hz
  actuationTimePeriod= 1000000/(2*VELOCITY_SAMPLING_RATE);// aiming for 1 khz update rate

}
void setGains(signed long kp, signed long ki,signed long kd){
   cli();
     xkp = ykp = kp;
     xki = yki = ki;
     xkd = ykd = kd;
   sei();
}
void setPoint(int mode_in, float v_left, float v_right){

   signed int dir_r = 0;
   signed int dir_l = 0;
   char logg[64];
   leftMotor->run(RELEASE);
   rightMotor->run(RELEASE);
   
   if (v_right < 0){
      sprintf(logg ,"Right wheel negative");
      Serial.println(logg);
      xoutMin = 0;
      xoutMax = max_pwm;
      rightMotor->run(FORWARD);
      dir_r = -1;
   }
   else
   {
       xoutMin = -max_pwm;
       xoutMax = 0;
       dir_r = -1;
       rightMotor->run(BACKWARD); 
   }

   if (v_left < 0){
     youtMin = -max_pwm;
     youtMax = 0;
     leftMotor->run(BACKWARD); 
   }
   else
   {
     youtMin = 0;
     youtMax = max_pwm;
     leftMotor->run(FORWARD);
   }
   
   cli();
      mode = mode_in;
      elapsedtime = 0;
      xSetPoint =  -1*v_right*speed_constant;
      ySetPoint =  v_left*speed_constant;
   sei();

   
   
}

void serialEvent(){

    while (Serial.available()) {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      serialcommand += inChar;
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
      if (inChar == '\n') {
        stringComplete = true;
        //Serial.print("Sometime arrived");
      }
    }

}

void SendValueCoordinatesDeg(uint16_t id, int32_t value)
{
  byte data[10];
  byte data2[20];
  byte msg[30];
  int m = 0;

  data[m++] = (byte) id;
  data[m++] = (byte) id >> 8;
  data[m++] =  (byte) value;
  data[m++] =  (byte) (value >> 8);
  data[m++] =  (byte) (value >> 16);
  data[m++] =  (byte) (value >> 24);

  //compose the final msg with checkSum
  int n = 0;
  int chSum = 0;
  for (byte i=0;i<m;i++)
  {
    data2[n++] = data[i];
    chSum ^= data2[n-1];
  }
  data2[n++] = (byte)chSum;
  data2[n++] = (byte)chSum >> 8;

 //Check for ST & ET within the data and checksum and mark them as data using ST
 int j = 0;
 msg[j++] = ST; //add start transmission
 for (byte i=0;i<n;i++)
  {
    msg[j++] = data2[i];
    if (data2[i] == ST or data2[i] == ET)
    {//mark as transparent
      msg[j++] = ST;
    }
  }
  msg[j++] = ST;
  msg[j++] = ET;

  //print
  Serial.write(msg,j);
}

void SendValue(uint16_t id, float value)
{
  byte data[10];
  byte data2[20];
  byte msg[30];
  int m = 0;
  FLOATUNION_t dataFloat;
  dataFloat.value = value;
  //data.length = sizeof(payload);

  data[m++] = (byte) id;
  data[m++] = (byte) id >> 8;
  //int32_t val = (int32_t) (value * pow(10,scale));
  data[m++] = dataFloat.bytes[0];
  data[m++] = dataFloat.bytes[1];
  data[m++] = dataFloat.bytes[2];
  data[m++] = dataFloat.bytes[3];
  //data[m++] = scale;

  //compose the final msg
  int n = 0;
  int chSum = 0;
  for (byte i = 0; i < m; i++)
  {
    data2[n++] = data[i];
    chSum ^= data2[n - 1];
  }
  data2[n++] = (byte)chSum;
  data2[n++] = (byte)chSum >> 8;

  //Check for ST & ET within the data and checksum and mark them as data using ST
  int j = 0;
  msg[j++] = ST; //add start transmission
  for (byte i = 0; i < n; i++)
  {
    msg[j++] = data2[i];
    if (data2[i] == ST or data2[i] == ET)
    { //mark as transparent
      msg[j++] = ST;
    }
  }
  msg[j++] = ST;
  msg[j++] = ET;

  Serial.write(msg, j);
}

// see.stanford.edu/materials/aiircs223a/handout6_Trajectory.pdf
// developer.mbed.org/cookbook/PID
// brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
// www.embeddedrelated.com/showarticle/121.php
