
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
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
// define pin connections
#define CS_PIN 10
#define CLOCK_PIN 13
#define MOSI_PIN 12
#define MISO_PIN 11

// definition of how the pins are set up
const int xQ1Pin = 15;  // pin for x axis quadrature encoder    J1 pin 4
const int xQ2Pin = 16;  // pin for x axis quadrature encoder    J1 pin 3

const int yQ1Pin = 3;  // pin for y axis quadrature encoder   J2 pin 4
const int yQ2Pin = 4;  // pin for y axis quadrature encoder   J2 pin 3

const int motorEnable = 2; // pin to enable the motor drivers (all)  goes to EA and EB of seeedstudio motor shield v2.0 

/*
For a 60mm wheel
const float circumference = 188.495592; //mm
const int encoders_wheelrotation = 1800;
const int encoders_baserotation = 2898;
*/
//For tracks
const float circumference = 122.5221135; //mm
const float wradius = 19.5; //mm
const float w2wradius = 48.3;//mm
const int encoders_wheelrotation = 1800; //pulses
const int encoders_baserotation = 4459; //pulses

float w_w2w_ratio=0.4;

signed int max_pwm = 150;

signed int xoutMin = 0;
signed int xoutMax = 0;
signed int youtMin = 0;
signed int youtMax = 0;

// Teensy 3.0 has the LED on pin 13
const int ledPin = 13;
int p=0;

// this is the decode table for the optical encoder
signed long enc_TAB[]={0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

// the setup() method runs once, when the sketch starts

// Create an IntervalTimer object 
IntervalTimer encoder;

//All things to do with serial
char* type;
String serialcommand, header, values;

#define POSITION 0
#define VELOCITY 1
#define TURNRATE 1

//dT 20 original
#define dT 20 //Interrupt frequency in microseconds
signed int PULSE_DEG = 5; //Number of encoder pulses per degree
signed int PULSE_DEG_BASE = 13; //Round off - should be 12.38 insted
signed int  VELOCITY_SAMPLING_RATE = 50; //Instantaneous velocity capture in hz
unsigned int freqRate=0;
  
volatile unsigned int mode = POSITION; // by default set it to position tracking
volatile bool modeAngular = false;
char msg[32];

volatile unsigned long encoderState = 0; // use volatile for shared variables

volatile signed long xEncoder = 0; // This is the position based on counting encoder pulses
volatile signed long yEncoder = 0; // This is the position based on counting encoder pulses
volatile signed long xSpeed = 0; // This is the position based on counting encoder pulses
volatile signed long ySpeed = 0; 
volatile signed int xError = 0;
volatile signed int yError = 0;
volatile signed long lastxEncoder = 0;
volatile signed long lastyEncoder = 0;
volatile signed long pulse_deg = 5;
volatile signed long elapsedtime = 0;

volatile signed long xSetPoint = 0; // this is where the PID loop will try to move the axis
volatile signed long ySetPoint = 0;


unsigned int count1 = 0;  // so that the encoder counting is faster than the PID updating
unsigned int channelCount = 0;  // 

volatile signed long xki=1024;  // PID integration constant
volatile signed long xkp=1024;  // PID proportional constant
volatile signed long xkd=1024;  // PID derivative constant
signed long xITerm=0;
signed long xLastInput=0;

volatile signed long yki=1024;  // PID integration constant
volatile signed long ykp=1024;  // PID proportional constant
volatile signed long ykd=1024;  // PID derivative constant
signed long yITerm=0;
signed long yLastInput=0;


String inputString = ""; 
boolean stringComplete = false;

//All things MotorHat Related
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);

void motorsStop(){
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}
void setup() {

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
  for (int i=0; i<100; i++) {
    leftMotor->setSpeed(i);  
    rightMotor->setSpeed(i); 
    delay(10);
  }
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
  for (int i=0; i<100; i++) {
    leftMotor->setSpeed(i);
    rightMotor->setSpeed(i);    
    delay(10);
  }
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);    
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

// functions called by IntervalTimer should be short, run as quickly as
// possible, and should avoid calling other functions if possible.
void encoderUpdate(void) {
  unsigned long nowState = 0 ;
  unsigned long encIndex =0;
  signed long pwm_output=0; // for PID loops
  signed long error = 0;
  signed long dInput = 0;
 
  nowState = (digitalReadFast(xQ2Pin)<<1) | digitalReadFast(xQ1Pin) | (digitalReadFast(yQ2Pin)<<5) | (digitalReadFast(yQ1Pin)<<4);
  encIndex= nowState | (encoderState << 2);
  xEncoder = xEncoder + enc_TAB[ (encIndex >> 0) & 0x0f];  // these 4 lines take about 0.7 microseconds, maybe can be optimized
  yEncoder = yEncoder + enc_TAB[ (encIndex >> 4) & 0x0f];
  encoderState = nowState; 

  elapsedtime++;
  if (elapsedtime*dT % (1000000/VELOCITY_SAMPLING_RATE) == 0){

    if(modeAngular){
       xSpeed =  w_w2w_ratio*(xEncoder-lastxEncoder); 
       ySpeed =  w_w2w_ratio*(yEncoder-lastyEncoder);    
    }
    else{
       xSpeed = (xEncoder-lastxEncoder); 
       ySpeed = (yEncoder-lastyEncoder); 
    }
    lastxEncoder=xEncoder;
    lastyEncoder=yEncoder;
    elapsedtime=0;
  }
  
  count1++;
  
  if (count1 >= freqRate) {  // aiming for 1 khz update rate
  count1 = 0; // restart the count
  channelCount++;
      if (channelCount >=2) {
      channelCount = 0;
      }
      switch (channelCount) 
      {  // actually each channel has its (freqRate/2) Hz proportionate to its requirements
      
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
                    
                    leftMotor->setSpeed(abs(pwm_output)); 
                    
                    switch(mode){
                      case POSITION:
                        xLastInput = xEncoder;
                      break;
                      case VELOCITY:
                        xLastInput = xSpeed;
                      break;
                    }
                    xError = error;
              break;
              
              // y channel
              case 1:
              
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
                         
                    rightMotor->setSpeed(abs(pwm_output));   
                      
                    switch(mode){
                      case POSITION:
                        yLastInput = yEncoder;
                      break;
                      case VELOCITY:
                        yLastInput = ySpeed;
                      break;
                    }
                    yError = error;   
              break;    
      }
  }

}

void loop() {
  

    float value = 0;
    
    cli(); // disable interrupt
      signed int copy_xEncoder = xSpeed;
      signed int copy_yEncoder = ySpeed;
      signed int copy_xError = xError;
      signed int copy_yError = yError;
      float xPercentError = (float) 100*copy_xError/xSetPoint;
      float yPercentError = (float) 100*copy_yError/ySetPoint;
  
    sei(); // reenable the interrupt 

    if(modeAngular){
        sprintf(msg,"%d\t%d\t%d\t%d", copy_xEncoder*VELOCITY_SAMPLING_RATE/PULSE_DEG_BASE, copy_yEncoder*VELOCITY_SAMPLING_RATE/PULSE_DEG_BASE, (int) xPercentError, (int) yPercentError);
    }
    else{
        sprintf(msg,"%d\t%d\t%d\t%d", copy_xEncoder*VELOCITY_SAMPLING_RATE/PULSE_DEG, copy_yEncoder*VELOCITY_SAMPLING_RATE/PULSE_DEG, (int) xPercentError, (int) yPercentError);
    }
    
    Serial.println(msg);
    
    if (stringComplete) {
        header = serialcommand.substring(0, serialcommand.indexOf("_"));
        values = serialcommand.substring(serialcommand.indexOf("_")+1 , serialcommand.length());
        value = values.toFloat();
        
        if (header == "stop" ){
          encoder.end();
          motorsStop();      
        }
        else if (header == "goto"){       
          setPoint(POSITION, false, (value/circumference)  * encoders_wheelrotation, -(value/circumference)  * encoders_wheelrotation);
          encoder.end();    
          setDirection();
          encoder.begin(encoderUpdate, dT);  // blinkLED to run every 166 microsecond ±6 KHz
        }
        else if (header == "turn"){
          setPoint(POSITION, false, -(value/360)  * encoders_baserotation, -(value/360)  * encoders_baserotation);
          encoder.end();    
          setDirection();
          encoder.begin(encoderUpdate, dT);  // blinkLED to run every 166 microsecond ±6 KHz
        }
        else if (header == "speed"){
          //Divide the value from deg/s to the sampling rate
          value = PULSE_DEG*value/VELOCITY_SAMPLING_RATE; 
          setPoint(VELOCITY, false,  value, -value);   
          encoder.end();
          setDirection(); 
          encoder.begin(encoderUpdate, dT);  // blinkLED to run every 166 microsecond ±6 KHz
          } 
        else if (header == "spin"){
          //Divide the value from deg/s to the sampling rate
          value = PULSE_DEG_BASE*value/VELOCITY_SAMPLING_RATE; 
          setPoint(VELOCITY, true, -value, -value);  
          encoder.end();
          setDirection(); 
          
          encoder.begin(encoderUpdate, dT);  // blinkLED to run every 166 microsecond ±6 KHz    
        } 
        else if(header == "gains"){
          char logg[16];
          signed long kpi,kii, kdi;
          sscanf(values.c_str(),"%ld %ld %ld", &kpi,&kii,&kdi);
          sprintf(logg ,"Setting PID to %ld %ld %ld", kpi,kii, kdi);
          Serial.println(logg);
          setGains(kpi,kii, kdi);
        }
        // clear all strings
        serialcommand = "";
        stringComplete = false;
        header ="";
    }
    
    delay(10);  
}

void setFreqRate(int mode){
  //Varies the feedforward rate - assumes the motors will get (freqRate/2) Hz e.g. if 50Hz is desired Feedforward rate, estimate for 100Hz
  switch(mode){
    case POSITION:
      freqRate=1000/dT;// aiming for 1 khz update rate
    break;
    case VELOCITY:
      freqRate=(1000/(2*VELOCITY_SAMPLING_RATE)*1000)/dT;// aiming for 100hz update rate
    break;
  }
}
void setGains(signed long kp, signed long ki,signed long kd){
   cli();
     xkp = ykp = kp;
     xki = yki = ki;
     xkd = ykd = kd;
   sei();
}
void setPoint(int mode_in, bool angular_mode, signed long value_left, signed long value_right){
   cli();
      mode = mode_in;
      setFreqRate(mode);
      modeAngular = angular_mode;
      xEncoder = 0 ;
      yEncoder = 0 ;  
      elapsedtime = 0;
      xSpeed = 0;
      ySpeed = 0;
      xSetPoint =  value_left;
      ySetPoint =  value_right;
   sei();
}
void setDirection(){
  
   leftMotor->run(RELEASE);
   rightMotor->run(RELEASE); 
   if (xSetPoint- xEncoder < 0 && xSetPoint < xEncoder) {
          xoutMin = -max_pwm;
          xoutMax = 0;
          leftMotor->run(BACKWARD);
        }
        //Turn forward
        else if (xSetPoint-xEncoder > 0 && xSetPoint > xEncoder) {
          xoutMin = 0;
          xoutMax = max_pwm;
          leftMotor->run(FORWARD);
        }

        if (ySetPoint- yEncoder < 0 && ySetPoint < yEncoder) {
          youtMin = -max_pwm;
          youtMax = 0;
          rightMotor->run(BACKWARD);
        }
        //Turn forward
        else if (ySetPoint-yEncoder > 0 && ySetPoint > yEncoder) {
          youtMin = 0;
          youtMax = max_pwm;
          rightMotor->run(FORWARD);
        }
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


// see.stanford.edu/materials/aiircs223a/handout6_Trajectory.pdf
// developer.mbed.org/cookbook/PID
// brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
// www.embeddedrelated.com/showarticle/121.php







