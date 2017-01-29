
/* code by fdavies

   This example code is in the public domain.
   
   The goal is to have a complete 3D printer driver (gcode => motion)
   that uses optical encoders and brushed DC motors.
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// definition of how the pins are set up
const int xQ1Pin = 15;  // pin for x axis quadrature encoder    J1 pin 4
const int xQ2Pin = 16;  // pin for x axis quadrature encoder    J1 pin 3

const int yQ1Pin = 3;  // pin for y axis quadrature encoder   J2 pin 4
const int yQ2Pin = 4;  // pin for y axis quadrature encoder   J2 pin 3

const int motorEnable = 2; // pin to enable the motor drivers (all)  goes to EA and EB of seeedstudio motor shield v2.0 

const float circumference = 188.495592; //mm
const int encoders_rev = 1800;

int max_pwm = 200;
//const int outMin = -100; // minimum output level (PWM)
//const int outMax = 100;  // maximum output level (PWM)

int xoutMin = 0;
int xoutMax = 0;
int youtMin = 0;
int youtMax = 0;
// Teensy 3.0 has the LED on pin 13
const int ledPin = 13;
int p=0;
// this is the decode table for the optical encoder
signed long enc_TAB[]={0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

// the setup() method runs once, when the sketch starts

// Create an IntervalTimer object 
IntervalTimer encoder;

//All things to do with serial

char msg[16];

volatile unsigned long encoderState = 0; // use volatile for shared variables

volatile signed long xEncoder = 0; // This is the position based on counting encoder pulses
volatile signed long yEncoder = 0; // This is the position based on counting encoder pulses

volatile signed long xSetPoint = 0; // this is where the PID loop will try to move the axis
volatile signed long ySetPoint = 0;
volatile signed long xTarget = 0;
volatile signed long yTarget = 0;

//Variables only for printing 
volatile signed long Output1=0; // for PID loops
volatile signed long Output2=0; // for PID loops

unsigned int count1 = 0;  // so that the encoder counting is faster than the PID updating
unsigned int channelCount = 0;  // 

signed long xki=0;  // PID integration constant
signed long xkp=1024;  // PID proportional constant
signed long xkd=1024;  // PID derivative constant
signed long xITerm=0;
signed long xLastInput=0;

signed long yki=0;  // PID integration constant
signed long ykp=1024;  // PID proportional constant
signed long ykd=1024;  // PID derivative constant
signed long yITerm=0;
signed long yLastInput=0;

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

  Serial.begin(9600);  
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
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

// functions called by IntervalTimer should be short, run as quickly as
// possible, and should avoid calling other functions if possible.
void encoderUpdate(void) {
  unsigned long nowState = 0 ;
  unsigned long encIndex =0;
 
  signed long dInput=0; // for PID loops
  signed long pwm_output=0; // for PID loops
  signed long error = 0;
  
  nowState = (digitalReadFast(xQ2Pin)<<1) | digitalReadFast(xQ1Pin)      | (digitalReadFast(yQ2Pin)<<5)  | (digitalReadFast(yQ1Pin)<<4);

  encIndex=nowState | (encoderState << 2);
             
  xEncoder = xEncoder + enc_TAB[ (encIndex >> 0) & 0x0f];  // these 4 lines take about 0.7 microseconds, maybe can be optimized
  yEncoder = yEncoder + enc_TAB[ (encIndex >> 4) & 0x0f];

  encoderState = nowState; 

  count1++;
  if (count1 >= 50) {  // aiming for 1 khz update rate
    count1 = 0; // restart the count
    channelCount++;
    if (channelCount >= 2) {
     channelCount = 0;
    }
    switch (channelCount) {  // actually each channel has its PID updated at 250Hz, which seems faste enough so far.
    case 0: // x channel
      {
        error = xSetPoint - xEncoder;
        xITerm+= (xki * error*1024);  // scaling for fixed point 
        
        if(xITerm > xoutMax*1024) xITerm= xoutMax*1024;
        else if(xITerm < xoutMin*1024) xITerm= xoutMin*1024;
   
        pwm_output = (xkp * error + xITerm- xkd * dInput)/1024;
       
        if(pwm_output > xoutMax) pwm_output = xoutMax;
        else if (pwm_output < xoutMin) pwm_output = xoutMin;

        Output1 = pwm_output;
        leftMotor->setSpeed(abs(pwm_output)); 
        xLastInput = xEncoder;  
      }
      break;
      
      case 1: // y channel
        error = ySetPoint - yEncoder;
        yITerm+= (yki * error*1024);  // scaling for fixed point         
 
        if(yITerm > youtMax*1024) yITerm= youtMax*1024;
        else if(yITerm < youtMin*1024) yITerm= youtMin*1024;
   
        pwm_output = (ykp * error + yITerm- ykd * dInput)/1024;
       
        if(pwm_output > youtMax) pwm_output = youtMax;
        else if (pwm_output < youtMin) pwm_output = youtMin;

        //To print on screen
        Output2 = pwm_output;
        rightMotor->setSpeed(abs(pwm_output));     
        yLastInput = yEncoder;        
      break;
    }
  }
  

}

char* type;
float  value = 0.0;

void loop() {
  
  signed long copy_xEncoder= 0;
  signed long copy_yEncoder= 0;
  signed long screenOutput = 0;
  signed long screenOutput1 = 0;
  String serialcommand, header, values;
  
    cli(); // disable interrupt
    copy_xEncoder = xEncoder;
    copy_yEncoder = yEncoder;
    screenOutput = Output1;
    screenOutput1 = Output2; 
    sei(); // reenable the interrupt 

    sprintf(msg,"%ld %ld %ld %ld", copy_xEncoder, copy_yEncoder,screenOutput,  screenOutput1);
    Serial.println(msg);
    //incoming command handler
    if(Serial.available()>0){
      serialcommand = Serial.readString();
      header = serialcommand.substring(0, serialcommand.indexOf(" "));
      values = serialcommand.substring(serialcommand.indexOf(" ") , serialcommand.length()+1);
      value = values.toFloat();
      Serial.println(value);
      if (header == "stop" ){
        encoder.end();
        Serial.println("Stopping motors");
        motorsStop();
      }
      else if (header == "drive"){
        cli();
          xEncoder = 0 ;
          yEncoder = 0 ; 
          xSetPoint =  (value/circumference)  * encoders_rev; //+ xEncoder;
          ySetPoint =  -(value/circumference)  * encoders_rev;// + yEncoder;
        sei();
        encoder.end();
        
        leftMotor->run(RELEASE);
        rightMotor->run(RELEASE);
                
        if (xSetPoint- xEncoder < 0 && xSetPoint < xEncoder) {
          Serial.println("Driving backward");
          xoutMin = -255;
          xoutMax = 0;
          leftMotor->run(BACKWARD);
          
        }
        //Turn forward
        else if (xSetPoint-xEncoder > 0 && xSetPoint > xEncoder) {
          Serial.println("Driving forward");
          xoutMin = 0;
          xoutMax = 255;
          leftMotor->run(FORWARD);
        }

        if (ySetPoint- yEncoder < 0 && ySetPoint < yEncoder) {
          Serial.println("Driving backward");
          youtMin = -255;
          youtMax = 0;
          rightMotor->run(BACKWARD);
        }
        //Turn forward
        else if (ySetPoint-yEncoder > 0 && ySetPoint > yEncoder) {
          Serial.println("Driving forward");
          youtMin = 0;
          youtMax = 255;
          rightMotor->run(FORWARD);
        }
        encoder.begin(encoderUpdate, 20);  // blinkLED to run every 166 microsecond ±6 KHz
      }
      else if (header == "disable"){    
         // Set the speed to start, from 0 (off) to 255 (max speed)
        encoder.end();  // blinkLED to run every 0.02 millisecond 50 KHz
        digitalWriteFast(ledPin,0); // for measuring interrupt duration with an oscilloscope
      }
  }
  delay(500);  

}
// see.stanford.edu/materials/aiircs223a/handout6_Trajectory.pdf
// developer.mbed.org/cookbook/PID
// brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
// www.embeddedrelated.com/showarticle/121.php







