
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
#include <string.h> 
#include <Adafruit_MotorShield.h>
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
#define xQ1Pin 15  // pin A for x axis quadrature encoder    J1 pin 4
#define xQ2Pin 16  // pin B for x axis quadrature encoder    J1 pin 3
#define yQ1Pin 3  // pin A for y axis quadrature encoder   J2 pin 4
#define yQ2Pin 4  // pin B for y axis quadrature encoder   J2 pin 3

//For encoders
int gear_ratio = 150;
int ppr=12;
int pulse_deg=5;//Number of encoder pulses per degree
int pulse_deg_base= 12.38; //Round off - should be 12.38 insted
int max_pwm=150;

//For tracks
float radius = 19.5; //mm
float base_radius = 48.3; //mm 
float circumference = 122.5221135; //mm
int encoders_wheelrotation = 1800; //pulses
int encoders_baserotation = 4459; //pulses

#define D2R 0.0174533
#define R2D 57.295
//Constants used to convert speed and spin targets into pulses/sec @ specified velocity sampling rate period

//Converts speed in m/s into pulses/sec @ specified velocity sampling rate period
const float speed_constant = (float) pulse_deg*(R2D/(radius*pow(10,-3)))/VELOCITY_SAMPLING_RATE;

//Converts speed in deg/s into pulses/sec @ specified velocity sampling rate period
const float spin_constant = (float) (radius/base_radius)*pulse_deg_base/VELOCITY_SAMPLING_RATE;

// this is the decode table for the optical encoder
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

volatile signed long lastxEncoder = 0;
volatile signed long lastyEncoder = 0;

volatile signed long xki=1024;  // PID integration constant
volatile signed long xkp=1024;  // PID proportional constant
volatile signed long xkd=1024;  // PID derivative constant

volatile signed long yki=1024;  // PID integration constant
volatile signed long ykp=1024;  // PID proportional constant
volatile signed long ykd=1024;  // PID derivative constant

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
long increment_counter =0;
boolean fbswitch = false;
//All things MotorHat Related
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);


void set_parameters(int _gr, int _ppr, float wheel_dia, float base_dia, int max_pwm = 150 ){
  
  radius = wheel_dia/2;
  base_radius = base_dia/2;
  circumference = PI*wheel_dia;
  encoders_wheelrotation =  _gr*_ppr;
  encoders_baserotation  =  encoders_wheelrotation*(PI*base_dia)/circumference;
  pulse_deg = encoders_wheelrotation/360;
  pulse_deg_base = encoders_baserotation/360;
  
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
  Serial.begin(57600);  
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
  
  //Figure out the frequency to read the error 
  setactuationTimePeriod(mode);
}

void loop() {
  
    float value = 0; 
    cli(); // disable interrupt
      float copy_xSpeed = xSpeed;
    sei(); // reenable the interrupt 
    
    if(fbswitch){

       //Calculate the elapsed time
       time_now = millis();
       dt  = time_now - time_last; 
       //Grab the current speed:
       v_now = copy_xSpeed/speed_constant;
       if(modeAngular == 1){         
          //Calculate the change in angle since last reading
          w_now = -1*v_now/(base_radius*pow(10,-3));
          th += w_now*dt*pow(10,-3);                
       }else if (modeAngular == 0){
          vx_now = cos(th)*v_now;
          vy_now = sin(th)*v_now;
          x += (vx_now)*dt*pow(10,-3);
          y += (vy_now)*dt*pow(10,-3);
        }     
       
        sprintf(msg,"%.3f %.3f %.3f %.3f %.3f %.3f\r", x, y, th, vx_now, vy_now, w_now);    
        Serial.println(msg);
        time_last = time_now;  
        increment_counter++;       
    }
       
    if (stringComplete) {
        header = serialcommand.substring(0, serialcommand.indexOf("_"));
        values = serialcommand.substring(serialcommand.indexOf("_")+1 , serialcommand.length());

     /*   std::string orbits ("686.97 365.24");
        std::string::size_type sz;     // alias of size_t
        float mars = std::stof (orbits,&sz);
        float earth = std::stof (orbits.substr(sz));
         std::cout << "One martian year takes " << (mars/earth) << " Earth years.\n";*/

        
        value = values.toFloat();
       
        if (header == "stop" ){
          encoder.end();
          motorsStop();       
          feedbackSwitch(false);  
          modeAngular = -1; //After a reset or stop, go into an unknown state     
          increment_counter=0;        
        }
        else if (header == "goto"){
          //in meters       
          setPoint(POSITION, 0, (value*pow(10,3)/circumference)*encoders_wheelrotation, -(value*pow(10,3)/circumference)  * encoders_wheelrotation);
          encoder.end();    
          setDirection();
          encoder.begin(encoderUpdate, dT);   
          feedbackSwitch(true);
        }
        else if (header == "turn"){
          setPoint(POSITION, 0, -(value/360)  * encoders_baserotation, -(value/360)  * encoders_baserotation);
          encoder.end();    
          setDirection();
          encoder.begin(encoderUpdate, dT);   
          feedbackSwitch(true);
        }
        else if (header == "speed"){
          feedbackSwitch(false);
          //Divide the value from m/s to the setpoint in degree/sec     
          float setpoint = value*speed_constant;
          setPoint(VELOCITY, 0,  setpoint, -setpoint);   
          encoder.end();
          setDirection(); 
          encoder.begin(encoderUpdate, dT);  // blinkLED to run every 166 microsecond ±6 KHz*/
          feedbackSwitch(true);
        } 
        else if (header == "spin"){
          feedbackSwitch(false);
          //Assume incoming request is in degrees/sec so convert into rad/s  
          float setpoint = D2R*value*speed_constant*base_radius*pow(10,-3);
          // 
          setPoint(VELOCITY, 1, -setpoint, -setpoint);  
          encoder.end();
          setDirection();        
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
                    
                    leftMotor->setSpeed(abs(pwm_output)); 
                    
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
                         
                    rightMotor->setSpeed(abs(pwm_output));   
                      
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
void setPoint(int mode_in, int angular_mode, float value_left, float value_right){
   cli();
      mode = mode_in;
      modeAngular = angular_mode;
      //xEncoder = 0 ;
      //yEncoder = 0 ;  
      elapsedtime = 0;
      xSetPoint =  value_left;
      ySetPoint =  value_right;
   sei();
}
void setDirection(){
  
   leftMotor->run(RELEASE);
   rightMotor->run(RELEASE); 
   if ((xSetPoint+xEncoder)- xEncoder < 0 && (xSetPoint+xEncoder) < xEncoder) {
          xoutMin = -max_pwm;
          xoutMax = 0;
          leftMotor->run(BACKWARD);
        }
        //Turn forward
        else if ((xSetPoint+xEncoder)-xEncoder > 0 && (xSetPoint+xEncoder) > xEncoder) {
          xoutMin = 0;
          xoutMax = max_pwm;
          leftMotor->run(FORWARD);
        }

        if ((ySetPoint+yEncoder)- yEncoder < 0 && (ySetPoint+yEncoder) < yEncoder) {
          youtMin = -max_pwm;
          youtMax = 0;
          rightMotor->run(BACKWARD);
        }
        //Turn forward
        else if ((ySetPoint+yEncoder)-yEncoder > 0 && (ySetPoint+yEncoder) > yEncoder) {
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







