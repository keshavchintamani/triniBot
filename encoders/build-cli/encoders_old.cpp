#include <Arduino.h>

/* sketch to read encoders for a Roomba 4000 series motor*/
const int encoderIn = 8; // input pin for the interrupter 
const int statusLED = 13; // Output pin for Status indicator
const int pulseOutput = 12; // Pulse output pin for external interfacing
int detectState=0; // Variable for reading the encoder status
int odo_counter = 0;
int turns = 0;
bool encState=LOW;
bool old_encState;
unsigned long time1, time2;
float dV;
float wheel_radius = 0.025; //Check Roomba
float angle_per_pulse = 360/32;
const float pi = 3.142;
int last_odo=0;
float res;
int a1=A1;
void setup()
{
   pinMode(a1, INPUT);
   pinMode(encoderIn, INPUT); //Set pin 8 as input
   pinMode(statusLED, OUTPUT); //Set pin 13 as output
   pinMode(pulseOutput, OUTPUT); // Set Pin 12 as output
   Serial.begin(9600

);
   
}


void loop() {
 
   
   old_encState=encState;
   //time1 = micros();
   res = (analogRead(a1)*5)/1023;
   if(res>4.5)
         encState = LOW;
   else
         encState = HIGH;
    


   if(encState != old_encState)
   {    
     //time2 = micros();
     odo_counter++;
     //dV = (angle_per_pulse*pi/180)*(last_odo-odo_counter)/((time2-time1));
     last_odo = odo_counter;
     Serial.println(odo_counter);
   }
   
}
