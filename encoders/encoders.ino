//Works See photo taken on Nexus on 01-09-2015
const int encoderIn = 8; // input pin for the interrupter 
const int statusLED = 13; // Output pin for Status indicator
const int pulseOutput = 12; // Pulse output pin for external interfacing
int detectState=0; // Variable for reading the encoder status
int a1 = A1;
int v1;
float res=0; 
int slots = 32;
int counter = 0;
int turns = 0;
bool encState=LOW;
bool old_encState;

unsigned long time1, time2;
float dV;
float wheel_radius = 0.0325; //Check Roomba
float angle_per_pulse = 360/32;
const float pi = 3.142;
int last_odo=0;

//Adding TimerOne related 
#include <TimerOne.h>
#define TIMER_US 200000
#define TICK_COUNTS 20 
#define uS2S 1/1000000
volatile long tick_count = TICK_COUNTS;         // Counter for 2S
volatile bool in_long_isr = false;              // True if in long interrupt
boolean toggle= false;

void timerIsr()
{
    toggle = !toggle;
    if (toggle)
        digitalWrite( statusLED, HIGH);   // Toggle LED 0
    else
        digitalWrite( statusLED, LOW);   // Toggle LED 0

    dV = (float)(angle_per_pulse*pi/180)*(counter-last_odo)/0.2;
    last_odo = counter;

}





void setup()
{
   pinMode(a1, INPUT);
   pinMode(encoderIn, INPUT); //Set pin 8 as input
   pinMode(statusLED, OUTPUT); //Set pin 13 as output
   pinMode(pulseOutput, OUTPUT); // Set Pin 12 as output
   Serial.begin(9600);
   
   Timer1.initialize(TIMER_US);                  // Initialise timer 1
   Timer1.attachInterrupt( timerIsr );           // attach the ISR routine here

}


void loop() {
  
   res = analogRead(a1);
   res = (res*5)/1023;
   if(res>4.8) encState = LOW;
   if(res<4.4) encState = HIGH;
     
   if(encState != old_encState)
   {      
     old_encState=encState;
     counter++;
   }
   Serial.println(dV,5);
   
}