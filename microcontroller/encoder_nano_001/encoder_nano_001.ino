#include <Encoder.h>

int interrupt_pin1_left = 17;
int interrupt_pin2_right = 15;
int direction_pin1_left = 16;
int direction_pin2_right = 14 ;

Encoder left(interrupt_pin1_left, direction_pin1_left);
Encoder right(interrupt_pin2_right, direction_pin2_right);

long pLeft = -999;
long pRight= -999;
char msg[16];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  left.write(0);
  right.write(0);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  long newLeft;
  long newRight;
  newLeft = left.read();
  newRight = right.read();
  
  //if (newLeft != pLeft || newRight != pRight){
  sprintf(msg, "%ld\t%ld", newLeft, newRight);
  Serial.println(msg);
  delay(10);
  //  pLeft = newLeft;
  //  pRight= newRight;
  //}

  if(Serial.available()>0){
    int received = Serial.read();
    if(received == 114){
      Serial.println("Resetting...");
      //left.write(0);
      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);               // wait for a second
      digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
      delay(500);   
      left.write(0);
      right.write(0);
      Serial.write(100);  
    }
  }
}
