/* 

<----Front--->
+------------+     ^
|            |     |
|            |  Forward
|            |     |
+------------+  
motor1       motor2

This program is designed to control a robot that using a 'Shopping Cart' drive train
 \Shopping Cart meaning that the front wheels of the robot is free spinning, and the back two motors are independing controlled. 

This program was created by Tyler Adam Martinez and Jorge M. Marmolejo. 

This program is free and open for educational use.
*/

#include <Servo.h> 


/*   This part of the code is for Rx for the RF module 
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif
RH_ASK driver;
// RH_ASK driver(2000, 4, 5, 0); // ESP8266 or ESP32: do not use pin 11 or 2
// RH_ASK driver(2000, 3, 4, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85), 
*/

#define MOTOR1 4  //pin number for motor1
#define MOTOR2 5  //pin number for motor2

#define SLIGHT_TURN 500 //number of milliseconds to hold turn when turning
#define FULL_TURN 0 // number of milliseconds to turn completely around

int POS = 0;
enum dir {fwd = 0, bwd, left, right};

//Declaring the varible to setup the Servo1 and Servo2 calculations

//bool fast = true;
int mspeedfwd = 1600; //1700    <--  Full speed //1600 <-- Half speed
int mspeedbwd = 1400; //1300    <--  Full speed //1400 <-- Half speed
int mspeedturn1 = 1550; //<-- fourth of the Full speed       NOTE: for safty turn the robot slowly. 
int mspeedturn2 = 1450; 

//Declaring Motors as Servos 
Servo motor1; //motor1
Servo motor2; //motor2

void setup() {

  motor1.attach(MOTOR1);  // attaches motor1 to its defined pin
  motor2.attach(MOTOR2);  // attaches motor2 to its defined pin
  
  Serial.begin(115200);
}

void loop() {

motorcontrol(fwd);
  delay(10000);

motorcontrol(bwd);
  delay(2000);

motorcontrol(left);
  delay(1000);

motorcontrol(right);
  delay(1000);  


}


void backup(){
  
  motor1.writeMicroseconds(mspeedbwd);
  motor2.writeMicroseconds(mspeedbwd);
  Serial.println("Reversed ");
  return;
}

void forward(){
 
  motor1.writeMicroseconds(mspeedfwd);  //1600    <--  half speed
  motor2.writeMicroseconds(mspeedfwd);  //1700    <--  Full speed
  Serial.println("Forward ");
  return;
}


void motorstop(){
  motor1.writeMicroseconds(1500);   //1500 <-- no speed
  motor2.writeMicroseconds(1500);  
  Serial.println("No Movement ");
  return;
}


void turnleft(){
  
  motor1.writeMicroseconds(mspeedturn1);
  motor2.writeMicroseconds(mspeedturn2);

  delay(SLIGHT_TURN);
  Serial.println("Turning Left ");
  return;
  }

void turnright(){
  motor1.writeMicroseconds(mspeedturn2);  
  motor2.writeMicroseconds(mspeedturn1);  
  
  delay(SLIGHT_TURN);
  Serial.println("Turning Right ");
  return;
  }
  
void motorcontrol(int dir){
  switch(dir)
  {
    case(fwd):
    forward();
    break;
    case(bwd):
    backup();
    break;
    case(left):
    turnleft();
    break;
    case(right):
    turnleft();
    break;
    default:
    motorstop();
    break;
  }
  return;
}



//© 2019 Tyler Adam Martinez 
