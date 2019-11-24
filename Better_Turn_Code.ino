/* 
Algorthim for better turns. 
 <----T---->
+------------+  ^
|            |  |
|            |  L
|            |  |
+------------+  |  
ai           ao
T is 29 cm
L is 50 cm
R is the control varible to turn the randius 
ai = arctan(L/(R - (T/2));
ao = arctan(L/(R + (T/2));

Fast Turn Mode
  ao = 21.1398
  ai = 49.2364

Slow Turn Mode 
  ao = 6.4881
  ai = 8.0518

  Assumptions are that 128 is Straight.
*/
#include <Servo.h> 
#include <math.h>

/*   This part of the code is for Rx for the RF module 
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

RH_ASK driver;
// RH_ASK driver(2000, 4, 5, 0); // ESP8266 or ESP32: do not use pin 11 or 2
// RH_ASK driver(2000, 3, 4, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85), 

*/


#define SERVO1 2  //pin number for servo1
#define SERVO2 3  //pin number for servo2
#define MOTOR1 4  //pin number for motor1
#define MOTOR2 5  //pin number for motor2

#define SLIGHT_TURN 500 //number of milliseconds to hold turn when turning
#define FULL_TURN 0 // number of milliseconds to turn completely around

int POS = 0;
enum dir {fwd = 0, bwd, lslow, lfast, rslow, rfast};

//Declaring the varible to setup the Servo1 and Servo2 calculations

//bool fast = true;
int mspeedfwd = 1600;
int mspeedturn = 1550;
int straight = 128;

int ao_fast = 22 + straight; //21.1398
int ai_fast = 49 + straight; //49.2364

int ao_slow = 6 + straight; //6.4881
int ai_slow = 8 + straight; //8.0518


//Declare Servos and Motors
Servo servo1; //servo1
Servo servo2; //servo2
Servo motor1; //motor1
Servo motor2; //motor2

void setup() {
  servo1.attach(SERVO1);  // attaches servo1 to its defined pin
  servo2.attach(SERVO2);  // attaches servo2 to its defined pin

  motor1.attach(MOTOR1);  // attaches motor1 to its defined pin
  motor2.attach(MOTOR2);  // attaches motor2 to its defined pin
  
  Serial.begin(115200);
}

void loop() {

 motorcontrol(lslow);
 delay(2000);
 motorcontrol(lfast);
 delay(2000);
 motorcontrol(rslow);
 delay(2000);
 motorcontrol(rfast);
 delay(2000);  
}


void backup(){
  servo1.write(140);
  servo2.write(140);
  motor1.writeMicroseconds(1300);
  motor2.writeMicroseconds(1300);
  Serial.println("Reversed ");
}
void spin(){
  servo1.write(130);
  servo2.write(130);
  motor1.writeMicroseconds(1300);
  motor2.writeMicroseconds(1300);
}

void forward(){
  servo1.write(128);  //140   <-- less turn
  servo2.write(128);  //140
  motor1.writeMicroseconds(1600);  //1600    <--  half speed
  motor2.writeMicroseconds(1600);  //1700    <--  Full speed
  Serial.println("Forward ");
}


void motorstop(){
  servo1.write(128);  //140   <-- less turn
  servo2.write(128);  //140
  motor1.writeMicroseconds(1500);  //1600    <--  half speed //1500 <-- no speed
  motor2.writeMicroseconds(1500);  //1700    <--  Full speed
  Serial.println("Forward ");
}


void turnleftfast(){
  //motor1.writeMicroseconds(mspeedturn);
  //motor2.writeMicroseconds(mspeedturn);
  for (POS = straight; POS >= ao_fast; POS -= 1) { 
    // in steps of 1 degree
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to Position in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  for (POS = ao_fast; POS <= straight; POS += 1) { 
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to POSition in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  Serial.println("Turning Left fastly ");
  }
void turnleftslow(){
   //motor1.writeMicroseconds(mspeedturn);
  //motor2.writeMicroseconds(mspeedturn);
  for (POS = straight; POS >= ao_slow ; POS -= 1) { 
    // in steps of 1 degree
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to Position in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  for (POS = ao_slow ; POS <= straight; POS += 1) { 
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to POSition in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  Serial.println("Turning Left slowly ");

  return; 
}




void turnrightfast(){
  //motor1.writeMicroseconds(mspeedturn);  //1700
  //motor2.writeMicroseconds(mspeedturn);  //1700
  for (POS = straight; POS <= ao_fast; POS += 1) { 
    // in steps of 1 degree
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to POSition in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  for (POS = ao_fast; POS >= straight; POS -= 1) { 
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to POSition in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  Serial.println("Turning Right fastly ");
  }
  
void turnrightslow(){ 
   //motor1.writeMicroseconds(mspeedturn);  //1700
  //motor2.writeMicroseconds(mspeedturn);  //1700
  for (POS = straight; POS <= ao_slow; POS += 1) { 
    // in steps of 1 degree
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to POSition in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  for (POS = ao_slow; POS >= straight; POS -= 1) { 
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to POSition in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  Serial.println("Turning Right slowly ");
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
    case(lslow):
    turnleftslow();
    break;
    case(lfast):
    turnleftfast();
    break;
    case(rslow):
    turnrightslow();
    break;
    case(rfast):
    turnrightfast();
    break;
    default:
    motorstop();
    break;
  }
  return;
}


void turningalg(int ao_fast, int ai_fast){
// loop 1
int currentao = straight;
int currentai = straight;
for ( POS = currentao; POS >= (1 *( ao_fast/ 8) + straight); POS -= 1) {
  servo1.write(POS);
  delay(30);
  currentao = (1 *( ao_fast/ 8) + straight);
}
for ( POS = currentai; POS >= (1*(ai_fast/ 8) + straight); POS -= 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (1*(ai_fast/ 8) + straight);
}

//loop 2 turn fwd
for ( POS = currentao; POS >= (2 *( ao_fast/ 8) + straight); POS -= 1) {
  servo1.write(POS);
  delay(30);
  currentao = (2 *( ao_fast/ 8) + straight);
}
for ( POS = currentai; POS >= (2 *(ai_fast/ 8) + straight); POS -= 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (2 *(ai_fast/ 8) + straight);
}

//loop 3 turn fwd
for ( POS = currentao; POS >= (3 *( ao_fast/ 8) + straight); POS -= 1) {
  servo1.write(POS);
  delay(30);
  currentao = (3 *( ao_fast/ 8) + straight);
}
for ( POS = currentai; POS >= (3 *(ai_fast/ 8) + straight); POS -= 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (3 *(ai_fast/ 8) + straight);
}

//loop 4 turn fwd
for ( POS = currentao; POS >= (4 *( ao_fast/ 8) + straight); POS -= 1) {
  servo1.write(POS);
  delay(30);
  currentao = (4 *( ao_fast/ 8) + straight);
}
for ( POS = currentai; POS >= (4 *(ai_fast/ 8) + straight); POS -= 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (4 *(ai_fast/ 8) + straight);
}

//loop 5 turn fwd
for ( POS = currentao; POS >= (5 *( ao_fast/ 8) + straight); POS -= 1) {
  servo1.write(POS);
  delay(30);
  currentao = (5 *( ao_fast/ 8) + straight);
}
for ( POS = currentai; POS >= (5 *(ai_fast/ 8) + straight); POS -= 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (5 *(ai_fast/ 8) + straight);
}

//loop 6 turn fwd
for ( POS = currentao; POS >= (6 *( ao_fast/ 8) + straight); POS -= 1) {
  servo1.write(POS);
  delay(30);
  currentao = (6 *( ao_fast/ 8) + straight);
}
for ( POS = currentai; POS >= (6 *(ai_fast/ 8) + straight); POS -= 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (6 *(ai_fast/ 8) + straight);
}

//loop 7 turn fwd
for ( POS = currentao; POS >= (7 *( ao_fast/ 8) + straight); POS -= 1) {
  servo1.write(POS);
  delay(30);
  currentao = (7 *( ao_fast/ 8) + straight);
}
for ( POS = currentai; POS >= (7 *(ai_fast/ 8) + straight); POS -= 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (7 *(ai_fast/ 8) + straight);
}

//loop 8 turn fwd
for ( POS = currentao; POS >= (8 *( ao_fast/ 8) + straight); POS -= 1) {
  servo1.write(POS);
  delay(30);
  currentao = (8 *( ao_fast/ 8) + straight);
}
for ( POS = currentai; POS >= (8 *(ai_fast/ 8) + straight); POS -= 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (8 *(ai_fast/ 8) + straight);
}

delay(SLIGHT_TURN);

//Write turning back part..  here.  
for ( POS = currentao; POS <= (straight - (1 *( ao_fast/ 8))); POS += 1) {
  servo1.write(POS);
  delay(30);
  currentao = (straight - (1 *( ao_fast/ 8)));
}
for ( POS = currentai; POS <= (straight - (1 *( ai_fast/ 8))); POS += 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (straight - (1 *( ai_fast/ 8)));
}

//loop 2 turn back
for ( POS = currentao; POS <= (straight - (2 *( ao_fast/ 8))); POS += 1) {
  servo1.write(POS);
  delay(30);
  currentao = (straight - (2 *( ao_fast/ 8)));
}
for ( POS = currentai; POS <= (straight - (2 *( ai_fast/ 8))); POS += 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (straight - (2 *( ai_fast/ 8)));
}

//loop 3 turn back
for ( POS = currentao; POS <= (straight - (3 *( ao_fast/ 8))); POS += 1) {
  servo1.write(POS);
  delay(30);
  currentao = (straight - (3 *( ao_fast/ 8)));
}
for ( POS = currentai; POS <= (straight - (3 *( ai_fast/ 8))); POS += 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (straight - (3 *( ai_fast/ 8)));
}

//loop 4 turn back
for ( POS = currentao; POS <= (straight - (4 *( ao_fast/ 8))); POS += 1) {
  servo1.write(POS);
  delay(30);
  currentao = (straight - (4 *( ao_fast/ 8)));
}
for ( POS = currentai; POS <= (straight - (4 *( ai_fast/ 8))); POS += 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (straight - (4 *( ai_fast/ 8)));
}

//loop 5 turn back
for ( POS = currentao; POS <= (straight - (5 *( ao_fast/ 8))); POS += 1) {
  servo1.write(POS);
  delay(30);
  currentao = (straight - (5 *( ao_fast/ 8)));
}
for ( POS = currentai; POS <= (straight - (5 *( ai_fast/ 8))); POS += 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (straight - (5 *( ai_fast/ 8)));
}

//loop 6 turn back
for ( POS = currentao; POS <= (straight - (6 *( ao_fast/ 8))); POS += 1) {
  servo1.write(POS);
  delay(30);
  currentao = (straight - (6 *( ao_fast/ 8)));
}
for ( POS = currentai; POS <= (straight - (6 *( ai_fast/ 8))); POS += 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (straight - (6 *( ai_fast/ 8)));
}

//loop 7 turn back
for ( POS = currentao; POS <= (straight - (7 *( ao_fast/ 8))); POS += 1) {
  servo1.write(POS);
  delay(30);
  currentao = (straight - (7 *( ao_fast/ 8)));
}
for ( POS = currentai; POS <= (straight - (7 *( ai_fast/ 8))); POS += 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (straight - (7 *( ai_fast/ 8)));
}

//loop 8 turn back
for ( POS = currentao; POS <= (straight - (8 *( ao_fast/ 8))); POS += 1) {
  servo1.write(POS);
  delay(30);
  currentao = (straight - (8 *( ao_fast/ 8)));
}
for ( POS = currentai; POS <= (straight - (8 *( ai_fast/ 8))); POS += 1) { 
  servo2.write(POS);
  delay(30);
  currentai = (straight - (8 *( ai_fast/ 8)));
}

delay(SLIGHT_TURN);

return; 
}
