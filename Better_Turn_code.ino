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

*/
#include <Servo.h> 
#include <math.h>

#define SERVO1 2  //pin number for servo1
#define SERVO2 3  //pin number for servo2

#define SLIGHT_TURN 500 //number of milliseconds to hold turn when turning
#define FULL_TURN 0 // number of milliseconds to turn completely around

int POS = 0;

//Declaring the varible to setup the Servo1 and Servo2 calculations
int R = 0;
double ai;
double ao;
//This is the code to tell the servo where to move and by how much.
//ai = atan(29/(R - 25));
//ao = atan(29/(R + 25));

//Declare Servos and Motors
Servo servo1; //servo1
Servo servo2; //servo2

void setup() {
  servo1.attach(SERVO1);  // attaches servo1 to its defined pin
  servo2.attach(SERVO2);  // attaches servo2 to its defined pin
}

void loop() {
  turnleft();
  turnright();
}




void turnleft(){
  //motor1.writeMicroseconds(1700);
  //motor2.writeMicroseconds(1700);
  for (POS = 128; POS >= 150; POS -= 1) { // goes from 122 degrees to 94 degrees
    // in steps of 1 degree
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to Position in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  for (POS = 150; POS <= 128; POS += 1) { // goes from 94 degrees to 122 degrees
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to POSition in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  Serial.println("Turning Left ");
}
void turnright(){
  //motor1.writeMicroseconds(1600);  //1700
  //motor2.writeMicroseconds(1600);  //1700
  for (POS = 128; POS <= 150; POS += 1) { // goes from 122 degrees to 150 degrees
    // in steps of 1 degree
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to POSition in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  for (POS = 150; POS >= 128; POS -= 1) { // goes from 150 degrees to 122 degrees
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to POSition in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  Serial.println("Turning Right ");
}
