#include <NewPing.h> // This is the library to read the ultrasounds 
#include<Servo.h> // Controls the servos as well as the motors

//This your definitions for the ultrasounds
#define SONAR_NUM     3 // Number or sensors.
#define MAX_DISTANCE 200 // Max distance in cm.
//#define PING_INTERVAL 33 // Milliseconds between pings.

//This your def for the motors 
#define SERVO1 2  //pin number for servo1
#define SERVO2 3  //pin number for servo2
#define MOTOR1 4  //pin number for motor1
#define MOTOR2 5  //pin number for motor2
int NeedtoTurn = 40;

//Extra Servos def
#define SLIGHT_TURN 500 //number of milliseconds to hold turn when turning
#define FULL_TURN 0 // number of milliseconds to turn completely around
int POS = 0;
 
unsigned long pingTimer[SONAR_NUM]; // When each pings.
unsigned int cm[SONAR_NUM]; // Store ping distances.
uint8_t currentSensor = 0; // Which sensor is active.
 
NewPing sonar[SONAR_NUM] = { // Sensor object array.
  NewPing(42, 43, MAX_DISTANCE), NewPing(40, 41, MAX_DISTANCE), NewPing(44, 45, MAX_DISTANCE)
};


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
  /*
  pingTimer[0] = millis() + 75; // First ping start in ms.
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
   */
}
 
void loop() {
  /*
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1)
        oneSensorCycle(); // Do something with results.
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
  // The rest of your code would go here.
}
 */
 /*
void echoCheck() { // If ping echo, set distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}  */
 
//void oneSensorCycle() { 
  for (uint8_t i = 0; i < SONAR_NUM; i++) { //(uint8_t i = 0; i < SONAR_NUM; i++)
    delay(50);
    Serial.print(i);
    Serial.print("=");
    Serial.print(sonar[i].ping_cm()); //     Serial.print(cm[i]);
    Serial.print("cm ");

      //Do something with the results.
      if ( sonar[i].ping_cm() < NeedtoTurn && sonar[i].ping_cm() != 0)  // ( cm[2] < NeedtoTurn && cm[2] != 0)
      {
          //Turn rights at 40cm ahead of it.
          turnright();
      }
      else
      {
          //Go forward.
          forward();
      }
    //Might need a small delay
    //delay(100);
    }
  Serial.println();
  }
 

void drive(int distance, float turn) {
  int fullSpeed = 1700;
  int stopSpeed = 1500;
  int reverseSpeed = 1300;
  float t = turn;

  while (t < -180) t += 180;
  while (t >  180) t -= 180;
  // drive to location
  int s = fullSpeed;
  if ( distance < 8 ) {
    int wouldBeSpeed = s - stopSpeed;
    wouldBeSpeed *= distance / 8.0f;
    if (turn > 180 || turn < -180) {
      wouldBeSpeed = -wouldBeSpeed;
    }
    s = stopSpeed + wouldBeSpeed;
  }
  
  int autoThrottle = constrain(s, reverseSpeed, fullSpeed);
  
  Serial.print("Turn: ");
  Serial.println(t);
  Serial.print("Original: ");
  Serial.println(turn);

  servo1.write(t);
  servo2.write(t);
  motor1.writeMicroseconds(autoThrottle);
  motor2.writeMicroseconds(autoThrottle);
  delay(SLIGHT_TURN);
  Serial.println("Following Person");
}

void turnleft(){
  motor1.writeMicroseconds(1700);
  motor2.writeMicroseconds(1700);
  for (POS = 128; POS >= 94; POS -= 1) { // goes from 122 degrees to 94 degrees
    // in steps of 1 degree
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to Position in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  for (POS = 94; POS <= 128; POS += 1) { // goes from 94 degrees to 122 degrees
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to POSition in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  Serial.println("Turning Left ");
}
void turnright(){
  motor1.writeMicroseconds(1600);  //1700
  motor2.writeMicroseconds(1600);  //1700
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
