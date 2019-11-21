#define BLYNK_USE_DIRECT_CONNECT
// Imports
#include <Adafruit_GPS.h> // Works for our current GPS module
#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h> // To communicate with Blynk app
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303DLH_Mag.h> // Specific Magnetometer library for Magnetometer in use
#include<Servo.h> // Controls the servos as well as the motors
#include "./Definitions.h" // Includes all Definitions 

BlynkTimer timers;

// Start GPS Code
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3
Adafruit_GPS GPS(&Serial3); //pins 15(RX) and 14(TX)
//If using software serial, comment above and uncomment below:
//SoftwareSerial mySerial(GPS_TX_PIN, GPS_RX_PIN);
//Adafruit_GPS GPS(&mySerial);

SoftwareSerial bluetoothSerial(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean);
// end gps code
int POS = 0;
//start compass code
int H2Degrees;
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
// end compass code

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

float geoBearing(struct GeoLoc &a, struct GeoLoc &b) {
  float y = sin(b.lon-a.lon) * cos(b.lat);
  float x = cos(a.lat)*sin(b.lat) - sin(a.lat)*cos(b.lat)*cos(b.lon-a.lon);
  return atan2(y, x) * RADTODEG;
}

float geoDistance(struct GeoLoc &a, struct GeoLoc &b) {
  const float R = 6371000; // km
  float p1 = a.lat * DEGTORAD;
  float p2 = b.lat * DEGTORAD;
  float dp = (b.lat-a.lat) * DEGTORAD;
  float dl = (b.lon-a.lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}
// Where the ping distances are stored.
unsigned int cm[SONAR_NUM];
unsigned int duration; 

//Declare Servos and Motors
Servo servo1; //servo1
Servo servo2; //servo2
Servo motor1; //motor1
Servo motor2; //motor2


void setup() {
  pinMode(TRIGGER1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIGGER2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIGGER3, OUTPUT);
  pinMode(ECHO3, INPUT);
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  GPS.begin(9600);
  //Bluetooth 
  bluetoothSerial.begin(9600);
  Blynk.begin(bluetoothSerial, auth);
  // Ping distance routine
  //PingTimer = timer.setInterval(250L, Sonar);

  servo1.attach(SERVO1);  // attaches servo1 to its defined pin
  servo2.attach(SERVO2);  // attaches servo2 to its defined pin

  motor1.attach(MOTOR1);  // attaches motor1 to its defined pin
  motor2.attach(MOTOR2);  // attaches motor2 to its defined pin

  delay(1000); // delay one second
  //start compass code
  {
    Serial.println("LSM303DLHC Magnetometer Test"); Serial.println("");
  
    /* Initialise the sensor */
    if(!mag.begin())
    {
      /* There was a problem detecting the LSM303DLHC ... check your connections */
      Serial.println("Ooops, no LSM303DLHC detected ... Check your wiring!");
      while(1);
    }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  }

  // end compass code

  // start gps setup code
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit GPS's - some use 4800
  GPS.begin(9600);
  
  // Uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  // mySerial.println(PMTK_Q_RELEASE);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

void gps1() {
    if (! usingInterrupt) {
      // read data from the GPS in the 'main loop'
      char c = GPS.read();
      // if you want to debug, this is a good time to do it!
      if (GPSECHO)
        if (c) Serial.print(c);
    }
  
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      // a tricky thing here is if we print the NMEA sentence, or data
      // we end up not listening and catching other sentences! 
      // so be very wary if using OUTPUT_ALLDATA and trying to print out data
      //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }

    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis())  timer = millis();

    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > 2000) { 
      timer = millis(); // reset the timer
    
      Serial.print("\nTime: ");
      Serial.print(GPS.hour, DEC); Serial.print(':');
      Serial.print(GPS.minute, DEC); Serial.print(':');
      Serial.print(GPS.seconds, DEC); Serial.print('.');
      Serial.println(GPS.milliseconds);
      Serial.print("Date: ");
      Serial.print(GPS.day, DEC); Serial.print('/');
      Serial.print(GPS.month, DEC); Serial.print("/20");
      Serial.println(GPS.year, DEC);
      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
      if (GPS.fix) {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        Serial.print(", "); 
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
          
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      }
    }
  }
  
  //start compass code
float Heading() {
    /* Get a new sensor event */ 
    sensors_event_t event; 
    mag.getEvent(&event);
 
    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
    delay(500);
    
    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(event.magnetic.y, event.magnetic.x);
  
    heading += DECLINATION_ANGLE;
  
    // Correct for when signs are reversed.
    if(heading < 0)
      heading += 2*PI;
    
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
      heading -= 2*PI;
   
    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/M_PI; 
  
    H2Degrees = headingDegrees;
    return H2Degrees;
    //delay(500);
  }
  // end compass code
void Sonar()
{
    // Pulse 1st Ultrasonic Sensor
    digitalWrite(TRIGGER1, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER1, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER1, LOW);

    // End Pulse & Calculate distance
    duration = pulseIn(ECHO1, HIGH);
    cm[0] = (duration / 2) / 29.1;

    // Pulse 2nd Ultrasonic Sensor
    digitalWrite(TRIGGER2, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER2, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER2, LOW);

    // End Pulse & Calculate distance
    duration = pulseIn(ECHO2, HIGH);
    cm[1] = (duration / 2) / 29.1;

    // Pulse 3rd Ultrasonic Sensor
    digitalWrite(TRIGGER3, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER3, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER3, LOW);

    // End Pulse & Calculate distance
    duration = pulseIn(ECHO3, HIGH);
    cm[2] = (duration / 2) / 29.1;

    Serial.print("Front sensor distance is  ");
    Serial.print(cm[0]);
    Serial.println(" cm");

    Serial.print("Left sensor distance is ");
    Serial.print(cm[1]);
    Serial.println(" cm");

    Serial.print("Right sensor distance is ");
    Serial.print(cm[2]);
    Serial.println(" cm");
}

int SonarTimer;

/*void turning()
{
  Sonar();
  while(cm[0]<FRONT_SENSOR || cm[1]<SIDE_SENSOR || cm[2]<SIDE_SENSOR) 
  { 
    char turndirection = scan(); // Goes to scan function to decide which way to turn based on l,r,s,b values.
    switch (turndirection){
    case 'l':
      turnleft();
      delay(SLIGHT_TURN);
      break; // exits the case
    case 'r':
      turnright();
      delay(SLIGHT_TURN);
      break;
    case 'b':
      backup();
      delay(SLIGHT_TURN);
      break;
    case 's':
      spin();
      break;
      delay(SLIGHT_TURN);
    }
    Sonar();
  }
}*/

//Driving the servo motors
/*void go(){
  servo1.write(90); //for now, we don't go anywhere, thus value is 90 for both which is stopped
  servo2.write(90);
  motor1.write(110);
  motor2.write(110);  
  Serial.println("Going forward  ");
}*/

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

void go(struct GeoLoc &loc, int timeout) {
//  turning();
  gps1();
  GeoLoc robotLoc;
  robotLoc.lat = GPS.latitude;
  robotLoc.lon = GPS.longitude;
  Serial.println("Reading onboard GPS: ");
  Serial.print(robotLoc.lat, 4); Serial.print(", "); Serial.println(robotLoc.lon, 4);
    
  bluetoothSerial.listen();

  if (robotLoc.lat != 0 && robotLoc.lon != 0) {
    float d = 0;
    //Start move loop here
    do {
      gps1();
      robotLoc.lat = GPS.latitude;
      robotLoc.lon = GPS.longitude;
      bluetoothSerial.listen();
      
      d = geoDistance(robotLoc, loc);
      float t = geoBearing(robotLoc, loc) - Heading();
      
      Serial.print("Distance: ");
      Serial.println(geoDistance(robotLoc, loc));
    
      Serial.print("Bearing: ");
      Serial.println(geoBearing(robotLoc, loc));

      Serial.print("Heading: ");
      Serial.println(Heading());
      
      drive(d, t);
      timeout -= 1;
    } 
    while (d > 3.0 && timeout > 0);
      servo1.write(128);
      servo2.write(128);
      motor1.writeMicroseconds(1700);
      motor2.writeMicroseconds(1700);
  }
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
  motor1.writeMicroseconds(1700);
  motor2.writeMicroseconds(1700);
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
char scan(){
  char choice;
  if (cm[0]<FRONT_SENSOR_REVERSE && cm[1]>SIDE_SENSOR_REVERSE){
    choice='s';
  }
  else if (cm[0]<FRONT_SENSOR_REVERSE && cm[2]>SIDE_SENSOR_REVERSE){
    choice='s';
  }
  else if (cm[0]<FRONT_SENSOR_REVERSE || cm[1]<SIDE_SENSOR_REVERSE || cm[2]<SIDE_SENSOR_REVERSE){
    choice='b';
  }
  else if (cm[1]>cm[2]){
    choice='l';
  }
  else if (cm[2]>cm[1]){
    choice='r';
  }
  else{
    choice='b';
  }
  Serial.print("Choice:  ");
  Serial.println(choice);
    return choice;
}


// GPS Streaming Hook
BLYNK_WRITE(V0) {
  Sonar();
  if (cm[0]<FRONT_SENSOR || cm[1]<SIDE_SENSOR || cm[2]<SIDE_SENSOR)
  {
    //delay(15000);
    //motor1.writeMicroseconds(1700);
    //motor2.writeMicroseconds(1700);
    char turndirection = scan(); // Goes to scan function to decide which way to turn based on l,r,s,b values.
    switch (turndirection){
    case 'l':
      turnleft();
      delay(SLIGHT_TURN);
      break; // exits the case
    case 'r':
      turnright();
      delay(SLIGHT_TURN);
      break;
    case 'b':
      backup();
      delay(SLIGHT_TURN);
      break;
    case 's':
      spin();
      break;
      delay(SLIGHT_TURN);
    }
  }
  else
  {
    GpsParam gps(param);
  
    Serial.println("Received remote GPS: ");
    // Print 4 decimal places for Lat and Lon
    Serial.print(gps.getLat(), 4); Serial.print(", "); Serial.println(gps.getLon(), 4);
    
    GeoLoc phoneLoc;
    phoneLoc.lat = gps.getLat();
    phoneLoc.lon = gps.getLon();

    go(phoneLoc, GPS_STREAM_TIMEOUT);
  }
}

// Terminal Hook
BLYNK_WRITE(V1) {
  Serial.print("Received Text: ");
  Serial.println(param.asStr());

  String rawInput(param.asStr());
  int colonIndex;
  int commaIndex;
  
  do {
    commaIndex = rawInput.indexOf(',');
    colonIndex = rawInput.indexOf(':');
    
    if (commaIndex != -1) {
      String latStr = rawInput.substring(0, commaIndex);
      String lonStr = rawInput.substring(commaIndex+1);

      if (colonIndex != -1) {
         lonStr = rawInput.substring(commaIndex+1, colonIndex);
      }
    
      float lat = latStr.toFloat();
      float lon = lonStr.toFloat();
    
      if (lat != 0 && lon != 0) {
        GeoLoc waypoint;
        waypoint.lat = lat;
        waypoint.lon = lon;
    
        Serial.print("Waypoint found: "); Serial.print(lat); Serial.println(lon);
        go(waypoint, GPS_WAYPOINT_TIMEOUT);
      }
    }
    
    rawInput = rawInput.substring(colonIndex + 1);
    
  } while (colonIndex != -1);
}


void loop()
{
  Blynk.run();
  timers.run();
}
