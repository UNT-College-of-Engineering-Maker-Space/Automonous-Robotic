#define BLYNK_USE_DIRECT_CONNECT
// Imports
#include <Adafruit_GPS.h> // Works for our current GPS module
#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h> // To communicate with Blynk app
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303DLH_Mag.h> // Specific Magnetometer library for Magnetometer in use
#include <Servo.h> // Controls the servos as well as the motors
#include <NewPing.h> // Used for the ultrasonic sensors
#include "./Definitions.h" // Includes all Definitions 

WidgetTerminal terminal(V1); // Allows us to print onto the Blynk app terminal

// Start GPS Code
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3
Adafruit_GPS GPS(&Serial3); //pins 15(RX) and 14(TX)
//If using software serial, comment above and uncomment below:
//SoftwareSerial mySerial(GPS_TX_PIN, GPS_RX_PIN);
//Adafruit_GPS GPS(&mySerial);

// Sets the RX and TX pins that the Bluetooth module will be using
SoftwareSerial bluetoothSerial(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);

// This keeps track of whether we're using the interrupt
// Off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean);
// end gps code

int POS = 90;
int joystick_x = 0;
int joystick_y = 0;

//start compass code
int H2Degrees;
// Assign a unique ID to this sensor at the same time
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
  float y = sin(b.Lon-a.Lon) * cos(b.Lat);
  float x = cos(a.Lat)*sin(b.Lat) - sin(a.Lat)*cos(b.Lat)*cos(b.Lon-a.Lon);
  return atan2(y, x) * RADTODEG;
}

float geoDistance(struct GeoLoc &a, struct GeoLoc &b) {
  const float R = 6371000; // km
  float p1 = a.Lat * DEGTORAD;
  float p2 = b.Lat * DEGTORAD;
  float dp = (b.Lat-a.Lat) * DEGTORAD;
  float dl = (b.Lon-a.Lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}
// Where the ping distances are stored.
unsigned int cm[SONAR_NUM]; // Store ping distances.


NewPing sonar[SONAR_NUM] = { // Sensor object array.
  NewPing(TRIGGER1, ECHO1, MAX_DISTANCE),
  NewPing(TRIGGER2, ECHO2, MAX_DISTANCE),
  NewPing(TRIGGER3, ECHO3, MAX_DISTANCE)
};

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
  // Open serial communications
  Serial.begin(115200);
  GPS.begin(9600);
  // Open Bluetooth communications
  bluetoothSerial.begin(9600);
  // Connect to the Blynk App
  Blynk.begin(bluetoothSerial, auth);
  // Clears off all data that was previously showing on the Blynk app terminal
  terminal.clear();
  
  servo1.attach(SERVO1);  // attaches servo1 to its defined pin
  servo2.attach(SERVO2);  // attaches servo2 to its defined pin

  motor1.attach(MOTOR1);  // attaches motor1 to its defined pin
  motor2.attach(MOTOR2);  // attaches motor2 to its defined pin

  //start compass code
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

  // end compass code

  // start gps setup code
  Serial.println("Adafruit GPS library basic test!");
  
  // Uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  // Request updates on antenna status, comment out if you are not using an antenna
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
    if (!usingInterrupt) {
      // read data from the GPS in the 'main loop'
      char c = GPS.read();
      // if you want to debug, this is a good time to do it!
      if (GPSECHO)
        if (c) Serial.print(c);
    }
  
    // if a sentence is received, we can parse it
    if (GPS.newNMEAreceived()) {
      // a tricky thing here is if we print the NMEA sentence, or data
      // we end up not listening and catching other sentences! 
      // so be very wary if using OUTPUT_ALLDATA and trying to print out data
      //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }

    // if millis() or timer wraps around, we'll just reset it
    // Uncomment below if you want to see all of the GPS data being retrieved
/*    if (timer > millis())  timer = millis();

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
        Serial.print("Location (DDMM.MMMM Format): ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        Serial.print(", "); 
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      }
    }*/
  }
  
  //start compass code
float Heading() {
    // Get a new sensor event
    sensors_event_t event; 
    mag.getEvent(&event);
 
    // Display the results (magnetic vector values are in micro-Tesla (uT))
    Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
    terminal.print("X: "); terminal.print(event.magnetic.x); terminal.print("  ");
    terminal.print("Y: "); terminal.print(event.magnetic.y); terminal.print("  ");
    terminal.print("Z: "); terminal.print(event.magnetic.z); terminal.print("  ");terminal.println("uT");
    terminal.flush();
    
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
}
  // end compass code
  
void Sonar()
{
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
    delay(PING_INTERVAL); // Currently set to wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    if (sonar[i].ping_cm() == 0) {
      cm[i] = MAX_DISTANCE;
    }
    else {
      cm[i] = sonar[i].ping_cm();
    }
  }

  Serial.print("Front sensor distance is ");
  Serial.print(cm[0]);
  Serial.println(" cm");

  Serial.print("Left sensor distance is ");
  Serial.print(cm[1]);
  Serial.println(" cm");

  Serial.print("Right sensor distance is ");
  Serial.print(cm[2]);
  Serial.println(" cm");

  terminal.print("Front sensor distance is ");
  terminal.print(cm[0]);
  terminal.println(" cm");

  terminal.print("Left sensor distance is ");
  terminal.print(cm[1]);
  terminal.println(" cm");

  terminal.print("Right sensor distance is ");
  terminal.print(cm[2]);
  terminal.println(" cm");

  terminal.flush();
}

//Driving the servo motors

void drive(int distance, float turn) {
  int fullSpeed = 1700;
  int stopSpeed = 1500;
  int reverseSpeed = 1300;
  float t = turn;
  float left = 0.00;
  float right = 0.00;
  while (t < -180) t += 180;
  while (t >  180) t -= 180;
  // drive to location
  int s = fullSpeed;
  if (distance < 8) {
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

  terminal.print("Turn: ");
  terminal.println(t);
  terminal.print("Original: ");
  terminal.println(turn);
  if (t < 20) {
    t = 20;
  }
  else if (t > 160) {
   t = 160; 
  }
  left = t - (90 - POS_LEFT);
  right = t - (90 - POS_RIGHT);

  servo1.write(left);
  servo2.write(right);
  motor1.writeMicroseconds(autoThrottle);
  motor2.writeMicroseconds(autoThrottle);
  Serial.println("Following Person");
  terminal.println("Following Person");

  terminal.flush();
}

float degWhole; //Variable for the whole part of position 
float degDec;  //Variable for the decimal part of degree

void go(struct GeoLoc &loc, int timeout) {
  gps1();
  GeoLoc robotLoc;

  robotLoc.Lat = GPS.latitude;
  degWhole = float(int(robotLoc.Lat/100)); //gives me the whole degree part of Longitude
  degDec = (robotLoc.Lat - degWhole*100)/60; //give me fractional part of longitude
  robotLoc.Lat = degWhole + degDec; //If in Southern Hemisphere, latitude should be negative
  if (GPS.lat == 'S') {
    robotLoc.Lat = -robotLoc.Lat;
  }
  
  robotLoc.Lon = GPS.longitude;
  degWhole = float(int(robotLoc.Lon/100)); //gives me the whole degree part of Longitude
  degDec = (robotLoc.Lon - degWhole*100)/60; //give me fractional part of longitude
  robotLoc.Lon = degWhole + degDec; //If in Western Hemisphere, longitude should be negative
  
  if (GPS.lon == 'W') {
    robotLoc.Lon = -robotLoc.Lon;
  }
  
  Serial.println("Reading onboard GPS: ");
  Serial.print(robotLoc.Lat, 7); Serial.print(", "); Serial.println(robotLoc.Lon, 7);

  terminal.println("Reading onboard GPS: ");
  terminal.print(robotLoc.Lat); terminal.print(", "); terminal.println(robotLoc.Lon);
    
  bluetoothSerial.listen();

  if (robotLoc.Lat != 0 && robotLoc.Lon != 0) {
    float d = 0;
    //Start move loop here
      gps1();
      robotLoc.Lat = GPS.latitude;
      degWhole = float(int(robotLoc.Lat/100)); // Gives the whole degrees part of latitude
      degDec = (robotLoc.Lat - degWhole*100)/60; // Gives the fractional part of latitudde
      robotLoc.Lat = degWhole + degDec; // Gives complete correct decimal form of latitude degrees
      if (GPS.lat == 'S') { // If in Southern Hemisphere, latitude should be negative
        robotLoc.Lat = -robotLoc.Lat;
      }
  
      robotLoc.Lon = GPS.longitude;
      degWhole = float(int(robotLoc.Lon/100)); // Gives the whole degree part of longitude
      degDec = (robotLoc.Lon - degWhole*100)/60; // Give the fractional part of longitude
      robotLoc.Lon = degWhole + degDec; // Gives complete correct decimal form of longitude degrees
  
      if (GPS.lon == 'W') { // If in Western Hemisphere, longitude should be negative
        robotLoc.Lon = -robotLoc.Lon;
      }
  
      bluetoothSerial.listen();
      
      d = geoDistance(robotLoc, loc);
      float t = geoBearing(robotLoc, loc) - Heading();
      
      Serial.print("Distance: ");
      Serial.println(d);
    
      Serial.print("Bearing: ");
      Serial.println(geoBearing(robotLoc, loc));

      Serial.print("Heading: ");
      Serial.println(Heading());

      terminal.print("Distance: ");
      terminal.println(d);
    
      terminal.print("Bearing: ");
      terminal.println(geoBearing(robotLoc, loc));

      terminal.print("Heading: ");
      terminal.println(Heading());
      
      drive(d, t);
      timeout -= 1;
  }
  terminal.flush();
}

void turnleft(){
  motor1.writeMicroseconds(1600);
  motor2.writeMicroseconds(1600);
  
  servo1.write(POS_LEFT - 40);
  servo2.write(POS_RIGHT - 40);
  
  /*for (POS = 128; POS >= 94; POS -= 1) { // goes from 128 degrees to 94 degrees
    // in steps of 1 degree
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to Position in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  for (POS = 94; POS <= 128; POS += 1) { // goes from 94 degrees to 128 degrees
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to POSition in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }*/
  //delay(SLIGHT_TURN);
  Serial.println("Turning Left ");
  terminal.println("Turning Left ");
  terminal.flush();
}
void turnright(){
  motor1.writeMicroseconds(1600);
  motor2.writeMicroseconds(1600);
  servo1.write(POS_LEFT + 40);
  servo2.write(POS_RIGHT + 40);
  /*for (POS = 128; POS <= 150; POS += 1) { // goes from 128 degrees to 150 degrees
    // in steps of 1 degree
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to POSition in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }
  delay(SLIGHT_TURN);
  for (POS = 150; POS >= 128; POS -= 1) { // goes from 150 degrees to 128 degrees
    servo1.write(POS);
    servo2.write(POS);// tell servo to go to POSition in variable 'POS'
    delay(30);                       // waits 15ms for the servo to reach the POSition
  }*/
  //delay(SLIGHT_TURN);
  Serial.println("Turning Right ");
  terminal.println("Turning Right ");
  terminal.flush();
}
void backup(){
  servo1.write(POS_LEFT);
  servo2.write(POS_RIGHT);
  motor1.writeMicroseconds(1300);
  motor2.writeMicroseconds(1300);
  Serial.println("Reversed ");
  terminal.println("Reversed ");
  terminal.flush();
}
void spin(){
  servo1.write(POS_LEFT + 50);
  servo2.write(POS_RIGHT + 50);
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
    char turndirection = scan(); // Goes to scan function to decide which way to turn based on l,r,s,b values.
    switch (turndirection){
    case 'l':
      turnleft();
      //delay(SLIGHT_TURN);
      break; // exits the case
    case 'r':
      turnright();
      //delay(SLIGHT_TURN);
      break;
    case 'b':
      backup();
      //delay(SLIGHT_TURN);
      break;
    case 's':
      spin();
      break;
      //delay(SLIGHT_TURN);
    }
  }
  else if ((joystick_x==90)&&(joystick_y==1500)){
  }
  else
  {
    GpsParam gps(param);
  
    Serial.println("Received remote GPS: ");
    // Print 7 decimal places for Lat and Lon
    Serial.print(gps.getLat(), 7); Serial.print(", "); Serial.println(gps.getLon(), 7);

    terminal.println("Received remote GPS: ");
    // Print 7 decimal places for Lat and Lon
    terminal.print(gps.getLat()); terminal.print(", "); terminal.println(gps.getLon());
    
    terminal.flush();
    
    GeoLoc phoneLoc;
    phoneLoc.Lat = gps.getLat();
    phoneLoc.Lon = gps.getLon();

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
    
      float Lat = latStr.toFloat();
      float Lon = lonStr.toFloat();
    
      if (Lat != 0 && Lon != 0) {
        GeoLoc waypoint;
        waypoint.Lat = Lat;
        waypoint.Lon = Lon;
    
        Serial.print("Waypoint found: "); Serial.print(Lat); Serial.println(Lon);
        go(waypoint, GPS_WAYPOINT_TIMEOUT);
      }
    }
    
    rawInput = rawInput.substring(colonIndex + 1);
    
  } while (colonIndex != -1);
}

BLYNK_WRITE(V2) {
  int left = 90;
  int right = 90;
  int Speed = 1500;
  joystick_x = param[0].asInt();
  joystick_y = param[1].asInt();
  int joystick_y1;

  if ((joystick_x!=90)&&(joystick_y!=1500))
  {
    left = joystick_x - (90-POS_LEFT);
    right = joystick_x - (90-POS_RIGHT);
    servo1.write(left);
    servo2.write(right);
    if
    motor1.writeMicroseconds(joystick_y);
    motor2.writeMicroseconds(joystick_y);
    // Printing x and y values helps for debugging of the joystick
    Serial.print("X = ");
    Serial.print(joystick_x);
    Serial.print("; Y = ");
    Serial.println(joystick_y);
    joystick_x = param[0].asInt();
    joystick_y = param[1].asInt();
  }
}

void loop()
{
  Blynk.run();
}
