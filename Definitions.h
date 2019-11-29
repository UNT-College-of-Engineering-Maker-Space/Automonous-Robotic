// Blynk Auth
char auth[] = "vsBftYjEwxJMcEknPkNPBAvs1uIQB_i2"; //jorge phone
//char auth[] = "ZltTLKOWzXGhahEvGLABvAwOoIbTJLHT"; //Alex's phone
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

//Declare Sensors
#define TRIGGER1 40 //front
#define TRIGGER2 42 //left
#define TRIGGER3 44 //right
#define ECHO1 41 //front
#define ECHO2 43 //left
#define ECHO3 45 //right
#define SONAR_NUM     3 // Number or ultra sonic sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 50 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

//Declare Servos
#define SLIGHT_TURN 500 //number of milliseconds to hold turn when turning
#define FULL_TURN 0 // number of milliseconds to turn completely around
#define POS_LEFT 85  //position of the left servo to be at 90 degrees
#define POS_RIGHT 70  //position of the right servo to be at 90 degrees
#define SERVO1 3  //pin number for right servo
#define SERVO2 2  //pin number for left servo
#define MOTOR1 4  //pin number for left servo
#define MOTOR2 5  //pin number for right servo

//Remove RX and TX pins when uploading, otherwise the GPS module will stop working overtime.
//If using software serial uncomment below and pick pins:
//#define GPS_TX_PIN 6
//#define GPS_RX_PIN 7

//Declare Sensor Limits In Centimeters
#define FRONT_SENSOR 70 // starts to turn
#define SIDE_SENSOR 35 // starts to turn

#define FRONT_SENSOR_REVERSE 50  //Reverses when this close to the front sensor
#define SIDE_SENSOR_REVERSE 20 //Reverses when this close to a side sensor

// Removex RX and TX pins when uploading, otherwise the bluetooth module will stop working overtime
#define BLUETOOTH_TX_PIN 10
#define BLUETOOTH_RX_PIN 11

// You must then add your 'Declination Angle' to the compass, which is the 'Error' of the magnetic field in your location.
// Can be found using this website: http://www.magnetic-declination.com/
// Must be in radians so convert from degrees to radians
#define DECLINATION_ANGLE 0.05f

// Number of changes in movement to timeout for GPS streaming
// Keeps the cooler from driving away if there is a problem
#define GPS_STREAM_TIMEOUT 18

// Number of changes in movement to timeout for GPS waypoints
// Keeps the cooler from driving away if there is a problem
#define GPS_WAYPOINT_TIMEOUT 45

// The offset of the mounting position to true north
// It would be best to run the /examples/magsensor sketch and compare to the compass on your smartphone
#define COMPASS_OFFSET 0.0f

struct GeoLoc {
  float Lat;
  float Lon;
};
