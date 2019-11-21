// Blynk Auth
char auth[] = "vsBftYjEwxJMcEknPkNPBAvs1uIQB_i2";

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true

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
#define SERVO1 2  //pin number for servo1
#define SERVO2 3  //pin number for servo2
#define MOTOR1 4  //pin number for motor1
#define MOTOR2 5  //pin number for motor2

//If using software serial uncomment below and pick pins:
//#define GPS_TX_PIN 6
//#define GPS_RX_PIN 7

//Declare Sensor Limits
#define FRONT_SENSOR 60// starts to turn
#define SIDE_SENSOR 35// starts to turn

#define FRONT_SENSOR_REVERSE 50  //Reverses when this close to the front sensor
#define SIDE_SENSOR_REVERSE 20 //Reverses when this close to a side sensor

#define BLUETOOTH_TX_PIN 10
#define BLUETOOTH_RX_PIN 9

// If one motor tends to spin faster than the other, add offset
#define MOTOR_A_OFFSET 0
#define MOTOR_B_OFFSET 0

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
  float lat;
  float lon;
};
