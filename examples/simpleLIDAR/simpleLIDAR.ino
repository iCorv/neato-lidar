#include "LIDAR.h"

// tested with teensy 3.6 board
LIDAR myLIDAR(&Serial2, A9);

void setup() {
  // serial to computer
  Serial.begin(115200);
  // serial to Neato LIDAR Sensor
  Serial2.begin(115200);
  // start motor and measurement
  myLIDAR.startLIDAR();
  // set this to see rpm of motor
  //myLIDAR.printMotorSpeedToSerial(true);
  
  // set this for distance readings, format: "angle" + "\t" + "distance"
  myLIDAR.printSensorDataToSerial(true);

}

void loop() {
  // call as often as possible
  myLIDAR.readLIDAR();

}
