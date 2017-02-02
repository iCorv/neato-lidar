/*
  LIDAR.cpp - Library for reading sensor data from the Neato LIDAR laser range scanner.
  Created by Arne C. Jaedicke, January 18, 2017.
  -------------------------------------------------------------------------------------
  The MIT License

  Copyright (c) 2017 Arne C. Jaedicke

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include "Arduino.h"
#include "LIDAR.h"

/*
   Serial has to provide 115200 baud to work with Neato LIDAR.
*/
LIDAR::LIDAR(HardwareSerial *serial, int motorPin) {
  _serial = serial;
  _motorPin = motorPin;
  _motorSpeed = MOTOR_MIN;
  outputPID = _motorSpeed;
  _printMotorSpeed = false;
  _printSensorData = false;

  _Kp = 0.5;
  _Ki = 0.5;
  _Kd = 0.0;
  _sampleTime = 20;
  _lastInput = 0;
  _intTerm = 0;
  _setPoint = 250;

  for (int i = 0; i < NUM_DISTANCE; i++) {
    dist[i] = 0;   // ein packet enthaelt 4 Distanzmessungen
  }

  inByte = 0;  // incoming serial byte
  byteNum = 0;
  for (int i = 0; i < DATA_LENGTH; i++) {
    data[i] = 0;
  }
  dataIndex = 0;
}

LIDAR::~LIDAR(void) {}

/*
   Starts the motor to start measurements
*/
void LIDAR::startLIDAR() {
  analogWrite(_motorPin, _motorSpeed);
  _lastTime = millis() - _sampleTime;
}

/*
   Prints motor speed readings to serial.
*/
void LIDAR::printMotorSpeedToSerial(bool printMotorSpeed) {
  _printMotorSpeed = printMotorSpeed;
}

/*
   Prints distance readings to serial.
*/
void LIDAR::printSensorDataToSerial(bool printSensorData) {
  _printSensorData = printSensorData;
}

/*
   Reads and interprets the serial data from the Neato Lidar.
   Should be called as often as possible.
*/
void LIDAR::readLIDAR() {
  if (_serial->available() > 0) {
    inByte = _serial->read();
    if (SEARCH_STATE) {
      if (inByte == START) {
        SEARCH_STATE = false;
        data[dataIndex++] = inByte;
      }

    } else {
      data[dataIndex++] = inByte;
      if (dataIndex == DATA_LENGTH) {
        uint16_t angle = data[1] - ANGLE_OFFSET;
        angle = angle * 4;
        processSpeed();
        setMotorSpeed(computeSpeed(_motorSpeed));
        if (_printMotorSpeed) {
          Serial.print("rpm: ");
          Serial.println(_motorSpeed);
        }
        if (_printSensorData) {
          for (int i = 0; i < 4; i++) {
            processDistance(i);
            Serial.print(angle + i);
            Serial.print("\t");
            Serial.println(dist[i]);
          }
        }
        dataIndex = 0;
        SEARCH_STATE = true;
      }
    }
  }
}

/*
   Method used to set the motor speed by PID control
*/
void LIDAR::setMotorSpeed(int motorSpeed) {
  analogWrite(_motorPin, motorSpeed);
}

/*
   PID controlled motor speed.
*/
int LIDAR::computeSpeed(float motorSpeed) {
  unsigned long now = millis();
  unsigned long timeChange = (now - _lastTime);
  if (timeChange >= _sampleTime) {
    float error = _setPoint - motorSpeed;
    _intTerm += _Ki * error;

    if (_intTerm > MOTOR_MAX) {
      _intTerm = MOTOR_MAX;
    }
    else if (_intTerm < MOTOR_MIN) {
      _intTerm = MOTOR_MIN;
    }

    float diffTerm = (motorSpeed - _lastInput);

    outputPID = _Kp * error + _intTerm - _Kd * diffTerm;

    if (outputPID > MOTOR_MAX) {
      outputPID = MOTOR_MAX;
    }
    else if (outputPID < MOTOR_MIN) {
      outputPID = MOTOR_MIN;
    }

    _lastInput = motorSpeed;
    _lastTime = now;
  }
  return int(outputPID);
}

/*
   Extract the motor speed from the serial input.
*/
void LIDAR::processSpeed() {
  uint8_t low_byte = data[OFFSET_TO_SPEED];
  uint8_t high_byte = data[OFFSET_TO_SPEED + 1];
  _motorSpeed = float( (high_byte << 8) | low_byte ) / 64.0;
}

/*
   Extract the distance measurements from the serial input.
*/
unsigned char LIDAR::processDistance(int iQuad) {
  uint8_t dataL, dataH;
  dist[iQuad] = 0;                     // initialize
  int iOffset = 4 + (iQuad * 4);
  dataH = data[iOffset + 1];           // invalid or strength data
  if (dataH & DATA_MASK)             // if data is invalid or low signal strength -> corrupted
    return dataH & DATA_MASK;        // non-zero if corrupted data
  dataL = data[iOffset];
  dist[iQuad] = dataL | ((dataH & 0x3F) << 8);
  return 0;                          // zero if distance reading was successful
}

/*
   Set new PID values if necessary.
*/
void LIDAR::setPID(float Kp, float Ki, float Kd, unsigned long sampleTime, float setPoint) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
  _sampleTime = sampleTime;
  _setPoint = setPoint;
}
