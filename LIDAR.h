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

#ifndef LIDAR_h
#define LIDAR_h

#define START 0xFA
#define ANGLE_OFFSET 0xA0
#define OFFSET_TO_DISTANCE 4
#define OFFSET_TO_SPEED 2
#define DATA_LENGTH 22
#define NUM_DISTANCE 4
#define OFFSET_TO_SPEED_LOW 2
#define OFFSET_TO_SPEED_HIGH 3
#define MOTOR_MAX 255
#define MOTOR_MIN 100

#include "Arduino.h"

class LIDAR
{
  public:
    LIDAR(HardwareSerial *serial, int motorPin);
    ~LIDAR(void);
    void printSensorDataToSerial(bool printSensorData);
    void startLIDAR();
    void printMotorSpeedToSerial(bool printMotorSpeed);
    void readLIDAR();
    void setPID(float Kp, float Ki, float Kd, unsigned long sampleTime, float setPoint);
  private:
    HardwareSerial * _serial;
    int _motorPin;
    float _motorSpeed;
    float outputPID;
    bool SEARCH_STATE;
    bool _printSensorData;
    bool _printMotorSpeed;
    float _Kp;
    float _Ki;
    float _Kd;
    float _setPoint;
    float _lastInput;
    float _intTerm;
    unsigned long _sampleTime;
    unsigned long _lastTime;
    static const unsigned char STRENGTH_FLAG = (1 << 6);
    static const unsigned char INVALID_FLAG = (1 << 7);
    static const unsigned char DATA_MASK = (INVALID_FLAG | STRENGTH_FLAG);
    uint16_t dist[NUM_DISTANCE];


    uint8_t inByte;
    int byteNum;
    int data[DATA_LENGTH];
    int dataIndex;

    void processSpeed();
    unsigned char processDistance(int iQuad);
    int computeSpeed(float motorSpeed);
    void setMotorSpeed(int motorSpeed);
};

#endif
