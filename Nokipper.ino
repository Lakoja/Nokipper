
/*
 * Copyright (C) 2018 Lakoja on github.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#include "I2Cdev.h"
#include "GY25.h"
#include "GY521.h"
#include "StepperMotors.h"
#include <PID_v1.h>

const uint8_t MOTOR_L_DIR = 25;
const uint8_t MOTOR_L_STEP = 33;
const uint8_t MOTOR_L_SLEEP = 32;
const uint8_t MOTOR_R_SLEEP = 26;
const uint8_t MOTOR_R_STEP = 27;
const uint8_t MOTOR_R_DIR = 14;
const uint16_t MOTOR_MAX_RPM = 60; // for steppers this can be higher (and weaker)
const uint16_t MOTOR_RESOLUTION = 800; // steps per rotation; this assumes a sub-step sampling (drv8834) of 4

const int LED_PIN = 2;

GY521 gy521;
GY25 gy25;
bool sensorOk = false;
uint32_t valueCounter = 0;
uint32_t systemStartTime = 0;
uint32_t lastTimeOut = 0;

double pidSetpoint = 90, pidInput = 90, pidOutput;
double Kp=0.4, Ki=0, Kd=0.0;
PID myPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

StepperMotors motor;

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);

  sensorOk = gy521.init();
  gy25.init();
  sensorOk = true;

  if (sensorOk) {
    digitalWrite(LED_PIN, HIGH);
    motor.setup(MOTOR_R_STEP, MOTOR_R_DIR, MOTOR_R_SLEEP, MOTOR_L_STEP, MOTOR_L_DIR, MOTOR_L_SLEEP, MOTOR_MAX_RPM, MOTOR_RESOLUTION);
    motor.start("motor");

    myPID.SetOutputLimits(-1, 1);
    myPID.SetSampleTime(2);

    systemStartTime = millis();
    //motor.requestForward(0.2, 1000);

    myPID.SetMode(AUTOMATIC);
  }
}


void loop() {

  gy25.drive();

  if (!sensorOk) {
    return;
  }

  gy521.drive();

  uint32_t now = millis();

  pidInput = gy25.getRoll() + 4; //gy521.getX() - 180;
  myPID.Compute();

  motor.requestMovement(pidOutput, 0);
  
  if (now - lastTimeOut >= 500) {
    //Serial.println("per/s "+String(valueCounter / ((now - systemStartTime) / 1000.0)));

    //Serial.println("input "+String(pidInput));
    //Serial.println("steps L "+String(motor.getMicroStepsLeft())+" R "+String(motor.getMicroStepsRight()));
    Serial.println("input "+String(pidInput)+" > output "+String(pidOutput));//+" roll "+String(gy25.getRoll()));
    lastTimeOut = now;
  }

  valueCounter++;

  delay(1);
}
