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

#ifndef __STEPPER_MOTORS_H__
#define __STEPPER_MOTORS_H__

#include <math.h>
#include "Task.h"

class StepperMotors: public Task
{
private:
  // the externally (or automatically) requested values
  float motorRSpeedDesired = 0;
  float motorLSpeedDesired = 0;
  double currentPwmRight = 1;
  double currentPwmLeft = 1;
  uint32_t motorREndTime;
  uint32_t motorLEndTime;
  uint64_t lastDriveLoopMicros = 0;
  uint32_t lastDriveLoopTime = 0;
  uint32_t lastCounterOutTime = 0;

  double microStepsLeft = 0; // can be negative; 0 means not real movement since start
  double microStepsRight = 0;

  uint8_t stepPinRight;
  uint8_t dirPinRight;
  uint8_t sleepPinRight;
  uint8_t stepPinLeft;
  uint8_t dirPinLeft;
  uint8_t sleepPinLeft;
  uint16_t motorMaxTurns;
  uint16_t stepsPerRotation;

public:
  StepperMotors()
  {
  }

  void setup(
    uint8_t stepRight,
    uint8_t dirRight,
    uint8_t sleepRight,
    uint8_t stepLeft,
    uint8_t dirLeft,
    uint8_t sleepLeft,
    uint16_t maxRpm,
    uint16_t steps)
  {
    stepPinRight = stepRight;
    dirPinRight = dirRight;
    sleepPinRight = sleepRight;
    stepPinLeft = stepLeft;
    dirPinLeft = dirLeft;
    sleepPinLeft = sleepLeft;
    motorMaxTurns = maxRpm;
    stepsPerRotation = steps;
  
    outputPin(stepPinRight);
    outputPin(dirPinRight);
    outputPin(sleepPinRight);
    outputPin(stepPinLeft);
    outputPin(dirPinLeft);
    outputPin(sleepPinLeft);

    // NOTE there are groups of timers at work here: The setting for channel 1 also resets the one for 0.
    // Some documentation would have been fine here generally...

    // For steppers: control is done with the "tone" (and only when driver enabled)
    ledcSetup(0, 600, 8); // precision bits 8: means we have 0..255 as steps, 4: 0..15
    ledcSetup(2, 600, 8);
    ledcAttachPin(stepPinRight, 0);
    ledcAttachPin(stepPinLeft, 2);

    // some arbitrary value: only falling flanks matter
    ledcWrite(0, 63);
    ledcWrite(2, 63);

    lastDriveLoopMicros = esp_timer_get_time();
  }

  void run()
  {
    while (true) {
      uint64_t nowMicros = esp_timer_get_time();
      uint32_t now = nowMicros/1000;
      
      if (lastDriveLoopMicros > 0) {
        uint32_t passedMicros = nowMicros - lastDriveLoopMicros;
        
        if (now >= motorREndTime) {
          motorRSpeedDesired = 0;
        }
  
        if (now >= motorLEndTime) {
          motorLSpeedDesired = 0;
        }

        double fromSecond = passedMicros / 1000000.0;
        
        microStepsLeft += currentPwmLeft * fromSecond;

        //Serial.print("m"+String(now-lastDriveLoopTime)+"p"+String(passedMicros)+"s"+String(microStepsLeft)+" ");
        //Serial.print("p"+String(passedMicros / 1000.0)+"s"+String(microStepsLeft)+" ");
       
        microStepsRight += currentPwmRight * fromSecond;
      }

      double rSpeed = motorRSpeedDesired * motorMaxTurns / 60.0 * stepsPerRotation;
      double lSpeed = motorLSpeedDesired * motorMaxTurns / 60.0 * stepsPerRotation;
      switchMotorR(rSpeed);
      switchMotorL(lSpeed);

      if (now - lastCounterOutTime > 1200) {
        //Serial.println("R r"+String(rSpeed)+" L r"+String(lSpeed));
        //Serial.println("msl "+String(microStepsLeft)+" msr "+String(microStepsRight));
        
        lastCounterOutTime = now;
      }
  
      lastDriveLoopMicros = nowMicros;
      lastDriveLoopTime = now;

      sleepAfterLoop(4, now);
    }
  }

  double getMicroStepsRight()
  {
    return microStepsRight;
  }
  
  double getMicroStepsLeft()
  {
    return microStepsLeft;
  }

  void requestMovement(float forward, float right, uint16_t durationMillis = 1000) {
    // This is the only one with value range -1 .. 1

    float rightSpeed = forward;
    float leftSpeed = forward;
    rightSpeed -= right / 2;
    leftSpeed += right / 2;
    
    requestRight(rightSpeed, durationMillis);
    requestLeft(leftSpeed, durationMillis);
  }

  void requestRight(float value, uint16_t durationMillis = 1000)
  {
    // TODO check for value range?
    
    uint32_t now = millis();
    motorRSpeedDesired = value;
    motorREndTime = now + durationMillis;
  }
  
  void requestLeft(float value, uint16_t durationMillis = 1000)
  {
    uint32_t now = millis();
    motorLSpeedDesired = value;
    motorLEndTime = now + durationMillis;
  }  
  
  void requestForward(float value, uint16_t durationMillis = 1000)
  {
    Serial.println("Request forward "+String(value));
    
    uint32_t desiredEndTime = millis() + durationMillis;
    motorRSpeedDesired = value;
    motorREndTime = desiredEndTime;
    motorLSpeedDesired = value;
    motorLEndTime = desiredEndTime;
  }
  
  void requestReverse(float value, uint16_t durationMillis = 1000)
  {
    uint32_t desiredEndTime = millis() + durationMillis;
    motorRSpeedDesired = -value;
    motorREndTime = desiredEndTime;
    motorLSpeedDesired = -value;
    motorLEndTime = desiredEndTime;
  }

private:
  void outputPin(int num)
  {
    digitalWrite(num, LOW);
    pinMode(num, OUTPUT);
  }

  void switchMotorR(double pwmValue)
  {
    if (pwmValue != currentPwmRight) {
      
      if (showDebug) {
        //(Serial.print("RRa"+String(pwmValue)+" ");
  
        //if (++outCounter % 20 == 0)
          //Serial.println();
      }
        
      int32_t speedInt = round(pwmValue);
      ledcWriteTone(0, abs(speedInt));

      digitalWrite(dirPinRight, pwmValue >= 0 ? 0 : 1);
      digitalWrite(sleepPinRight, pwmValue != 0 ? 1 : 0);

      currentPwmRight = pwmValue;
    }
  }

  void switchMotorL(double pwmValue)
  {
    if (pwmValue != currentPwmLeft) {
  
      int32_t speedInt = round(pwmValue);
      ledcWriteTone(2, abs(speedInt));

      
      if (showDebug) { // && random(5) == 4) {
        //Serial.print("LRai"+String(speedInt)+" ");
  
        //if (++outCounter % 20 == 0)
          //Serial.println();
      }

      digitalWrite(dirPinLeft, pwmValue >= 0 ? 1 : 0); // inverted to right
      digitalWrite(sleepPinLeft, pwmValue != 0 ? 1 : 0);
      
      currentPwmLeft = pwmValue;
    }
  }

  uint16_t outCounter = 0;
  bool showDebug = true;
};

#endif
