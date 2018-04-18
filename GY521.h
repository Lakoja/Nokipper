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

#ifndef __GY521_CONTROL_H__
#define __GY521_CONTROL_H__

#include "Wire.h"
#include "MPU6050.h"
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

class GY521
{
private:
  Kalman kalmanX; // Kalman demo code copied from http://forum.arduino.cc/index.php?topic=261200.0
  Kalman kalmanY;
  MPU6050 mpu; //(0x69); // <-- use for AD0 high
  bool sensorOk = false;
  bool first = true;
  uint64_t lastMicros = 0;
  uint32_t lastTimeOut = 0;
  double kalAngleX;
  double kalAngleY;

public:
  bool init()
  {
    Wire.begin();

    Serial.println("Initializing I2C devices...");
    bool b = mpu.initialize();

    Serial.println("Testing device connections...");
    sensorOk = mpu.testConnection();
  
    Serial.println(String(b)+" " +String()+ (sensorOk ? "MPU6050 connection successful" : "MPU6050 connection failed"));

    if (sensorOk) {
      int16_t gx, gy, gz;
      mpu.getRotation(&gx, &gy, &gz);
    
      Serial.print("First Gyro values:\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.println(gz);
    
      // TODO supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(220);
      mpu.setYGyroOffset(76);
      mpu.setZGyroOffset(-85);
      mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    }

    lastMicros = esp_timer_get_time();

    return sensorOk;
  }

  double getX()
  {
    return kalAngleX;
  }

  double getY()
  {
    return kalAngleY;
  }

  void drive()
  {
    uint64_t now = esp_timer_get_time();
    double passedSeconds = (now - lastMicros) / 1000000.0;
    
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    
    double accXangle, accYangle; // Angle calculate using the accelerometer
    double gyroXangle, gyroYangle; // Angle calculate using the gyro
  
    accYangle = (atan2(ax, az) + PI) * RAD_TO_DEG;
    accXangle = (atan2(ay, az) + PI) * RAD_TO_DEG;
      
    if (first) {
      kalmanX.setAngle(accXangle); // Set starting angle
      kalmanY.setAngle(accYangle);
      gyroXangle = accXangle;
      gyroYangle = accYangle;
    
      first = false;
    } else {
      double gyroXrate = (double)gx / 131.0;
      double gyroYrate = -((double)gy / 131.0);
      gyroXangle += gyroXrate * passedSeconds; // Calculate gyro angle without any filter
      gyroYangle += gyroYrate * passedSeconds;

      kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, passedSeconds); // Calculate the angle using a Kalman filter
      kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, passedSeconds);
  
      uint32_t now = millis();
      if (now - lastTimeOut >= 1000) {
        /*
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);*/
        
        //Serial.println(String(kalAngleX)+" , "+String(kalAngleY));
        lastTimeOut = now;
      }
    }
  }
};

#endif
