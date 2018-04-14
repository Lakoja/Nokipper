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
#include "MPU6050.h"
#include "GY25.h"
#include "Wire.h"
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "StepperMotors.h"

const uint8_t MOTOR_L_STEP = 33;
const uint8_t MOTOR_L_DIR = 32;
const uint8_t MOTOR_L_SLEEP = 25;
const uint8_t MOTOR_R_STEP = 27;
const uint8_t MOTOR_R_DIR = 26;
const uint8_t MOTOR_R_SLEEP = 14;
const uint16_t MOTOR_MAX_RPM = 120; // for steppers this can be higher (and weaker)
const uint16_t MOTOR_RESOLUTION = 800; // steps per rotation; this assumes a sub-step sampling (drv8834) of 4


// Kalman demo code copied from http://forum.arduino.cc/index.php?topic=261200.0
Kalman kalmanX;
Kalman kalmanY;

MPU6050 mpu; //(0x69); // <-- use for AD0 high
GY25 gy25;
bool sensorOk = false;

StepperMotors motor;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Initializing I2C devices...");
  mpu.initialize();

  gy25.init();

  // verify connection
  Serial.println("Testing device connections...");
  sensorOk = mpu.testConnection();

  for (int i=0; i<3 && !sensorOk; i++) {
      delay(500);
      Serial.print("R");
      sensorOk = mpu.testConnection();
  }
  Serial.println(sensorOk ? "MPU6050 connection successful" : "MPU6050 connection failed");

  if (sensorOk) {
    motor.setup(MOTOR_R_STEP, MOTOR_R_DIR, MOTOR_R_SLEEP, MOTOR_L_STEP, MOTOR_L_DIR, MOTOR_L_SLEEP, MOTOR_MAX_RPM, MOTOR_RESOLUTION);
    
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
  
    Serial.print("First Gyro values:\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
  
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    motor.start("motor");

    motor.requestForward(0.1, 2000);
  }
}

bool first = true;
uint32_t timer = 0;
uint32_t lastTimeOut = 0;

void loop() {

  gy25.drive();

  if (!sensorOk) {
    return;
  }

  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);


  uint32_t now = millis();
  if (now - lastTimeOut >= 1000) {
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
  }

  double accXangle, accYangle; // Angle calculate using the accelerometer
  double gyroXangle, gyroYangle; // Angle calculate using the gyro
  double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter

  accYangle = (atan2(ax, az) + PI) * RAD_TO_DEG;
  accXangle = (atan2(ay, az) + PI) * RAD_TO_DEG;
    
  if (first) {
    kalmanX.setAngle(accXangle); // Set starting angle
    kalmanY.setAngle(accYangle);
    gyroXangle = accXangle;
    gyroYangle = accYangle;
  
    first = false;

    timer = micros();
  } else {

    double gyroXrate = (double)gx / 131.0;
    double gyroYrate = -((double)gy / 131.0);
    gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000);
  
    kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); // Calculate the angle using a Kalman filter
    kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);

    timer = micros();

    uint32_t now = millis();
    if (now - lastTimeOut >= 1000) {
      Serial.println(String(kalAngleX)+" , "+String(kalAngleY));
      lastTimeOut = now;
    }
  }

  delay(1);
}
