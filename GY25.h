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

#ifndef __GY25_CONTROL_H__
#define __GY25_CONTROL_H__
 
#include "HardwareSerial.h"

class GY25
{
private:
  HardwareSerial Serial2(2);
   
  float yaw, pitch, roll;
  unsigned char Re_buf[8];
  unsigned char counter=0;
  bool dataPresent=false;
  uint32_t valueCounter = 0;
  
  uint32_t systemStartTime = 0;
  uint32_t lastTimeOut = 0;

public:
  void init()
  {
    Serial2.begin(115200);
    
    Serial2.write(0XA5); 
    Serial2.write(0X52);
    systemStartTime = millis();
  }
  
  void drive()
  {
    if(dataPresent)
    { 
       dataPresent=false;
  
        uint32_t now = millis();
        if (now - lastTimeOut > 200) {
  
          float perSecond = valueCounter / ((now - systemStartTime) / 1000.0f);
         Serial.println(String(yaw)+" "+String(pitch)+" "+String(roll)+" r "+String(perSecond));
         lastTimeOut = now;
        }
    } else {
      readValue();
    }
  
    delay(1);
  }
  
  void readValue()
  {
      while (Serial2.available()) {   
        Re_buf[counter]=(unsigned char)Serial2.read();
        if(counter==0&&Re_buf[0]!=0xAA)  {
          Serial.print("w");
          break;       
        }
        counter++;       
        if(counter==8)
        {    
          valueCounter++;
           counter=0;
           dataPresent=true;
        }
      }
      
     if(Re_buf[0]==0xAA && Re_buf[7]==0x55)
     {           
        yaw=(Re_buf[1]<<8|Re_buf[2])/100.0f;
        pitch=(Re_buf[3]<<8|Re_buf[4])/100.0f;
        roll=(Re_buf[5]<<8|Re_buf[6])/100.0f;
     }
  
  /*
    if (yaw > 180) {
      yaw = yaw - (655 + 180);
    }
    if (pitch > 180) {
      pitch = pitch - 835;
    }
    if (roll > 180) {
      roll = roll - 835;
    }*/
  }
};

#endif
