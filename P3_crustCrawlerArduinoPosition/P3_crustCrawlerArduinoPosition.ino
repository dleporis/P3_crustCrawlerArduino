
/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1 //RS485
#define DEBUG_SERIAL Serial  // PC
#define TEST_LED 13l
String serialdata = "";
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN ???????
uint8_t preGripperDXL_ID;
uint8_t DXL_ID = 1; //ID of motor, can be changed by user input
const float DXL_PROTOCOL_VERSION = 2.0;
int8_t angVelGear = 0;
uint8_t gripperState = 0;
uint8_t rawVelRatio = 10;
uint16_t DXL_P_GAIN[] = {850, 750, 850, 850, 850};
uint8_t DXL_I_GAIN[] = {0, 0, 0, 0, 0};
uint16_t DXL_D_GAIN[] = {10, 1600, 10, 10000, 10000};
uint8_t DXL_FWD1_GAIN[] = {1, 100, 1, 1, 1};
uint8_t DXL_FWD2_GAIN[] = {1, 1, 1, 1, 1};
uint8_t DXL_VELOCITY_LIMIT[] = {200, 210, 200, 200, 200};
uint16_t DXL_ACCELERATION_LIMIT[] = {200, 200, 200, 200, 200};
//uint16_t DXL_PROFILE_VELOCITY[] = {50, 1000, 100, 100, 100}; //range: 0 ~ VELOCITY_LIMIT, which has range 0 ~ 1,023 unit[0.229rpm] 
uint16_t DXL_PROFILE_VELOCITY[] = {0, 0, 0, 0, 0};
//uint16_t DXL_PROFILE_ACCELERATION[] = {10, 1000, 10, 10, 10};
uint16_t DXL_PROFILE_ACCELERATION[] = {0, 10, 0, 0, 0};
int16_t DXL_PWM_LIMIT[] = {250,250,250,250,250};
int16_t DXL_GOAL_PWM[] = {250,250,250,250,250};// 200, 200, 200, 200}; //PWM limit range 0(0 [%]) ~ 885(100 [%])
uint8_t DXL_DRIVE_MODE[]={0,0,0,0,1};
uint8_t DXL_OPERATING_MODE[]={4,4,4,4,4}; //extended position mode // position mode
int16_t DXL_HOMING_OFFSET[] = {1030, 0, -3081, -2030, 2017}; //position mode requires different homing offset than vel mode
float input[3];
int16_t DXL_GOAL_POSITION[5];

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  // begin communication with PC
  DEBUG_SERIAL.begin(38400);
  Serial.println("<Arduino is ready>");
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  dxl.scan();

  DEBUG_SERIAL.println("INITIALISING POSITION MODE");
  for (int DXL_ID = 1; DXL_ID <=5; DXL_ID++){
    dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 0);
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID, DXL_P_GAIN[DXL_ID -1] );
    DEBUG_SERIAL.print(DXL_ID);
    DEBUG_SERIAL.print(" P gain: ");
    DEBUG_SERIAL.println(dxl.readControlTableItem(POSITION_P_GAIN, DXL_ID));
    dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID, DXL_I_GAIN[DXL_ID -1] );
    dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID, DXL_D_GAIN[DXL_ID -1] );
    dxl.writeControlTableItem(FEEDFORWARD_1ST_GAIN, DXL_ID, DXL_FWD1_GAIN[DXL_ID -1] );
    dxl.writeControlTableItem(FEEDFORWARD_2ND_GAIN, DXL_ID, DXL_FWD2_GAIN[DXL_ID -1] );
    dxl.writeControlTableItem(VELOCITY_LIMIT, DXL_ID, DXL_VELOCITY_LIMIT[DXL_ID -1]  );
    dxl.writeControlTableItem(ACCELERATION_LIMIT, DXL_ID, DXL_ACCELERATION_LIMIT[DXL_ID -1] );
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, DXL_PROFILE_VELOCITY[DXL_ID -1]  );
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, DXL_PROFILE_ACCELERATION[DXL_ID -1]  );
    dxl.writeControlTableItem(PWM_LIMIT, DXL_ID, DXL_PWM_LIMIT[DXL_ID -1]  );//PWM limit range 0(0 [%]) ~ 885(100 [%])
    dxl.writeControlTableItem(GOAL_PWM, DXL_ID, DXL_GOAL_PWM[DXL_ID -1]  );
    dxl.writeControlTableItem(OPERATING_MODE, DXL_ID, DXL_OPERATING_MODE[DXL_ID -1]); // position mode
    dxl.writeControlTableItem(DRIVE_MODE, DXL_ID,DXL_DRIVE_MODE[DXL_ID -1] );
    dxl.writeControlTableItem(HOMING_OFFSET, DXL_ID, /*0);//**/ DXL_HOMING_OFFSET[DXL_ID -1]);//DXL_HOMING_OFFSET[DXL_ID -1] );
    // Turn on torque
    dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 1);
    DEBUG_SERIAL.print(DXL_ID);
    DEBUG_SERIAL.print(" torque enabled: ");
    DEBUG_SERIAL.println(dxl.readControlTableItem(TORQUE_ENABLE, DXL_ID));
    
  }
  
//  PWM BASED POSITION CONTROL OF 2ND SERVO
//  DXL_ID = 2;
//  dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 0);
//  dxl.writeControlTableItem(OPERATING_MODE, DXL_ID, 5); //current based position
//  dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID, 2000);
//  dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID, DXL_P_GAIN[DXL_ID -1] );
//  dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID, DXL_I_GAIN[DXL_ID -1] );
//  dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID, DXL_D_GAIN[DXL_ID -1] );
//  dxl.writeControlTableItem(FEEDFORWARD_1ST_GAIN, DXL_ID, DXL_FWD1_GAIN[DXL_ID -1] );
//  dxl.writeControlTableItem(FEEDFORWARD_2ND_GAIN, DXL_ID, DXL_FWD2_GAIN[DXL_ID -1] );
//  dxl.writeControlTableItem(VELOCITY_LIMIT, DXL_ID, DXL_VELOCITY_LIMIT[DXL_ID -1]  );
//  dxl.writeControlTableItem(ACCELERATION_LIMIT, DXL_ID, DXL_ACCELERATION_LIMIT[DXL_ID -1] );
//  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, DXL_PROFILE_VELOCITY[DXL_ID -1]  );
//  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, DXL_PROFILE_ACCELERATION[DXL_ID -1]  );
//  dxl.writeControlTableItem(PWM_LIMIT, DXL_ID, DXL_PWM_LIMIT[DXL_ID -1]  );//PWM limit range 0(0 [%]) ~ 885(100 [%])
//  dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 1);
//  dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, 0);
//  dxl.writeControlTableItem(GOAL_CURRENT, DXL_ID, 1000);
  DXL_ID =1;
}


void loop() {
  int prevDXL_ID = DXL_ID;
  if(millis() % 2500 == 0){
    DEBUG_SERIAL.println("");
    DEBUG_SERIAL.println(":::::::::::::::::::::::::::::");
    for (DXL_ID = 1; DXL_ID <=5; DXL_ID++){
      
      DEBUG_SERIAL.print("Present position of servo ");
      DEBUG_SERIAL.print(DXL_ID);
      DEBUG_SERIAL.print(" : RAW: ");
      DEBUG_SERIAL.print(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID));
      DEBUG_SERIAL.print(" degrees: ");
      DEBUG_SERIAL.println(raw2Deg(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID)));
    }
  }
  
  DXL_ID = prevDXL_ID;  
  
  while (DEBUG_SERIAL.available() > 0) {
    String serialdata = DEBUG_SERIAL.readStringUntil('\n');
    //int inChar = DEBUG_SERIAL.read();
    //if (inChar != '\n') { 
      
      // As long as the incoming byte is not a newline,
      // convert the incoming byte to a char and add it to the string
      //serialdata += char(inChar);
    //}
    // if you get a newline, print the string,
    // then the string's value as a float:
    //else {
        
//      Serial.print("Input string: ");
//      Serial.print(inString);
//      Serial.print("\tAfter conversion to float:");
//      Serial.println(inString.toFloat());
//      // clear the string for new input:
//      inString = "";
    
    // recieving data from processing (theta1.00, theta2.00, theta3.00)  example (100.00,020.29,005.2)
      if (serialdata.charAt(0) == 'j'|| serialdata.charAt(0) == 'k' || serialdata.charAt(0) == 'l'){
        DEBUG_SERIAL.println("before remove:"+ serialdata);
        if (serialdata.charAt(0) == 'j'){
          serialdata.remove(0,1);
          input[0] = serialdata.toFloat();
          DEBUG_SERIAL.print("After conversion to float: ");
          DEBUG_SERIAL.println(input[0]);
          DXL_GOAL_POSITION[0] = input[0];
          DXL_ID = 1;
          dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, DXL_PROFILE_VELOCITY[DXL_ID -1]  );
          dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, DXL_PROFILE_ACCELERATION[DXL_ID -1]  );
          dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
          
          
        }
        else if (serialdata.charAt(0) == 'k'){
          serialdata.remove(0,1);
          input[1] = serialdata.toFloat();
          DEBUG_SERIAL.print("\tAfter conversion to float: ");
          DEBUG_SERIAL.println(input[1]);
          DXL_GOAL_POSITION[1] = input[1];
          DXL_ID = 2;
          //if (DXL_GOAL_POSITION[1] < 0)
          //  dxl.writeControlTableItem(GOAL_PWM, DXL_ID, -200 );
          //if (DXL_GOAL_POSITION[1] > 0)
          //  dxl.writeControlTableItem(GOAL_PWM, DXL_ID, 200 );
            
          dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, DXL_PROFILE_VELOCITY[DXL_ID -1]  );
          dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, DXL_PROFILE_ACCELERATION[DXL_ID -1]  );
          dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
          //dxl.writeControlTableItem(GOAL_CURRENT, DXL_ID, 2000 );
        }
        else if (serialdata.charAt(0) == 'l'){
          serialdata.remove(0,1);
          input[2] = serialdata.toFloat();
          DEBUG_SERIAL.println(input[2]);
          DXL_GOAL_POSITION[2] = input[2];
          DXL_ID = 3;
          dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, DXL_PROFILE_VELOCITY[DXL_ID -1]  );
          dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, DXL_PROFILE_ACCELERATION[DXL_ID -1]  );
          dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
        }
        DEBUG_SERIAL.print(DXL_ID);
        DEBUG_SERIAL.print("Goal PWM of servo: ");
        DEBUG_SERIAL.println(dxl.readControlTableItem(GOAL_PWM, DXL_ID));
        DEBUG_SERIAL.print(DXL_ID);
        DEBUG_SERIAL.print("Present PWM of servo: ");
        DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_PWM, DXL_ID));
        DEBUG_SERIAL.print(DXL_ID);
        DEBUG_SERIAL.print("Present current of servo: ");
        DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_CURRENT, DXL_ID));
        DEBUG_SERIAL.print(DXL_ID);
        DEBUG_SERIAL.print("Present profile acceleration of servo: ");
        DEBUG_SERIAL.println(dxl.readControlTableItem(PROFILE_ACCELERATION, DXL_ID));
        DEBUG_SERIAL.print(DXL_ID);
        DEBUG_SERIAL.print("present load of servo: ");
        DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_LOAD, DXL_ID));
          
        serialdata = "";
      }
      //TEST - send angle, watch servo 3 to obtain new position
      else if (serialdata.charAt(0) == 't'){
        DEBUG_SERIAL.println("position_mode_test in 2 secs");
        DXL_ID = 3;
        DXL_GOAL_POSITION[DXL_ID -1] = 0;
        DEBUG_SERIAL.print("Goal position deg: ");
        DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
        dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1])); //angle input
        delay(1000);
        DXL_GOAL_POSITION[DXL_ID -1] = -30;
        DEBUG_SERIAL.print("Goal position deg: ");
        DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
        dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
      }
     
      
      else if (serialdata.charAt(0) == 'c'){
        DEBUG_SERIAL.println("closing Gripper");
        DXL_ID = 4;
        DXL_GOAL_POSITION[DXL_ID -1] = -4;
        DEBUG_SERIAL.print("Goal position deg: ");
        DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
        dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1])); //angle input
        DXL_ID = 5;
        DXL_GOAL_POSITION[DXL_ID -1] = -4;
        DEBUG_SERIAL.print("Goal position deg: ");
        DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
        dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
      }
      else if (serialdata.charAt(0) == 'o'){
        DEBUG_SERIAL.println("closing Gripper");
        DXL_ID = 4;
        DXL_GOAL_POSITION[DXL_ID -1] = 30;
        DEBUG_SERIAL.print("Goal position deg: ");
        DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
        dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1])); //angle input
        DXL_ID = 5;
        DXL_GOAL_POSITION[DXL_ID -1] = 30;
        DEBUG_SERIAL.print("Goal position deg: ");
        DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
        dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
      }
      else if (serialdata.charAt(0) == ' '){
        for (int DXL_ID = 1; DXL_ID <=5; DXL_ID++){
          dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, 0);
          dxl.writeControlTableItem(GOAL_CURRENT, DXL_ID, 0);
        }
      }
    //}
    serialdata ="";
  }
}
float deg2Raw(float degAng){
  float rawAng = degAng * 4095 / 360;
  return rawAng;
}
float raw2Deg(float rawAng){
  float degAng = rawAng * 360 / 4095;
  return degAng;
}
float CSVtoFloat(String serialdata){
  float angle = 100*serialdata.charAt(1) + 10*serialdata.charAt(2) + 1*serialdata.charAt(3) 
  + 0.1*serialdata.charAt(5)+0.01*serialdata.charAt(6);
  return angle;
}
//void removeCharFromString(char * p, char c){
//  if (NULL == p)
//    return;  
//  char * pDest = p;
//
//  while (*p){
//    if( *p != c)
//      *pDest++ = *p;
//    p++;
//  }
//  *pDest = '\0';
//}
//
//void printString(char stringChar[], int lng){
//  for (int i = 0; i < lng; i++ ){
//     DEBUG_SERIAL.print(stringChar[i]);
//    }
//  }
