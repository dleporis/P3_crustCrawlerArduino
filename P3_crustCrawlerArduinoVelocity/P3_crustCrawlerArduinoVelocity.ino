
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
#define TEST_LED 13

const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN ???????
uint8_t preGripperDXL_ID;
uint8_t DXL_ID = 1; //ID of motor, can be changed by user input
const float DXL_PROTOCOL_VERSION = 2.0;
int8_t angVelGear = 0;
uint8_t gripperState = 0;
uint8_t rawVelRatio = 50;
int8_t DXL_GOAL_VELOCITY;
uint8_t DXL_P_GAIN[] = {100, 100, 2, 2, 2};
uint8_t DXL_I_GAIN[] = {1, 1, 1, 1, 1};
//uint8_t DXL_D_GAIN[] = {1, 1, 1, 1, 1};
uint8_t DXL_VELOCITY_LIMIT[] = {200, 500, 200, 200, 200};
uint8_t DXL_PROFILE_VELOCITY[] = {100, 300, 100, 100, 100}; //range: 0 ~ VELOCITY_LIMIT, which has range 0 ~ 1,023 unit[0.229rpm]
uint8_t DXL_PROFILE_ACCELERATION[] = {40, 200, 10, 10, 10};
uint8_t DXL_PWM_LIMIT[] = {885,885,885,885,885};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  // begin communication with PC
  DEBUG_SERIAL.begin(9600);
  Serial.println("<Arduino is ready>");
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  dxl.scan();
  
  
  for (DXL_ID = 1; DXL_ID <=5; DXL_ID++){
    // Turn off torque
    dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 0);
  
    /* set velocity(speed) mode */
    // MX2.0, X serise
    dxl.writeControlTableItem(OPERATING_MODE, DXL_ID, 1);//velocity mode
    DEBUG_SERIAL.print("read op mode:");
    DEBUG_SERIAL.println(dxl.readControlTableItem(OPERATING_MODE, DXL_ID));
    dxl.setOperatingMode(DXL_ID, OP_VELOCITY); //
    DEBUG_SERIAL.print("get op mode:");
    DEBUG_SERIAL.println(dxl.readControlTableItem(OPERATING_MODE, DXL_ID));
    // AX~MX
    dxl.writeControlTableItem(CW_ANGLE_LIMIT, DXL_ID, 0);
    dxl.writeControlTableItem(CCW_ANGLE_LIMIT, DXL_ID, 0);
    dxl.writeControlTableItem(PWM_LIMIT, DXL_ID, DXL_PWM_LIMIT[DXL_ID -1]  );
    // Turn on torque
    dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 1);
    DEBUG_SERIAL.print(DXL_ID);
    DEBUG_SERIAL.print(" torque enabled: ");
    DEBUG_SERIAL.println(dxl.readControlTableItem(TORQUE_ENABLE, DXL_ID));
  }
  //dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 0);
  dxl.writeControlTableItem(DRIVE_MODE, 1, 0);
  dxl.writeControlTableItem(DRIVE_MODE, 2, 0);
  dxl.writeControlTableItem(DRIVE_MODE, 3, 0);
  dxl.writeControlTableItem(DRIVE_MODE, 4, 0);
  dxl.writeControlTableItem(DRIVE_MODE, 5, 1);

  dxl.writeControlTableItem(HOMING_OFFSET, 1, -3071);
  dxl.writeControlTableItem(HOMING_OFFSET, 2, -2560);
  dxl.writeControlTableItem(HOMING_OFFSET, 3, -3055);
  dxl.writeControlTableItem(HOMING_OFFSET, 4, -2021);
  dxl.writeControlTableItem(HOMING_OFFSET, 5, 2028);

  DXL_ID =2;
  dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, 300);
}


void loop() {
//  int prevDXL_ID = DXL_ID;
//  DEBUG_SERIAL.println("");
//  DEBUG_SERIAL.println(":::::::::::::::::::::::::::::");
//  for (DXL_ID = 1; DXL_ID <=5; DXL_ID++){
//    
//    DEBUG_SERIAL.print("Present position of servo ");
//    DEBUG_SERIAL.print(DXL_ID);
//    DEBUG_SERIAL.print(" is: ");
//    DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID));
//  }
//  DXL_ID = prevDXL_ID;  
  String serialdata = DEBUG_SERIAL.readStringUntil('\n');
  if (serialdata == NULL) {
    return;
  }
  //DEBUG_SERIAL.println("A: rec data: "+serialdata);
  //
  
  if (serialdata.charAt(0) == 'i'){
    switch (serialdata.charAt(1)){
      case '1':
        DXL_ID = 1;
        break;
      case '2':
        DXL_ID = 2;
        break;
      case '3':
        DXL_ID = 3;
        break;
    }
    DEBUG_SERIAL.print("A:IDshift:");
    DEBUG_SERIAL.println(DXL_ID);
  }
  else if (serialdata.charAt(0) == 'v'){
    switch (serialdata.charAt(1)){
      case '0':
        angVelGear = 0;
        break;
      case '1':
        angVelGear = 1;
        break;
      case '2':
        angVelGear = 2;
        break;
      case '3':
        angVelGear = 3;
        break;
      case '-':
        switch (serialdata.charAt(2)){
          case '1':
            angVelGear = -1;
            break;
          case '2':
            angVelGear = -2;
            break;
          case '3':
            angVelGear = -3;
            break;
        }
        break;
    }
    DEBUG_SERIAL.print("A:Vel_gr:");
    DEBUG_SERIAL.println(angVelGear);
    //dxl.setGoalVelocity(DXL_ID, rawVelRatio*angVelGear); //rawVel = rawVelRatio = 10 
    DXL_GOAL_VELOCITY = rawVelRatio*angVelGear;
    DEBUG_SERIAL.print("A: pre write GOAL_VELOCITY IS:");
    DEBUG_SERIAL.println(DXL_GOAL_VELOCITY);
    dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, DXL_GOAL_VELOCITY);
    delay(1000);
    DEBUG_SERIAL.print("A: written GOAL_VELOCITY IS:");
    DEBUG_SERIAL.println(dxl.readControlTableItem(GOAL_VELOCITY, DXL_ID));
    DEBUG_SERIAL.print("A:ID:");
    DEBUG_SERIAL.print(DXL_ID);
    DEBUG_SERIAL.print("RAW_vel:");
    DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_VELOCITY, DXL_ID));
     DEBUG_SERIAL.print("Present PWM of servo ");
    DEBUG_SERIAL.print(DXL_ID);
    DEBUG_SERIAL.print(" is: ");
    DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_PWM, DXL_ID));
    
  }
  else if (serialdata.charAt(0) == 'c'){
    // close the gripper
    DEBUG_SERIAL.println("A:Gripp_closed.");
    //angVelGear = 0;
    //dxl.setGoalVelocity(DXL_ID, rawVelRatio*angVelGear);
    //dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, rawVelRatio*angVelGear);
    preGripperDXL_ID = DXL_ID;
    delay(100);
    DXL_ID = 4;
    //dxl.setGoalVelocity(DXL_ID, 10);
    dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, -10);
    DXL_ID = 5;
    //dxl.setGoalVelocity(DXL_ID, 10);
    dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, 10);
    delay(10);
    DEBUG_SERIAL.print("A:ID:5RAW_vel:");
    DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID));
    DXL_ID = 4;
    DEBUG_SERIAL.print("A:ID:4RAW_vel:");
    DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID));
    DXL_ID = preGripperDXL_ID;
    
  }
  else if (serialdata.charAt(0) == 'o'){
    // open the gripper
    //angVelGear = 0;
    //dxl.setGoalVelocity(DXL_ID, rawVelRatio*angVelGear);
    //dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, rawVelRatio*angVelGear);
    preGripperDXL_ID = DXL_ID;
    DEBUG_SERIAL.println("A:Gripp_open.");
    DXL_ID = 4;
    //dxl.setGoalVelocity(DXL_ID, -10);
//    dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, 10 );
    DXL_ID = 5;
    //dxl.setGoalVelocity(DXL_ID, -10);
    dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, -10 );
    delay(10);
    DEBUG_SERIAL.print("A:ID:5RAW_vel:");
    DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID));
    DXL_ID = 4;
    DEBUG_SERIAL.print("A:ID:4RAW_vel:");
    DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID));
    DXL_ID = preGripperDXL_ID;
  }
  //check position of servo, click t like theta
//  else if (serialdata.charAt(0) == 't'){
//    //readStringUntill
//    DXL_ID = 4;
//    DEBUG_SERIAL.print("Present position of servo ");
//    DEBUG_SERIAL.print(DXL_ID);
//    DEBUG_SERIAL.print(" is: ");
//    DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID));
//    DXL_ID = 5;
//    DEBUG_SERIAL.print("Present position of servo ");
//    DEBUG_SERIAL.print(DXL_ID);
//    DEBUG_SERIAL.print(" is: ");
//    DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID));
//    serialdata = "";
//  }
  //PIZZA position mode
  else if (serialdata.charAt(0) == 'p'){
      DEBUG_SERIAL.println("PIZZA POSITION MODE");
      for (int DXL_ID = 1; DXL_ID <=5; DXL_ID++){
      dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 0);
      dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID, DXL_P_GAIN[DXL_ID -1] );
      dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID, DXL_I_GAIN[DXL_ID -1] );
      //dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID, DXL_D_GAIN[DXL_ID -1] );
      //dxl.writeControlTableItem(ACCELERATION_LIMIT, DXL_ID, );
      dxl.writeControlTableItem(VELOCITY_LIMIT, DXL_ID, DXL_VELOCITY_LIMIT[DXL_ID -1]  );
      dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, DXL_PROFILE_VELOCITY[DXL_ID -1]  );
      //dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, DXL_PROFILE_ACCELERATION[DXL_ID -1]  );
      
      dxl.writeControlTableItem(OPERATING_MODE, DXL_ID, 3); //position mode
      // Turn on torque
      //dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 1);
    }
    dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 0);
    dxl.writeControlTableItem(DRIVE_MODE, 1, 0);
    dxl.writeControlTableItem(DRIVE_MODE, 2, 0);
    dxl.writeControlTableItem(DRIVE_MODE, 3, 0);
    dxl.writeControlTableItem(DRIVE_MODE, 4, 0);
    dxl.writeControlTableItem(DRIVE_MODE, 5, 1);
  
    dxl.writeControlTableItem(HOMING_OFFSET, 1, -3071);
    dxl.writeControlTableItem(HOMING_OFFSET, 2, -2560);
    dxl.writeControlTableItem(HOMING_OFFSET, 3, 1024); //position mode requires different homing offset than vel
    dxl.writeControlTableItem(HOMING_OFFSET, 4, -2021);
    dxl.writeControlTableItem(HOMING_OFFSET, 5, 2028);
    // TEST - send angle, watch servo 3 to obtain new position
//    delay(2000);
//    DXL_ID = 3;
//    DXL_GOAL_POSITION[DXL_ID -1] = 0;
//    DEBUG_SERIAL.println("position_mode_test in 2 secs");
//    DEBUG_SERIAL.print("Goal position: ");
//    DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
//    delay(2000);
//    
//    dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1])); //angle input
    serialdata ="";
  }
  else if (serialdata.charAt(0) == ' '){
    for (int DXL_ID = 1; DXL_ID <=5; DXL_ID++){
      dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, 0);
      dxl.writeControlTableItem(GOAL_CURRENT, DXL_ID, 0);
      //dxl.writeControlTableItem(GOAL_PWM, DXL_ID, 0);
    }
  }
// VELOCITY MODE.ino example:_____________
 // Please refer to e-Manual(http://emanual.robotis.com) for available range of value.
  // Set Goal Velocity using RAW unit
  //dxl.setGoalVelocity(DXL_ID, 200);
  //delay(1000);
  // Print present velocity
  //DEBUG_SERIAL.print("Present Velocity(raw) : ");
  //DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID));
  //delay(1000);

  // Set Goal Velocity using RPM
//  dxl.setGoalVelocity(DXL_ID, 25.8, UNIT_RPM);
//  delay(1000);
//  DEBUG_SERIAL.print("Present Velocity(rpm) : ");
//  DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID, UNIT_RPM));
//  delay(1000);

  // Set Goal Velocity using percentage (-100.0 [%] ~ 100.0 [%])
//  dxl.setGoalVelocity(DXL_ID, -10.2, UNIT_PERCENT);
//  delay(1000);
//  DEBUG_SERIAL.print("Present Velocity(ratio) : ");
//  DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID, UNIT_PERCENT));
//  delay(1000);
}

float deg2Raw(float degAng){
  float rawAng = degAng * 4095 / 360;
  return rawAng;
}
float raw2Deg(float rawAng){
  float degAng = rawAng * 360 / 4095;
  return degAng;
}
