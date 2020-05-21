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
#include <LiquidCrystal.h>
#include "Robot.h"
#include <TimerOne.h>

#define DXL_SERIAL   Serial1 //RS485
#define DEBUG_SERIAL Serial  // PC

long testCounter = 0;

// DH, kinemtics
// Receive with start- and end-markers
String serialdata = "";
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;
boolean checkStatusNow = false;
uint8_t recMode = 0; //0 not recording, 1 recording, 2 playback

// SETUP KINEMATIC MODEL
//setup DH parameters 
float TCP = 120; // in millimeters, from line between motors//distance between chosen TCP and
float DH_a[] = {0, 0, 218.5, 145}; // in millimeters
float DH_d[] = {185, 0, 0, 0}; // in millimeters
float DH_alpha[] = {0, -90, 0, 90}; // in degrees
float DH_theta_offset[] = {0, 0, 0, 0}; // in degrees, theta[0] is not being used
int configuration;
float prevTheta[4];
float input[3]; // cleaned gui input angle
float inputR[2]; // cleaned gui input, Xr Zr position
int presentTemp[5];
Robot crustCrawler(DH_a, DH_d, DH_alpha, DH_theta_offset, TCP); //declare robot var of Robot type



long currentMillis;
long prevMillis;
long loopTime;
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN ???????
uint8_t prevDXL_ID;
uint8_t DXL_ID = 1; //ID of motor, can be changed by user input
const float DXL_PROTOCOL_VERSION = 2.0;
int8_t angVelGear = 0;
uint8_t gripperState = 0;
uint8_t rawVelRatio = 10;
uint16_t DXL_P_GAIN[] = {750, 850, 750, 100, 100};
uint8_t DXL_I_GAIN[] = {0, 0, 0, 0, 0};
uint16_t DXL_D_GAIN[] = {100, 1600, 1100, 10000, 10000};
uint8_t DXL_FWD1_GAIN[] = {1, 1, 1, 1, 1};
uint8_t DXL_FWD2_GAIN[] = {1, 1, 1, 1, 1};
uint8_t DXL_VELOCITY_LIMIT[] = {200, 210, 200, 200, 200};
uint16_t DXL_ACCELERATION_LIMIT[] = {200, 200, 200, 200, 200};
//uint16_t DXL_PROFILE_VELOCITY[] = {50, 1000, 100, 100, 100}; //range: 0 ~ VELOCITY_LIMIT, which has range 0 ~ 1,023 unit[0.229rpm] 
uint16_t DXL_PROFILE_VELOCITY[] = {0, 0, 0, 0, 0};
//uint16_t DXL_PROFILE_ACCELERATION[] = {10, 1000, 10, 10, 10};
uint16_t DXL_PROFILE_ACCELERATION[] = {0, 10, 0, 0, 0};
int16_t DXL_PWM_LIMIT[] = {200,400,270,200,200};
int16_t DXL_GOAL_PWM[] = {200,260,200,200,200};// 200, 200, 200, 200}; //PWM limit range 0(0 [%]) ~ 885(100 [%])
uint8_t DXL_DRIVE_MODE[]={0,0,0,0,1};
uint8_t DXL_OPERATING_MODE[]={4,4,4,4,4}; //extended position mode // position mode
int16_t DXL_HOMING_OFFSET[] = {-3083, /*0/*/-4095/**/, -3081, -2030, 2017}; //position mode requires different homing offset than vel mode

int16_t DXL_GOAL_POSITION[5];

LiquidCrystal lcd(11, 12, 4, 5, 6, 7);
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

float deg2Raw(float degAng){
  float rawAng = degAng * 4095 / 360;
  return rawAng;
}
float raw2Deg(float rawAng){
  float degAng = rawAng * 360 / 4095;
  return degAng;
}

void  clearArray(char charArray[]){
  for (int x = 0; x < sizeof(charArray) / sizeof(charArray[0]); x++){
  charArray[x] = '0';
  }
}

void printStatus(){
   DEBUG_SERIAL.println("Servo ");
   DEBUG_SERIAL.print(DXL_ID); 
   DEBUG_SERIAL.print(" status:" );
   DEBUG_SERIAL.print("Position degrees: " );
   DEBUG_SERIAL.println(raw2Deg(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID)));
//   DEBUG_SERIAL.print("Position RAW: " );
//   DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID));
//   DEBUG_SERIAL.print("Velocity:" );
//   DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_VELOCITY, DXL_ID));
//   DEBUG_SERIAL.print("PWM:" );
//   DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_PWM, DXL_ID)); 
}

void closeGripper(){
  prevDXL_ID = DXL_ID;
  gripperState = 0;
  // //DEBUG_SERIAL.println("closing Gripper");
  DXL_ID = 4;
  DXL_GOAL_POSITION[DXL_ID -1] = -4;
  // //DEBUG_SERIAL.print("Goal position deg: ");
  // //DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
  dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1])); //angle input
  DXL_ID = 5;
  DXL_GOAL_POSITION[DXL_ID -1] = -4;
  // //DEBUG_SERIAL.print("Goal position deg: ");
  // //DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
  dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1])); 
  DXL_ID = prevDXL_ID;
}
void openGripper(){
  prevDXL_ID = DXL_ID;
  gripperState = 1;
  // //DEBUG_SERIAL.println("opening Gripper");
  DXL_ID = 4;
  DXL_GOAL_POSITION[DXL_ID -1] = 30;
  // //DEBUG_SERIAL.print("Goal position deg: ");
  // //DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
  dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1])); //angle input
  DXL_ID = 5;
  DXL_GOAL_POSITION[DXL_ID -1] = 30;
  // //DEBUG_SERIAL.print("Goal position deg: ");
  // //DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
  dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));  
  DXL_ID = prevDXL_ID;
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
          receivedChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars) {
            ndx = numChars - 1;
          }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true; 
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
  serialdata = String(receivedChars);
  
}

void showNewData() {
        DEBUG_SERIAL.print("This just in ... ");
        
        DEBUG_SERIAL.println(serialdata);
        lcd.clear();
        lcd.setCursor(1,1);
        lcd.print(serialdata);
        newData = false;
}

void synchroniseGUI(){
    crustCrawler.set_theta(raw2Deg(dxl.readControlTableItem(PRESENT_POSITION, 1)), 1);
    //DEBUG_SERIAL.println(crustCrawler.get_theta(1));
    crustCrawler.set_theta(raw2Deg(dxl.readControlTableItem(PRESENT_POSITION, 2)), 2);
    //DEBUG_SERIAL.println(crustCrawler.get_theta(2));
    crustCrawler.set_theta(raw2Deg(dxl.readControlTableItem(PRESENT_POSITION, 3)), 3);
    //DEBUG_SERIAL.println(crustCrawler.get_theta(3));
    //forward kinematics
    crustCrawler.forKinem();
    String sendS ="";
    sendS = (String)"<j"+crustCrawler.get_theta(1)+">"; 
    //DEBUG_SERIAL.print(sendS);
    
    sendS = (String)"<x"+crustCrawler.get_Xr()+">";
    //DEBUG_SERIAL.print(sendS);
    sendS = (String)"<z"+crustCrawler.get_Zr()+">";
    //DEBUG_SERIAL.print(sendS);
    sendS = (String)"<g"+gripperState+">";
    //DEBUG_SERIAL.print(sendS);
    //update them in gui
}

void checkStatus(){
  checkStatusNow = true;
}

//stop    
void stopNow(){
  for (int DXL_ID = 1; DXL_ID <=5; DXL_ID++){
    dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, 0);
    dxl.writeControlTableItem(GOAL_PWM , DXL_ID, 0);
  }
  delay(2000);

  for (int DXL_ID = 1; DXL_ID <=5; DXL_ID++){
    dxl.writeControlTableItem(GOAL_CURRENT, DXL_ID, 0);
    dxl.writeControlTableItem(GOAL_PWM , DXL_ID, DXL_GOAL_PWM[DXL_ID-1]);
  }
  // if playing back recorded motion, return to normal mode
  if (recMode == 2){ //playback
    recMode = 0; //normal
  }
}

void setup() {
  
  
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  dxl.scan();

  // //DEBUG_SERIAL.println("INITIALISING POSITION MODE");
  for (DXL_ID = 1; DXL_ID <=5; DXL_ID++){
    dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 0);
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID, DXL_P_GAIN[DXL_ID -1] );
    // //DEBUG_SERIAL.print(DXL_ID);
    // //DEBUG_SERIAL.print(" P gain: ");
    // //DEBUG_SERIAL.println(dxl.readControlTableItem(POSITION_P_GAIN, DXL_ID));
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
    dxl.writeControlTableItem(HOMING_OFFSET, DXL_ID, DXL_HOMING_OFFSET[DXL_ID -1]);
    // Turn on torque
    dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 1);
    // //DEBUG_SERIAL.print(DXL_ID);//);
    // //DEBUG_SERIAL.print(" torque enabled: ");
    // //DEBUG_SERIAL.println(dxl.readControlTableItem(TORQUE_ENABLE, DXL_ID));
    // //DEBUG_SERIAL.print("MIN_POSITION_LIMIT:");
    // //DEBUG_SERIAL.println(dxl.readControlTableItem(MIN_POSITION_LIMIT,DXL_ID));
    // //DEBUG_SERIAL.print("MAX_POSITION_LIMIT:");
    // //DEBUG_SERIAL.println(dxl.readControlTableItem(MAX_POSITION_LIMIT,DXL_ID));
    
  }
  for (DXL_ID = 1; DXL_ID <=3; DXL_ID++){
    crustCrawler.set_theta(raw2Deg(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID)), DXL_ID-1);
  }
  crustCrawler.forKinem();
  crustCrawler.findConfiguration();
  configuration = crustCrawler.get_configuration();
  
    
  DXL_ID =1;
  
  // begin communication with PC
  DEBUG_SERIAL.begin(9600);
  lcd.begin(16,2);
  crustCrawler.begin();
  Serial.println("Arduino is ready");
  lcd.clear();
  lcd.print("Ready to recieve commands...");

  Timer1.initialize(100000); //microsec 100 millis 0.1 sec 10Hz
  Timer1.attachInterrupt(checkStatus);
}


void loop() {
  
//  prevMillis = currentMillis;
//  currentMillis =micros();
//  
//  loopTime= currentMillis - prevMillis;
//  // //DEBUG_SERIAL.println(loopTime);
  
  /*prevDXL_ID = DXL_ID;
  if(millis() % 100 == 0){
   DEBUG_SERIAL.println(":::::::::::::::::::::::::::::");
    for (DXL_ID = 1; DXL_ID <=5; DXL_ID++){
      
      DEBUG_SERIAL.print("Present position of servo ");
      DEBUG_SERIAL.print(DXL_ID);
      DEBUG_SERIAL.print(" --- RAW: ");
      DEBUG_SERIAL.print(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID));
      DEBUG_SERIAL.print(" --- degrees: ");
      DEBUG_SERIAL.println(raw2Deg(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID)));
    }
  }
  DXL_ID = prevDXL_ID; 
  */
  if (checkStatusNow == true){
    /*for (DXL_ID = 1; DXL_ID <=5; DXL_ID++){
      
      DEBUG_SERIAL.print("Present position of servo ");
      DEBUG_SERIAL.print(DXL_ID);
      DEBUG_SERIAL.print(" --- RAW: ");
      DEBUG_SERIAL.print(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID));
      DEBUG_SERIAL.print(" --- degrees: ");
      DEBUG_SERIAL.print(raw2Deg(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID)));
    }
    DEBUG_SERIAL.print("\n");
    */
    for (int DXL_ID = 1; DXL_ID <=3; DXL_ID++){
      prevTheta[DXL_ID] = raw2Deg(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID));
      crustCrawler.set_theta(prevTheta[DXL_ID], DXL_ID-1);
      //prevTheta[DXL_ID] = testCounter; // sends 123456789.... series as a fake position recording
    }
    
    if (recMode == 1){
      //testCounter++;
      DEBUG_SERIAL.print("<");
      ////DEBUG_SERIAL.print("j");
      DEBUG_SERIAL.print(prevTheta[1]);
      DEBUG_SERIAL.print(",");
      ////DEBUG_SERIAL.print("k");
      DEBUG_SERIAL.print(prevTheta[2]);
      DEBUG_SERIAL.print(",");
      ////DEBUG_SERIAL.print("l");
      DEBUG_SERIAL.print(prevTheta[3]);
      DEBUG_SERIAL.print(",");
      DEBUG_SERIAL.print(gripperState);
      DEBUG_SERIAL.print(">");
    }
    ////DEBUG_SERIAL.println("temptemptemptemptemptemptemptemp"); 
    for (int DXL_ID = 1; DXL_ID <=5; DXL_ID++){
      presentTemp[DXL_ID-1] = dxl.readControlTableItem(PRESENT_TEMPERATURE,DXL_ID);
      if (presentTemp[DXL_ID-1] >= 50){
        //DEBUG_SERIAL.print("TEMPERATURE TOO HIGH. STOPPING SERVO ");
        //DEBUG_SERIAL.println(DXL_ID);
        dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, 0);
        dxl.writeControlTableItem(GOAL_CURRENT, DXL_ID, 0);
        dxl.writeControlTableItem(GOAL_PWM , DXL_ID, 0);
      }
    }
    checkStatusNow = false;
  }
 
  recvWithStartEndMarkers();
  
  if (newData == true){
    showNewData(); // includes newData = false;
    if (serialdata.charAt(0) == 's'){
      stopNow();  
    }
    if (recMode == 2){
      if (serialdata.charAt(0) == 'j'){ //<j1k2l3g0>
//        DEBUG_SERIAL.print("serialdata.charAt(0) "); 
//        DEBUG_SERIAL.println(serialdata.charAt(0)); 
//        DEBUG_SERIAL.print("before remove:");
//        DEBUG_SERIAL.println(serialdata);
        serialdata.remove(0,1);
        input[0] = serialdata.toFloat();
//        DEBUG_SERIAL.print("Recieved j angle after conversion to float: ");
//        DEBUG_SERIAL.println(input[0]);
        
        for (int i = 0; i < numChars; i++){
          if(serialdata.charAt(i) == 'k'){
//            DEBUG_SERIAL.print("serialdata.charAt(i) "); 
//            DEBUG_SERIAL.println(serialdata.charAt(i));
//            DEBUG_SERIAL.print("before remove:");
//            DEBUG_SERIAL.println(serialdata);
            serialdata.remove(0,i+1);
            input[1] = serialdata.toFloat();
//            DEBUG_SERIAL.print("Recieved k angle after conversion to float: ");
//            DEBUG_SERIAL.println(input[1]);
          }
        }
        for (int i = 0; i < numChars; i++){
          if(serialdata.charAt(i) == 'l'){
//            DEBUG_SERIAL.print("serialdata.charAt(i) "); 
//            DEBUG_SERIAL.println(serialdata.charAt(i));
//            DEBUG_SERIAL.print("before remove:");
//            DEBUG_SERIAL.println(serialdata);
            serialdata.remove(0,i+1);
            input[2] = serialdata.toFloat();
//            DEBUG_SERIAL.print("Recieved l angle after conversion to float: ");
//            DEBUG_SERIAL.println(input[2]);
          }
        }
        for (int i = 0; i < numChars; i++){
          if(serialdata.charAt(i) == 'g'){
            DEBUG_SERIAL.print("serialdata.charAt(i) "); 
            DEBUG_SERIAL.println(serialdata.charAt(i));
            DEBUG_SERIAL.print("before remove:");
            DEBUG_SERIAL.println(serialdata);
            serialdata.remove(0,i+1);
            gripperState = serialdata.toInt();
            DEBUG_SERIAL.print("Recieved gripper after conv to int: ");
            DEBUG_SERIAL.println(gripperState);
            if (gripperState == 0)
              closeGripper();
            else if (gripperState == 1)
              openGripper();
          }
        }
         
        for (int DXL_ID = 1; DXL_ID <=3; DXL_ID++){
          dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(input[DXL_ID-1]));
        }
      }
      serialdata = "";
    }
    if (recMode == 0 || recMode == 1){
      //recieving x z plane position data
      if (serialdata.charAt(0) == 'x'){
        //// //DEBUG_SERIAL.println("before remove:"+ serialdata);
        serialdata.remove(0,1);
        inputR[0] = serialdata.toFloat();
        //DEBUG_SERIAL.print("GOAL Position Xr after conversion to float: ");
        //DEBUG_SERIAL.println(inputR[0]);
        for (int i = 0; i < numChars; i++){
          if(serialdata.charAt(i) == 'z'){
            serialdata.remove(0,i+1);
            inputR[1] = serialdata.toFloat();
          } 
        }
        for (int i = 0; i < numChars; i++){
          if(serialdata.charAt(i) == 'k'){ // K line Konfiguration
            if (serialdata.charAt(i+1) == '1'){
              configuration = 1;
            }
            else if (serialdata.charAt(i+1) == '0'){
              configuration = 0;
            }
          } 
        }
        //DEBUG_SERIAL.print("GOAL Position Zr after conversion to float: ");
        //DEBUG_SERIAL.println(inputR[1]);
        
        //INVERSE KINEMATICS....
        // read present position
        // determine configuration from it
        // compute IK with desired config
        
        for (int DXL_ID = 1; DXL_ID <=3; DXL_ID++){
          prevTheta[DXL_ID] = raw2Deg(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID));
          crustCrawler.set_theta(prevTheta[DXL_ID], DXL_ID-1);
        }
        //crustCrawler.forKinem();
        //crustCrawler.findConfiguration(); // finds configuration of the prev position
        //crustCrawler. set_Xr(inputR[0]);
        //crustCrawler. set_Zr(inputR[0]);
        //bool reachable = crustCrawler.poseReachableR();
        //if (reachable == 0)
        //  //DEBUG_SERIAL.println("pose not reachable, downscaled. ");
        //crustCrawler.tooClose();
        //DEBUG_SERIAL.print("Up to compute IK with config");
        //DEBUG_SERIAL.print(configuration);
        lcd.clear();
        lcd.print(configuration);
        crustCrawler.invKinemR(inputR[0], inputR[1], prevTheta[2], prevTheta[3] , /*1);//**/configuration/**/);
        
        DXL_GOAL_POSITION[1] = crustCrawler.get_theta(2);
        // apply joint limits
        if (DXL_GOAL_POSITION[1] >= 20){
          DXL_GOAL_POSITION[1] = 20;
          //DEBUG_SERIAL.println("Hitting max angle limit for joint2");
        }
        else if (DXL_GOAL_POSITION[1] <= -190){
          DXL_GOAL_POSITION[1] = -190;
          //DEBUG_SERIAL.println("Hitting min angle limit for joint2");
        }
        DXL_GOAL_POSITION[2] = crustCrawler.get_theta(3);
        if (DXL_GOAL_POSITION[2] >= 110){
          DXL_GOAL_POSITION[2] = 110;
          //DEBUG_SERIAL.println("Hitting max angle limit for joint2");
        }
        else if (DXL_GOAL_POSITION[2] <= -110)
          DXL_GOAL_POSITION[2] = -110;
          //DEBUG_SERIAL.println("Hitting max angle limit for joint3");
              
        //DEBUG_SERIAL.print("Goal position2 deg: ");
        //DEBUG_SERIAL.println(DXL_GOAL_POSITION[1]);
        //DEBUG_SERIAL.print("Goal position3 deg: ");
        //DEBUG_SERIAL.println(DXL_GOAL_POSITION[2]);
        
        prevDXL_ID = DXL_ID;
        //send commands to Dynamixel
        DXL_ID = 2;
        // adjust PWM according to direction
        if (prevTheta[DXL_ID-1] > DXL_GOAL_POSITION[DXL_ID-1]){
          dxl.writeControlTableItem(GOAL_PWM, DXL_ID, DXL_GOAL_PWM[DXL_ID-1] + 50);
          dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID-1]));
        }  
        else if (prevTheta[DXL_ID-1] <= DXL_GOAL_POSITION[DXL_ID-1]){
          dxl.writeControlTableItem(GOAL_PWM, DXL_ID, DXL_GOAL_PWM[DXL_ID-1] - 50);
          dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID-1]));  
        }
        DXL_ID = 3;
        if (prevTheta[DXL_ID-1] > DXL_GOAL_POSITION[DXL_ID-1]){
          dxl.writeControlTableItem(GOAL_PWM, DXL_ID, DXL_GOAL_PWM[DXL_ID-1] + 70);
          dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID-1]));
        }  
        else if (prevTheta[DXL_ID-1] <= DXL_GOAL_POSITION[DXL_ID-1]){
          dxl.writeControlTableItem(GOAL_PWM, DXL_ID, DXL_GOAL_PWM[DXL_ID-1] - 50);
          dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID-1]));  
        }
        DXL_ID = prevDXL_ID;  
        //send commands to Dynamixel
        
        //INVERSE KINEMATICS^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        serialdata = "";
      }
      // recieving data from processing (theta1.00, theta2.00, theta3.00)  example (100.00,020.29,005.2)
      else if (serialdata.charAt(0) == 'j'|| serialdata.charAt(0) == 'k' || serialdata.charAt(0) == 'l'){
        // //DEBUG_SERIAL.println("before remove:"+ serialdata);
        
        if (serialdata.charAt(0) == 'j'){
          DXL_ID = 1;
          if (serialdata.charAt(1) == '?')
            printStatus();
          else{
            serialdata.remove(0,1);
            input[0] = serialdata.toFloat();
            DEBUG_SERIAL.print("After conversion to float: ");
            DEBUG_SERIAL.println(input[0]);
            DXL_GOAL_POSITION[0] = input[0];
            //joint limit
            if (DXL_GOAL_POSITION[0] >= 220){
              DXL_GOAL_POSITION[0] = 220;
              //DEBUG_SERIAL.println("Hitting max angle limit for base joint");
            }
            else if (DXL_GOAL_POSITION[0] <= -220){
              DXL_GOAL_POSITION[0] = -220;
              //DEBUG_SERIAL.println("Hitting min angle limit for base joint");
            }
            lcd.print("j:g"+DXL_GOAL_POSITION[0]);
            dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
          } 
        }
        else if (serialdata.charAt(0) == 'k'){
          DXL_ID = 2;
          if (serialdata.charAt(1) == '?')
            printStatus();
          else{  
            serialdata.remove(0,1);
            input[1] = serialdata.toFloat();
            // //DEBUG_SERIAL.print("After conversion to float: ");
            // //DEBUG_SERIAL.println(input[1]);
            DXL_GOAL_POSITION[1] = input[1];
            //joint limit
            if (DXL_GOAL_POSITION[1] >= 20){
              DXL_GOAL_POSITION[1] = 20;
              //DEBUG_SERIAL.println("Hitting max angle limit for joint1");
              }
            else if (DXL_GOAL_POSITION[1] <= -190){
              DXL_GOAL_POSITION[1] = -190;
              //DEBUG_SERIAL.println("Hitting min angle limit for joint1");
            }
            lcd.print("k:g"+DXL_GOAL_POSITION[1]);
            dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
          }
        }
        else if (serialdata.charAt(0) == 'l'){
          DXL_ID = 3;
          if (serialdata.charAt(1) == '?')
            printStatus();
          else{  
            serialdata.remove(0,1);
            input[2] = serialdata.toFloat();
            // //DEBUG_SERIAL.println(input[2]);
            DXL_GOAL_POSITION[2] = input[2];
            // joint limit
            if (DXL_GOAL_POSITION[2] >= 110){
              DXL_GOAL_POSITION[2] = 110;
              //DEBUG_SERIAL.println("Hitting max angle limit for joint2");
            }
              
            else if (DXL_GOAL_POSITION[2] <= -110){
              DXL_GOAL_POSITION[2] = -110;
              //DEBUG_SERIAL.println("Hitting min angle limit for joint2");
            }
            lcd.print("l:g"+DXL_GOAL_POSITION[2]);
            dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
          }
        }
        // //DEBUG_SERIAL.print("Present position of servo deg: ");
        // //DEBUG_SERIAL.print(DXL_ID);
        // //DEBUG_SERIAL.print(": ");
        // //DEBUG_SERIAL.println( raw2Deg(dxl.readControlTableItem(PRESENT_POSITION, DXL_ID)));
        // //DEBUG_SERIAL.print("Goal position of servo deg (angle limit): ");
        // //DEBUG_SERIAL.print(DXL_ID);
        // //DEBUG_SERIAL.print(": ");
        // //DEBUG_SERIAL.println( raw2Deg(dxl.readControlTableItem(GOAL_POSITION, DXL_ID)));
        serialdata = "";
      }
      //else if (serialdata.charAt(0) == 's'){
        //synchroniseGUI();
  //    &&}
      else if (serialdata.charAt(0) == 'h'){
        dxl.writeControlTableItem(GOAL_POSITION, 2, 0);
        dxl.writeControlTableItem(GOAL_POSITION, 3, 0);
        dxl.writeControlTableItem(GOAL_POSITION, 1, 0);
        closeGripper();
      }
    /*  else if (serialdata.charAt(0) == 't'){
        delay(3000);
        // //DEBUG_SERIAL.println("position_mode_test in 2 secs");
        DXL_ID = 1;
        DXL_GOAL_POSITION[DXL_ID -1] = 90;
        // //DEBUG_SERIAL.print("Goal position deg: ");
        // //DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
        dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1])); //angle input
        openGripper();
        DXL_ID = 2;
        DXL_GOAL_POSITION[DXL_ID -1] = -60;
        // //DEBUG_SERIAL.print("Goal position deg: ");
        // //DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
        dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
        openGripper();
        DXL_ID = 3;
        DXL_GOAL_POSITION[DXL_ID -1] = -30;
        // //DEBUG_SERIAL.print("Goal position deg: ");
        // //DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
        dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
        closeGripper();
        delay(1000);
        DXL_GOAL_POSITION[DXL_ID -1] = 30;
        // //DEBUG_SERIAL.print("Goal position deg: ");
        // //DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
        dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
        delay(1000);
        DXL_GOAL_POSITION[DXL_ID -1] = -30;
        // //DEBUG_SERIAL.print("Goal position deg: ");
        // //DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
        dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
        closeGripper();
        delay(1000);
        DXL_GOAL_POSITION[DXL_ID -1] = 30;
        // //DEBUG_SERIAL.print("Goal position deg: ");
        // //DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
        dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
        openGripper();
        delay(1000);
        DXL_GOAL_POSITION[DXL_ID -1] = -30;
        // //DEBUG_SERIAL.print("Goal position deg: ");
        // //DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
        dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
        closeGripper();
        delay(1000);
        DXL_GOAL_POSITION[DXL_ID -1] = 30;
        // //DEBUG_SERIAL.print("Goal position deg: ");
        // //DEBUG_SERIAL.println(deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]) );
        dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
        for(int i = 0; i<20; i++){
          openGripper();
          delay(500);
          closeGripper();
          delay(500);
        }
        for (DXL_ID = 1; DXL_ID <= 3; DXL_ID++){
          DXL_GOAL_POSITION[DXL_ID -1] = 0;
          dxl.writeControlTableItem(GOAL_POSITION, DXL_ID, deg2Raw(DXL_GOAL_POSITION[DXL_ID -1]));
        }
      }*/
     
      
      else if (serialdata.charAt(0) == 'c'){
        closeGripper();
      }
      else if (serialdata.charAt(0) == 'o'){
        openGripper();
      }
    }
    
    // RECORD MODES
    // n - normal
    if (serialdata.charAt(0) == 'n'){
      recMode = 0;
      DEBUG_SERIAL.print("recMode");
      DEBUG_SERIAL.println(recMode);
    }
    // r - record
    else if (serialdata.charAt(0) == 'r'){
      recMode = 1;
      testCounter = 0;
      DEBUG_SERIAL.print("recMode");
      DEBUG_SERIAL.println(recMode);
    }
    //p - playback
    else if (serialdata.charAt(0) == 'p'){
      recMode = 2;
      DEBUG_SERIAL.print("recMode");
      DEBUG_SERIAL.println(recMode);
    }
    //TEST - send angle, watch servo 3 to obtain new position
    else if (serialdata.charAt(0) == 't'){
      if (serialdata.charAt(1) == 'n'){
        for (DXL_ID = 1; DXL_ID <=5; DXL_ID++){
          dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 1);
        }
      }
      else if (serialdata.charAt(1) == 'f'){
        for (DXL_ID = 1; DXL_ID <=3; DXL_ID++){
          dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 0);
        }
      }
    }
    else if (serialdata.charAt(0) == '?'){
      delay(3000);
      DEBUG_SERIAL.println(":::::::::::::::::::::::::::::::::");
      for (DXL_ID = 1; DXL_ID <=5; DXL_ID++){
        printStatus();
      }  
    }
    serialdata = "";
  }
  serialdata = "";
}
