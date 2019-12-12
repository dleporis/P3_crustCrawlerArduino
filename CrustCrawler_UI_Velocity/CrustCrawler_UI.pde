// program for 1-at-time control of crustrawler Dynamixel servos with PC keyboard and simple UI
import processing.serial.*;     // Serial connection. Default does not need to be installed

//Serial Communication
Serial myPort;  // Create object from Serial class
String serialData;     
//UI variables
PImage imgOpen;
PImage imgClosed;
int x, y, xServo, yServo;
boolean [] keys = new boolean[128];
int gripperState; // false is closed, true is open
int activeServoID;
int angVelGear; // -3, -2, -1, 0, 1, 2, 3
color servoCol = color(255, 204, 0); // yellow for servo indicator
color arrowsCol = color(255, 0, 0); // red for arrows indicator

//------------------------Functions------------------------------------------------

String readSerialData(int dly){
  String  RawSerialdata = myPort.readStringUntil('\n');
  if (RawSerialdata != null) {  // if the data is not empty... 
    //trim whitespace and formatting characters (like carriage return)
    String serialData = trim(RawSerialdata);
    println(serialData);
    return serialData; 
  } else {
    //println ("No serial available");
    delay(dly);
    return " ";
  }
}

//------------------------ Void Setup ----------------------------------------------
void setup()
{
  // Setup Serial Comm -----------------------------------------
  // List all the available serial ports. 
  printArray(Serial.list());
  // Open whatever port is the one you're using.
  String portName = Serial.list()[1]; //change the 4 to a 1 or 2 etc. to match the port your Ardruino is connected to
  myPort = new Serial(this, portName,9600);   // Change the last number:  "... portName, XXX)  to the baud rate of serial connection.
  myPort.bufferUntil('\n');
  
  //Setup UI
  size (500, 400);
  x = width/2; y = height/2;
  imgOpen = loadImage("CrustCrawlerOpen.jpg");
  imgClosed = loadImage("CrustCrawlerClosed.jpg");
  
  //setup robot
  gripperState = 0; // 0 is closed, 1 is open
  myPort.write("c\n");
  activeServoID = 1;
  myPort.write("i1\n"); // Processing writes i1, i2, i3, Arduino writes I1, I2, I3
  angVelGear = 0;
  myPort.write("v0\n"); 
}

//------------------------ Void  draw (== to void loop in Ard) ---------------------------------------
void draw() {
  //serialData = readSerialData(0); // check if arduino actually changed 
  // UI drawing -----------------------------------------------
  background(0);
  switch (gripperState){
    case 0:
      image(imgClosed, 0, 0);
      break;
    case 1:
      image(imgOpen, 0, 0);
      break;
  }
  //draw active servo indicator
  fill(servoCol);  // Use color variable 'c' as fill color
  stroke(255);  // Don't draw a stroke around shapes
  switch(activeServoID){
    case 1:
      xServo = 382; yServo = 315;
      break;
    case 2:
      xServo = 382; yServo = 250;
      break;
    case 3:
      xServo = 250; yServo = 130;
      break;
  }
  ellipse(xServo, yServo, 30, 30);
  fill(arrowsCol);
  ellipse(x, y, 20, 20);
} //end of draw

// what happens if I press key?
void keyReleased(){
  int prevAngVelGear = angVelGear;
  int prevActiveServoID = activeServoID;
  int prevGripperState = gripperState;
  print("Pressed key: ");
  print(key);
  println( "------------------------");
  if (key == 'a' || key == 'A'){
    x = x-5;
    angVelGear = angVelGear - 1;
    if (angVelGear < -3)
      angVelGear = -3;
    println("P: Velocity of servo " + activeServoID + ": " + angVelGear);
  }
  else if (key == 'd' || key == 'D'){
    x = x+5;
    angVelGear = angVelGear + 1;
    if (angVelGear > 3)
      angVelGear = 3;
    println("P: Velocity of servo " + activeServoID + ": " + angVelGear);
  }
  else if (key == 'w'|| key == 'W'){
    y = y-5;
    if (angVelGear == 0 ){
      activeServoID = activeServoID +1;
      if (activeServoID > 3)
        activeServoID = 1;
      println("P: Swiching to servo " + activeServoID);
    }
    else{ 
      angVelGear = 0; 
      println("P: Velocity of servo " + activeServoID + ": " + angVelGear);
    }
  }
  else if (key == 's' || key == 'S'){
    y = y+5;
    if (angVelGear == 0 ){
      activeServoID = activeServoID -1;
      if (activeServoID < 1)
        activeServoID = 3;
      println("P: Swiching to servo " + activeServoID);
    }
    else {
      angVelGear = 0;
      println("P: Velocity of servo " + activeServoID + ": " + angVelGear);
    }
  }
  else if (key == 'c' || key == 'C'){
    gripperState = 0;
    if (gripperState != prevGripperState){
      myPort.write("c\n");
      println("P: Gripper closed");
    }
  }
  else if (key == 'o'|| key == 'O'){
    gripperState = 1;
    if (gripperState != prevGripperState){
      myPort.write("o\n");
      println("P: Gripper open");
    } 
  }
  if (activeServoID != prevActiveServoID){
    myPort.write(String.format("i%s\n", activeServoID));
    println(String.format("i%s", activeServoID));
  }   
  else if(angVelGear != prevAngVelGear){
    myPort.write(String.format("v%s\n", angVelGear));
    println(String.format("v%s", angVelGear));
  }
}
