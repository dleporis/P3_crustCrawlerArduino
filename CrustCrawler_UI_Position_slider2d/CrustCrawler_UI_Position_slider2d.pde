//slider
import controlP5.*;
import processing.serial.*; 

Serial myPort;
ControlP5 cp5;
Slider2D s2;

String serialdata;
int numChars = 32;
boolean recvInProgress = false;
int ndx = 0;
char receivedChars[];
char startMarker = '<';
char endMarker = '>';
char rc;
boolean newData = false;
int xShift = 0;
int myColor = color(0,0,0);
float baseJoint = 0;
float prevBaseJoint;
float joint2 = 0;
float prevJoint2;
float joint3 = 0;
float prevJoint3;
int configuration = 1;
String outputString = "";
color targetCol = color(255, 100, 0); 
int targetX, targetY;
int sendXr, sendZr;
int prevSendXr, prevSendZr;
int sz = 40;
int knobJointX = 700;
int knobJointY = 550;
int knobRadius = 55;
//float baseAngle = 180; //in deg
//float knobX, knobY;
Segment[] segment = new Segment[2];
//Segment baseJoint;
PVector base;


void setup(){
  // Open whatever port is the one you're using.
  String portName = Serial.list()[1]; //change the 4 to a 1 or 2 etc. to match the port your Ardruino is connected to
  myPort = new Serial(this, portName, 9600);   // Change the last number:  "... portName, XXX)  to the baud rate of serial connection
  size( 1200, 685);
  frameRate(30);
  // load knob image
  
  targetX = width/2 - (218+145+120)/2 - 30/2 /*<initial distance from TCP */ + xShift; // 218+145+120 = link 2, link 3, TCP
  targetY = (height/2)-(185/2);
  colorMode(HSB);
  background(100);
  
  

  segment[0] = new Segment(300, 100, 218/2, 0);
  for (int i = 1; i<2; i++){
    segment[i] =new Segment(segment[i-1], (145+120)/2, 0); // 145+120 is link 3 + TCP
  }
  base = new PVector(width/2 + xShift, height/2 - 185/2);
  

  noStroke();
  cp5 = new ControlP5(this);
  //
  cp5.addToggle("gripper")
     .setPosition(40,500)
     .setSize(100,100)
     .setValue(false)
     ;
  cp5.addToggle("configuration")
     .setPosition(40,350)
     .setSize(100,100)
     .setValue(true)
     ;
  cp5.addButton("home_Position")
     .setValue(128)
     .setPosition(1000,500)
     .setSize(100,100)
     .updateSize()
     ;
   cp5.addButton("synchronise")
     .setValue(128)
     .setPosition(1000,350)
     .setSize(100,100)
     .updateSize()
     ;
  cp5.addButton("STOP")
     .setValue(128)
     .setPosition(300,550)
     .setSize(width-600,70)
     .updateSize()
     ;
  
  
  
    //base joint slider
  cp5.addSlider("Base Joint")
     .setPosition(width/4,height/4*3)
     .setSize(600,20)
     .setRange(-220,220) // values can range from big to small as well
     .setNumberOfTickMarks(89)
     .setValue(0)
     .snapToTickMarks(false)
     .setSliderMode(Slider.FLEXIBLE)
     .setScrollSensitivity(0.3)
     ;
     outputString = String.format("<j%s>", nf(baseJoint, 0, 3));
     myPort.write(outputString);
     print("sending J: "); println(outputString);
  
  cp5.addSlider("Joint2")
     .setPosition(width/4+15,height/4*3-30)
     .setSize(600,20)
     .setRange(-190,20) // values can range from big to small as well
     .setNumberOfTickMarks(43)
     .setValue(0)
     .snapToTickMarks(false)
     .setSliderMode(Slider.FLEXIBLE)
     .setScrollSensitivity(0.3)
     ;
     outputString = String.format("<k%s>", nf(joint2, 0, 3));
     myPort.write(outputString);
     print("sending K: "); println(outputString);
     
  cp5.addSlider("Joint3")
     .setPosition(width/4,height/4*3-60)
     .setSize(600,20)
     .setRange(-110,110) // values can range from big to small as well
     .setNumberOfTickMarks(45)
     .setValue(0)
     .snapToTickMarks(false)
     .setSliderMode(Slider.FLEXIBLE)
     .setScrollSensitivity(0.3)
     ;
     outputString = String.format("<l%s>", nf(joint3, 0, 3));
     myPort.write(outputString);
     print("sending K: "); println(outputString);
  
  /*cp5 = new ControlP5(this);
  s2 = cp5.addSlider2D("XrZ Plane")
     .setPosition(0,0)
     .setSize(1000/2,686/2)
     .setMinMax(-500, 0, 500, 685)// values can range from big to small as well
     .setValue(0,685)
     ;
  println("Xr:"+ s2.getArrayValue()[0]);
  println("Z:"+ s2.getArrayValue()[1]);
  //*/
  
//  public void controlEvent(ControlEvent theEvent) {
//  println(theEvent.getController().getName());
//  n = 0;
//}
// function colorC will receive changes from 
// controller with name colorC


  
}
void draw(){
  //draw background and axes
  background(100);
  stroke(255);
  strokeWeight(1);
  line(0, height/2, width, height/2);
  line(width/2 + xShift, 0, width/2 + xShift, height);
  
  //base joint and joint3 zero indicator
  strokeWeight(3);
  line(width/2 + xShift, height/4*3-70, width/2 + xShift, height/4*3+30);
  //joint2 zero indicator
  line(width/4*3 + xShift -48, height/4*3-50, width/4*3 + xShift-48, height/4*3);
  
  //draw FAT BASE
  strokeWeight(16);
  line(width/2 + xShift, height/2, width/2 + xShift, height/2 - 185/2);
  
  
  
  // TARGET POINT DRAG
  fill(targetCol);
  if(dist(targetX,targetY,mouseX, mouseY) < sz/2){
    stroke(255);// inside the target circle
    cursor(HAND);
    if (mousePressed){
      strokeWeight(5);
    }
    else{
      strokeWeight(2);
      }
  }
  else {
    noStroke();
    cursor(ARROW);
  }
  ellipse(targetX,targetY, sz, sz);
  
  // TARGET POINT DRAG
  
  
  int total = segment.length;
  Segment end = segment[total-1];
  
  end.follow(targetX, targetY);
  end.update();
  
  for (int i = total-2; i>= 0; i--){
    segment[i].follow(segment[i+1]);
    segment[i].update();
  }
  segment[0].setA(base);
  
  for(int i = 1; i<total; i++){
  segment[i].setA(segment[i-1].b);
  }
  
  for(int i = 0; i<total; i++){
  segment[i].show(); 
  }
  
  // screen x ----->  -1* Xr of robot. 1 pixel = 2mm
  //print("targetX " + targetX + " corresponds to: ");
  sendXr = (width/2 - targetX) * 2; 
  //println(sendXr + " sendXr.");
  //SEND screen y ----->  -1* Zr of robot. 1 pixel = 2mm
  //print("targetY " + targetY + " corresponds to: ");
  sendZr = (height/2 - targetY) * 2;
  
  // sending Xr Zr
  if(sendXr != prevSendXr || sendZr != prevSendZr){
    outputString = String.format("<x%sz%sc%s>", nf(sendXr, 0, 1), nf(sendZr, 0, 1), configuration);
    myPort.write(outputString);
    print("sending Xr, Zr: "); println(outputString); 
    
  }
  prevSendXr = sendXr;
  prevSendZr = sendZr;
  
  
  // base joint slider
  baseJoint = cp5.getValue("Base Joint");
  if(baseJoint != prevBaseJoint){
    outputString = String.format("<j%s>", nf(baseJoint, 0, 3));
    myPort.write(outputString);
    print("sending J: "); println(outputString);
  }
  prevBaseJoint = baseJoint;
  
  //joint2 slider
  joint2 = cp5.getValue("Joint2");
  if(joint2 != prevJoint2){
    outputString = String.format("<k%s>", nf(joint2, 0, 3));
    myPort.write(outputString);
    print("sending K: "); println(outputString);
  }
  prevJoint2 =joint2;
  
  // joint3 slider
  joint3 = cp5.getValue("Joint3");
  if(joint3 != prevJoint3){
    outputString = String.format("<l%s>", nf(joint3, 0, 3));
    myPort.write(outputString);
    print("sending L: "); println(outputString);
  }
  prevJoint3 = joint3;
}

////////////////////////////////////////////////////////////////////////////////////////
void mouseDragged(){
  if (mouseY >= height/2){
  }  
  else{
  targetX = mouseX;
  targetY = mouseY;
  // calculate Xr and Z from targetX targetY
  // send them to arduino
  }
}


public void gripper(int theValue){
  if (theValue == 1){
    println("openning gripper");
    myPort.write("<o>");
  }
  if (theValue == 0){
    println("closing gripper");
    myPort.write("<c>");
  }
} 
public void configuration(int theValue){
  if (theValue == 1){
    println("configuration 1");
    configuration  = 1;
  }
  if (theValue == 0){
    println("configuration 0");
    configuration  = 0;
  }
} 
public void home_Position(int theValue){
  if (theValue == 128){
    println("Home position...");
    myPort.write("<h>");
    cp5.addSlider("Base Joint")
     .setPosition(width/4,height/4*3)
     .setSize(600,20)
     .setRange(-220,220) // values can range from big to small as well
     .setNumberOfTickMarks(89)
     .setValue(0)
     .snapToTickMarks(false)
     .setSliderMode(Slider.FLEXIBLE)
     .setScrollSensitivity(0.3)
     ;
  cp5.addSlider("Joint2")
     .setPosition(width/4+15,height/4*3-30)
     .setSize(600,20)
     .setRange(-190,20) // values can range from big to small as well
     .setNumberOfTickMarks(43)
     .setValue(0)
     .snapToTickMarks(false)
     .setSliderMode(Slider.FLEXIBLE)
     .setScrollSensitivity(0.3)
     ;
   cp5.addSlider("Joint3")
     .setPosition(width/4,height/4*3-60)
     .setSize(600,20)
     .setRange(-110,110) // values can range from big to small as well
     .setNumberOfTickMarks(45)
     .setValue(0)
     .snapToTickMarks(false)
     .setSliderMode(Slider.FLEXIBLE)
     .setScrollSensitivity(0.3)
     ;
   targetX = width/2 - (218+145+120)/2 - 30/2 /*<initial distance from TCP */ + xShift; // 218+145+120 = link 2, link 3, TCP
   targetY = (height/2)-(185/2);
}
} 

void recvWithStartEndMarkers() {
  while (myPort.available() > 0 && newData == false) {
      rc = myPort.readChar();
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
  serialdata = new String(receivedChars);
}

public void STOP() {
  myPort.write("< >");
  println("STOP!!! ");
  println("< >");
}  

public void synchronise(int theValue) {
  if (theValue == 128){
    println("Synchronysing with robot... ");
    println("<s>");
    myPort.write("<s>");
    syncListen();
  }  
}

void syncListen(){
   float putFloat;
    int putInt;
    int i = 0;
  while(i < 4){
    recvWithStartEndMarkers();
    if (serialdata.charAt(0) == 'j'){
      putFloat = parseFloat(serialdata.substring(1));
      println(putFloat);
     cp5.addSlider("Base Joint")
     .setPosition(width/4,height/4*3)
     .setSize(600,20)
     .setRange(-220,220) // values can range from big to small as well
     .setNumberOfTickMarks(89)
     .setValue(putFloat)
     .snapToTickMarks(false)
     .setSliderMode(Slider.FLEXIBLE)
     .setScrollSensitivity(0.3)
     ;
      i++;
    }
    if (serialdata.charAt(0) == 'x'){
     
     putInt = parseInt(serialdata.substring(1));
     targetX = putInt;
     // cp5.addSlider("Base Joint")
     //.setPosition(width/4,height/4*3)
     //.setSize(600,20)
     //.setRange(-220,220) // values can range from big to small as well
     //.setNumberOfTickMarks(89)
     //.setValue(putIn)
     //.snapToTickMarks(false)
     //.setSliderMode(Slider.FLEXIBLE)
     //.setScrollSensitivity(0.3)
     //;
      i++;
    }
    if (serialdata.charAt(0) == 'z'){
      putInt = parseInt(serialdata.substring(1));
      targetY = putInt;
      i++;
    }
    if (serialdata.charAt(0) == 'g'){
      putInt = parseInt(serialdata.substring(1));
      targetX = putInt;
      i++;
    }
  }
}

  
void keyReleased(){
  if (key == ' '){
    myPort.write("< >");
    println("STOP!!!");
    println("< >");
  }
  if (key == 'o'||key =='O'){
    myPort.write("<o>");
    println("opening gripper");
    println("<o>");
  }
  if (key == 'c'||key =='c'){
    myPort.write("<c>");
    println("closing gripper");
    println("<c>");
  }
}
