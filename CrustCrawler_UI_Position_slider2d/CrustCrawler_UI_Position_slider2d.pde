// source code handling GUI, that controls Arduino and CrustCrawler robot

//https://processing.org/reference/libraries/io/GPIO_attachInterrupt_.html

//slider
import controlP5.*;
import processing.serial.*; 
import java.io.*;
//import processing.io.*;
import java.util.Timer;
import java.util.TimerTask;

Serial myPort;
ControlP5 cp5;
Slider2D s2;


public Timer myTimer = new Timer();
public TimerTask task = new TimerTask() {
  public void run() {
    if (recMode == 2)
      playback();
  }
};

float sliderButtonIncr = 0.2;
float newSliderVal;
//recieve variables
String serialdata;
int numChars = 32;
boolean newData = false;
boolean recvInProgress = false;
char receivedChars[] = new char[numChars]; 

char startMarker = '<';
char endMarker = '>';
int ndx = 0; //index
char rc; //recieved
// record
int recMode = 0; //0 not recording, 1 recording, 2 playback
boolean displayPauseText = false;
int csvLineCounter = 0;
Table table;
TableRow row;
int rowCount;
int columnCount;
//GUI
PFont f; //Declare PFont variable
int xShift = 0;
int myColor = color(0, 0, 0);
color targetCol = color(255, 100, 0);
Segment[] segment = new Segment[2];
PVector base;
//joint values
float baseJoint = 0;
float prevBaseJoint;
float joint2 = 0;
float prevJoint2;
float joint3 = 0;
float prevJoint3;
int configuration = 1;
//output 
int targetX, targetY;
int sz = 40; // target size
int sendXr, sendZr;
int prevSendXr, prevSendZr;
String outputString = "";
int prevMillis;
int gripperState = 0;


void setup() {
  // Open whatever port is the one you're using.
  String portName = Serial.list()[1]; //change the 4 to a 1 or 2 etc. to match the port your Ardruino is connected to
  myPort = new Serial(this, portName, 9600);   // Change the last number:  "... portName, XXX)  to the baud rate of serial connection
  myPort.bufferUntil('\n');
  size( 1200, 685);
  background(100);
  frameRate(60);

  myTimer.scheduleAtFixedRate(task, 0, 100);
  println("timer scheduled at fixed rate");
  table = new Table();
  rowCount = table.getRowCount();  
  println("Table rowCount: " + rowCount);
  columnCount = table.getColumnCount();
  println("Table columnCount: " + columnCount);
  table.addColumn("j", Table.STRING);
  table.addColumn("k", Table.STRING);
  table.addColumn("l", Table.STRING);
  table.addColumn("g", Table.STRING);
  columnCount = table.getColumnCount();
  println("Table columnCount: " + columnCount);

  targetX = width/2 - (218+145+120)/2 - 30/2 /*<initial distance from TCP */ + xShift; // 218+145+120 = link 2, link 3, TCP
  targetY = (height/2)-(185/2);
  colorMode(HSB);


  segment[0] = new Segment(300, 100, 218/2, 0);
  for (int i = 1; i<2; i++) {
    segment[i] =new Segment(segment[i-1], (145+120)/2, 0); // 145+120 is link 3 + TCP
  }
  base = new PVector(width/2 + xShift, height/2 - 185/2);

  f = createFont("Arial", 12, true); // STEP 2 Create Font
  PImage[] imgs = {loadImage("STOP_BUTTON_a.png"), loadImage("STOP_BUTTON_b.png"), loadImage("STOP_BUTTON_c.png")};

  noStroke();
  cp5 = new ControlP5(this);
  //
  cp5.addToggle("torque_on_off")
    .setValue(true)
    .setPosition(1000, 350)
    .setSize(30, 30)
    .updateSize()
    ;
  cp5.addToggle("gripper")
    .setPosition(40, 500)
    .setSize(100, 100)
    .setValue(false)
    ;
  cp5.addToggle("configuration")
    .setPosition(40, 350)
    .setSize(100, 100)
    .setValue(true)
    ;
  cp5.addButton("home_Position")
    .setValue(128)
    .setPosition(1000, 500)
    .setSize(100, 100)
    .updateSize()
    ;
  cp5.addToggle("record")
    .setValue(false)
    .setPosition(1000-260, 350)
    .setSize(50, 50)
    .updateSize()
    ;
  cp5.addToggle("playback_pause")
    .setValue(false)
    .setPosition(1000-200, 350)
    .setSize(50, 50)
    .updateSize()
    ;
  cp5.addButton("STOP")
    .setValue(128)
    .setPosition(300, 550)
    .setSize(width-600, 70)
    .setImages(imgs)
    .updateSize()
    ;

  // joint "ARROW" buttons
  cp5.addButton("B+")
    .setValue(128)
    .setPosition(width/4*3+50, height/4*3)
    .setSize(40, 20)
    .updateSize()
    ;
  cp5.addButton("B-")
    .setValue(128)
    .setPosition(width/4-50, height/4*3)
    .setSize(40, 20)
    .updateSize()
    ;

  cp5.addButton("2+")
    .setValue(128)
    .setPosition(width/4*3+50, height/4*3-30)
    .setSize(40, 20)
    .updateSize()
    ;
  cp5.addButton("2-")
    .setValue(128)
    .setPosition(width/4-50, height/4*3-30)
    .setSize(40, 20)
    .updateSize()
    ;

  cp5.addButton("3+")
    .setValue(128)
    .setPosition(width/4*3+50, height/4*3-60)
    .setSize(40, 20)
    .updateSize()
    ;
  cp5.addButton("3-")
    .setValue(128)
    .setPosition(width/4-50, height/4*3-60)
    .setSize(40, 20)
    .updateSize()
    ;
  cp5.addButton("UP")
    .setValue(128)
    .setPosition(width/3, height/2+9)
    .setSize(40, 40)
    .updateSize()
    ;
  cp5.addButton("DOWN")
    .setValue(128)
    .setPosition(width/3, height/2+50)
    .setSize(40, 40)
    .updateSize()
    ;
  cp5.addButton("RIGHT")
    .setValue(128)
    .setPosition(width/3+41, height/2+50)
    .setSize(40, 40)
    .updateSize()
    ;
  cp5.addButton("LEFT")
    .setValue(128)
    .setPosition(width/3-41, height/2+50)
    .setSize(40, 40)
    .updateSize()
    ;

  cp5.addSlider("Speed (no effect on sliders and orange target)")
    .setPosition(width/2, height/4*3-100)
    .setSize(150, 20)
    .setRange(0.05, 5) // values can range from big to small as well
    .setNumberOfTickMarks(10)
    .setValue(0)
    .snapToTickMarks(false)
    .setSliderMode(Slider.FLEXIBLE)
    .setScrollSensitivity(0.3)
    ;
  //base joint slider
  cp5.addSlider("Base Joint")
    .setPosition(width/4, height/4*3)
    .setSize(600, 20)
    .setRange(-220, 220) // values can range from big to small as well
    .setNumberOfTickMarks(89)
    .setValue(0)
    .snapToTickMarks(false)
    .setSliderMode(Slider.FLEXIBLE)
    .setScrollSensitivity(0.3)
    ;
  outputString = String.format("<j%s>", nf(baseJoint, 0, 3));
  myPort.write(outputString);
  print("sending J: "); 
  println(outputString);

  cp5.addSlider("Joint2")
    .setPosition(width/4+15, height/4*3-30)
    .setSize(600, 20)
    .setRange(-190, 20) // values can range from big to small as well
    .setNumberOfTickMarks(43)
    .setValue(0)
    .snapToTickMarks(false)
    .setSliderMode(Slider.FLEXIBLE)
    .setScrollSensitivity(0.3)
    ;
  outputString = String.format("<k%s>", nf(joint2, 0, 3));
  myPort.write(outputString);
  print("sending K: "); 
  println(outputString);

  cp5.addSlider("Joint3")
    .setPosition(width/4, height/4*3-60)
    .setSize(600, 20)
    .setRange(-110, 110) // values can range from big to small as well
    .setNumberOfTickMarks(45)
    .setValue(0)
    .snapToTickMarks(false)
    .setSliderMode(Slider.FLEXIBLE)
    .setScrollSensitivity(0.3)
    ;
  outputString = String.format("<l%s>", nf(joint3, 0, 3));
  myPort.write(outputString);
  print("sending L: "); 
  println(outputString);

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


void draw() {
  //print(".");
  background(100);
  //recording
  if (displayPauseText == true) {
    textFont(f, 16);                  // STEP 3 Specify font to be used
    fill(0);                         // STEP 4 Specify font color 
    text("PLAYBACK\nPAUSED", 1000-120, height/2+40);   // STEP 5 Display Text
  } else if (displayPauseText == false) {
    background(100);
  }
  if (recMode == 1) {// recording
  
    recvWithStartEndMarkers();
    if (newData == true){
      showNewData(serialdata); // includes newData = false;
      saveToCSV(serialdata);
      serialdata ="";
    }
    
    
  }

  //draw and axes
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

  if (mousePressed == true) {
    sliderButtonIncr = cp5.getValue("Speed (no effect on sliders and orange target)");

    if (cp5.getController("B+").isMousePressed()  == true) {
      newSliderVal = cp5.getController("Base Joint").getValue()+sliderButtonIncr;
      cp5.getController("Base Joint").setValue(newSliderVal);
    } else if (cp5.getController("B-").isMousePressed()  == true) {
      newSliderVal = cp5.getController("Base Joint").getValue() -sliderButtonIncr;
      cp5.getController("Base Joint").setValue(newSliderVal);
    } else if (cp5.getController("2+").isMousePressed()  == true) {
      newSliderVal = cp5.getController("Joint2").getValue()+sliderButtonIncr;
      cp5.getController("Joint2").setValue(newSliderVal);
    } else if (cp5.getController("2-").isMousePressed()  == true) {
      newSliderVal = cp5.getController("Joint2").getValue() -sliderButtonIncr;
      cp5.getController("Joint2").setValue(newSliderVal);
    } else if (cp5.getController("3+").isMousePressed()  == true) {
      newSliderVal = cp5.getController("Joint3").getValue()+sliderButtonIncr;
      cp5.getController("Joint3").setValue(newSliderVal);
    } else if (cp5.getController("3-").isMousePressed()  == true) {
      newSliderVal = cp5.getController("Joint3").getValue() -sliderButtonIncr;
      cp5.getController("Joint3").setValue(newSliderVal);
    } else {
      if (sliderButtonIncr < 1) {
        sliderButtonIncr = 1;
      }
      if (cp5.getController("UP").isMousePressed()  == true) {
        targetY = targetY-int(sliderButtonIncr);
      } else if (cp5.getController("DOWN").isMousePressed()  == true) {
        targetY = targetY+int(sliderButtonIncr);
      } else if (cp5.getController("RIGHT").isMousePressed()  == true) {
        targetX = targetX+int(sliderButtonIncr);
      } else if (cp5.getController("LEFT").isMousePressed()  == true) {
        targetX = targetX-int(sliderButtonIncr);
      }
      if (targetY > height/2) {
        targetY = height/2;
      }
    }
  }

  // TARGET POINT DRAG
  fill(targetCol);
  if (dist(targetX, targetY, mouseX, mouseY) < sz/2) {
    stroke(255);// inside the target circle
    cursor(HAND);
    if (mousePressed) {
      strokeWeight(5);
    } else {
      strokeWeight(2);
    }
  } else {
    noStroke();
    cursor(ARROW);
  }
  ellipse(targetX, targetY, sz, sz);

  // TARGET POINT DRAG


  int total = segment.length;
  Segment end = segment[total-1];

  end.follow(targetX, targetY);
  end.update();

  for (int i = total-2; i>= 0; i--) {
    segment[i].follow(segment[i+1]);
    segment[i].update();
  }
  segment[0].setA(base);

  for (int i = 1; i<total; i++) {
    segment[i].setA(segment[i-1].b);
  }

  for (int i = 0; i<total; i++) {
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
  if (sendXr != prevSendXr || sendZr != prevSendZr) {
    outputString = String.format("<x%sz%sk%s>", nf(sendXr, 0, 1), nf(sendZr, 0, 1), configuration);
    myPort.write(outputString);
    print("sending Xr, Zr: "); 
    println(outputString);
  }
  prevSendXr = sendXr;
  prevSendZr = sendZr;



  baseJoint = cp5.getValue("Base Joint");
  if (baseJoint != prevBaseJoint) {
    //  cp5.getController("Base Joint").setValue(baseJoint); // put to arrow button

    outputString = String.format("<j%s>", nf(baseJoint, 0, 3));
    myPort.write(outputString);
    print("sending J: "); 
    println(outputString);
  }
  prevBaseJoint = baseJoint;

  //joint2 slider
  joint2 = cp5.getValue("Joint2");
  if (joint2 != prevJoint2) {
    outputString = String.format("<k%s>", nf(joint2, 0, 3));
    myPort.write(outputString);
    print("sending K: "); 
    println(outputString);
  }
  prevJoint2 =joint2;

  // joint3 slider
  joint3 = cp5.getValue("Joint3");
  if (joint3 != prevJoint3) {
    outputString = String.format("<l%s>", nf(joint3, 0, 3));
    myPort.write(outputString);
    print("sending L: "); 
    println(outputString);
  }
  prevJoint3 = joint3;
}

////////////////////////////////////////////////////////////////////////////////////////
void Button(float theValue) {
  println("got a button press", theValue);
}

void mouseDragged() {
  if (mouseY >= height/2) {
  } else {
    targetX = mouseX;
    targetY = mouseY;
    // calculate Xr and Z from targetX targetY
    // send them to arduino
  }
}


public void gripper(int theValue) {
  if (theValue == 1) {
    println("openning gripper");
    myPort.write("<o>");
  }
  if (theValue == 0) {
    println("closing gripper");
    myPort.write("<c>");
  }
} 
public void configuration(int theValue) {
  if (theValue == 1) {
    println("configuration 1");
    configuration  = 1;
  }
  if (theValue == 0) {
    println("configuration 0");
    configuration  = 0;
  }
} 
public void home_Position(int theValue) {
  if (theValue == 128) {
    println("Home position...");
    myPort.write("<h>");
    cp5.getController("Base Joint").setValue(0);
    cp5.getController("Joint2").setValue(0);
    cp5.getController("Joint3").setValue(0);
    targetX = width/2 - (218+145+120)/2 - 30/2 /*<initial distance from TCP */ + xShift; // 218+145+120 = link 2, link 3, TCP
    targetY = (height/2)-(185/2);
  }
} 
public void STOP() {
  myPort.write("<s>");
  println("STOP!!!");
  println("<s>");
  if (recMode == 2) { //playback
    recMode = 0; //normal
    print("recMode is "); 
    println(recMode);
    csvLineCounter = 0;
    cp5.getController("playback_pause").setValue(0);
  }
  else{
    print("recMode is "); 
    println(recMode);
  }
  if (displayPauseText == true) {
    displayPauseText = false;
    csvLineCounter=0;
  }
}  
public void record(int theValue) {
  if (theValue == 1) {
    if (recMode == 0) {
      recMode = 1; //0 not recording, 1 recording, 2 playback
      table = new Table();
      rowCount = table.getRowCount();  
      println("Table rowCount: " + rowCount);
      columnCount = table.getColumnCount();
      println("Table columnCount: " + columnCount);
      table.addColumn("j", Table.STRING);
      table.addColumn("k", Table.STRING);
      table.addColumn("l", Table.STRING);
      table.addColumn("g", Table.STRING);
      println("Table columnCount: " + columnCount);
      columnCount = table.getColumnCount();
      println("Recording the movement");
      myPort.write("<r>");
      displayPauseText = false;
    } 
    else if (recMode == 1) {
      println("Already recording");
    } 
    else if (recMode == 2) {
      println("Cannot record during playback, Press 'STOP' or 'pause' in order to enable record.");
      cp5.getController("record").setValue(0);
      return;
    }
  }
  if (theValue == 0) {
    if (recMode == 0) {
      recMode = 0; //0 not recording, 1 recording, 2 playback
      println("not recording anyway.");
    } else if (recMode == 1) {
      recMode = 0; //0 not recording, 1 recording, 2 playback
      println("Recording stopped"); 
      rowCount = table.getRowCount();
      print("table length is "); print(rowCount); println(" lines");
      saveTable(table, "data/Position_log.csv", "csv");
      myPort.write("<n>"); //n -normal
    }
    //else if (recMode == 2){
    //  println("Mistake within GUI, but playback continues.");
    //}
  }
  print("recMode is "); 
  println(recMode);
} 
public void torque_on_off(int theValue){
  if (theValue == 1) {
    println("torque on");
    myPort.write("<tn>");
  }
  if (theValue == 0) {
    println("torque off");
    myPort.write("<tf>");
  }
} 
  
public void playback_pause(int theValue) {
  if (theValue == 1) {
    if (recMode == 0) {
      recMode = 2; //0 not recording, 1 recording, 2 playback
      println("Motion playback started.");
      myPort.write("<p>"); // p-playback
      if (displayPauseText == false){
        table = loadTable ("data/Position_log.csv", "header");
        rowCount = table.getRowCount();
        println("Table rowCount: " + rowCount);
        columnCount = table.getColumnCount();
        println("Table columnCount: " + columnCount);
      }
      else{
        displayPauseText = false;
      }
    } 
    else if (recMode == 1) {
      println("Playback is not possible during recording. Press 'record' toggle to stop recording.");
      cp5.getController("playback_pause").setValue(0);
    } 
    else if (recMode == 2) {
      println("Playback already running.");
    }
  }
  if (theValue == 0) {
    if (recMode == 0) {
      //  recMode = 0; //0 not recording, 1 recording, 2 playback
      //  println("Playbeck not running anyway.");
    } else if (recMode == 1) {
      println("Recording continues. Press 'record' toggle to stop recording.");
    } else if (recMode == 2) {
      recMode = 0; //0 not recording/normal, 1 recording, 2 playback
      println("Motion playback paused, Press 'playback' button to continue.");
      myPort.write("<n>"); //n - normal 
      displayPauseText = true;
      //remember line in .csv
    }
  }
  print("recMode is "); 
  println(recMode);
  // save the present csv line
}

public void saveToCSV(String inputString) {
  println("inputString "+ inputString+">");
  table.addRow();
  row = table.getRow(csvLineCounter);
  String[] tableValues = split(inputString, ','); 

  int arrayLength = tableValues.length;
  //println("arrayLength: "+arrayLength);
  if (arrayLength < 4) {
    row.setString("j", "problem");
    println("receiving "+row.getString("j"));
  } else {
    row.setString("j", tableValues[0]);
    row.setString("k", tableValues[1]);
    row.setString("l", tableValues[2]);
    println("tableValues[3] "+tableValues[3]);
    println("int(tableValues[3]) "+int(tableValues[3]));
    row.setString("g", tableValues[3]);
    println("recorded positions: "+row.getString("j")+" | "+row.getString("k")+" | "+row.getString("k")+" | gripper: "+row.getString("g"));
  }
  csvLineCounter++;
}

void playback() {
  //rowCount = table.getRowCount();
  row = table.getRow(csvLineCounter);
  if (csvLineCounter == rowCount) {
    csvLineCounter = 0;
    row = table.getRow(csvLineCounter);
    while (row.getString("j") == "problem"){
      csvLineCounter++;
      row = table.getRow(csvLineCounter);
    }
    outputString = String.format("<j%sk%sl%s>", row.getString("j"), row.getString("k"), row.getString("l"));
    myPort.write(outputString);
    print("sending jkl: "); 
    println(outputString); 
    println("End of motion playback. Press 'playback' toggle to repeat.");
    print("table length was "); 
    print(rowCount); 
    println(" lines");
    recMode = 0;
    cp5.getController("playback_pause").setValue(0);
    csvLineCounter = 0;
  } 
  else if (displayPauseText == true) {
    println("paused");
  } 
  else {
    if (row.getString("j") == "problem") {
      csvLineCounter++;
    } else {
      outputString = String.format("<j%sk%sl%sg%s>", row.getString("j"), row.getString("k"), row.getString("l"), row.getString("g"));
      myPort.write(outputString);
      print("sending jkl: "); 
      println(outputString); 
      if (row.getString("g") == "1" && gripperState == 0 ) {
        myPort.write("<o>");
        gripperState = 1;
        cp5.getController("gripper").setValue(128);
      } else if (row.getString("g") == "1" && gripperState == 1 ) {
        myPort.write("<c>");
        gripperState = 0;
        cp5.getController("gripper").setValue(0);
      }
      //print(millis()-prevMillis);
      //prevMillis = millis();
      csvLineCounter++;
    }
  }
}

void keyReleased() {
  if (key == ' ') {
    myPort.write("<s>");
    println("STOP!!!");
    println("<s>");
    if (recMode == 2) { //playback
      recMode = 0; //normal
      print("recMode is "); 
      println(recMode);
      csvLineCounter = 0;
      cp5.getController("playback_pause").setValue(0);
    }
    else{
      print("recMode is "); 
      println(recMode);
    }
    if (displayPauseText == true) {
      displayPauseText = false;
      csvLineCounter=0;
    }
  }
  if (key == 'o'||key =='O') {
    myPort.write("<o>");
    println("opening gripper");
    println("<o>");
  }
  if (key == 'c'||key =='c') {
    myPort.write("<c>");
    println("closing gripper");
    println("<c>");
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
      } else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
  serialdata = new String(receivedChars);
}

void showNewData(String data) {
  //println("recorded angles: "+data);
  newData = false;
}
