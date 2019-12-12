//slider
import controlP5.*;

ControlP5 cp5;
int myColor = color(0,0,0);
float baseJoint = 0;
float prevBaseJoint;

color targetCol = color(255, 100, 0); 
int targetX, targetY;
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
  size( 900, 700);
  // load knob image
  
  targetX = width/2 - 100;
  targetY = height/2-100;
  colorMode(HSB);
  background(100);
  

  segment[0] = new Segment(300, 100, 100, 0);
  for (int i = 1; i<2; i++){
    segment[i] =new Segment(segment[i-1], 100, 0);
  }
  base = new PVector(width/2, height/2 - 185/2);
  
  //base joint slider
  noStroke();
  cp5 = new ControlP5(this);
  cp5.addSlider("Base Joint")
     .setPosition(width/4-23,height/4*3)
     .setSize(500,20)
     .setRange(-270,270) // values can range from big to small as well
     .setNumberOfTickMarks(55)
     .setValue(0)
     .snapToTickMarks(false)
     .setSliderMode(Slider.FLEXIBLE)
     .setScrollSensitivity(0.03)
     ;
  baseJoint = cp5.getValue("Base Joint");
  println("sending joint"+ baseJoint);
}
void draw(){
  //draw background and axes
  background(100);
  stroke(255);
  strokeWeight(1);
  line(0, height/2, width, height/2);
  line(width/2, 0, width/2, height);
  //draw base
  strokeWeight(6);
  line(width/2, height/2, width/2, height/2 - 185/2);
  
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
  
  // base joint slider
  baseJoint = cp5.getValue("Base Joint");
  if(baseJoint != prevBaseJoint){
    println("sending joint"+ baseJoint); 
  }
  else{
  println("not sending anything");
  }
  prevBaseJoint =baseJoint;
  }
  
  //if(dist(knobX,knobY,mouseX, mouseY) < sz*3/2 || dist(knobX,knobY,mouseX, mouseY) > sz){
  //  stroke(255);// inside the target circle
  //  cursor(HAND);
  //  if (mousePressed){
  //    strokeWeight(5);
  //  }
  //  else{
  //    strokeWeight(2);
  //    }
  //}
  //else {
  //  noStroke();
  //  cursor(ARROW);
  //}
  // strokeWeight(2);
  // fill(255);
  // ellipse(knobJointX, knobJointY, sz*4, sz*4);
  // fill(100);
  // ellipse(knobJointX, knobJointY, sz*2, sz*2);
  // //ellipse(knobX, knobY, sz, sz);
  // fill(targetCol);
  // ellipse(knobX, knobY, sz, sz);
  // cursor(ARROW);
}

void mouseDragged(){
  if (mouseY >= height/2){
  }  
  else{
  targetX = mouseX;
  targetY = mouseY;
  
  }
}
