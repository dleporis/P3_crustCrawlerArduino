class Segment{
  PVector a;
  float angle;
  float len;
  PVector b = new PVector();
  Segment parent;
  Segment child;
  
  Segment(float x, float y, float len_, float angle_) {
    a = new PVector(x,y);
    angle = angle_;
    len = len_;
    Segment parent = null;
    calculateB();
  }
  
  Segment(Segment parent_, float len_, float angle_) {
    parent = parent_;
    a = parent.b.copy();
    angle = angle_;
    len = len_;
    calculateB();
  }
  
  void follow(Segment child){
    float targetX = child.a.x;
    float targetY = child.a.y;
    follow(targetX, targetY);
  }
  
  void follow(float tx, float ty){
    PVector target = new PVector(tx, ty);
    PVector dir = PVector.sub(target, a);
    angle = dir.heading(); //angle of  vector, uses atan2
    
   dir.setMag(len);
   dir.mult(-1);
   a = PVector.add(target, dir);
  
  }
  
  void setA(PVector pos){
    a = pos.copy();
    calculateB();
  }
  void setB(int xin, int yin){
    b.set(xin, yin);
  }
  void calculateB(){
    
    float dx = len*cos(angle);
    float dy = len*sin(angle);
    b.set(a.x+dx, a.y+dy);
  }
  
  void update(){
     calculateB();
  }
  
  void show(){
    stroke(100, 20, 30);
    strokeWeight(7);
    line( a.x, a.y, b.x, b.y);
    
  }
}
