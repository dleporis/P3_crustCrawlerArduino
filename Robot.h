
#include <Arduino.h>

class Robot{
  
    float TCP;
    float a[4];
    float d[4];
    float alpha[4];
    float theta_offset[4];
    float theta[4]; // we are not using theta[0], theta[1] = base, theta[2] = 2nd joint theta[3] = 3rd joint
    float prevTheta[4]; // we are not using theta[0], theta[1] = base, theta[2] = 2nd joint theta[3] = 3rd joint
    float Xee;
    float Yee;
    float Zee;
    float pitchEE;
    float Xr;
    float Zr;
    float Zb;
    float goalDistance; // vector length from 2nd motor to desired position in mm
     int configuration; // 1 is upper configuration, 0 is lower configuration
     int prevConfiguration;
     float pi = 3.14157;
  
  public: 
  
    //Robot(float DH_a[], float DH_d[], float DH_alpha[], float DH_theta_offset[], float tcp); // constructor
//    ~Robot(); // destructor
//  // SETTERS _________________________________________________________
//    void begin(void);
//  // DH setters
//    void set_a(float val, int i);
//    void set_d(float val, int i);
//    void set_alpha(float val, int i);
//    void set_theta_offset(float val, int i);
//    void set_TCP(float val);
//    // CARTESIAN setters
//    void set_Xee(float val);
//    void set_Yee(float val);
//    void set_Zee(float val);
//    // R plane setters
//    void set_Xr(float val);
//    void set_Zr(float val);
//    void set_Zb(float val);
//    // Angle, configuration setters
//    void set_configuration(bool val);
//    void set_theta(float val, int i);
//    void set_pitchEE(float val);
//    
//  // GETTERS _________________________________________________________
//  
//  // DH getters  
//    float get_a(int i);
//    float get_d(int i);
//    float get_alpha(int i);
//    float get_theta_offset(int i);
//    float get_TCP();
//    // CARTESIAN getters
//    float get_Xee();
//    float get_Yee();
//    float get_Zee();
//    // R plane getters
//    float get_Xr();
//    float get_Zr();
//    float get_Zb();
//    // Angle, configuration getters
//    bool get_configuration();
//    float get_theta(int i);
//    float get_pitchEE();
//    float get_goalDistance();
//    
//  // ANGLE CONVERSIONS _________________________________________________________
//    float rad2Deg(float radAng);
//     float deg2Rad(float degAng);
//     //float DH_a[] = {0, 0, 218.5, (145+TCP)}; // in millimeters
//     //float DH_d[] = {185, 0, 0, 0}; // in millimeters
//     //float DH_alpha[] = {0, -90, 0, 90}; // in degrees
//     //float DH_theta_offset[] = {0, 0, 0, 0}; // in degrees, theta[0] is not being used
//    //TODO: output vector
//    void forKinem();
//    void findConfiguration();
//    bool poseReachable();
//    void tooClose();
//    void invKinemCart();
//    void invKinemR(); // need to use setters to update Xr and Zr 

  Robot(float DH_a[], float DH_d[], float DH_alpha[], float DH_theta_offset[], float tcp){
     
    for (int i = 0; i<4; i++) {
      a[i] = DH_a[i];
      d[i] =  DH_d[i];
      alpha[i] = deg2Rad(DH_alpha[i]);
      theta_offset[i] =  deg2Rad(DH_theta_offset[i]);
      theta[i] = 0 + theta_offset[i];
    }
    TCP = tcp;
    a[3] = a[3]+TCP;
  }
// SETTERS _________________________________________________________
  void begin(void){
    }
// DH setters
  void set_a(float val, int i){
    a[i] = val + TCP;
  }
  void set_d(float val, int i){
    d[i] = val;
  }
  void set_alpha(float val, int i){
    alpha[i] = val;
  }
  void set_theta_offset(float val, int i){
    theta_offset[i] = val;
  }
  void set_TCP(float val){
    TCP = val;
  }
  
  // CARTESIAN setters
  void set_Xee(float val){
    Xee = val;
  }
  void set_Yee(float val){
    Yee = val;
  }
  void set_Zee(float val){
    Zee = val;
    Zr = Zee;
  }
  
  // R plane setters
  void set_Xr(float val){
    Xr = val;
  }
  void set_Zr(float val){
    Zr = val;
    Zee = Zr;
  }
  void set_Zb(float val){
    Zb = val;
  }
  
  // Angle, configuration setters
  void set_configuration(bool val){
    configuration = val;
  }
  void set_theta(float val, int i){
    theta[i] = deg2Rad(val) +theta_offset[i] ;
    Serial.print("theta[");
    Serial.print(i);
    Serial.print("] ");
    Serial.print(" in rad: ");
    Serial.println(deg2Rad(val));
  }
  void set_prevTheta(float val, int i){
    prevTheta[i] = deg2Rad(val) +theta_offset[i] ;
    Serial.print("prevTheta[");
    Serial.print(i);
    Serial.print("] ");
    Serial.print(" in rad: ");
    Serial.println(deg2Rad(val));
  }
  void set_pitchEE(float val){
    pitchEE = deg2Rad(val);
  }
  
// GETTERS _________________________________________________________

// DH getters  
  float get_a(int i){
    return a[i];
  }
  float get_d(int i){
    return d[i];
  }
  float get_alpha(int i){
    return alpha[i];
  }
  float get_theta_offset(int i){
    return rad2Deg(theta_offset[i]);
  }
  float get_TCP(){
    return TCP;
  }
  
  // CARTESIAN getters
  float get_Xee(){
    return Xee;
  }
  float get_Yee(){
    return Yee;
  }
  float get_Zee(){
    return Zee;
  }
  
  // R plane getters
  float get_Xr(){
    return Xr;
  }
  float get_Zr(){
    return Zr;
  }
  float get_Zb(){
    return Zb;
  }
  
  // Angle, configuration getters
  int get_configuration(){
    return configuration;
  }
  float get_theta(int i){
    return rad2Deg(theta[i]) ;
  }
  
  float get_prevTheta(int i){
    return rad2Deg(prevTheta[i]) ;
  }
  float get_pitchEE(){
    return rad2Deg(pitchEE) ;
  }

  //float get_goalDistance(){
  //  return goalDistance;
  //}
// ANGLE CONVERSIONS _________________________________________________________
  float rad2Deg(float radAng){
    float degAng = radAng * 57296 / 1000;
    return degAng;
  }  

  float deg2Rad(float degAng){
    float radAng = degAng * 1000 / 57296;
    return radAng;
  }
   //float DH_a[] = {0, 0, 218.5, (145+TCP)}; // in millimeters
   //float DH_d[] = {185, 0, 0, 0}; // in millimeters
   //float DH_alpha[] = {0, -90, 0, 90}; // in degrees
   //float DH_theta_offset[] = {0, 0, 0, 0}; // in degrees, theta[0] is not being used
  
  void forKinem(){
    //vector r of planar 2DOF robot
    Xr = a[2]*cos(theta[2]) + a[3]*cos(theta[2]+theta[3]);
    Zb = a[2]*sin(theta[2])+a[3]*sin(theta[2]+theta[3]);
    Zr = d[0]+ Zb;
    //going to 3D by introducing base joint theta[1]
    Xee = Xr*cos(theta[1]);
    Yee = Xr*sin(theta[1]);
    Zee = Zr;
    pitchEE =  theta[2] + theta[3]; // MX 106 and MX64
    //determine configuration
  }
  void forKinem(float th1, float th2, float th3){
    set_theta(th1, 1);
    set_theta(th2, 2);
    set_theta(th3, 3);
    //vector r of planar 2DOF robot
    Xr = a[2]*cos(theta[2]) + a[3]*cos(theta[2]+theta[3]);
    Zb = a[2]*sin(theta[2])+a[3]*sin(theta[2]+theta[3]);
    Zr = d[0]+ Zb;
    //going to 3D by introducing base joint theta[1]
    Xee = Xr*cos(theta[1]);
    Yee = Xr*sin(theta[1]);
    Zee = Zr;
    pitchEE =  theta[2] + theta[3]; // MX 106 and MX64
    //determine configuration
  }
  void findConfiguration(){
    if (theta[3] < 0 && theta[3] > -pi)
      configuration = 0;
    else
      configuration = 1; 
  }
  void findPrevConfiguration(){
    if (prevTheta[3] < 0 && prevTheta[3] > -3.14)
      prevConfiguration = 0;
    else
      prevConfiguration = 1; 
  }
  
  bool poseReachableC(){
    // should I move it somehow?2
    Zb = Zee-d[0];
    float r1 = sqrt(sq(Xee)+sq(Yee)+sq(Zb));
    if (r1 <= (a[2]+a[3])){
      // position is reachable
      return 1;
    }
    else {
      Xee = Xee/r1*(a[2]+a[3]); // normalize and scale to reachable position
      Yee = Yee/r1*(a[2]+a[3]); // normalize and scale to reachable position
      Zee = Zee/r1*(a[2]+a[3]);// normalize and scale to reachable position
      set_theta(0,3);
      return 0; // position is not reachable
    } 
  }
  
  bool poseReachableR(){
    // should I move it somehow?2
    Zb = Zr - d[0];
    float r1 = sqrt(sq(Xr)+sq(Zb));
    if (r1 < (a[2]+a[3])){
      // position is reachable
      return 1;
    }
    else {
      Xr = Xr/r1*(a[2]+a[3]); // normalize and scale to reachable position
      Zb = Zb/r1*(a[2]+a[3]);// normalize and scale to reachable position
      set_theta(0,3);
      return 0; // position is not reachable
    } 
  }
  void tooClose(){
    if (Xr < 200 && Zr <= d[0]){
        Xr = 200;
    }
  }
  
  void invKinemCart(){
    theta[1] = atan2(Yee,Xee);
    Xr = sqrt( sq(Xee) + sq(Yee)); // X component of r vector in a "planar" 2DOF robot
    Zr = Zee;
    Zb = Zr - d[0];
    // planar 2DOF robot Xr Zr with theta2 and theta3 
    theta[2] =  atan2(Zb,Xr) + acos(( - sq(a[3]) - sq(Xr) - sq(Zb) - sq(a[2]))/(2*sqrt(sq(Xr)+sq(Zb))*a[3]));
    theta[3] = acos( (sq(Xr) + sq(Zb) - sq(a[2]) - sq(a[3]))/(2* a[2] * a[3]) );
     //difference between 2nd joint and end effector 
  }
  
  
  void invKinemR(float x, float z, float prevTh2, float prevTh3, int conf){
  //float Xr = 100;
  //float Zr = 100;
  //float Zb = Zr - d[0];
  set_Xr(x);
  set_Zr(z);
  set_prevTheta(prevTh2, 2);
  set_prevTheta(prevTh3, 3);
  set_Zb(Zr - d[0]);
  float a = get_a(2);
  float b = get_a(3);
  float c, beta, gamma, delta;
  
  findPrevConfiguration();
  
  Serial.print("prevConfiguration: ");
  Serial.println(prevConfiguration);
 
  if (Zb<0 &&Xr <200){
      Xr =200;  
  }
  
  if (Zb == 0){
    c = Xr;
  }
  else{
    c = sqrt( sq(Xr) + sq(Zb)) ; // pythagoras theorem  
  }
  if (prevConfiguration == 1){
    if ( conf == 0)
      set_configuration(conf);// 1 is upper configuration, 0 is lower configuration
    else configuration = 1;
    // unless specified in message to force configuration 0
  }
  else if (prevConfiguration == 0){
    if(Zb < 0){
      configuration = 1;
    }
    if(c <= cos(deg2Rad(20)) *b*2){
      configuration = 1;
    } 
  }
  if (c <= a+b){
    // position is reachable
  }
  else {
    // position is not reachable
    theta[3] = 0;
    delta = (asin ( Zb/c ));
    theta[2] =  -1* delta;
    Serial.println("Pose non-reachable, stretching in direction");
    Serial.print("theta[2]: ");
    Serial.println(rad2Deg(theta[2]));
    return;
  } 
  
  beta = (acos ( (sq(b) - sq(a)-sq(c)) / (-2*a*c) ) ) ; // law of cossines: beta angle in the picture, oppisite a[3]
  gamma = (acos ( (sq(c) - sq(b)-sq(a)) / (-2*a*b) ) ); //  law of cossines: gama angle in the picture, opposite c(from start to tool
  delta = (asin ( Zb/c ));
  Serial.print("beta :");
  Serial.println(beta);
  Serial.print("gamma :");
  Serial.println(gamma);
  Serial.print("delta :");
  Serial.println(delta);
  // upper configuration
  if (configuration == 1){ 
    theta[2] = -1*(delta + beta);
    theta[3] = pi-gamma;
  } 
  // lower configuration
 else if (configuration == 0){
    theta[2] = delta + beta;
    theta[3] = -1* (pi-gamma);
  }
  Serial.print("Inverse kinematics for Xr: ");
  Serial.print(Xr);
  Serial.print(" Zb:");
  Serial.print(Zb);
  Serial.print(" configuration: ");
  Serial.print(configuration);
  Serial.println(":");
  Serial.print("theta[2]: ");
  Serial.println(rad2Deg(theta[2]));
  Serial.print(" theta[3]: ");
  Serial.println(rad2Deg(theta[3]));
}    
};
