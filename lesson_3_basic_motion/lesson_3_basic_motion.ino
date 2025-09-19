/* 
By Whit Whittall
COSGC New Robotics Workshop code for lesson 3 basic motion

Finds wheel speeds based on input direction vector and drives brushed DC motors with dual H-bridge motor driver

left and right directions referenced in comments are from the robot's perspective
*/ 

//--------------------rover geometry parameters--------------------
// motor_controller() uses these parameters to calculate wheel velocities
const float r = 0.030;   // radius of drive wheels in meters
const float L = 0.146;   // width separating the drive wheels in meters

//--------------------declare motor pins--------------------
// setup() and drive() use these variables to control Arduino pins
// declare pins to control right motor
const int R1 = 8;    //AI1  -> D8
const int R2 = 7;    //AI2  -> D7
const int pwmR = 6;  //PWMA -> D6

// declare pins to control left motor
const int L1 = 5;    //BI1  -> D5
const int L2 = 4;    //BI2  -> D4
const int pwmL = 3;  //PWMB -> D3

void setup() {
  // put your setup code here, to run once:

  //--------------------setup motor pins--------------------
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(pwmL, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  //--------------------declare velocity variables--------------------
  // declaring v and w here limits their scope to loop() so there aren't conflicts with motor_controller()
  float v = 0.346;    // linear velocity (0.346 is forward full, -0.346 is back full)
  float w = 0;        // angular velocity (4.73 is rotate left full, -4.73 is rotate right full)
  motor_controller(v, w);
  delay(1000);

  // is a normal comment
  /* is a block comment */

  // remove the block comment below to see the rover move in all directions
  /*
  v = -0.346;
  w = 0;
  motor_controller(v,w);
  delay(1000);

  v = 0;
  w = 4.73;
  motor_controller(v,w);
  delay(1000);

  v = 0;
  w = -4.73;
  motor_controller(v,w);
  delay(1000);
  */
  // play around with the velocities and the delay() function and see what you can get your robot to do
}

//--------------------CUSTOM FXNS--------------------
// void motor_controller(v, w)
// void drive(vel_L, vel_R)

void motor_controller(float v, float w) {
  // determines required wheel speeds (in rad/s) based on linear and angular velocities (m/s, rad/s)
  // maps required wheel speeds to PWM duty cycle
  // expects -0.346 < v < 0.346 m/s, -4.73 < w < 4.73 rad/s
  // motors will saturate if desired velocity vector is too large, best to keep desired velocities low

}

void drive(int duty_L, int duty_R) {
  // based on PWM duty cycle setting, assigns motor driver pin values
  // expects duty_L and duty_R to be between -255 and 255

}