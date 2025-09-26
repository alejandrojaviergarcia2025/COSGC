/*
By Whit Whittall
COSGC New Robotics Workshop code for lesson 3 basic motion

Finds wheel speeds based on input direction vector and drives brushed DC motors with dual H-bridge motor driver

left and right directions referenced in comments are from the robot's perspective
*/

//--------------------rover geometry parameters--------------------
// motor_controller() uses these parameters to calculate wheel velocities
const float r = 0.030f;   // radius of drive wheels in meters
const float L = 0.146f;   // width separating the drive wheels in meters

//--------------------declare motor pins--------------------
// declare pins to control right motor
const int R1   = 8;   // AI1  -> D8
const int R2   = 7;   // AI2  -> D7
const int pwmR = 6;   // PWMA -> D6 (PWM)

// declare pins to control left motor
const int L1   = 5;   // BI1  -> D5
const int L2   = 4;   // BI2  -> D4
const int pwmL = 3;   // PWMB -> D3 (PWM)

//--------------------controller scaling--------------------
const float MAX_WHEEL_RAD_S = 11.52f;        // max wheel speed (rad/s) used for scaling
const float PWM_PER_RAD_S   = 255.0f / MAX_WHEEL_RAD_S;
const int   PWM_MAX         = 255;
const float DUTY_DEADBAND   = 0.5f;          // small deadband to avoid buzzing around 0

void setup() {
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(pwmL, OUTPUT);
}

void loop() {
  //--------------------declare velocity variables--------------------
  // declaring v and w here limits their scope to loop() so there aren't conflicts with motor_controller()
  float v = 0.346f;  // linear velocity (0.346 is forward full, -0.346 is back full)
  float w = 0.0f;    // angular velocity (4.73 is rotate left full, -4.73 is rotate right full)
  motor_controller(v, w);
  delay(2000);

  v = -0.346f;
  w = 0.0f;
  motor_controller(v, w);
  delay(2000);

  v = 0.0f;
  w = 4.73f;
  motor_controller(v, w);
  delay(2000);

  v = 0.0f;
  w = -4.73f;
  motor_controller(v, w);
  delay(2000);

  // remove the block comment below to see the rover move in all directions
  /*
  v = -0.346f; w = 0.0f;   motor_controller(v,w); delay(1000);
  v =  0.0f;   w = 4.73f;  motor_controller(v,w); delay(1000);
  v =  0.0f;   w = -4.73f; motor_controller(v,w); delay(1000);
  */
}

//--------------------CUSTOM FXNS--------------------
// void motor_controller(v, w)
// void drive(vel_L, vel_R)

void motor_controller(float v, float w) {
  // determines required wheel speeds (in rad/s) based on linear and angular velocities (m/s, rad/s)
  // expects roughly -0.346 < v < 0.346 m/s, -4.73 < w < 4.73 rad/s
  float dphi_L = (v / r) - (L * w) / (2.0f * r);
  float dphi_R = (v / r) + (L * w) / (2.0f * r);

  // keep within physical/driver limits to avoid mis-scaling
  dphi_L = constrain(dphi_L, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);
  dphi_R = constrain(dphi_R, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);

  // map rad/s -> PWM using float math (avoid Arduino's integer-only map)
  float dutyLf = dphi_L * PWM_PER_RAD_S;
  float dutyRf = dphi_R * PWM_PER_RAD_S;

  // apply a small deadband around zero so motors truly stop
  if (fabs(dutyLf) < DUTY_DEADBAND) dutyLf = 0.0f;
  if (fabs(dutyRf) < DUTY_DEADBAND) dutyRf = 0.0f;

  // clamp and convert to int for analogWrite
  int duty_L = (int) round(constrain(dutyLf, -PWM_MAX, PWM_MAX));
  int duty_R = (int) round(constrain(dutyRf, -PWM_MAX, PWM_MAX));

  drive(duty_L, duty_R);
}

void drive(int duty_L, int duty_R) {
  // based on PWM duty cycle setting, assigns motor driver pin values
  // expects duty_L and duty_R to be between -255 and 255

  // left motor
  if (duty_L > 0) {               // forward
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
  } else if (duty_L < 0) {        // backward
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
  } else {                        // stop (coast). For brake: set both HIGH if your driver supports it.
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
  }

  // right motor
  if (duty_R > 0) {               // forward
    digitalWrite(R1, HIGH);
    digitalWrite(R2, LOW);
  } else if (duty_R < 0) {        // backward
    digitalWrite(R1, LOW);
    digitalWrite(R2, HIGH);
  } else {                        // stop (coast). For brake: set both HIGH if your driver supports it.
    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
  }

  analogWrite(pwmL, abs(duty_L));
  analogWrite(pwmR, abs(duty_R));
}
