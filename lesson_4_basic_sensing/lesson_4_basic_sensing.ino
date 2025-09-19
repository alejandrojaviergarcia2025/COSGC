/* 
By Whit Whittall
COSGC New Robotics Workshop code for lesson 4 basic sensing
Verified to work using Robotics Workshop BOM hardware on 7/16/2024

Adds three common hobby robotics sensors: ultrasonic time of flight, IR reflectance, and quadrature encoders
Demonstrates use of finite state machine for robotic decision making

left and right directions referenced in comments are from the robot's perspective
*/ 

// include the QuadratureEncoder library

//--------------------rover geometry parameters--------------------
// motor_controller() uses these parameters to calculate wheel velocities
// get_odom() uses them to track distance travelled
const float r = 0.030;   // radius of drive wheels in meters
const float L = 0.146;   // width separating the drive wheels in meters

//--------------------declare motor pins--------------------
// setup() and drive() use these variables to control Arduino pins
const int R1 = 8;    //AI1  -> D8
const int R2 = 7;    //AI2  -> D7
const int pwmR = 6;  //PWMA -> D6

// declare pins to control left motor
const int L1 = 5;    //BI1  -> D5
const int L2 = 4;    //BI2  -> D4
const int pwmL = 3;  //PWMB -> D3

//--------------------declare sensor pins--------------------
// declare echo and trig pins for ultrasonic sensor

// declare pin for infrared sensor

// declare a and b pins for encoders (can be analog pins)
// right encoder

// left encoder

// lets also create our encoder objects

//--------------------set up FSM--------------------

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); // initialize the serial monitor so we can use it to check our work

  //--------------------setup motor pins--------------------
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(pwmL, OUTPUT);

  //--------------------setup sensor pins--------------------
  // no need to setup our encoder pins, the library takes care of that

}

void loop() {
  // put your main code here, to run repeatedly:

  // uncomment to test get_distance()
  //Serial.println(get_distance());
  //delay(1000);

  // uncomment to test get_line()
  //Serial.println(get_line());

  // uncomment to test get_odom()
  //motor_controller(0.346, 0);
  //delay(2000);
  //Serial.println(get_odom()); // returns in meters
  // if get_odom() returns negative numbers, try reversing the yellow/white signal wires of both encoders
  // if get_odom() grows much slower than you expect (0.346 m/s), try reversing the yellow/white signal wires of one encoder at a time

  // Put your FSM in here:
  /* // FSM currently commented out so it doesn't interfere with sensor testing code
  switch (current_state) {
    case follow_right :
      break;
    case turn_right :
      break;
    case follow_left :
      break;
    case turn_left :
      break;
    case stop :
      break;
  }

  // update states

  */
}


//--------------------CUSTOM FXNS--------------------
// float get_distance();
// bool get_line();
// long get_odom();
// void motor_controller(v, w)
// void drive(vel_L, vel_R)


float get_distance() {
  // finds the distance of an object in front of the ultrasonic sensor
  // if this returns 0.00, check that the sensor is getting 5V
  // also check trig and echo pins aren't reversed
  float echo_time;             // var to store time of flight
  float calculated_distance;   // var to store distance calculated from time of flight

  // send out an ultrasonic pulse thats 10ms long

  // use pulseIn function to see how long it takes for the pulse to return to the sensor

  // calculate distance using formula from ToF sensor section

}

bool get_line() {
  // returns 0 (false) if no reflection seen (over black surface)
  // returns 1 (true) if reflection seen (over white surface)
  // this only works when the lipo was plugged in as well as the serial cable
  // the IR sensor NEEDS to be fed ~5V
  
}

float get_odom() {
  // get encoder counts using getEncoderCount method from the Encoders class

  // find the angular position of our wheels

  // calculate the linear position of our robot from the angular position of the wheels

}

void motor_controller(float v, float w) {
// determines required wheel speeds (in rad/s) based on linear and angular velocities (m/s, rad/s)
// maps required wheel speeds to PWM duty cycle
// expects -0.346 < v < 0.346 m/s, -4.73 < w < 4.73 rad/s
// motors will saturate if desired velocity vector is too large, best to keep desired velocities low

  float dphi_L = (v/r) - (L * w)/(2 * r);
  float dphi_R = (v/r) + (L * w)/(2 * r);

  // use the constrain function to keep dphi_L and dphi_R within certain boundaries
  // this prevents unintended behavior of the map function
  dphi_L = constrain(dphi_L, -11.52, 11.52);
  dphi_R = constrain(dphi_R, -11.52, 11.52);

  // need to confirm map() behaves well when given non-int input
  // map() uses integer math, returns only integers which is not a problem in this case
  // would be a problem if it misbehaves with float input
  int duty_L = map(dphi_L, -11.52, 11.52, -255, 255);
  int duty_R = map(dphi_R, -11.52, 11.52, -255, 255);

  drive(duty_L, duty_R);
}

void drive(int duty_L, int duty_R) {
// based on PWM duty cycle setting, assigns motor driver pin values
// expects duty_L and duty_R to be between -255 and 255

  // left motor
  if (duty_L > 0) {  // left motor forward
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
  }
  if (duty_L < 0) {  // left motor backward
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
  }
  if (duty_L == 0) {  // left motor stop
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
  }
  // right motor
  if (duty_R > 0) {  // right motor forward
    digitalWrite(R1, HIGH);
    digitalWrite(R2, LOW);
  }
  if (duty_R < 0) {  // right motor backward
    digitalWrite(R1, LOW);
    digitalWrite(R2, HIGH);
  }
  if (duty_R == 0) {  // right motor stop
    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
  }
  analogWrite(pwmL, abs(duty_L));
  analogWrite(pwmR, abs(duty_R));
}