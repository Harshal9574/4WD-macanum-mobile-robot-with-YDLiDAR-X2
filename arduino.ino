#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

////////////////// Tick Data Publishing Variables and Constants ///////////////

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT1_A 2
#define ENC_IN_RIGHT1_A 3
#define ENC_IN_LEFT2_A 18
#define ENC_IN_RIGHT2_A 19

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT1_B 22
#define ENC_IN_RIGHT1_B 23
#define ENC_IN_LEFT2_B 32
#define ENC_IN_RIGHT2_B 33

// True = Forward; False = Reverse
boolean Direction_left1 = true;
boolean Direction_right1 = true;
boolean Direction_left2 = true;
boolean Direction_right2 = true;

// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
std_msgs::Int16 left1_wheel_tick_count;
ros::Publisher Left1Pub("left1_ticks", &left1_wheel_tick_count);

std_msgs::Int16 right1_wheel_tick_count;
ros::Publisher Right1Pub("right1_ticks", &right1_wheel_tick_count);

std_msgs::Int16 left2_wheel_tick_count;
ros::Publisher Left2Pub("left2_ticks", &left2_wheel_tick_count);

std_msgs::Int16 right2_wheel_tick_count;
ros::Publisher Right2Pub("right2_ticks", &right2_wheel_tick_count);

// Time interval for measurements in milliseconds
const int interval = 200;
long previousMillis = 0;
long currentMillis = 0;

////////////////// Motor Controller Variables and Constants ///////////////////

const int enA_left1 = 6;
const int in1_left1 = 24;
const int in2_left1 = 25;

const int enB_right1 = 7;
const int in3_right1 = 26;
const int in4_right1 = 27;

const int enA_left2 = 8;
const int in1_left2 = 28;
const int in2_left2 = 29;

const int enB_right2 = 9;
const int in3_right2 = 30;
const int in4_right2 = 31;

// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;
 
// Number of ticks per wheel revolution.
const int TICKS_PER_REVOLUTION = 102; 
 
// Wheel radius in meters
const double WHEEL_RADIUS = 0.04; 
 
// Distance from center of the left tire to the center of the right tire in m
const double WHEEL_BASE = 0.218;
 
// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
const double TICKS_PER_METER = 406; 
 
// Proportional constant, which was measured by measuring the 
// PWM-Linear Velocity relationship for the robot.
const int K_P = 278;
 
// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 52;
 
// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 120;
 
// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = 255;
 
// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 200; 
const int PWM_MAX = 255; 

// Set linear velocity and PWM variable values for each wheel
double velLeft1Wheel = 0;
double velRight1Wheel = 0;
double velLeft2Wheel = 0;
double velRight2Wheel = 0;
double pwmLeft1Req = 0;
double pwmRight1Req = 0;
double pwmLeft2Req = 0;
double pwmRight2Req = 0;

// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;

/////////////////////// Tick Data Publishing Functions ////////////////////////
 
// Increment the number of ticks

void right1_wheel_tick() {
    int val = digitalRead(ENC_IN_RIGHT1_B);
    if (val == LOW) {
        Direction_right1 = false; // Reverse
    }
    else {
        Direction_right1 = true; // Forward
    }
    if (Direction_right1) {
        if (right1_wheel_tick_count.data == encoder_maximum) {
            right1_wheel_tick_count.data = encoder_minimum;
        }
        else {
            right1_wheel_tick_count.data++;  
        }    
    }
    else {
        if (right1_wheel_tick_count.data == encoder_minimum) {
            right1_wheel_tick_count.data = encoder_maximum;
        }
        else {
            right1_wheel_tick_count.data--;  
        }   
    }
}

void left1_wheel_tick() {
    // Read the value for the encoder for the left wheel
    int val = digitalRead(ENC_IN_LEFT1_B);
 
    if (val == LOW) {
      Direction_left1 = true; // Reverse
    }
    else {
      Direction_left1 = false; // Forward
    }
   
    if (Direction_left1) {
      if (left1_wheel_tick_count.data == encoder_maximum) {
        left1_wheel_tick_count.data = encoder_minimum;
      }
      else {
        left1_wheel_tick_count.data++;  
      }  
    }
    else {
      if (left1_wheel_tick_count.data == encoder_minimum) {
        left1_wheel_tick_count.data = encoder_maximum;
      }
      else {
        left1_wheel_tick_count.data--;  
      }   
    }
}

void right2_wheel_tick() {
    int val = digitalRead(ENC_IN_RIGHT2_B);
    if (val == LOW) {
        Direction_right2 = false; // Reverse
    }
    else {
        Direction_right2 = true; // Forward
    }
    if (Direction_right2) {
        if (right2_wheel_tick_count.data == encoder_maximum) {
            right2_wheel_tick_count.data = encoder_minimum;
        }
        else {
            right2_wheel_tick_count.data++;  
        }    
    }
    else {
        if (right2_wheel_tick_count.data == encoder_minimum) {
            right2_wheel_tick_count.data = encoder_maximum;
        }
        else {
            right2_wheel_tick_count.data--;  
        }   
    }
}

void left2_wheel_tick() {
    int val = digitalRead(ENC_IN_LEFT2_B);
    if (val == LOW) {
        Direction_left2 = false; // Reverse
    }
    else {
        Direction_left2 = true; // Forward
    }
   if (Direction_left2) {
        if (left2_wheel_tick_count.data == encoder_maximum) {
            left2_wheel_tick_count.data = encoder_minimum;
        }
        else {
            left2_wheel_tick_count.data++;  
        }    
    }
    else {
        if (left2_wheel_tick_count.data == encoder_minimum) {
            left2_wheel_tick_count.data = encoder_maximum;
        }
        else {
            left2_wheel_tick_count.data--;  
        }   
    }
}

/////////////////////// Motor Controller Functions ////////////////////////////
 
void calc_vel_left1_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevLeft1Count = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + left1_wheel_tick_count.data - prevLeft1Count) % 65535;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velLeft1Wheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
 
  // Keep track of the previous tick count
  prevLeft1Count = left1_wheel_tick_count.data; 
 
  // Update the timestamp
  prevTime = (millis()/1000);
}
 
void calc_vel_right1_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevRight1Count = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 +  right1_wheel_tick_count.data - prevRight1Count) % 65535;
 
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velRight1Wheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
 
  prevRight1Count = right1_wheel_tick_count.data;
   
  prevTime = (millis()/1000);
 
}

void calc_vel_left2_wheel(){
   
  // Previous timestamp
   static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
   static int prevLeft2Count = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
   int numOfTicks = (65535 + left2_wheel_tick_count.data - prevLeft2Count) % 65535;
 
  // If we have had a big jump, it means the tick count has rolled over.
   if (numOfTicks > 10000) {
         numOfTicks = 0 - (65535 - numOfTicks);
   }
 
  // Calculate wheel velocity in meters per second
   velLeft2Wheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
 
  // Keep track of the previous tick count
   prevLeft2Count = left2_wheel_tick_count.data;
 
  // Update the timestamp
   prevTime = (millis()/1000);
}

void calc_vel_right2_wheel(){
   
  // Previous timestamp
   static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
   static int prevRight2Count = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
   int numOfTicks = (65535 + right2_wheel_tick_count.data - prevRight2Count) % 65535;
 
   if (numOfTicks > 10000) {
         numOfTicks = 0 - (65535 - numOfTicks);
   }
 
  // Calculate wheel velocity in meters per second
   velRight2Wheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
 
   prevRight2Count = right2_wheel_tick_count.data;
   
   prevTime = (millis()/1000);
 
}

// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
   
  // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis()/1000);
   
  // Calculate the PWM value given the desired velocity
  pwmLeft1Req = K_P * cmdVel.linear.x + b;
  pwmRight1Req = K_P * cmdVel.linear.x + b;
  pwmLeft2Req = K_P * cmdVel.linear.x + b;
  pwmRight2Req = K_P * cmdVel.linear.x + b;

  // Check if we need to turn 
  if (cmdVel.linear.x == 0 && cmdVel.angular.z != 0.0) {

    if (cmdVel.angular.z > 0.0) {
      pwmLeft1Req = -PWM_TURN;
      pwmRight1Req = PWM_TURN;
      pwmLeft2Req = -PWM_TURN;
      pwmRight2Req = PWM_TURN;
    }
    // Turn right    
    else {
      pwmLeft1Req = PWM_TURN;
      pwmRight1Req = -PWM_TURN;
      pwmLeft2Req = PWM_TURN;
      pwmRight2Req = -PWM_TURN;
    }
  }

  else if (cmdVel.linear.x != 0 && cmdVel.angular.z == 0.0) {

    static double prevDiff1 = 0;
    static double prevDiff2 = 0;
    static double prevPrevDiff1 = 0;
    static double prevPrevDiff2 = 0;
    double currDifference1 = velLeft1Wheel - velRight1Wheel; 
    double currDifference2 = velLeft2Wheel - velRight2Wheel; 
    double avgDifference1 = (prevDiff1+prevPrevDiff1+currDifference1)/3;
    double avgDifference2 = (prevDiff2+prevPrevDiff2+currDifference2)/3;
    prevPrevDiff1 = prevDiff1;
    prevPrevDiff2 = prevDiff2;
    prevDiff1 = currDifference1;
    prevDiff2 = currDifference2;
 
    // Correct PWM values of both wheels to make the vehicle go straight
    pwmLeft1Req -= (int)(avgDifference1 * DRIFT_MULTIPLIER);
    pwmRight1Req += (int)(avgDifference1 * DRIFT_MULTIPLIER);
    pwmLeft2Req -= (int)(avgDifference2 * DRIFT_MULTIPLIER);
    pwmRight2Req += (int)(avgDifference2 * DRIFT_MULTIPLIER); 
  }

  else {
    pwmLeft1Req = 0;
    pwmRight1Req = 0;
    pwmLeft2Req = 0;
    pwmRight2Req = 0;
  }
  
  // Handle low PWM values
  if (abs(pwmLeft1Req) < PWM_MIN) {
    pwmLeft1Req = 0;
  }
  if (abs(pwmRight1Req) < PWM_MIN) {
    pwmRight1Req = 0;  
  }  
  if (abs(pwmLeft2Req) < PWM_MIN) {
    pwmLeft2Req = 0;
  }
  if (abs(pwmRight2Req) < PWM_MIN) {
    pwmRight2Req = 0;  
  } 
}

void set_pwm_values() {
 
  // These variables will hold our desired PWM values
  static int pwmLeft1Out = 0;
  static int pwmRight1Out = 0;
  static int pwmLeft2Out = 0;
  static int pwmRight2Out = 0;
 
  // If the required PWM is of opposite sign as the output PWM, we want to
  // stop the car before switching direction
  static bool stopped = false;
  if ((pwmLeft1Req * velLeft1Wheel < 0 && pwmLeft1Out != 0) ||
      (pwmRight1Req * velRight1Wheel < 0 && pwmRight1Out != 0)) {
    pwmLeft1Req = 0;
    pwmRight1Req = 0;
  }
  
  if ((pwmLeft2Req * velLeft2Wheel < 0 && pwmLeft2Out != 0) ||
      (pwmRight2Req * velRight2Wheel < 0 && pwmRight2Out != 0)) {
    pwmLeft2Req = 0;
    pwmRight2Req = 0;
  }

  // Set the direction of the motors
  if (pwmLeft1Req > 0) { // Front left wheel forward
    digitalWrite(in1_left1, HIGH);
    digitalWrite(in2_left1, LOW);
  }
  else if (pwmLeft1Req < 0) { // Front left wheel reverse
    digitalWrite(in1_left1, LOW);
    digitalWrite(in2_left1, HIGH);
  }
  else if (pwmLeft1Req == 0 && pwmLeft1Out == 0 ) { // Front left wheel stop
    digitalWrite(in1_left1, LOW);
    digitalWrite(in2_left1, LOW);
  }
  else { // Front left wheel stop
    digitalWrite(in1_left1, LOW);
    digitalWrite(in2_left1, LOW); 
  }

  if (pwmRight1Req > 0) { // Front right wheel forward
    digitalWrite(in3_right1, HIGH);
    digitalWrite(in4_right1, LOW);
  }
  else if (pwmRight1Req < 0) { // Front right wheel reverse
    digitalWrite(in3_right1, LOW);
    digitalWrite(in4_right1, HIGH);
  }
  else if (pwmRight1Req == 0 && pwmRight1Out == 0) { // Front right wheel stop
    digitalWrite(in3_right1, LOW);
    digitalWrite(in4_right1, LOW);
  }
  else { // Front right wheel stop
    digitalWrite(in3_right1, LOW);
    digitalWrite(in4_right1, LOW); 
  }

  if (pwmLeft2Req > 0) { // rear left wheel forward
    digitalWrite(in1_left2, HIGH);
    digitalWrite(in2_left2, LOW);
  }
  else if (pwmLeft2Req < 0) { // rear left wheel reverse
    digitalWrite(in1_left2, LOW);
    digitalWrite(in2_left2, HIGH);
  }
  else if (pwmLeft2Req == 0 && pwmLeft2Out == 0 ) { // rear left wheel stop
    digitalWrite(in1_left2, LOW);
    digitalWrite(in2_left2, LOW);
  }
  else { // rear left wheel stop
    digitalWrite(in1_left2, LOW);
    digitalWrite(in2_left2, LOW); 
  }

  if (pwmRight2Req > 0) { // rear right wheel forward
    digitalWrite(in3_right2, HIGH);
    digitalWrite(in4_right2, LOW);
  }
  else if (pwmRight2Req < 0) { // rear right wheel reverse
    digitalWrite(in3_right2, LOW);
    digitalWrite(in4_right2, HIGH);
  }
  else if (pwmRight2Req == 0 && pwmRight2Out == 0) { // rear right wheel stop
    digitalWrite(in3_right2, LOW);
    digitalWrite(in4_right2, LOW);
  }
  else { // rear right wheel stop
    digitalWrite(in3_right2, LOW);
    digitalWrite(in4_right2, LOW); 
  }

  // Increase the required PWM if the robot is not moving
  if (pwmLeft1Req != 0 && velLeft1Wheel == 0) {
    pwmLeft1Req *= 1.5;
  }
  if (pwmRight1Req != 0 && velRight1Wheel == 0) {
    pwmRight1Req *= 1.5;
  }
  if (pwmLeft2Req != 0 && velLeft2Wheel == 0) {
    pwmLeft2Req *= 1.5;
  }
  if (pwmRight2Req != 0 && velRight2Wheel == 0) {
    pwmRight2Req *= 1.5;
  }

  // Calculate the output PWM value by making slow changes to the current value
  if (abs(pwmLeft1Req) > pwmLeft1Out) {
    pwmLeft1Out += PWM_INCREMENT;
  }
  else if (abs(pwmLeft1Req) < pwmLeft1Out) {
    pwmLeft1Out -= PWM_INCREMENT;
  }
  else{}
   
  if (abs(pwmRight1Req) > pwmRight1Out) {
    pwmRight1Out += PWM_INCREMENT;
  }
  else if(abs(pwmRight1Req) < pwmRight1Out) {
    pwmRight1Out -= PWM_INCREMENT;
  }
  else{}

  if (abs(pwmLeft2Req) > pwmLeft2Out) {
    pwmLeft2Out += PWM_INCREMENT;
  }
  else if (abs(pwmLeft2Req) < pwmLeft2Out) {
    pwmLeft2Out -= PWM_INCREMENT;
  }
  else{}
   
  if (abs(pwmRight2Req) > pwmRight2Out) {
    pwmRight2Out += PWM_INCREMENT;
  }
  else if(abs(pwmRight2Req) < pwmRight2Out) {
    pwmRight2Out -= PWM_INCREMENT;
  }
  else{}

  // Conditional operator to limit PWM output at the maximum 
  pwmLeft1Out = (pwmLeft1Out > PWM_MAX) ? PWM_MAX : pwmLeft1Out;
  pwmRight1Out = (pwmRight1Out > PWM_MAX) ? PWM_MAX : pwmRight1Out;
  pwmLeft2Out = (pwmLeft2Out > PWM_MAX) ? PWM_MAX : pwmLeft2Out;
  pwmRight2Out = (pwmRight2Out > PWM_MAX) ? PWM_MAX : pwmRight2Out;

  // PWM output cannot be less than 0
  pwmLeft1Out = (pwmLeft1Out < 0) ? 0 : pwmLeft1Out;
  pwmRight1Out = (pwmRight1Out < 0) ? 0 : pwmRight1Out;
  pwmLeft2Out = (pwmLeft2Out < 0) ? 0 : pwmLeft2Out;
  pwmRight2Out = (pwmRight2Out < 0) ? 0 : pwmRight2Out;
 
  // Set the PWM value on the pins
  analogWrite(enA_left1, pwmLeft1Out); 
  analogWrite(enB_right1, pwmRight1Out);
  analogWrite(enA_left2, pwmLeft2Out); 
  analogWrite(enB_right2, pwmRight2Out); 
}
 
// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values);

void setup() {
  // Set pin states of the encoders
  pinMode(ENC_IN_LEFT1_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT1_B , INPUT);
  pinMode(ENC_IN_RIGHT1_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT1_B , INPUT);
  pinMode(ENC_IN_LEFT2_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT2_B , INPUT);
  pinMode(ENC_IN_RIGHT2_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT2_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT1_A), left1_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT1_A), right1_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT2_A), left2_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT2_A), right2_wheel_tick, RISING);
   
  // Motor control pins are outputs
  pinMode(enA_left1, OUTPUT);
  pinMode(enB_right1, OUTPUT);
  pinMode(enA_left2, OUTPUT);
  pinMode(enB_right2, OUTPUT);
  pinMode(in1_left1, OUTPUT);
  pinMode(in2_left1, OUTPUT);
  pinMode(in3_right1, OUTPUT);
  pinMode(in4_right1, OUTPUT);
  pinMode(in1_left2, OUTPUT);
  pinMode(in2_left2, OUTPUT);
  pinMode(in3_right2, OUTPUT);
  pinMode(in4_right2, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1_left1, LOW);
  digitalWrite(in2_left1, LOW);
  digitalWrite(in3_right1, LOW);
  digitalWrite(in4_right1, LOW);
  digitalWrite(in1_left2, LOW);
  digitalWrite(in2_left2, LOW);
  digitalWrite(in3_right2, LOW);
  digitalWrite(in4_right2, LOW);
  
  // Set the motor speed
  analogWrite(enA_left1, 0); 
  analogWrite(enB_right1, 0);
  analogWrite(enA_left2, 0); 
  analogWrite(enB_right2, 0);
 
  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(Left1Pub);
  nh.advertise(Right1Pub);
  nh.advertise(Left2Pub);
  nh.advertise(Right2Pub);
  nh.subscribe(subCmdVel);
}

void loop() {

  nh.spinOnce();
   
  // Record the time
  currentMillis = millis();
 
  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;

    // Publish tick counts to topics
    Left1Pub.publish( &left1_wheel_tick_count );
    Right1Pub.publish( &right1_wheel_tick_count );
    Left2Pub.publish( &left2_wheel_tick_count );
    Right2Pub.publish( &right2_wheel_tick_count );
 
    // Calculate the velocity of the right and left wheels
    calc_vel_right1_wheel();
    calc_vel_left1_wheel();
    calc_vel_right2_wheel();
    calc_vel_left2_wheel();
     
  }
   
  // Stop the car if there are no cmd_vel messages
  if((millis()/1000) - lastCmdVelReceived > 1) {
    pwmLeft1Req = 0;
    pwmRight1Req = 0;
    pwmLeft2Req = 0;
    pwmRight2Req = 0;
  }
 
  set_pwm_values();
} 
