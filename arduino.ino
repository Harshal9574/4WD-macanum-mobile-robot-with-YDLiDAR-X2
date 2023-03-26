#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
ros::NodeHandle nh;
// Encoder ENC_A pin to Arduino Interrupt pin, which tracks the encoder tick count.
// Encoder ENC_B pin to Arduino Interrupt pin, which tracks the direction of encoder.
#define ENC_IN_FRONTLEFT_A 2
#define ENC_IN_FRONTLEFT_B 22
#define ENC_IN_FRONTRIGHT_A 3
#define ENC_IN_FRONTRIGHT_B 23
#define ENC_IN_REARLEFT_A 18
#define ENC_IN_REARLEFT_B 32
#define ENC_IN_REARRIGHT_A 19
#define ENC_IN_REARRIGHT_B 33
// true fOR forward direction and false for reverse direction 
boolean Direction_frontleft = true;
boolean Direction_frontright = true;
boolean Direction_rearleft = true;
boolean Direction_rearright = true;
// Minumum and maximum values for 16-bit integers, Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
// publisher for a message representing the tick count of wheels
std_msgs::Int16 frontleft_wheel_tick_count;
ros::Publisher FrontLeftPub("frontleft_ticks", &frontleft_wheel_tick_count);
std_msgs::Int16 frontright_wheel_tick_count;
ros::Publisher FrontRightPub("frontright_ticks", &frontright_wheel_tick_count);
std_msgs::Int16 rearleft_wheel_tick_count;
ros::Publisher RearLeftPub("rearleft_ticks", &rearleft_wheel_tick_count);
std_msgs::Int16 rearright_wheel_tick_count;
ros::Publisher RearRightPub("rrearight_ticks", &rearright_wheel_tick_count);
// Time interval for measurements in milliseconds
const int interval = 200;
long previousMillis = 0;
long currentMillis = 0;
// DC motors connections
const int enA_frontleft = 6;
const int in1_frontleft = 24;
const int in2_frontleft = 25;
const int enB_frontright = 7;
const int in3_frontright = 26;
const int in4_frontright = 27;
const int enA_rearleft = 8;
const int in1_rearleft = 28;
const int in2_rearleft = 29;
const int enB_rearright = 9;
const int in3_rearright = 30;
const int in4_rearright = 31;
// PWM value change in each cycle
const int PWM_INCREMENT = 1;
// Number of ticks per wheel revolution.
const int TICKS_PER_REVOLUTION = 102;  
// Wheel radius in meters
const double WHEEL_RADIUS = 0.04;
// Distance from center of the left wheel to the center of the right wheel in m
const double WHEEL_BASE = 0.218; 
// A wheel moves a 1 meter linear distance in the specified number of ticks. (TICKS_PER_REVOLUTION / 2*pi*r)
const double TICKS_PER_METER = 406; 
const int K_P = 278;
const int b = 52;
const int DRIFT_MULTIPLIER = 120;
const int PWM_TURN = 255;
const int PWM_MIN = 200; // about 0.5 m/s
const int PWM_MAX = 255; // about 0.7 m/s
double velFrontLeftWheel = 0;
double velFrontRightWheel = 0;
double velRearLeftWheel = 0;
double velRearRightWheel = 0;
double pwmFrontLeftReq = 0;
double pwmFrontRightReq = 0;
double pwmRearLeftReq = 0;
double pwmRearRightReq = 0;
// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;
// counts the number of ticks of each encoder
void frontright_wheel_tick() {
    int val = digitalRead(ENC_IN_FRONTRIGHT_B);
    if (val == LOW) {
        Direction_frontright = false; 
    }
    else {
        Direction_frontright = true;
    }
    if (Direction_frontright) {
        if (frontright_wheel_tick_count.data == encoder_maximum) {
            frontright_wheel_tick_count.data = encoder_minimum;
        }
        else {
            frontright_wheel_tick_count.data++;  
        }    
    }
    else {
        if (frontright_wheel_tick_count.data == encoder_minimum) {
            frontright_wheel_tick_count.data = encoder_maximum;
        }
        else {
            frontright_wheel_tick_count.data--;  
        }   
    }
}
void frontleft_wheel_tick() {
    int val = digitalRead(ENC_IN_FRONTLEFT_B);
    if (val == LOW) {
      Direction_frontleft = true; 
    }
    else {
      Direction_frontleft = false; 
    }
    if (Direction_frontleft) {
      if (frontleft_wheel_tick_count.data == encoder_maximum) {
        frontleft_wheel_tick_count.data = encoder_minimum;
      }
      else {
        frontleft_wheel_tick_count.data++;  
      }  
    }
    else {
      if (frontleft_wheel_tick_count.data == encoder_minimum) {
        frontleft_wheel_tick_count.data = encoder_maximum;
      }
      else {
        frontleft_wheel_tick_count.data--;  
      }   
    }
}
void rearright_wheel_tick() {
    int val = digitalRead(ENC_IN_REARRIGHT_B);
    if (val == LOW) {
        Direction_rearright = false; 
    }
    else {
        Direction_rearright = true; 
    }
    if (Direction_rearright) {
        if (rearright_wheel_tick_count.data == encoder_maximum) {
            rearright_wheel_tick_count.data = encoder_minimum;
        }
        else {
            rearright_wheel_tick_count.data++;  
        }    
    }
    else {
        if (rearright_wheel_tick_count.data == encoder_minimum) {
            rearright_wheel_tick_count.data = encoder_maximum;
        }
        else {
            rearright_wheel_tick_count.data--;  
        }   
    }
}
void rearleft_wheel_tick() {
    int val = digitalRead(ENC_IN_REARLEFT_B);
    if (val == LOW) {
        Direction_rearleft = false; 
    }
    else {
        Direction_rearleft = true; 
    }
   if (Direction_rearleft) {
        if (rearleft_wheel_tick_count.data == encoder_maximum) {
            rearleft_wheel_tick_count.data = encoder_minimum;
        }
        else {
            rearleft_wheel_tick_count.data++;  
        }    
    }
    else {
        if (rearleft_wheel_tick_count.data == encoder_minimum) {
            rearleft_wheel_tick_count.data = encoder_maximum;
        }
        else {
            rearleft_wheel_tick_count.data--;  
        }   
    }
}
// Calculate the wheels linear velocity in m/s every time 
void calc_vel_frontleft_wheel(){
  static double prevTime = 0;
  static int prevFrontLeftCount = 0;
  int numOfTicks = (65535 + frontleft_wheel_tick_count.data - prevFrontLeftCount) % 65535;
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
  velFrontLeftWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
 
  prevFrontLeftCount = frontleft_wheel_tick_count.data; 
  prevTime = (millis()/1000);
}
void calc_vel_frontright_wheel(){
  static double prevTime = 0;
  static int prevFrontRightCount = 0;
  int numOfTicks = (65535 +  frontright_wheel_tick_count.data - prevFrontRightCount) % 65535;
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
  velFrontRightWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
  prevFrontRightCount = frontright_wheel_tick_count.data;
  prevTime = (millis()/1000);
}
void calc_vel_rearleft_wheel(){
   static double prevTime = 0;
   static int prevRearLeftCount = 0; 
   int numOfTicks = (65535 + rearleft_wheel_tick_count.data - prevRearLeftCount) % 65535;
   if (numOfTicks > 10000) {
         numOfTicks = 0 - (65535 - numOfTicks);
   }
   velRearLeftWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
   prevRearLeftCount = rearleft_wheel_tick_count.data;
   prevTime = (millis()/1000);
}
void calc_vel_rearright_wheel(){
   static double prevTime = 0;
   static int prevRearRightCount = 0;
   int numOfTicks = (65535 + rearright_wheel_tick_count.data - prevRearRightCount) % 65535;
   if (numOfTicks > 10000) {
         numOfTicks = 0 - (65535 - numOfTicks);
   }
   velRearRightWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
   prevRearRightCount = rearright_wheel_tick_count.data;
   prevTime = (millis()/1000);
}
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
  lastCmdVelReceived = (millis()/1000);
  // Calculate the PWM value given the desired velocity
  pwmFrontLeftReq = K_P * cmdVel.linear.x + b;
  pwmFrontRightReq = K_P * cmdVel.linear.x + b;
  pwmFrontLeftReq = K_P * cmdVel.linear.x + b;
  pwmFrontRightReq = K_P * cmdVel.linear.x + b;
  if (cmdVel.linear.x == 0 && cmdVel.angular.z != 0.0) {
  // turn robot in left direction
    if (cmdVel.angular.z > 0.0) {
      pwmFrontLeftReq = -PWM_TURN;
      pwmFrontRightReq = PWM_TURN;
      pwmRearLeftReq = -PWM_TURN;
      pwmRearRightReq = PWM_TURN;
    }  
    // turn robot in right direction 
    else {
      pwmFrontLeftReq = PWM_TURN;
      pwmFrontRightReq = -PWM_TURN;
      pwmRearLeftReq = PWM_TURN;
      pwmRearRightReq = -PWM_TURN;
    }
  }
 // forward direction & reverse direction
  else if (cmdVel.linear.x != 0 && cmdVel.angular.z == 0.0) {

    static double prevFrontDiff = 0;
    static double prevRearDiff = 0;
    static double prevFrontPrevDiff = 0;
    static double prevRearPrevDiff = 0;
    double currFrontDifference = velFrontLeftWheel - velFrontRightWheel; 
    double currRearDifference = velRearLeftWheel - velRearRightWheel; 
    double avgFrontDifference = (prevFrontDiff+prevFrontDiff+currFrontDifference)/3;
    double avgRearDifference = (prevRearDiff+prevRearPrevDiff+currRearDifference)/3;
    prevFrontPrevDiff = prevFrontDiff;
    prevRearPrevDiff = prevRearDiff;
    prevFrontDiff = currFrontDifference;
    prevRearDiff = currRearDifference;
    pwmFrontLeftReq -= (int)(avgFrontDifference * DRIFT_MULTIPLIER);
    pwmFrontRightReq += (int)(avgFrontDifference * DRIFT_MULTIPLIER);
    pwmRearLeftReq -= (int)(avgRearDifference * DRIFT_MULTIPLIER);
    pwmRearRightReq += (int)(avgRearDifference * DRIFT_MULTIPLIER); 
  }
  else {
    pwmFrontLeftReq = 0;
    pwmFrontRightReq = 0;
    pwmRearLeftReq = 0;
    pwmRearRightReq = 0;
  }
  if (abs(pwmFrontLeftReq) < PWM_MIN) {
    pwmFrontLeftReq = 0;
  }
  if (abs(pwmFrontRightReq) < PWM_MIN) {
    pwmFrontRightReq = 0;  
  }  
  if (abs(pwmRearLeftReq) < PWM_MIN) {
    pwmRearLeftReq = 0;
  }
  if (abs(pwmRearRightReq) < PWM_MIN) {
    pwmRearRightReq = 0;  
  } 
}
void set_pwm_values() {
  static int pwmFrontLeftOut = 0;
  static int pwmFrontRightOut = 0;
  static int pwmRearLeftOut = 0;
  static int pwmRearRightOut = 0;
  static bool stopped = false;
  if ((pwmFrontLeftReq * velFrontLeftWheel < 0 && pwmFrontLeftOut != 0) ||
      (pwmFrontRightReq * velFrontRightWheel < 0 && pwmFrontRightOut != 0)) {
    pwmFrontLeftReq = 0;
    pwmFrontRightReq = 0;
  }
  if ((pwmRearLeftReq * velRearLeftWheel < 0 && pwmRearLeftOut != 0) ||
      (pwmRearRightReq * velRearRightWheel < 0 && pwmRearRightOut != 0)) {
    pwmRearLeftReq = 0;
    pwmRearRightReq = 0;
  }
  // Set the direction of the motors
  // Front left wheel forward
  if (pwmFrontLeftReq > 0) { 
    digitalWrite(in1_frontleft, HIGH);
    digitalWrite(in2_frontleft, LOW);
  }
  // Front left wheel reverse
  else if (pwmFrontLeftReq < 0) { 
    digitalWrite(in1_frontleft, LOW);
    digitalWrite(in2_frontleft, HIGH);
  }
  // Front left wheel stop
  else if (pwmFrontLeftReq == 0 && pwmFrontLeftOut == 0 ) { 
    digitalWrite(in1_frontleft, LOW);
    digitalWrite(in2_frontleft, LOW);
  }
  else {
    digitalWrite(in1_frontleft, LOW);
    digitalWrite(in2_frontleft, LOW); 
  }
  // Front right wheel forward
  if (pwmFrontRightReq > 0) { 
    digitalWrite(in3_frontright, HIGH);
    digitalWrite(in4_frontright, LOW);
  }
  // Front right wheel reverse
  else if (pwmFrontRightReq < 0) { 
    digitalWrite(in3_frontright, LOW);
    digitalWrite(in4_frontright, HIGH);
  }
  // Front right wheel stop
  else if (pwmFrontRightReq == 0 && pwmFrontRightOut == 0) { 
    digitalWrite(in3_frontright, LOW);
    digitalWrite(in4_frontright, LOW);
  }
  else {
    digitalWrite(in3_frontright, LOW);
    digitalWrite(in4_frontright, LOW); 
  }
  // rear left wheel forward
  if (pwmRearLeftReq > 0) { 
    digitalWrite(in1_rearleft, HIGH);
    digitalWrite(in2_rearleft, LOW);
  }
  // rear left wheel reverse
  else if (pwmRearLeftReq < 0) { 
    digitalWrite(in1_rearleft, LOW);
    digitalWrite(in2_rearleft, HIGH);
  }
  // rear left wheel stop
  else if (pwmRearLeftReq == 0 && pwmRearLeftOut == 0 ) { 
    digitalWrite(in1_rearleft, LOW);
    digitalWrite(in2_rearleft, LOW);
  }
  else { 
    digitalWrite(in1_rearleft, LOW);
    digitalWrite(in2_rearleft, LOW); 
  }
  // rear right wheel forward
  if (pwmRearRightReq > 0) { 
    digitalWrite(in3_rearright, HIGH);
    digitalWrite(in4_rearright, LOW);
  }
  // rear right wheel reverse
  else if (pwmRearRightReq < 0) { 
    digitalWrite(in3_rearright, LOW);
    digitalWrite(in4_rearright, HIGH);
  }
  // rear right wheel stop
  else if (pwmRearRightReq == 0 && pwmRearRightOut == 0) { 
    digitalWrite(in3_rearright, LOW);
    digitalWrite(in4_rearright, LOW);
  }
  else { 
    digitalWrite(in3_rearright, LOW);
    digitalWrite(in4_rearright, LOW); 
  }
  // Increase the required PWM when the robot is not moving
  if (pwmFrontLeftReq != 0 && velFrontLeftWheel == 0) {
    pwmFrontLeftReq *= 1.5;
  }
  if (pwmFrontRightReq != 0 && velFrontRightWheel == 0) {
    pwmFrontRightReq *= 1.5;
  }
  if (pwmRearLeftReq != 0 && velRearLeftWheel == 0) {
    pwmRearLeftReq *= 1.5;
  }
  if (pwmRearRightReq != 0 && velRearRightWheel == 0) {
    pwmRearRightReq *= 1.5;
  }
  if (abs(pwmFrontLeftReq) > pwmFrontLeftOut) {
    pwmFrontLeftOut += PWM_INCREMENT;
  }
  else if (abs(pwmFrontLeftReq) < pwmFrontLeftOut) {
    pwmFrontLeftOut -= PWM_INCREMENT;
  }
  else{}
  if (abs(pwmFrontRightReq) > pwmFrontRightOut) {
    pwmFrontRightOut += PWM_INCREMENT;
  }
  else if(abs(pwmFrontRightReq) < pwmFrontRightOut) {
    pwmFrontRightOut -= PWM_INCREMENT;
  }
  else{}
  if (abs(pwmRearLeftReq) > pwmRearLeftOut) {
    pwmRearLeftOut += PWM_INCREMENT;
  }
  else if (abs(pwmRearLeftReq) < pwmRearLeftOut) {
    pwmRearLeftOut -= PWM_INCREMENT;
  }
  else{}
  if (abs(pwmRearRightReq) > pwmRearRightOut) {
    pwmRearRightOut += PWM_INCREMENT;
  }
  else if(abs(pwmRearRightReq) < pwmRearRightOut) {
    pwmRearRightOut -= PWM_INCREMENT;
  }
  else{}
  // Conditional limit of PWM output to operate at the maximum 
  pwmFrontLeftOut = (pwmFrontLeftOut > PWM_MAX) ? PWM_MAX : pwmFrontLeftOut;
  pwmFrontRightOut = (pwmFrontRightOut > PWM_MAX) ? PWM_MAX : pwmFrontRightOut;
  pwmRearLeftOut = (pwmRearLeftOut > PWM_MAX) ? PWM_MAX : pwmRearLeftOut;
  pwmRearRightOut = (pwmRearRightOut > PWM_MAX) ? PWM_MAX : pwmRearRightOut;
  // PWM output cannot be less than 0
  pwmFrontLeftOut = (pwmFrontLeftOut < 0) ? 0 : pwmFrontLeftOut;
  pwmFrontRightOut = (pwmFrontRightOut < 0) ? 0 : pwmFrontRightOut;
  pwmRearLeftOut = (pwmRearLeftOut < 0) ? 0 : pwmRearLeftOut;
  pwmRearRightOut = (pwmRearRightOut < 0) ? 0 : pwmRearRightOut;
  analogWrite(enA_frontleft, pwmFrontLeftOut); 
  analogWrite(enB_frontright, pwmFrontRightOut);
  analogWrite(enA_rearleft, pwmRearLeftOut); 
  analogWrite(enB_rearright, pwmRearRightOut); 
}
// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values);
void setup() {
  pinMode(ENC_IN_FRONTLEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_FRONTLEFT_B , INPUT);
  pinMode(ENC_IN_FRONTRIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_FRONTRIGHT_B , INPUT);
  pinMode(ENC_IN_REARLEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_REARLEFT_B , INPUT);
  pinMode(ENC_IN_REARRIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_REARRIGHT_B , INPUT);
  // the ISR should trigger when the input signal goes from low to high
  attachInterrupt(digitalPinToInterrupt(ENC_IN_FRONTLEFT_A), frontleft_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_FRONTRIGHT_A), frontright_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_REARLEFT_A), rearleft_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_REARRIGHT_A), rearright_wheel_tick, RISING);
  pinMode(enA_frontleft, OUTPUT);
  pinMode(enB_frontright, OUTPUT);
  pinMode(enA_rearleft, OUTPUT);
  pinMode(enB_rearright, OUTPUT);
  pinMode(in1_frontleft, OUTPUT);
  pinMode(in2_frontleft, OUTPUT);
  pinMode(in3_frontright, OUTPUT);
  pinMode(in4_frontright, OUTPUT);
  pinMode(in1_rearleft, OUTPUT);
  pinMode(in2_rearleft, OUTPUT);
  pinMode(in3_rearright, OUTPUT);
  pinMode(in4_rearright, OUTPUT);
  // Turn off motors in initial state
  digitalWrite(in1_frontleft, LOW);
  digitalWrite(in2_frontleft, LOW);
  digitalWrite(in3_frontright, LOW);
  digitalWrite(in4_frontright, LOW);
  digitalWrite(in1_rearleft, LOW);
  digitalWrite(in2_rearleft, LOW);
  digitalWrite(in3_rearright, LOW);
  digitalWrite(in4_rearright, LOW);
  // Set the motor speed in initial state
  analogWrite(enA_frontleft, 0); 
  analogWrite(enB_frontright, 0);
  analogWrite(enA_rearleft, 0); 
  analogWrite(enB_rearright, 0);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(FrontLeftPub);
  nh.advertise(FrontRightPub);
  nh.advertise(RearLeftPub);
  nh.advertise(RearRightPub);
  nh.subscribe(subCmdVel);
}
void loop() {
  nh.spinOnce(); 
  currentMillis = millis();
  // If the time interval has passed, publish the number of ticks, and calculate the velocities.
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    // Publish tick counts to topics
    FrontLeftPub.publish( &frontleft_wheel_tick_count );
    FrontRightPub.publish( &frontright_wheel_tick_count );
    RearLeftPub.publish( &rearleft_wheel_tick_count );
    RearRightPub.publish( &rearright_wheel_tick_count );
    // Calculate the velocity of the wheels
    calc_vel_frontright_wheel();
    calc_vel_frontleft_wheel();
    calc_vel_rearright_wheel();
    calc_vel_rearleft_wheel();
     
  }
  if((millis()/1000) - lastCmdVelReceived > 1) {
    pwmFrontLeftReq = 0;
    pwmFrontRightReq = 0;
    pwmRearLeftReq = 0;
    pwmRearRightReq = 0;
  }
  set_pwm_values();
} 
