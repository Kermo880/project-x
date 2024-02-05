#include <IBusBM.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

IBusBM ibus;
ros::NodeHandle nh;

const int motorPin = 7;
const int motorPin2 = 6;
const int stby = 6;

void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmd_vel_callback);

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = defaultValue ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void setup() {
  Serial1.begin(57600);
  ibus.begin(Serial1);
  pinMode(motorPin, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(stby, OUTPUT);
  digitalWrite(stby, HIGH);

  nh.initNode();
  nh.subscribe(cmd_vel_sub);
}

void loop() {
  int rcCH5 = readSwitch(4, false);

  if(rcCH5 == true){
    remoteControl();
  } else {
    nh.spinOnce();
  }
}

void remoteControl(){
  int rcCH1 = readChannel(0, -100, 100, 0);
  int rcCH3 = readChannel(2, 105, 250, 0);
  int rcCH6 = readSwitch(5, false);

  int pwmValue = rcCH3;
  int pwmValue1 = rcCH3;
  if (rcCH3 >= 180 && rcCH6 == true && rcCH1 >= 10) {
    // Right motion
    pwmValue = map(rcCH3, 175, 105, 180, 250);
    pwmValue1 = rcCH3;
    analogWrite(motorPin, pwmValue);
    analogWrite(motorPin2, pwmValue1);
  } else if (rcCH3 <= 175 && rcCH6 == true && rcCH1 <= -10) {
    // Left motion
    pwmValue = map(rcCH3, 180, 250, 175, 105);
    pwmValue1 = rcCH3;
    analogWrite(motorPin, pwmValue);
    analogWrite(motorPin2, pwmValue1);
  } else if (rcCH3 >= 180 && rcCH6 == true){
    // Forward motion
    pwmValue = rcCH3;
    pwmValue1 = rcCH3;
    analogWrite(motorPin, pwmValue);
    analogWrite(motorPin2, pwmValue1);
  } else if(rcCH3 <= 175 && rcCH6 == true) {
    // Backward motion
    pwmValue = rcCH3;
    pwmValue1 = rcCH3;
    analogWrite(motorPin, pwmValue);
    analogWrite(motorPin2, pwmValue1);
  } else {
    pwmValue = 0;
    pwmValue1 = 0;
    analogWrite(motorPin, pwmValue);
    analogWrite(motorPin2, pwmValue1);
  }
}

void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg) {
  float linear_velocity = cmd_vel_msg.linear.x;
  float angular_velocity = cmd_vel_msg.angular.z;

  // Calculate motor speeds based on linear and angular velocities
  int pwmValue = 0;  // Placeholder for motor 1
  int pwmValue1 = 0; // Placeholder for motor 2

  if (linear_velocity > 0.0) {
    // Forward motion
    pwmValue = map(linear_velocity + angular_velocity, 1.5, -1.5, 180, 210);
    pwmValue1 = map(linear_velocity + angular_velocity, 1.5, -1.5, 180, 210);
  } else if (linear_velocity < 0.0) {
    // Backward motion
    pwmValue = map(linear_velocity - angular_velocity, 1.5, -1.5, 175, 145);
    pwmValue1 = map(linear_velocity - angular_velocity, 1.5, -1.5, 175, 145);
  } else if (angular_velocity > 0.0) {
    // Left motion
    pwmValue = map(angular_velocity, -1.0, 1.0, 180, 210);
    pwmValue1 = map(angular_velocity, -1.0, 1.0, 175, 145);
  } else if (angular_velocity < 0.0) {
    // Right motion
    pwmValue = map(angular_velocity, 1.0, -1.0, 175, 145);
    pwmValue1 = map(angular_velocity, 1.0, -1.0, 180, 210);
  }

  // Control each motor independently using pwmValue
  analogWrite(motorPin, pwmValue);
  analogWrite(motorPin2, pwmValue1);
}