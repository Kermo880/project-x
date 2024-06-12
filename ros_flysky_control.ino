#include <IBusBM.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>

IBusBM ibus;
ros::NodeHandle nh;

const int motorPin = 6;
const int motorPin2 = 7;
const int filterRes = 1000;
const int filterCap = 100;

float robot_x, robot_y;

void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg); // Function declaration (remove duplicate)
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmd_vel_callback);

// Function to read RC channel with error handling
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
  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);
  pinMode(filterCap, OUTPUT);
  digitalWrite(filterCap, LOW);

  nh.initNode();
  nh.subscribe(cmd_vel_sub);
}

void loop() {
  int rcCH5 = readSwitch(4, false);

  if (rcCH5 == true) {
    digitalWrite(filterCap, HIGH);
    delayMicroseconds(filterRes * filterCap / 1000000.0);
    digitalWrite(filterCap, LOW);
    remoteControl();
  } else {
    nh.spinOnce();
  }
}

void remoteControl() {
  // Read the values from RC channels 1, 3, and 6
  int rcCH1 = readChannel(0, -100, 100, 0);  // Steering adjustment
  int rcCH3 = readChannel(2, 100, 250, 175);  // Motor 1 speed
  int rcCH6 = readSwitch(5, false);       // ROS/Remote switch

  int pwmValue = 0;
  int pwmValue1 = 0;

  // Map the RC channel 3 value to the PWM range for motor 1
  pwmValue = map(rcCH3, 105, 255, 105, 255);

  // Calculate motor speeds based on steering adjustment
  pwmValue = max(min(rcCH3 + rcCH1, 210), 145);
  pwmValue1 = max(min(rcCH3 - rcCH1, 210), 145);

  // Write the motor speeds to the motor pins
  analogWrite(motorPin, pwmValue);
  analogWrite(motorPin2, pwmValue1);
}

// Function definition for cmd_vel_callback
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg) {
  float linear_velocity = cmd_vel_msg.linear.x;
  float angular_velocity = cmd_vel_msg.angular.z;

  int pwmValue = 0;
  int pwmValue1 = 0;

  int pwmValue_linear = map(linear_velocity, -1.5, 1.5, 100, 250);
  int pwmValue_angular = map(angular_velocity, -1.0, 1.0, -100, 100);

  pwmValue = pwmValue_linear + pwmValue_angular;
  pwmValue1 = pwmValue_linear - pwmValue_angular;

  pwmValue = constrain(pwmValue, 145, 210);
  pwmValue1 = constrain(pwmValue1, 145, 210);

  analogWrite(motorPin, pwmValue);
  analogWrite(motorPin2, pwmValue1);
}
