#include <ros.h>
#include <IBusBM.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;
IBusBM ibus;

const int motorPin = 7;
const int motorPin2 = 6;
const int stby = 6;

int pwmValue = 0;
int pwmValue1 = 0;

bool obstacle_detected = false;
std_msgs::Bool obstacle_msg;
void obstacle_callback(const std_msgs::Bool& msg);

geometry_msgs::Twist cmd_vel_msg;
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg);

ros::Subscriber<std_msgs::Bool> obstacle_sub("/lidar_pimp", &obstacle_callback);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmd_vel_callback);

void obstacle_callback(const std_msgs::Bool& msg) {
  obstacle_detected = msg.data;
}

void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg) {
  float linear_velocity = cmd_vel_msg.linear.x;
  float angular_velocity = cmd_vel_msg.angular.z;

  if (!obstacle_detected) {
    if (linear_velocity > 0.0) {
    // Forward motion
    pwmValue = map(linear_velocity + angular_velocity, 1.5, -1.5, 180, 210);
    pwmValue1 = map(linear_velocity + angular_velocity, 1.5, -1.5, 180, 210);
  } else if (linear_velocity < 0.0) {
    // Backward motion
    pwmValue = map(linear_velocity - angular_velocity, 1.5, -1.5, 185, 155);
    pwmValue1 = map(linear_velocity - angular_velocity, 1.5, -1.5, 175, 145);
  } else if (angular_velocity > 0.0) {
    // Left motion
    pwmValue = map(angular_velocity, -1.0, 1.0, 180, 210);
    pwmValue1 = map(angular_velocity, -1.0, 1.0, 175, 145);
  } else if (angular_velocity < 0.0) {
    // Right motion
    pwmValue = map(angular_velocity, 1.0, -1.0, 185, 155);
    pwmValue1 = map(angular_velocity, 1.0, -1.0, 180, 210);
  }

    analogWrite(motorPin, pwmValue);
    analogWrite(motorPin2, pwmValue1);
  }
}

void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(obstacle_sub);
  nh.subscribe(cmd_vel_sub);
  pinMode(motorPin, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(stby, OUTPUT);
  digitalWrite(stby, HIGH);
}

void loop() {
  nh.spinOnce();

  if (obstacle_detected) {
    analogWrite(motorPin, 175);
    analogWrite(motorPin2, 175);
  }
}
