#include <ros.h>
#include <IBusBM.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h>

// Create iBus Object
IBusBM ibus;
ros::NodeHandle nh;

void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg);

int MotorSpeedA;
int MotorSpeedB;

// Channel Values
int rcCH1 = 0; // Left - Right
int rcCH2 = 0; // Forward - Reverse
int rcCH3 = 0; // Acceleration
int rcCH5 = 0; // Spin Control
bool rcCH6 = 0; // Mode Control

// LED Connection
const int carLED = 13;

// Motor A Control Connections
const int pwmA = 7;
const int in1A = 42;
const int in2A = 40;
const int stby = 6;

// Motor B Control Connections
const int pwmB = 6;
const int in1B = 22;
const int in2B = 24;

// Dead zone to avoid small joystick fluctuations
const int deadZone = 10;

// Function to control a motor
void controlMotor(int mspeed, int in1, int in2, int pwm) {
  // Constrain speed
  int constrainedSpeed = constrain(mspeed, -155, 155);

  // Determine direction
  if (constrainedSpeed >= 0) {
    // Motor forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    // Motor backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  // Control motor
  int pwm_value = map(abs(constrainedSpeed), 0, 155, 1000, 2000);
  pwm_value = constrain(pwm_value, 1000, 2000); // Constrain within acceptable PWM range
  analogWrite(pwm, pwm_value);
}

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
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

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmd_vel_callback);

bool rosNodeActive = false;

void setup() {
  // Start serial monitor for debugging
  Serial1.begin(115200);
  Serial.begin(57600);

  // Attach iBus object to the serial port
  ibus.begin(Serial1);

  nh.initNode();
  nh.subscribe(cmd_vel_sub);

  // Set all the motor control pins to outputs
  pinMode(pwmA, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(stby, OUTPUT);

  // Set Motor B control pins to outputs
  pinMode(pwmB, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  // Set LED pin as an output
  pinMode(carLED, OUTPUT);

  // Keep motors on standby for two seconds & flash LED
  digitalWrite(stby, LOW);
  digitalWrite(carLED, HIGH);
  delay(1000);
  digitalWrite(carLED, LOW);
  delay(1000);
  digitalWrite(stby, HIGH);
}

void loop() {
  if (Serial.available()) {
    char data = Serial.read();
    Serial.write(data);
    Serial.print("Sent to Serial: ");
    Serial.println(data);
  }

  if (Serial1.available()) {
    char data = Serial1.read();
    Serial1.write(data);
    Serial.print("Received from Serial1: ");
    Serial.println(data);
  }
  // Get RC channel values
  rcCH1 = readChannel(0, -255, -1, 0);
  rcCH2 = readChannel(1, -155, 155, 0);
  rcCH3 = readChannel(2, -60, 35, 0);
  rcCH5 = readChannel(4, -100, 100, 0);
  rcCH6 = readSwitch(5, false);

  // Print values to the serial monitor for debugging
  Serial.print("Ch1 = ");
  Serial.print(rcCH1);

  Serial.print(" Ch2 = ");
  Serial.print(rcCH2);

  Serial.print(" Ch3 = ");
  Serial.print(rcCH3);

  Serial.print(" Ch5 = ");
  Serial.print(rcCH5);

  Serial.print(" Ch6 = ");
  Serial.println(rcCH6);

  // Set speeds with channel 3 value
  MotorSpeedA = rcCH3;
  MotorSpeedB = rcCH3;

  // Apply dead zone to joystick input
  if (abs(rcCH1) < deadZone) {
    rcCH1 = 0;
  }

  if (abs(rcCH2) < deadZone) {
    rcCH2 = 0;
  }

  // Debounce joystick input
  static unsigned long lastDebounceTime1 = 0;
  static unsigned long lastDebounceTime2 = 0;
  unsigned long debounceDelay = 50; // Adjust this value as needed

  if (millis() - lastDebounceTime1 > debounceDelay) {
    lastDebounceTime1 = millis();
    // Control Motor A
    controlMotor(rcCH1, in1A, in2A, pwmA);
  }

  if (millis() - lastDebounceTime2 > debounceDelay) {
    lastDebounceTime2 = millis();
    // Control Motor B
    controlMotor(rcCH2, in1B, in2B, pwmB);
  }

  nh.spinOnce();
}

void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg) {
  float linear_velocity = 2 * cmd_vel_msg.linear.x;
  float angular_velocity = 2* cmd_vel_msg.angular.z;

  // Calculate motor speeds based on linear and angular velocities
  MotorSpeedA = linear_velocity - angular_velocity;
  MotorSpeedB = linear_velocity + angular_velocity;

  // Control each motor independently
  controlMotor(MotorSpeedA, in1A, in2A, pwmA); // Motor A
  controlMotor(MotorSpeedB, in1B, in2B, pwmB); // Motor B
}