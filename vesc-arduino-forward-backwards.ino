const int stby = 6;
const int motorPin = 7;
const int minimumPWM = 180;

void setup() {
  Serial.begin(115200);
  pinMode(motorPin, OUTPUT);
  digitalWrite(stby, HIGH);
}

void loop() {
  // Forward motion for 5 seconds
  for (int pwmValue = minimumPWM; pwmValue <= 255; pwmValue += 10) {
    analogWrite(motorPin, pwmValue);
    Serial.print("PWM Value (Forward): ");
    Serial.println(pwmValue);
    delay(500);
  }

  // Stop before going backward
  analogWrite(motorPin, 0);
  delay(1000);

  // Backward motion for 5 seconds
  for (int pwmValue = 175; pwmValue >= 105; pwmValue -= 10) {
    analogWrite(motorPin, pwmValue);
    Serial.print("PWM Value (Backward): ");
    Serial.println(pwmValue);
    delay(500);
  }

  // Stop and reset for the next iteration
  analogWrite(motorPin, 0);
  delay(1000);
}