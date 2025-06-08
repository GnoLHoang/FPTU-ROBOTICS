#include "MotorDriver.h"

MotorDriver::MotorDriver() : pwm(Adafruit_PWMServoDriver()) {}

void MotorDriver::begin() {
  pwm.begin();
  pwm.setPWMFreq(50);
  pinMode(NOTIFY_LED, OUTPUT);
  Serial.begin(115200);
  Serial.println("Motor test firmware for MakerBot 2024");
}

void MotorDriver::setMotorSpeed(int speedR1, int speedR2, int speedR3, int speedR4) {
  auto setSingleMotor = [&](int speed, int chA, int chB) {
    if (speed >= 0) {
      pwm.setPin(chA, speed);
      pwm.setPin(chB, 0);
    } else {
      pwm.setPin(chA, 0);
      pwm.setPin(chB, abs(speed));
    }
  };

  setSingleMotor(speedR1, MOTOR_1_CHANNEL_A, MOTOR_1_CHANNEL_B);
  setSingleMotor(speedR2, MOTOR_2_CHANNEL_A, MOTOR_2_CHANNEL_B);
  setSingleMotor(speedR3, MOTOR_3_CHANNEL_A, MOTOR_3_CHANNEL_B);
  setSingleMotor(speedR4, MOTOR_4_CHANNEL_A, MOTOR_4_CHANNEL_B);
}

void MotorDriver::testMotors() {
  Serial.println("Starting sequentially test...");

  for (int i = 0; i < 3; i++) {
    Serial.println(3 - i);
    digitalWrite(NOTIFY_LED, HIGH);
    delay(500);
    digitalWrite(NOTIFY_LED, LOW);
    delay(500);
  }

  for (int i = 8; i <= 15; i++) {
    for (int j = 8; j <= 15; j++) pwm.setPin(j, 0);

    for (int pwm_val = 0; pwm_val <= 4000; pwm_val += 200) {
      Serial.print("Motor channel ");
      Serial.print(i);
      Serial.print(" running at ");
      Serial.println(pwm_val);
      pwm.setPin(i, pwm_val);
      delay(200);
    }

    pwm.setPin(i, 0);
  }

  Serial.println("Starting simultaneously test...");

  digitalWrite(NOTIFY_LED, HIGH);
  delay(500);
  digitalWrite(NOTIFY_LED, LOW);
  delay(200);

  Serial.println("Motor 1 & 3 forward, Motor 2 & 4 backward");
  setMotorSpeed(2000, -2000, 2000, -2000);
  delay(3000);

  Serial.println("All motor stop");
  setMotorSpeed(0, 0, 0, 0);
  delay(500);

  Serial.println("Motor 1 & 3 backward, Motor 2 & 4 forward");
  setMotorSpeed(-3500, 3500, -3500, 3500);
  delay(3000);

  Serial.println("All motor stop");
  setMotorSpeed(0, 0, 0, 0);
  delay(500);

  Serial.println("Motor 1 & 3 forward, Motor 2 & 4 backward");
  setMotorSpeed(3500, -3500, 3500, -3500);
  delay(3000);

  Serial.println("All motor stop");
  setMotorSpeed(0, 0, 0, 0);
  delay(500);

  Serial.println("Motor 1 & 3 backward, Motor 2 & 4 forward");
  setMotorSpeed(-3500, 3500, -3500, 3500);
  delay(3000);

  Serial.println("All motor stop");
  setMotorSpeed(0, 0, 0, 0);

  Serial.println("Test completed. Restarting in 5 seconds.");
  delay(5000);
}
