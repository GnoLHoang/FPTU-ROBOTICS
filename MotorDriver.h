#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

class MotorDriver {
  public:
    MotorDriver();

    void begin();
    void setMotorSpeed(int speedR1, int speedR2, int speedR3, int speedR4);
    void testMotors();

  private:
    Adafruit_PWMServoDriver pwm;

    const int MOTOR_1_CHANNEL_A = 8;
    const int MOTOR_1_CHANNEL_B = 9;
    const int MOTOR_2_CHANNEL_A = 10;
    const int MOTOR_2_CHANNEL_B = 11;
    const int MOTOR_3_CHANNEL_A = 12;
    const int MOTOR_3_CHANNEL_B = 13;
    const int MOTOR_4_CHANNEL_A = 14;
    const int MOTOR_4_CHANNEL_B = 15;

    const int NOTIFY_LED = 13;
};

#endif
