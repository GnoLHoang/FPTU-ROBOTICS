#ifndef Servo.h
#define Servo.h

#include <Arduino.h>

class SimpleServo {
public:
    SimpleServo();
    void attach(uint8_t pin);
    void write(int angle);     // Góc từ 0 đến 180
    void detach();

private:
    uint8_t _pin;
    bool _attached;
    int _angle;
    void writePulse(int angle);
};

#endif