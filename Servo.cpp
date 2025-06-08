#include "Servo.h"

SimpleServo::SimpleServo() {
    _pin = 255;
    _attached = false;
    _angle = 90;
}

void SimpleServo::attach(uint8_t pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
    _attached = true;
    write(_angle);
}

void SimpleServo::write(int angle) {
    if (!_attached) return;
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    _angle = angle;
    writePulse(angle);
}

void SimpleServo::writePulse(int angle) {
    int pulseWidth = map(angle, 0, 180, 500, 2400);  // microseconds
    digitalWrite(_pin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(_pin, LOW);
    delay(20);  // Refresh rate ~50Hz
}

void SimpleServo::detach() {
    if (_attached) {
        digitalWrite(_pin, LOW);
        _attached = false;
    }
}