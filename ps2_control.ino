#include <PS2X_lib.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <MotorDriver.h>
#include <Servo.h>

// Initialization
PS2X ps2x;
MotorDriver motor;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
SimpleServo myServo;

int servoAngle = 90;
unsigned long pressed_time;
unsigned long released_time;
unsigned long lastInputTime;
int left_speed = 0;
int right_speed = 0;

void setServo(uint8_t channel, uint16_t pulse) {
  pwm.setPWM(channel, 0, pulse);
}

void deceleration(int speed) {
    const int step = 1000;

    if (speed > 0) {
        speed -= step;
        if (speed < 0) speed = 0;
    } else if (speed < 0) {
        speed += step;
        if (speed > 0) speed = 0;
    }
    return speed;
}

void setup() {
  Serial.begin(115200);

  // Motor setup
  motor.begin();
  pwm.begin();
  pwm.setPWMFreq(50); 
  motor.setMotorSpeed(0, 0, 0, 0);

  // PS2 setup
  int error = -1; 
  for (int i = 0; i < 10; i++) // thử kết nối với tay cầm ps2 trong 10 lần 
  {
    delay(1000); // đợi 1 giây 
    // cài đặt chân và các chế độ: GamePad
    error = ps2x.config_gamepad(14, 13, 15, 12, false, false); 
    Serial.print("."); 
    if(!error) //kiểm tra nếu tay cầm đã kết nối thành công 
    break; // thoát khỏi vòng lặp 
  } 
}

void loop() {
  bool anyPressed = false;
  ps2x.read_gamepad(false, false);

  // Đọc joystick trái (điều khiển motor tiến/lùi)
  int ly = ps2x.Analog(PSS_LY); // 0 - 255, giữa là ~128

  // Movement
  if (ps2x.Button(PSB_PAD_LEFT)) {    // LEFT
    lastInputTime = millis();
    anyPressed = true;
    right_speed += 400;
  }
  if (ps2x.Button(PSB_PAD_RIGHT)) {   // RIGHT
    lastInputTime = millis();
    anyPressed = true;
    left_speed -= 400;
  }
  if (ps2x.Button(PSB_PAD_UP)) {      // UP
    lastInputTime = millis();
    anyPressed = true;
    left_speed -= 300;
    right_speed += 300;
  }
  if (ps2x.Button(PSB_PAD_DOWN)) {    // DOWN
    lastInputTime = millis();
    anyPressed = true;
    left_speed += 300;
    right_speed -= 300;
  }

  if (!anyPressed && (millis() - lastInputTime > 500)) { // Reduce speed to 0 when idling for more than 500ms
    left_speed = deceleration(left_speed)
    right_speed = deceleration(right_speed)
  }

  motor.setMotorSpeed(0, 0, left_speed, right_speed); // Execute
  //delay(50); No longer required, best to add to reduce serial speed for debugging
  

  //Servo điều khiển bằng joystick phải trục X
  int rx = ps2x.Analog(PSS_RX); // 0 - 255
  servoAngle = map(rx, 0, 255, 100, 550);
  pwm.setPWM(2, 0, servoAngle);

  delay(20);
}
