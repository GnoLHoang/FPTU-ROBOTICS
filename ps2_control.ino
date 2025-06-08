#include <PS2X_lib.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <MotorDriver.h>
#include <Servo.h>

PS2X ps2x;
MotorDriver motor;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
SimpleServo myServo;

int servoAngle = 90;

void setServo(uint8_t channel, uint16_t pulse) {
  pwm.setPWM(channel, 0, pulse);
}

void setup() {
  Serial.begin(115200);

  // Motor setup
  motor.begin();
  pwm.begin();
  pwm.setPWMFreq(50); 
  motor.setMotorSpeed(0, 0, 0, 0);

  // Servo setup
  // myServo.attach(6); // Gắn servo vào chân số 6
  // myServo.write(servoAngle);

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
  int speed = 0;
  ps2x.read_gamepad(false, false);

  // Đọc joystick trái (điều khiển motor tiến/lùi)
  int ly = ps2x.Analog(PSS_LY); // 0 - 255, giữa là ~128

  speed = map(ly, 0, 255, 4095, -4095); // Joystick lên là tiến

  // Xử lý xoay trái/phải nếu bấm nút
  if (ps2x.Button(PSB_R1)) {
    motor.setMotorSpeed(0, 0, 2000, -2000); // quay phải
  } else if (ps2x.Button(PSB_L1)) {
    motor.setMotorSpeed(0, 0, -2000, 2000); // quay trái
  } else {
    motor.setMotorSpeed(0,0,0,0);
  }

  //servo
  // Set servo to 0 before test
  // setServo(2, 205); 
  // Serial.print("set 0");
  // delay(3000);

  // Sweep from 0 to 4000 in steps of 100
// for (int angle = 0; angle <= 180; angle += 10) {
//   int pwm_val = map(angle, 0, 180, 100, 550); // ánh xạ góc về pwm
//   Serial.print("Angle: ");
//   Serial.print(angle);
//   Serial.print(" => PWM: ");
//   Serial.println(pwm_val);

//   pwm.setPWM(2, 0, pwm_val);
//   delay(300);
// }

// for (int pwm_val = 100; pwm_val <= 600; pwm_val += 10) {
//   pwm.setPWM(2, 0, pwm_val);
//   Serial.print("PWM: ");
//   Serial.println(pwm_val);
//   delay(500);
// }


  // Set servo to 0 after test
  // setServo(2, 0); 

  if (speed >= 0)
  {
    motor.setMotorSpeed(0, 0, speed, speed);
  }
  else
  {
    motor.setMotorSpeed(0, 0, 0, 0);
  }
  motor.setMotorSpeed(0, 0, 0, 0);

  //Servo điều khiển bằng joystick phải trục X
  int rx = ps2x.Analog(PSS_RX); // 0 - 255
  servoAngle = map(rx, 0, 255, 100, 550);
  pwm.setPWM(2, 0, servoAngle);

  delay(20);
}
