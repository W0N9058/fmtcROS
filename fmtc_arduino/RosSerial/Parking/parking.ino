#include <Servo.h>
#include <math.h>

constexpr float BACKMOTOR_MAX = 255.0;
constexpr float BACKMOTOR_MIN = 0.0;
constexpr float FRONTMOTOR_MAX = 800; // potentiometer 값으로 매핑하도록...
constexpr float FRONTMOTOR_MIN = 115;
constexpr float backmotor_synchro_coeff = 0.9; // 왼쪽 바퀴가 빠른 현상 억제 변수

// class
class MyMotorControl {
  protected:
    int input1; // digital
    int input2; // digital
    int pwm; // pwm

  public:
    MyMotorControl(int input1, int input2, int pwm);
    void stop();
    void changePower(int power);
};

class BackMyMotorControl : public MyMotorControl {
  public:
    BackMyMotorControl(int input1, int input2, int pwm);
    void go(int power);
    void back(int power);
};

class FrontMyMotorControl : public MyMotorControl {
  private:
    int prev_error;
    double integral;
  public:
    FrontMyMotorControl(int input1, int input2, int pwm);
    void move(int power);
};

// map function for float input
int mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh);

// 포텐시오미터값 전역변수
int pot_value = 500;

// 핀 설정
const int potPin = A0; // 포텐시오미터 핀 설정
BackMyMotorControl leftMotor(22, 23, 5); // 왼쪽 모터를 위한 핀 설정
BackMyMotorControl rightMotor(24, 25, 6); // 오른쪽 모터를 위한 핀 설정
FrontMyMotorControl steeringMotor(26, 27, 9); // 조향 모터를 위한 핀 설정

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  leftMotor.stop();
  rightMotor.stop();
  steeringMotor.stop();
  
}

void loop() {
    GO(1000);
    STEER('left');
    GO(1000);
    STEER('right');
    BACK(1000);
    STEER('center');
    BACK(1000);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// MyMotorControl
MyMotorControl::MyMotorControl(int input1, int input2, int pwm) {
  this->input1 = input1;
  this->input2 = input2;
  this->pwm = pwm;

  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);
  pinMode(pwm, OUTPUT);
}

void MyMotorControl::stop() {
  digitalWrite(input1, LOW);
  digitalWrite(input2, LOW);
  analogWrite(pwm, 0);
}

void MyMotorControl::changePower(int power) {
  analogWrite(pwm, power);
}

// BackMyMotorControl
BackMyMotorControl::BackMyMotorControl(int input1, int input2, int pwm)
  : MyMotorControl(input1, input2, pwm) {}

void BackMyMotorControl::go(int power) {
  digitalWrite(input1, LOW);
  digitalWrite(input2, HIGH);
  analogWrite(pwm, power);
}

void BackMyMotorControl::back(int power) {
  digitalWrite(input1, HIGH);
  digitalWrite(input2, LOW);
  analogWrite(pwm, power);
}

// FrontMyMotorControl
FrontMyMotorControl::FrontMyMotorControl(int input1, int input2, int pwm)
  : MyMotorControl(input1, input2, pwm) {}

void FrontMyMotorControl::move(int target_angle) {
  int error_angle = target_angle - pot_value;
  double Kp = 1; 
  double Ki = 0.01;  
  double Kd = 0.02;

  integral += error_angle;
  double derivative = error_angle - prev_error;

  double control_value = Kp * error_angle + Ki * integral + Kd * derivative;
  control_value = constrain(control_value, -255, 255);

  // 방향 제어 및 속도 제어
  if (control_value > 0) { // 좌회전해야하는 상황
    digitalWrite(input1, LOW);
    digitalWrite(input2, HIGH);
    analogWrite(pwm, abs(control_value));
  } else if (control_value == 0) {
    stop();
  } else { // 우회전해야하는 상황
    digitalWrite(input1, HIGH);
    digitalWrite(input2, LOW);
    analogWrite(pwm, abs(control_value));
  }
  prev_error = error_angle;
  
}

int mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return int(round(toLow + (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow)));
}

//////////////////// 주차 미션을 위해 추가된 함수들
void GO(int duration) {
  int power = 150;
  leftMotor.go(power*backmotor_synchro_coeff);
  rightMotor.go(power);
  delay(duration);
  leftMotor.stop();
  rightMotor.stop();
}

void BACK(int duration) {
  int power = 150;
  leftMotor.back(power*backmotor_synchro_coeff);
  rightMotor.back(power);
  delay(duration);
  leftMotor.stop();
  rightMotor.stop();
}

void STEER(const char* direction) {
  int target_angle;
  if (strcmp(direction, "left") == 0) {
    target_angle = FRONTMOTOR_MAX;
  } else if (strcmp(direction, "right") == 0) {
    target_angle = FRONTMOTOR_MIN;
  } else if (strcmp(direction, "center") == 0) {
    target_angle = (FRONTMOTOR_MAX + FRONTMOTOR_MIN) / 2;
  } else {
    return; // Invalid direction
  }

  unsigned long start_time = millis();
  while (millis() - start_time < 1000) {
    steeringMotor.move(target_angle);
    delay(10);
  }
  steeringMotor.stop();
}
