#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <Servo.h>
#include <math.h>
#include <std_msgs/Float32MultiArray.h>

constexpr float BACKMOTOR_MAX = 255.0;
constexpr float BACKMOTOR_MIN = 0.0;
constexpr float FRONTMOTOR_MAX = 800; // potentiometer 값으로 매핑하도록...
constexpr float FRONTMOTOR_MIN = 115;

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
  public:
    FrontMyMotorControl(int input1, int input2, int pwm);
    void move(int power);
};

// 키보드입력 받아서 모터 조작하는 함수
void keyCallback(const std_msgs::Float32MultiArray &msg);

// map function for float input
int mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh);

// 포텐시오미터값 전역변수
int pot_value = 500;

// ROS NodeHandle
ros::NodeHandle nh;

// 포텐시오미터 메시지와 퍼블리셔 선언
std_msgs::Int32 pot_angle_msg;
std_msgs::Int32 forward_msg;
std_msgs::Int32 backward_msg;
std_msgs::Int32 joy_angle_msg;
ros::Publisher pot_angle_pub("pot_angle", &pot_angle_msg);
ros::Publisher forward_pub("forward", &forward_msg);
ros::Publisher backward_pub("backward", &backward_msg);
ros::Publisher joy_angle_pub("joy_angle", &joy_angle_msg);

// 키보드 입력 메시지와 구독자 선언
ros::Subscriber<std_msgs::Float32MultiArray> key_sub("control_input", &keyCallback);

// 핀 설정
const int potPin = A0; // 포텐시오미터 핀 설정
BackMyMotorControl leftMotor(22, 23, 5); // 왼쪽 모터를 위한 핀 설정
BackMyMotorControl rightMotor(24, 25, 6); // 오른쪽 모터를 위한 핀 설정
FrontMyMotorControl steeringMotor(26, 27, 9); // 조향 모터를 위한 핀 설정

void setup() {
  // ROS 노드 초기화
  nh.initNode();
  
  // 퍼블리셔와 구독자 설정
  nh.advertise(pot_angle_pub);
  nh.advertise(forward_pub);
  nh.advertise(backward_pub);
  nh.advertise(joy_angle_pub);
  nh.subscribe(key_sub);
}

void loop() {
  // 포텐시오미터 값 읽기
  potValue = analogRead(potPin);
  
  // 메시지에 값 설정
  pot_angle_msg.data = potValue;

  // 메시지 퍼블리시
  pot_angle_pub.publish(&pot_angle_msg);
  
  // ROS 핸들러 스핀
  nh.spinOnce();

  // 100ms 지연
  delay(10);
}

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
  int error_angle = target_angle - pot_value
  if (error_angle > 0){ // 좌회전해야하는 상황
    digitalWrite(input1, HIGH);
    digitalWrite(input2, LOW);
    analogWrite(pwm, abs(power));
  } else if (error_angle == 0) {
    stop()
  } else { // 우회전해야하는 상황
    digitalWrite(input1, LOW);
    digitalWrite(input2, HIGH);
    analogWrite(pwm, abs(power));
  }
}


// Xbox input callback function
void keyCallback(const std_msgs::Float32MultiArray &msg) {
  int forward = mapFloat(msg.data[0], 1.0, -1.0, BACKMOTOR_MIN, BACKMOTOR_MAX);
  int backward = mapFloat(msg.data[1], 1.0, -1.0, BACKMOTOR_MIN, BACKMOTOR_MAX);
  int joy_angle = mapFloat(msg.data[2], -1.0, 1.0, -FRONTMOTOR_MAX, FRONTMOTOR_MAX);

  //debugging
//  pwm_msg.data = [forward, backward, angle];
  forward_msg.data = forward;
  backward_msg.data = backward;
  joy_angle_msg.data = joy_angle;
  forward_pub.publish(&forward_msg);
  backward_pub.publish(&backward_msg);
  joy_angle_pub.publish(&joy_angle_msg);


  if (forward >= backward) {
    leftMotor.go(forward);
    rightMotor.go(forward);
  } else {
    leftMotor.back(backward);
    rightMotor.back(backward);
  }

  // positive input is left!!
  steeringMotor.move(joy_angle);
}

int mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return int(round(toLow + (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow)));
}
