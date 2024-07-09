#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <Servo.h>
#include <math.h>

constexpr float BACKMOTOR_MAX = 255.0;
constexpr float BACKMOTOR_MIN = 0.0;
constexpr float FRONTMOTOR_MAX = 255.0;
constexpr float FRONTMOTOR_MIN = 0.0;

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
    void move(int power); // Modify for steering
};

void keyCallback(const std_msgs::Float32MultiArray &msg);

int mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh);

ros::NodeHandle nh;

std_msgs::Int32 pot_msg;
std_msgs::Int32 forward_msg;
std_msgs::Int32 backward_msg;
std_msgs::Int32 angle_msg;
ros::Publisher angle_pub("angle_topic", &pot_msg);
ros::Publisher forward_pub("forward", &forward_msg);
ros::Publisher backward_pub("backward", &backward_msg);
ros::Publisher angle_pub("angle", &angle_msg);

ros::Subscriber<std_msgs::Float32MultiArray> key_sub("control_input", &keyCallback);

const int potPin = A0; 
BackMyMotorControl leftMotor(22, 23, 5); 
BackMyMotorControl rightMotor(24, 25, 6); 
FrontMyMotorControl steeringMotor(26, 27, 9); 

void setup() {
    nh.initNode();
    nh.advertise(angle_pub);
    nh.advertise(forward_pub);
    nh.advertise(backward_pub);
    nh.advertise(angle_pub);
    nh.subscribe(key_sub);
}

void loop() {
    int potValue = analogRead(potPin);
    pot_msg.data = potValue;
    angle_pub.publish(&pot_msg);
    nh.spinOnce();
    delay(10);
}

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

FrontMyMotorControl::FrontMyMotorControl(int input1, int input2, int pwm)
: MyMotorControl(input1, input2, pwm) {}

void FrontMyMotorControl::move(int power) {
    if (power == 0) {
        stop();
    } else if (power > 0) {
        digitalWrite(input1, HIGH);
        digitalWrite(input2, LOW);
        analogWrite(pwm, abs(power));
    } else {
        digitalWrite(input1, LOW);
        digitalWrite(input2, HIGH);
        analogWrite(pwm, abs(power));
    }
}

void keyCallback(const std_msgs::Float32MultiArray &msg) {
    int forward = mapFloat(msg.data[0], 1.0, -1.0, BACKMOTOR_MIN, BACKMOTOR_MAX);
    int backward = mapFloat(msg.data[1], 1.0, -1.0, BACKMOTOR_MIN, BACKMOTOR_MAX);
    int angle = mapFloat(msg.data[2], -1.0, 1.0, FRONTMOTOR_MIN, FRONTMOTOR_MAX);

    forward_msg.data = forward;
    backward_msg.data = backward;
    angle_msg.data = angle;

    forward_pub.publish(&forward_msg);
    backward_pub.publish(&backward_msg);
    angle_pub.publish(&angle_msg);

    if (forward >= backward) {
        leftMotor.go(forward);
        rightMotor.go(forward);
    } else {
        leftMotor.back(backward);
        rightMotor.back(backward);
    }

    steeringMotor.move(angle);
}

int mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
    return int(round(toLow + (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow)));
}
