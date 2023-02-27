#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Bool.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Broken PIN: 4,12,52,53

// PUB/SUB sampling time
#define SAMPLING_TIME 10

// Ultrasonic
#define TRIGPIN 11
#define ECHOPIN 10

// PCA9685
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define S0MIN 80
#define S0MAX 265 
#define S1MIN 80
#define S1MAX 265
#define S2MIN 80
#define S2MAX 265
#define S3MIN 80
#define S3MAX 265

// Trolley Motor
#define RPWM1 6
#define R_EN1 26
#define R_IS1 28
#define LPWM1 5
#define L_EN1 24
#define L_IS1 22

// Hoisting Motor
#define RPWM2 3
#define R_EN2 27
#define R_IS2 29
#define LPWM2 2
#define L_EN2 25
#define L_IS2 23

// Limit Switch
#define LEFT 47
#define RIGHT 38
#define LEDEBUG 44

// Incremental Encoder
#define ENC_A 18
#define ENC_B 19

volatile unsigned int counter = 0;

void ai0(){
if(digitalRead(ENC_B)==LOW) {
    counter++;
  }
  else{
    counter--;
  }
}  

void ai1(){
if(digitalRead(ENC_A)==LOW) {
    counter--;
  }
  else{
    counter++;
  }
}

// create NodeHandle to create publisher/subcsriber
ros::NodeHandle nh;

// pubsub initialization
std_msgs::UInt16 ultradist_msg;
ros::Publisher ultradist("ultradist", &ultradist_msg);
std_msgs::Bool lim_left_msg;
ros::Publisher lim_left("lim_left", &lim_left_msg);
std_msgs::Bool lim_right_msg;
ros::Publisher lim_right("lim_right", &lim_right_msg);
std_msgs::UInt16 encoder_msg;
ros::Publisher encoder("encoder", &encoder_msg);

void motor1Cb(const std_msgs::Int16& pwm1_msg){
  runMotor1(pwm1_msg.data);
//  digitalWrite(LEDEBUG,HIGH);
}

void motor2Cb(const std_msgs::Int16& pwm2_msg){
  runMotor2(pwm2_msg.data);
//  digitalWrite(LEDEBUG,LOW);
}

void servoCb(const std_msgs::Bool& servo_msg){
  if (servo_msg.data == true){
    servoClose();
  }
  else{
    servoOpen();
  }
}

byte mode_now = 0;

void modeCb(const std_msgs::Byte& mode_msg){
  mode_now = mode_msg.data;
  nh.loginfo("Mode Change");
}

void counterCb(const std_msgs::UInt16& cCalib_msg){
  counter = cCalib_msg.data;
  digitalWrite(LEDEBUG,HIGH);
}

ros::Subscriber<std_msgs::Int16> pwm1("pwm1", &motor1Cb);
ros::Subscriber<std_msgs::Int16> pwm2("pwm2", &motor2Cb);
ros::Subscriber<std_msgs::Bool> servo("servo", &servoCb);
ros::Subscriber<std_msgs::Byte> mode("mode", &modeCb);
ros::Subscriber<std_msgs::UInt16> cCalib("cCalib", &counterCb);

// I2C address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

long duration;
int distance;

int cableLength;

void setup() {
  // HC-SR04
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);

 // PCA 9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);

  // Motor 1 pinout
  pinMode(RPWM1, OUTPUT);
  pinMode(R_EN1, OUTPUT);
  pinMode(R_IS1, OUTPUT); //Optional Current Sensing
  pinMode(LPWM1, OUTPUT);
  pinMode(L_EN1, OUTPUT);
  pinMode(L_IS1, OUTPUT); //Optional Current Sensing
  digitalWrite(R_IS1, LOW);
  digitalWrite(R_IS1, LOW);
  digitalWrite(R_EN1, HIGH);
  digitalWrite(L_EN1, HIGH);
  
  // Motor 2 pinout
  pinMode(RPWM2, OUTPUT);
  pinMode(R_EN2, OUTPUT);
  pinMode(R_IS2, OUTPUT); //Optional Current Sensing
  pinMode(LPWM2, OUTPUT);
  pinMode(L_EN2, OUTPUT);
  pinMode(L_IS2, OUTPUT); //Optional Current Sensing
  digitalWrite(R_IS2, LOW);
  digitalWrite(R_IS2, LOW);
  digitalWrite(R_EN2, HIGH);
  digitalWrite(L_EN2, HIGH);

  // Limit Switch
  pinMode(LEFT, INPUT_PULLUP);
  pinMode(RIGHT, INPUT_PULLUP);
  pinMode(LEDEBUG, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Incremental Encoder
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(5, ai0, RISING);
  attachInterrupt(4, ai1, RISING);

  // ROS
  nh.initNode();
  nh.advertise(ultradist);
  nh.advertise(lim_left);
  nh.advertise(lim_right);
  nh.advertise(encoder);
  nh.subscribe(pwm1);
  nh.subscribe(pwm2);
  nh.subscribe(servo);
  nh.subscribe(mode);
  nh.subscribe(cCalib);
}


int prevTime = 0;

void loop() {
  if(millis()-prevTime >= SAMPLING_TIME){
    prevTime = millis();
    if (mode_now == 1 || mode_now == 2 || mode_now == 5 || mode_now == 99){
      // 1: Calibration Mode
      // 2: Scanning Mode
      // 5: PID_PD Control
      // 5:  Control
      // Only publish limit switches and encoder
      lim_left_msg.data = !digitalRead(LEFT);
      lim_right_msg.data = !digitalRead(RIGHT);
      encoder_msg.data = counter;
      lim_left.publish(&lim_left_msg);
      lim_right.publish(&lim_right_msg);
      encoder.publish(&encoder_msg);
    }
    else if (mode_now == 8 || mode_now == 9 || mode_now == 10){
      // 8: Lift-off Maneuver
      // 9: Lift-on Maneuver
      // 10: Reshuffling Maneuver
      lim_left_msg.data = !digitalRead(LEFT);
      lim_right_msg.data = !digitalRead(RIGHT);
      encoder_msg.data = counter;
      lim_left.publish(&lim_left_msg);
      lim_right.publish(&lim_right_msg);
      encoder.publish(&encoder_msg);
      ultra();
      ultradist.publish(&ultradist_msg);
    }
    else if (mode_now == 3 || mode_now == 4 || mode_now == 6){
      // 3: Hoisting Mode Down (pick)
      // 4: Hoisting Mode Up (pick)
      // 6: Hoisting Mode 
      // Only publish ultradist
      ultra();
      ultradist.publish(&ultradist_msg);
    }
    if (mode_now == 0){
      // 0: Standby Mode
      // Do nothing
    }
    nh.spinOnce();
    delay(1);
  }
}

void lim(){
  if(digitalRead(LEFT) == false){
    lim_left_msg.data = true;
  }
  else{
    lim_left_msg.data = false;
  }
  if(digitalRead(RIGHT) == false){
    lim_right_msg.data = true;
  }
  else{
    lim_right_msg.data = false;
  }
}

void ultra(){
  ultradist_msg.data = measureDist();
}

void servoOpen(){
  pwm.setPWM(12,0,S0MIN);
  pwm.setPWM(13,0,S1MIN);
  pwm.setPWM(14,0,S2MIN);
  pwm.setPWM(15,0,S3MIN);
}

void servoClose(){
  pwm.setPWM(12,0,S0MAX);
  pwm.setPWM(13,0,S1MAX);
  pwm.setPWM(14,0,S2MAX);
  pwm.setPWM(15,0,S3MAX);
}

int measureDist(){
  // Clears trig pin
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  // Sets trig pin HIGH for 10 us
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  // Reads echo pin, returns the duration
  duration = pulseIn(ECHOPIN, HIGH);
  distance = duration*0.034/2;
  return distance;
}

void runMotor1(int in){
  if (in>=0){
    digitalWrite(RPWM1, LOW);
    if(in >=0 && in <=255 ){  
      analogWrite(LPWM1, in);
    }
  }
  else{
    in=in*-1;
    digitalWrite(LPWM1, LOW);
    if(in >=0 && in <=255 ){
      analogWrite(RPWM1, in);
    }
  }
}

void runMotor2(int in){
  if (in>=0){
    digitalWrite(RPWM2, LOW);
    if(in >=0 && in <=255 ){
      analogWrite(LPWM2, in);
    }
  }
  else{
    in=in*-1;
    digitalWrite(LPWM2, LOW);
    if(in >=0 && in <=255 ){
      analogWrite(RPWM2, in);
    }
  }
}
