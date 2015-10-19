/*

Key function introduction:

* reflexMovement         logic          - write the motion logic based on the sensor reading and generate move direction and move speed 
* OmniControlProcessor   data converter - convert move direction and speed to 3 wheels speed value
* MotorOutput            actuator       - output pwm signal to motor driver
* debugSerial            

Key variables introduction:

Acutator
* moveDirection        - set move direction (degree) from 0 - 360
* moveSpeed            - set move speed              from 100 - 255
* motionX              - directly set X axis move speed 100 - 255
* motionY              - directly set Y axis move speed 100 - 255
* theta                - directly set rotation speed    suggested value: < 20


# Project Name    OmniHCRTest
# Editor          Lauren / Jason Ling
# Date            20150919

Hardware connection:
* Suggest to use Mega work as the microcontroller
- connect motor driver to D2-D7 based on the motion model

*/

#include "Metro.h"

// pin configuration
#define MOTORADIRPIN  4  // WHITE CABLE
#define MOTORAPWMPIN  5   // BLUE CABLE

#define MOTORBDIRPIN  2  // WHITE CABLE
#define MOTORBPWMPIN  3   // BLUE CABLE

#define MOTORCDIRPIN  7  // WHITE CABLE
#define MOTORCPWMPIN  6   // BLUE CABLE

int motorApwm,motorBpwm,motorCpwm;


void setup(){
  Serial.begin(9600);
  Serial.println("Project Magic Chair!");

  initMotorA();   // init pwm output and direction control pin for motor A
  initMotorB();   // init pwm output and direction control pin for motor B
  initMotorC();   // init pwm output and direction control pin for motor C

}

Metro distanceReadingTimer = Metro(30,true);  // generate the multi thread using timer
Metro debugTimer = Metro(100,true);
Metro simpleLogicTimer = Metro(30,true);
Metro motorControlTimer = Metro(30,true);

float moveDirection = 0;    // degree
float moveSpeed = 0;        // speed value from 0-20 map

float motionX;    // x axis movement
float motionY;    // y axis movement
float theta;      // rotation angle

void loop(){
  if(debugTimer.check())  DebugSerial();

  if(simpleLogicTimer.check())  ReflexMovement();

  if(motorControlTimer.check()){
    OmniControlProcessor();
    MotorOutput();
  }
}

void ReflexMovement(){
  
  // logic sample:
  // move forward for 1s
  // move reverse for 1s
  
  static int steps = 0;
  
  if(steps == 0){
    moveDirection = 90;  // set direction
    moveSpeed = 180;     // set speed
	theta = 0;
	
    simpleLogicTimer.interval(3000);  // move 2s 
	steps ++;
  }
  else{
	theta = 0;
    moveDirection = 270;   
    moveSpeed = 180;
	
    simpleLogicTimer.interval(3000);
	steps = 0;
  }

}


void DebugSerial(){

//  Serial.print("v1 v2 v3: ");
//  Serial.print(motorApwm);
//  Serial.print(" ");
//  Serial.print(motorBpwm);
//  Serial.print(" ");
//  Serial.print(motorCpwm);
//  Serial.println(" ");

}



void OmniControlProcessor(){
  
  //
  motionX = cos(degreeToRad(moveDirection)) * moveSpeed;
  motionY = sin(degreeToRad(moveDirection)) * moveSpeed;

  float phi = 120;                     // phi is the degree between each motor axis
  float L = 10;                        // estimated length
  
  // code is based on the motion formula, please check the picture attached in the folder
  float v1 = sin((60 - theta) * PI / 180.0) * motionX + cos((60 - theta) * PI / 180.0) * motionY + L * theta;    //-14.00  254.00  0.00  -237	-246	246
  float v2 = -1 * sin((60 + theta) * PI / 180.0) * motionX + cos((60 + theta) * PI / 180.0) * motionY + L * theta;
  float v3 = sin(theta * PI / 180.0) * motionX - cos(theta * PI / 180.0) * motionY + L * theta;

  motorApwm = v1;
  motorBpwm = v2;
  motorCpwm = v3;

}

float degreeToRad(float _degree){
  float z = _degree / 180;
  z *= PI;
  return z;
}

void MotorOutput(){
  motorASpeedControl(motorApwm);
  motorBSpeedControl(motorBpwm);
  motorCSpeedControl(motorCpwm);
}


void initMotorA(){
  pinMode(MOTORAPWMPIN,OUTPUT);
  pinMode(MOTORADIRPIN,OUTPUT);
  motorApwm = 0;
  motorASpeedControl(motorApwm);

}

void initMotorB(){
  pinMode(MOTORBPWMPIN,OUTPUT);
  pinMode(MOTORBDIRPIN,OUTPUT);
  motorBpwm = 0;
  motorBSpeedControl(motorBpwm);

}

void initMotorC(){
  pinMode(MOTORCPWMPIN,OUTPUT);
  pinMode(MOTORCDIRPIN,OUTPUT);
  motorCpwm = 0;
  motorCSpeedControl(motorCpwm);

}

void motorASpeedControl(int pwm){
  pwm = constrain(pwm, -255, 255);
  if(pwm >= 0){
    digitalWrite(MOTORADIRPIN,HIGH);
    analogWrite(MOTORAPWMPIN,pwm);
  }
  else{
    digitalWrite(MOTORADIRPIN,LOW);
    analogWrite(MOTORAPWMPIN,abs(pwm));
  }
}

void motorBSpeedControl(int pwm){
  pwm = constrain(pwm, -255, 255);
  if(pwm >= 0){
    digitalWrite(MOTORBDIRPIN,HIGH);
    analogWrite(MOTORBPWMPIN,pwm);
  }
  else{
    digitalWrite(MOTORBDIRPIN,LOW);
    analogWrite(MOTORBPWMPIN,abs(pwm));
  }
}


void motorCSpeedControl(int pwm){
  pwm = constrain(pwm, -255, 255);
  if(pwm >= 0){
    digitalWrite(MOTORCDIRPIN,HIGH);
    analogWrite(MOTORCPWMPIN,pwm);
  }
  else{
    digitalWrite(MOTORCDIRPIN,LOW);
    analogWrite(MOTORCPWMPIN,abs(pwm));
  }
}



