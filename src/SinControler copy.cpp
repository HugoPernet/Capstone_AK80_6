#include "Arduino.h"
#include <potentiometerUtilities.h>
#include <MotorUtilitiesCopy.h>
#include <IMUutilities.h>


//POTS for torque amplitude
#define POT_K1 17   // A3
#define POT_C1 18   // A4
#define POT_D1 19   // A5

//POTS for torque slope
#define POT_K2 16   // A2
#define POT_C2 15   // A1
#define POT_D2 14   // A0

//////// Variable definition ////////

//structs
CAN_Tx Motor_Tx; // initialize Motor command and Motor reply variables
CAN_Rx Motor_Rx_L;
CAN_Rx Motor_Rx_R;
Joint_origines origine;
bias bias_pitch;

// initialize IMU Data
float HipVel = 0.0;
float HipAngle = 0.0;

// initialize homing parameters
float P0_shoulder = 0;
float P0_hip = 0;
float P0_midpt = 0;

//Loop() Period
float dt = 20;

//Mechanical constant:
const float StaticFrictionTorque = 0.25;
float As = 4.0;
float K2, C2, D2 = 0.1; // torque slope
float R = 5;
float midpoint;


void setup() {
  //starts Serial Com
  Serial.begin(11250);
  while (!Serial) delay(10);

  //starts Can com
  SetupCan();
  delay(500);

  initializeIMU();
  bias_pitch = calculate_bias();

  //Setup motor
  EnterMotorMode(); 
  delay(50);

  SetZero();
  delay(50);

  Motor_Rx_L = unpack_reply_L();
  HomingL(Motor_Rx_L,1.0,Motor_Tx,origine);
  Motor_Rx_R = unpack_reply_R();
  HomingR(Motor_Rx_R,1.0,Motor_Tx,origine);

  Serial.println("Homed shoulder & hip");
  Serial.println("CONTROL START");
  delay(1000);
  Motor_Tx.kd_in_L = 1;
  Motor_Tx.kd_in_R = 1;
  Motor_Tx.kp_in_L = 0;
  Motor_Tx.kp_in_R = 0;
  Serial1.println(origine.leg_L);
  Serial1.println(origine.leg_R);
  Serial1.println(origine.shoulder_L);
  Serial1.println(origine.shoulder_R);
}




void loop() {

  float time_now = millis();
  HipVel = readgyro()-bias_pitch.velocity;

  if (HipVel<0){
    Motor_Tx.kd_in_L = 0.3;
    Motor_Tx.kd_in_R = 0.3;
  }
  else{
    Motor_Tx.kd_in_L = 0.5;
    Motor_Tx.kd_in_R = 0.5;
  }

  //Read IMU
  HipAngle = (round(readIMU())-bias_pitch.angle)-2.0; //deg

  ////// Left /////
  float Ts = As*sin((Motor_Rx_L.position-origine.shoulder_L)*R);
  float Tleg = -As*sin(radians(HipAngle));

  float Switch_leg = (1/PI)*atan(HipAngle -3)+0.5; 
  float Switch_shoulder = ((1/PI)*atan(degrees((Motor_Rx_L.position-origine.shoulder_L)*R) -3)+0.5);
  float Ts_switch = Ts*Switch_shoulder + 0.5;
  float Tleg_switch = Tleg*Switch_leg;
    
  Motor_Tx.t_in_L = Ts_switch+Tleg_switch;
  Motor_Tx.t_in_L = constrain(Motor_Tx.t_in_L, T_MIN, T_MAX);

  ////// Left /////
  Ts = As*sin((Motor_Rx_R.position-origine.shoulder_R)*R);
  Tleg = As*sin(radians(HipAngle));

  Switch_leg = (1/PI)*atan(HipAngle -3)+0.5; 
  Switch_shoulder = 1-((1/PI)*atan(HipAngle-10)+0.5);

  Ts_switch = Ts*Switch_shoulder*(1-((1/PI)*atan(degrees((Motor_Rx_R.position-origine.shoulder_R)*R) -3) +0.5));
  Tleg_switch = Tleg*Switch_leg;

  Motor_Tx.t_in_R = Ts_switch+Tleg_switch-0.5;
  Motor_Tx.t_in_R = constrain(Motor_Tx.t_in_R, T_MIN, T_MAX);

  pack_cmd(Motor_Tx);
  while (millis()-time_now < dt) {}
}