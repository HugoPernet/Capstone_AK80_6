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
CAN_Tx Motor_Tx_L;
CAN_Tx Motor_Tx_R; // initialize Motor command and Motor reply variables
CAN_Rx Motor_Rx_L;
CAN_Rx Motor_Rx_R;
Joint_origines origine_L;
Joint_origines origine_R;
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

  //CAN.onReceive(onCANReceive);


  Motor_Rx_L = unpack_reply_L();
  origine_L = HomingL(Motor_Rx_L,1.0,Motor_Tx_L);
  
  Motor_Rx_R = unpack_reply_R();
  origine_R = HomingR(Motor_Rx_R,1.0,Motor_Tx_R);

  Serial.println("Homed shoulder & hip");
  Serial.println("CONTROL START");
  delay(1000);
  Motor_Tx_L.kd_in = 1;
  Motor_Tx_R.kd_in = 1;
  Motor_Tx_L.kp_in = 0;
  Motor_Tx_R.kp_in = 0;
  Serial.println(origine_L.leg);
  Serial.println(origine_R.leg);
  Serial.println(origine_L.shoulder);
  Serial.println(origine_R.shoulder);
}




void loop() {


  
  
  HipVel = readgyro()-bias_pitch.velocity;

  //Read IMU
  HipAngle = (round(readIMU())-bias_pitch.angle)-2.0; //deg
  torqueL(HipAngle,origine_L);
  torqueR(HipAngle,origine_R);


  //Serial.println("pL :"+String(degrees(Motor_Rx_L.position-origine_L.leg))+ "  pR :"+String(degrees(Motor_Rx_R.position-origine_L.leg))+ "  T_L :"+String(-Motor_Rx_L.torque) +"  T_R :"+String(-Motor_Rx_R.torque)+ " hip: "+String(HipAngle) );

}