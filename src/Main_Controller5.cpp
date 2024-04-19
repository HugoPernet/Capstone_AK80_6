
#include "Arduino.h"
#include <MotorUtilities.h>
#include <potentiometerUtilities.h>
#include <IMUutilities.h>


//POTS for torque amplitude
#define POT_K1 17   // A3

//structs
MotorCommand MotorIn;
MotorReply MotorOut;
Joint_origines origines;
bias bias_pitch;

// initialize IMU Data
float HipVel = 0.0;
float HipAngle = 0.0;

// initialize homing parameters
float P0_shoulder = 0;
float P0_hip = 0;
float P0_midpt = 0;

//Loop() Period
float dt = 10;

//Mechanical constant:
const float StaticFrictionTorque = 0.25;
float TorqueAmplitude = 1.0;
float K2, C2, D2 = 0.1; // torque slope
float Slack;



void setup() {
  //////////// initialize POTS ////////////
  pinMode(POT_K1, INPUT);

  //////////// starts Serial Com /////////
  Serial.begin(11250);
  while (!Serial) delay(10);

  //////////// starts Can com ////////////
  SetupCan();
  delay(500);

  //////////// initialize IMU ////////////
  initializeIMU();
  bias_pitch = calculate_bias();

  ////////////  Setup motor   ////////////
  EnterMotorMode(); delay(50);
  SetZero(); delay(50);
  MotorOut = unpack_reply();
  origines = Homing(MotorOut,1.0,MotorIn);
  Slack = abs(origines.shoulder-origines.leg);

  MotorIn.kp_in = 0;
  Serial.println("Zeroed and Homed at Midpoint of shoulder & hip");
  delay(1000);
}



void loop() {
  float time_now = millis();
  MotorIn.kp_in = 0.0;

  ////////////    Read IMU    ////////////
  HipAngle = (round(readIMU())-bias_pitch.angle)-2.0; //deg
  HipVel = readgyro()-bias_pitch.velocity;
  
  ////////////    Read POTs   ////////////
  float POT_reading1 = analogRead(POT_K1);
  float K1 = map_float(POT_reading1, 0, 1023, 2, 4.5);
  Serial.print(" K1: "+String(K1)); // about 2.5-3 is good

  ////////////    Torque Eq   ////////////
  float As = 2; float Al = 1; float Ss = 1;
  float shoulder = As*(1/PI)*atan(degrees(MotorOut.position-Slack/2)-10) + As/2;
  float leg = Al*(1/PI)*atan(-degrees(MotorOut.position-Slack/2)+10) - As/2 - 2*Ss + 1;
  float switching = -4*(1/PI)*atan(HipAngle-20)+1;

  float Dyn_shoulder = MotorOut.velocity*((1/PI)*atan(degrees(MotorOut.position - origines.shoulder)-10)+0.5) ;
  float Dyn_leg = 0.02*HipVel*((1/PI)*atan(HipAngle -20)+0.5)*((1/PI)*atan(degrees(-MotorOut.velocity)-30)+0.5);

  MotorIn.t_in = shoulder + leg + switching; //Dyn_leg+Dyn_shoulder;
  MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);

  ////////////    PID modulation    ////////////
  float LB_kd = 0.2; // lower bound kd
  float Akd = 2*LB_kd;
  MotorIn.kd_in = K1*(Akd*cos((PI/origines.midpoint) *(MotorOut.position-origines.leg-radians(HipAngle)))+(1-Akd));

  ////////////    Send CAN command    ////////////
  pack_cmd(MotorIn);

  ////////////  Listen Motor output   ////////////
  MotorOut = unpack_reply();
 
  Serial.println(" Dyn_shoulder: " + String(Dyn_shoulder) + " Dyn_leg: " + String(Dyn_leg));
  while (millis()-time_now < dt) {}
}