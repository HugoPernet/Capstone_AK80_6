//Edits from Controller 3:
// kd changing as function of motor angle
// kp set to zero
// kd max about 2.5-3 is good
// As = 2; Al = 3; Ss = 1;
// Added torque limits at ends (saturates to zero)

#include "Arduino.h"
#include <MotorUtilities.h>
#include <potentiometerUtilities.h>
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
MotorCommand MotorIn; // initialize Motor command and Motor reply variables
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

void setup() {
  //starts Serial Com
  Serial.begin(11250);
  while (!Serial) delay(10);

  //starts Can com
  SetupCan();
  delay(500);

  // initialize POTS
  pinMode(POT_K1, INPUT);
  pinMode(POT_C1, INPUT);
  pinMode(POT_D1, INPUT);
  pinMode(POT_K2, INPUT);
  pinMode(POT_C2, INPUT);
  pinMode(POT_D2, INPUT);

  initializeIMU();
  bias_pitch = calculate_bias();

  //Setup motor
  EnterMotorMode(); delay(50);
  SetZero(); delay(50);
  MotorOut = unpack_reply();
  origines = Homing(MotorOut,1.0,MotorIn);

  MotorIn.kp_in = 0;
<<<<<<< HEAD
  Serial.println("Zeroed and Homed at Midpoint of shoulder & hip");
=======
  Serial.println("Homed shoulder & hip");
  Serial.println("CONTROL START");
>>>>>>> 3fa066ea3d0e4058b44c59e4e9e69f8c5584c740
  delay(1000);
}



void loop() {
  float time_now = millis();

<<<<<<< HEAD
  //Read IMU
  HipAngle = (round(readIMU())-bias_pitch.angle)-2.0; //deg
  HipVel = readgyro()-bias_pitch.velocity;
  
  //Read POTs
  float POT_reading1 = analogRead(POT_K1); // between 0 to 1023
  float K1 = map_float(POT_reading1, 0, 1023, 2, 4.5); // Shoulder Angle
=======
  //Read POTs
  float POT_reading1 = analogRead(POT_K1); // between 0 to 1023
  float K1 = map_float(POT_reading1, 0, 1023, 2, 4.5); // Shoulder Angle
<<<<<<<< HEAD:Sample codes/Archive/Main_Controller5.cpp
  Serial.print("    K1: "+String(K1)); // about 2.5-3 is good

  //Read IMU
  HipAngle = (round(readIMU())-bias_pitch.angle)-2.0; //deg
  HipVel = readgyro()-bias_pitch.velocity;
 
  // Torque Eq
  float As = 2; float Al = 1; float Ss = 1;
  float shoulder = As*(1/PI)*atan(degrees(MotorOut.position-origines.shoulder)-10) + As/2;
  float leg = Al*(1/PI)*atan(-degrees(MotorOut.position-origines.leg)+10) - As/2 - 2*Ss + 1;
  float switching = -4*(1/PI)*atan(HipAngle-20)+1;

  float shoulder_vel = pow((1/PI),2)*(atan(degrees(MotorOut.position-origines.shoulder))+0.5)*(atan(-degrees(MotorOut.velocity)+10)+0.5);
  float leg_vel = 1/PI*(atan(HipAngle-10)+0.5);
========
>>>>>>> 3fa066ea3d0e4058b44c59e4e9e69f8c5584c740
//   float POT_reading2 = analogRead(POT_C1); // between 0 to 1023
//   float C1 = map_float(POT_reading2, 0, 1023, 0.5, 3); // Shoulder Velocity
//   float POT_reading3 = analogRead(POT_D1);
//   float D1 = map_float(POT_reading3, 0, 1023, 0.5, 3); // Hip Angle
//   float POT_reading4 = analogRead(POT_K2);
//   float K2 = map_float(POT_reading4, 0, 1023, 0.05,0.5); // Shoulder Angle ~0.3
//   float POT_reading5 = analogRead(POT_C2);
//   float C2 = map_float(POT_reading5, 0, 1023, 0.05,0.15); // Shoulder Velocity ~0.05
//   float POT_reading6 = analogRead(POT_D2);
//   float D2 = map_float(POT_reading6, 0, 1023, 0.05,0.5); //Hip Angle, ~0.2

<<<<<<< HEAD
  Serial.print("    K1: "+String(K1)); // about 2.5-3 is good
  //Serial.print("    K1: "+String(K1)+" C1: "+String(C1)+ " D1:" + String(D1) + " K2: " + String(K2)+" C2: "+String(C2)+" D2: "+String(D2));

  // float Akd =3;
  float Slack = abs(origines.shoulder-origines.leg);
  // Torque Eq
  float As = 2; float Al = 1; float Ss = 1;
  float shoulder = As*(1/PI)*atan(degrees(MotorOut.position-Slack/2)-10) + As/2;
  float leg = Al*(1/PI)*atan(-degrees(MotorOut.position-Slack/2)+10) - As/2 - 2*Ss + 1;
  float switching = -4*(1/PI)*atan(HipAngle-20)+1;
  //float KD = Akd*cos((2*PI/Slack)*(MotorOut.position - origines.shoulder-HipAngle)) + Akd + 0.2;

<<<<<<<< HEAD:Sample codes/Archive/Main_Controller4.cpp
  MotorIn.t_in = shoulder + leg + switching;
  MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);

========
  // float KD = Akd*(1/PI)*atan(degrees(MotorOut.position-(Slack/2))-10) + Akd*(1/PI)*atan(degrees(-MotorOut.position -origines.shoulder-HipAngle) +10) +(Akd +0.5);
  // MotorIn.kd_in = KD;
>>>>>>>> 3fa066ea3d0e4058b44c59e4e9e69f8c5584c740:src/Main_Controller5.cpp
=======

   // Torque Eq
  float As = 2; float Al = 1; float Ss = 1; 
  float theta_dot_IMU = readgyro(); // deg/s

  float shoulder = As*(1/PI)*atan(degrees(MotorOut.position-origines.shoulder)-10) + As/2;
  float leg = Al*(1/PI)*atan(-degrees(MotorOut.position-origines.leg)-10) - As/2 - 2*Ss + 1;
  float switching = -4*(1/PI)*atan(HipAngle-20)+1;
  float Dyn_shoulder = 0.01*theta_dot_IMU*((1/PI)*atan(degrees(MotorOut.position - origines.shoulder)-10)+0.5) ;
  float Dyn_leg = 0.01*MotorOut.velocity*((1/PI)*atan(-theta_dot_IMU)+0.5); 
>>>>>>>> 3fa066ea3d0e4058b44c59e4e9e69f8c5584c740:Sample codes/Archive/Main_Controller4.cpp

  MotorIn.t_in = shoulder + leg + switching;
  MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);

<<<<<<<< HEAD:Sample codes/Archive/Main_Controller5.cpp
  float LB_kd = 0.2; // lower bound kd
  float Akd = 2*LB_kd;
  MotorIn.kd_in = K1*(Akd*cos((PI/origines.midpoint) *(MotorOut.position-origines.leg-radians(HipAngle)))+(1-Akd));
========
  
>>>>>>> 3fa066ea3d0e4058b44c59e4e9e69f8c5584c740
  float LB_kd = 0.2; // lower bound kd
  float Akd = 2*LB_kd;
 
  MotorIn.kd_in = K1*(Akd*cos((PI/origines.midpoint) *(MotorOut.position-origines.leg-radians(HipAngle)))+(1-Akd));

<<<<<<< HEAD


<<<<<<<< HEAD:Sample codes/Archive/Main_Controller4.cpp
========
  float theta_dot_IMU = readgyro(); // deg/s
  float Dyn_shoulder = MotorOut.velocity*((1/PI)*atan(degrees(MotorOut.position - origines.shoulder)-10)+0.5) ;
  float Dyn_leg = 0.02*HipVel*((1/PI)*atan(-HipVel)+0.5); 


  MotorIn.t_in = shoulder + leg + switching +Dyn_leg+Dyn_shoulder;
  MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);
>>>>>>>> 3fa066ea3d0e4058b44c59e4e9e69f8c5584c740:src/Main_Controller5.cpp
  //pack & unpack msgs
  pack_cmd(MotorIn);
  MotorOut = unpack_reply();
  // Serial.print("  kd: " + String(MotorIn.kd_in)+"  T_out: "+String(MotorOut.torque));
  // Serial.print("  T_in: " + String(MotorIn.t_in));
  // Serial.println("    MEASURING:  IMU_Ang: " + String(HipAngle)+  "  P_out: "+String(degrees(MotorOut.position)) + "  P_l: "+String(degrees(MotorOut.position-origines.leg))+"  P_s: "+String(degrees(MotorOut.position-origines.shoulder))+"  v_out: "+String(MotorOut.velocity));

  Serial.println(" Dyn_shoulder: " + String(Dyn_shoulder) + " Dyn_leg: " + String(Dyn_leg));
=======
>>>>>>>> 3fa066ea3d0e4058b44c59e4e9e69f8c5584c740:Sample codes/Archive/Main_Controller4.cpp

  //pack & unpack msgs
  pack_cmd(MotorIn);
  MotorOut = unpack_reply();
<<<<<<<< HEAD:Sample codes/Archive/Main_Controller5.cpp
  Serial.print("  kd: " + String(MotorIn.kd_in));
  Serial.print("  T_in: " + String(MotorIn.t_in));
  Serial.print("   shoulder_vel: "+String(shoulder_vel)+"  leg_vel: "+String(leg_vel));
  Serial.println("  IMU_Ang: " + String(HipAngle)+  "  P_s: "+String(degrees(MotorOut.position-origines.shoulder))+ "  P_l: "+String(degrees(MotorOut.position-origines.leg)));
========
  Serial.println(" Dyn_shoulder: " + String(Dyn_shoulder) + " Dyn_leg: " + String(Dyn_leg));
>>>>>>>> 3fa066ea3d0e4058b44c59e4e9e69f8c5584c740:Sample codes/Archive/Main_Controller4.cpp

>>>>>>> 3fa066ea3d0e4058b44c59e4e9e69f8c5584c740
  while (millis()-time_now < dt) {}
}