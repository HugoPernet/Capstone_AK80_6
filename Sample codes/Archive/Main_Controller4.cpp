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
  Serial.println("Zeroed and Homed at Midpoint of shoulder & hip");
  delay(1000);
}



void loop() {
  float time_now = millis();

  //Read IMU
  HipAngle = (round(readIMU())-bias_pitch.angle)-2.0; //deg
  HipVel = readgyro()-bias_pitch.velocity;
  
  //Read POTs
  float POT_reading1 = analogRead(POT_K1); // between 0 to 1023
  float K1 = map_float(POT_reading1, 0, 1023, 2, 4.5); // Shoulder Angle
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


   // Torque Eq
  float As = 2; float Al = 1; float Ss = 1; 
  float theta_dot_IMU = readgyro(); // deg/s

  float shoulder = As*(1/PI)*atan(degrees(MotorOut.position-origines.shoulder)-10) + As/2;
  float leg = Al*(1/PI)*atan(-degrees(MotorOut.position-origines.leg)-10) - As/2 - 2*Ss + 1;
  float switching = -4*(1/PI)*atan(HipAngle-20)+1;
  float Dyn_shoulder = 0.01*theta_dot_IMU*((1/PI)*atan(degrees(MotorOut.position - origines.shoulder)-10)+0.5) ;
  float Dyn_leg = 0.01*MotorOut.velocity*((1/PI)*atan(-theta_dot_IMU)+0.5); 

  MotorIn.t_in = shoulder + leg + switching;
  MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);

  
  float LB_kd = 0.2; // lower bound kd
  float Akd = 2*LB_kd;
 
  MotorIn.kd_in = K1*(Akd*cos((PI/origines.midpoint) *(MotorOut.position-origines.leg-radians(HipAngle)))+(1-Akd));


  //pack & unpack msgs
  pack_cmd(MotorIn);
  MotorOut = unpack_reply();
  Serial.println(" Dyn_shoulder: " + String(Dyn_shoulder) + " Dyn_leg: " + String(Dyn_leg));

  while (millis()-time_now < dt) {}
}