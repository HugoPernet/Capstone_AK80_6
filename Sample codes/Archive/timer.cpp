//Edits from Main_dynamic:
// changing dt depending on shoulder vs. leg assist

#include "Arduino.h"
#include <MotorUtilities.h>
#include <potentiometerUtilities.h>
#include <IMUutilities.h>
#include <ctime>


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
float tstart, tend; 
int timer_countersize = 10000; //

//Mechanical constant:
const float StaticFrictionTorque = 0.25;
float TorqueAmplitude = 1.0;
float K2, C2, D2 = 0.1; // torque slope
String dyn = "OFF";
bool timer_complete = 0;

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
//   origines = Homing(MotorOut,1.0,MotorIn);

  MotorIn.kp_in = 0;
  Serial.println("Homed shoulder & hip");
  Serial.println("CONTROL START");
  delay(1000);
}

float POT_reading1, K1, As, Al, Ss, shoulder, leg, switching, shoulder_vel, LB_kd, Akd;

void timer() {
  tstart = micros();
  for (int i = 0; i < timer_countersize; i++) {
    //Read POTs
    POT_reading1 = analogRead(POT_K1); // between 0 to 1023
    K1 = map_float(POT_reading1, 0, 1023, 1, 4.5); // Shoulder Angle


    //Read IMU
    HipAngle = (round(readIMU())-bias_pitch.angle)-2.0; //deg
    HipVel = readgyro()-bias_pitch.velocity;
    
    // Torque Eq
    As = 2; Al = 2; Ss = 1;
    shoulder = As*(1/PI)*atan(degrees(MotorOut.position-origines.shoulder)-3) + As/2 + 1;
    leg = Al*(1/PI)*atan(-degrees(MotorOut.position-origines.leg)+30) - As/2 - 2*Ss + 0.5;
    switching = -5*(1/PI)*atan(HipAngle-20)+1.5;

    // shoulder_vel is between 0 and 1
    shoulder_vel = ((1/PI)*atan(degrees(MotorOut.position-origines.shoulder))+0.5)*(1/PI*atan(-degrees(MotorOut.velocity)-2)+0.5);
    //float leg_vel = 1/PI*(atan(HipAngle-10)+0.5);

        if (shoulder_vel > 0.1) {
            dyn = "ON";
        }
        else {
            dyn = "OFF";
        }
        
    MotorIn.t_in = shoulder + leg + switching - shoulder_vel;
    MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);

    LB_kd = 0.2; // lower bound kd
    Akd = 2*LB_kd;
    MotorIn.kd_in = K1*(Akd*cos((PI/origines.midpoint) *(MotorOut.position-origines.leg-radians(HipAngle)))+(1-Akd));

    //pack & unpack msgs
    pack_cmd(MotorIn);
    MotorOut = unpack_reply();
    Serial.println("  count: " + String(i));
    }

  tend = micros(); 
  Serial.println("It took "+ String(tend-tstart) + " microseconds(s).");
  
  timer_complete = 1;
}

void loop() {
  if (timer_complete == 0) {
    timer();
  }
}