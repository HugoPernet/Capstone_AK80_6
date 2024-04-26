//left motor 0x01

#include "Arduino.h"
#include <MotorUtilities.h>
#include <potentiometerUtilities.h>
#include <IMUutilities.h>

//////// Variable definition ////////
struct measurement_avg {
    float HipAngle;
    float MotorAngle;
};

//structs
MotorCommand MotorIn; // initialize Motor command and Motor reply variables
MotorReply MotorOut;
Joint_origines origines;
bias bias_pitch;
measurement_avg avgs;

// initialize IMU Data
float HipVel = 0.0;
float HipAngle = 0.0;

// average of last few points
const int sample_pts = 5;
float HipAngle_avg = 0;
float HipAngle_arr[sample_pts] = {0.0f};
float MotorAngle_avg = 0;
float MotorAngle_arr[sample_pts] = {0.0f};

// initialize homing parameters
float P0_shoulder = 0;
float P0_hip = 0;
float P0_midpt = 0;

//Loop() Period
float dt = 10;

//Mechanical constant:
const float StaticFrictionTorque = 0.25;
float As = 2.0;
float Al = 2.0;
float K2, C2, D2 = 0.1; // torque slope
float R = 7.8;

// case switching
int control_case = 1;
float IMU_threshold = 12; // degrees
float compliance_angle = 2;

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
  EnterMotorMode(); delay(50);
  SetZero(); delay(50);
  MotorOut = unpack_reply();
  origines = Homing(MotorOut,1.0,MotorIn);

  //MotorIn.kp_in = 0;
  Serial.println("Homed shoulder & hip");
  Serial.println("CONTROL START");
  delay(1000);
}



measurement_avg update_avg() {
    measurement_avg avgs;
    for (int i = 0; i<sample_pts-1; i++) {
        HipAngle_arr[i] = HipAngle_arr[i+1];
        MotorAngle_arr[i] = MotorAngle_arr[i+1];
    }
    HipAngle_arr[sample_pts-1] = HipAngle;
    MotorAngle_arr[sample_pts-1] = MotorOut.position;
    avgs.HipAngle = calculateMean(HipAngle_arr, sample_pts);
    avgs.MotorAngle = calculateMean(MotorAngle_arr, sample_pts);
    return avgs;
}

void loop() {
  float time_now = millis();

  //Read IMU
  HipAngle = (round(readIMU())-bias_pitch.angle)-2.0; //deg
  HipVel = readgyro()-bias_pitch.velocity;
  avgs = update_avg();

  switch (control_case) {
    case 1: {// PC shoulder
        Serial.print("PC_shoulder");
        if (avgs.HipAngle > IMU_threshold) {
            control_case = 3;} // PC hip
        if ((avgs.HipAngle < IMU_threshold) & (abs(avgs.MotorAngle-origines.shoulder)<compliance_angle)) {
            control_case = 2;} // shoulder control
        MotorIn.t_in = 0; MotorIn.p_in = origines.shoulder;
        break; }

    case 2: {//shoulder control
        Serial.print("shoulder_control");
        float Ts = As*sin((MotorOut.position-origines.shoulder)*R)+0.5;
        if (HipAngle_avg > IMU_threshold) {
            control_case = 3;} //PC hip
        MotorIn.p_in = 0; MotorIn.t_in = Ts;
        break; }

    case 3: {//PC hip
        Serial.print("PC_hip");
        if (avgs.HipAngle < IMU_threshold) {
            control_case = 1; } // PC shoulder
        if ((avgs.HipAngle > IMU_threshold) & (abs(avgs.MotorAngle-origines.leg)<compliance_angle)) {
            control_case = 4; } // hip_control
        MotorIn.t_in = 0; MotorIn.p_in = origines.leg;
        break; }

    case 4: {// hip control
        Serial.print("hip_control");
        float Tleg = -Al*sin(radians(HipAngle));
        if (HipAngle_avg < IMU_threshold) {
            control_case = 1;} //PC hip
        MotorIn.p_in = 0; MotorIn.t_in = Tleg;
        break; }
  }

  MotorIn.p_in = constrain(MotorIn.p_in, P_MIN, P_MAX);
  MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);

  float LB_kd = 0.2; // lower bound kd
  float Akd = 2*LB_kd;
  //MotorIn.kd_in = 4*cos((2*PI/(2*origines.midpoint-radians(HipAngle)))*((MotorOut.position-radians(HipAngle))-origines.leg));

    MotorIn.kd_in = 2;
//   if (MotorOut.velocity > 0) {
//     MotorIn.kd_in = 0.1;
//   }
//   else {
//     MotorIn.kd_in = 0.5;
//   }

  //pack & unpack msgs
  pack_cmd(MotorIn);
  MotorOut = unpack_reply();
  Serial.print("  T_in: " + String(MotorIn.t_in)+ "  p_in: " + String(MotorIn.p_in));
  Serial.println("  pout: " + String(MotorOut.position)+"  IMU_Ang: " + String(HipAngle));

  while (millis()-time_now < dt) {}
}