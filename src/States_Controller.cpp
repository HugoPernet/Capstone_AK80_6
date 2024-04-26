//left motor 0x01

#include "Arduino.h"
#include <MotorUtilities.h>
#include <potentiometerUtilities.h>
#include <IMUutilities.h>

//////// Variable definition ////////

//structs
MotorCommand MotorIn; // initialize Motor command and Motor reply variables
MotorReply MotorOut;
Joint_origines origines;
bias bias_pitch;

//Loop() Period
float dt = 10;

// initialize homing parameters
float P0_shoulder = 0;
float P0_hip = 0;
float P0_midpt = 0;

// initialize IMU Data
float HipVel = 0.0;
float HipAngle = 0.0;

// average of last few points
const int sample_pts = 5;
float HipAngle_avg = 0;
float HipAngle_arr[sample_pts] = {0};
float MotorAngle_avg = 0;
float MotorAngle_arr[sample_pts] = {0};


//Mechanical constant:
const float StaticFrictionTorque = 0.25;
float As = 2.0;
float Al = 2.0;
float K2, C2, D2 = 0.1; // torque slope
float R = 5;

// case switching
int control_case = 1;
float IMU_threshold1 = 10.0f; // degrees
float IMU_threshold2 = 15.0f; // degrees
float compliance_angle_s = radians(10);
float compliance_angle_l = radians(40);

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

//   HipAngle_arr[sample_pts] = {round(readIMU())-bias_pitch.angle-2};
//   MotorAngle_arr[sample_pts] = {origines.leg+origines.midpoint};

  Serial.println("Homed shoulder & hip");
  Serial.println("CONTROL START");
  delay(1000);
}

void loop() {
  float time_now = millis();
  MotorOut = unpack_reply();

  //Read IMU
  HipAngle = (round(readIMU())-bias_pitch.angle)-2.0; //deg
  HipVel = readgyro()-bias_pitch.velocity;
  MotorAngle_avg = update_avg(MotorAngle_arr, MotorOut.position, sample_pts);
  HipAngle_avg = update_avg(HipAngle_arr, HipAngle, sample_pts);

  switch (control_case) {
    case 1: {// PC shoulder
        Serial.print("PC_shoulder");
        if (HipAngle_avg > IMU_threshold2) {
            control_case = 3;} // PC hip
        if ((abs(HipAngle) <= IMU_threshold2) && (MotorOut.position>=(origines.leg+origines.midpoint)) && (abs(MotorOut.torque) >= 0.1)) {
            control_case = 2;} // shoulder control
        //Serial.print("  delta1 = " + String(degrees(abs(MotorOut.position-origines.shoulder))));
        bool a = HipAngle_avg < IMU_threshold2;
        bool b = MotorOut.position>=(origines.leg+origines.midpoint);
        bool c = abs(MotorOut.torque) >= 0.1;
        Serial.print( "  a,b,c= " + String(a)+" "+String(b)+" "+String(c) + " imu = " + String(HipAngle));
        MotorIn.kp_in = 2;
        MotorIn.p_in = origines.shoulder;
        MotorIn.t_in = 0.5; 
        break; }

    case 2: {//shoulder control
        Serial.print("shoulder_control");
        float Ts = As*sin((MotorOut.position-origines.shoulder)*R)+0.5;
        if (HipAngle_avg > IMU_threshold2) {
            control_case = 3;} //PC hip
        MotorIn.kp_in = 0; 
        MotorIn.t_in = Ts;
        break; }

    case 3: {//PC hip
        Serial.print("PC_hip");
        if (HipAngle_avg < IMU_threshold1) {
            control_case = 1; } // PC shoulder
        if ((HipAngle_avg > IMU_threshold1) && (MotorOut.position<=(origines.leg+origines.midpoint)) && (abs(MotorOut.torque) >= 0.5)) {
            control_case = 4; } // hip_control
        //Serial.print("  delta2 = " + String(degrees(abs(MotorOut.position-origines.leg))));
        MotorIn.kp_in = 2;
        MotorIn.p_in = origines.leg;
        MotorIn.t_in = 0; 
        break; }

    case 4: {// hip control
        Serial.print("hip_control");
        float Tleg = -Al*sin(radians(HipAngle));
        if (HipAngle_avg < IMU_threshold1) {
            control_case = 1;} //PC hip
        MotorIn.kp_in = 0; 
        MotorIn.t_in = Tleg;
        break; }
  }

  MotorIn.p_in = constrain(MotorIn.p_in, P_MIN, P_MAX);
  MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);

    MotorIn.kd_in = 0.5;
//   if (MotorOut.velocity > 0) {
//     MotorIn.kd_in = 0.1;
//   }
//   else {
//     MotorIn.kd_in = 0.5;
//   }

  //pack & unpack msgs
  pack_cmd(MotorIn);
  MotorOut = unpack_reply();
  Serial.print("  T_in: " + String(MotorIn.t_in)+ "  T_out: " + String(MotorOut.torque) + "  p_in: " + String(MotorIn.p_in));
  Serial.println("  pout: " + String(MotorOut.position)+"  IMU_Ang: " + String(HipAngle));

  while (millis()-time_now < dt) {}
}