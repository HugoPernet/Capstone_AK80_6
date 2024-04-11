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

//Button
#define IMU_RESET_BTN 9 // IMU reset button

//////// Variable definition ////////

// initialize Motor command and Motor reply variables
MotorCommand MotorIn;
MotorReply MotorOut;
Joint_origines origines;

// initialize IMU Data
float HipVel = 0.0;
float HipAngle = 0.0;
float bias_pitch = 0.0;
float bias_pitch_rate = 0.0;

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

// initialize button
int IMU_BTN_STATE = 0;
unsigned long debounceDelay = 50;
unsigned long myTime=millis();

void isr() {  // the function to be called when interrupt is triggered
  IMU_BTN_STATE = 1;
}

// float homing() {
//   while(MotorOut.torque< StaticFrictionTorque){
//       MotorIn.p_in = constrain(MotorIn.p_in + Step, P_MIN, P_MAX);
//       pack_cmd(MotorIn);
//       MotorOut = unpack_reply();
//       Serial.println("P_out:"+String(MotorOut.position*180/PI)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity*180/PI));
//   }
//   P0_shoulder = MotorOut.position*180/PI;

//   while(MotorOut.torque>= -0.5*StaticFrictionTorque){
//       MotorIn.p_in = constrain(MotorIn.p_in - Step, P_MIN, P_MAX);
//       pack_cmd(MotorIn);
//       MotorOut = unpack_reply();
//       Serial.println("P_out:"+String(MotorOut.position*180/PI)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity*180/PI));
//   }
//   P0_hip = MotorOut.position*180/PI;
//   Serial.println("Homing P0_shoulder, P0_hip complete");
//   P0_midpt = abs((P0_shoulder-P0_hip)/2); // in degrees

//   while(abs(MotorOut.position*180/PI - (P0_hip+P0_midpt)) >= 3) {
//     MotorIn.p_in = constrain(MotorIn.p_in + 0.01, P_MIN, P_MAX);
//     pack_cmd(MotorIn);
//     MotorOut = unpack_reply();
//     delay(10);
//   }
//   Serial.println("midpoint = " +String(P0_midpt));
//   delay(2000);
//   return P0_midpt;
// }

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

  //button for IMU zero
  pinMode(IMU_RESET_BTN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU_RESET_BTN), isr, RISING);

  bias_pitch = initializeIMU();

  //Setup motor
  EnterMotorMode(); delay(50);
  SetZero(); delay(50);
  MotorOut = unpack_reply();
  P0_midpt = Homing(MotorOut,1.0,MotorIn);
  //homing(); delay(50);
  SetZero(); delay(500);

  // while(MotorOut.torque< StaticFrictionTorque){
  //     MotorIn.p_in = constrain(MotorIn.p_in + Step, P_MIN, P_MAX);
  //     pack_cmd(MotorIn);
  //     MotorOut = unpack_reply();
  //     Serial.println("P_out:"+String(MotorOut.position*180/PI)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity*180/PI));
  // }
  Serial.println("Zeroed and Homed at Midpoint of shoulder & hip");
  delay(1000);
}



void loop() {
  float time_now = millis();
  //Zero IMU reading
  // if (IMU_BTN_STATE == 1) {
  //   bias_pitch = initializeIMU();
  //   IMU_BTN_STATE = 0;
  // }

  //Read IMU
  HipAngle = (readIMU()-bias_pitch)-5; // deg
  // HipVel = (readgyro()-bias_pitch); // deg
  
  
  //Read POTs
    //Torque Amplitudes
  float POT_reading1 = analogRead(POT_K1); // between 0 to 1023
  float K1 = map_float(POT_reading1, 0, 1023, 0.5, 3); // Shoulder Angle
  float POT_reading2 = analogRead(POT_C1); // between 0 to 1023
  float C1 = map_float(POT_reading2, 0, 1023, 0.5, 3); // Shoulder Velocity
  float POT_reading3 = analogRead(POT_D1);
  float D1 = map_float(POT_reading3, 0, 1023, 0.5, 3); // Hip Angle

    //Torque Slopes
  float POT_reading4 = analogRead(POT_K2);
  float K2 = map_float(POT_reading4, 0, 1023, 0.05,0.5); // Shoulder Angle ~0.3
  float POT_reading5 = analogRead(POT_C2);
  float C2 = map_float(POT_reading5, 0, 1023, 0.05,0.15); // Shoulder Velocity ~0.05
  float POT_reading6 = analogRead(POT_D2);
  float D2 = map_float(POT_reading6, 0, 1023, 0.05,0.5); //Hip Angle, ~0.2
  Serial.print("    K1: "+String(K1)+" C1: "+String(C1)+ " D1:" + String(D1) + " K2: " + String(K2)+" C2: "+String(C2)+" D2: "+String(D2));

  //maintains position
  MotorIn.p_in = MotorOut.position;
  MotorIn.t_in = K1*atan(K2*(MotorOut.position*180/PI-P0_midpt)*180/PI) + C1*atan(C2*MotorOut.velocity*180/PI) - D1*atan(D2*HipAngle);
  //MotorIn.t_in = 0;

  //test torque
  float As = 5; float Al = 5; float Ss = 1; float midpoint = P0_midpt;
  float shoulder = As*(1/PI)*atan((MotorOut.position*180/PI-midpoint)-10);
  float leg = Al*(1/PI)*atan((MotorOut.position*180/PI+midpoint)+20);
  float switching = -2*Ss*(1/PI)*atan(HipAngle-10);
  float test_torque = shoulder + leg + switching; //"T_test: " + String(test_torque)+
  
  MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);
  pack_cmd(MotorIn);
  MotorOut = unpack_reply();
  Serial.println(" T_in: " + String(MotorIn.t_in) +  "   MEASURING:  Hip_A: " + String(HipAngle)+ " Hip_V: " + String(HipVel)+" P_out:"+String(MotorOut.position*180/PI-P0_midpt)+ " t_out:"+String(MotorOut.torque)+" v_out"+ String(MotorOut.velocity*180/PI));

  while (millis()-time_now < dt) {}
}