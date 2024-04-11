#include "Arduino.h"
#include <MotorUtilities.h>
#include <potentiometerUtilities.h>
#include <IMUutilities.h>

#define POT_B1 16
#define POT_K1 17 // pin number of potentiometer
#define POT_C1 18
#define POT_D1 19 // 

#define IMU_RESET_BTN 9 // IMU reset button

//////// Variable definition ////////

// initialize Motor command and Motor reply variables
MotorCommand MotorIn;
MotorReply MotorOut;

// initialize IMU Data
float HipVel = 0.0;
float HipAngle = 0.0;
float dt = 50;
float bias_pitch = 0.0;
float bias_pitch_rate = 0.0;

//Mechanical constant:
const float StaticFrictionTorque = 0.25;
float TorqueAmplitude = 1.0;
float P0_shoulder = 0;
float P0_hip = 0;
float K2, C2, D2, G2 = 0.1;
float P0_midpt = 0;

// initialize button
int IMU_BTN_STATE = 0;
unsigned long debounceDelay = 50;
unsigned long myTime=millis();

void isr() {  // the function to be called when interrupt is triggered
  IMU_BTN_STATE = 1;
}

float homing() {
  while(MotorOut.torque< 1.1*StaticFrictionTorque){
      MotorIn.p_in = constrain(MotorIn.p_in + Step, P_MIN, P_MAX);
      pack_cmd(MotorIn);
      MotorOut = unpack_reply();
      Serial.println("P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));
  }
  P0_shoulder = MotorOut.position;

  while(MotorOut.torque>= -0.7*StaticFrictionTorque){
      MotorIn.p_in = constrain(MotorIn.p_in - Step, P_MIN, P_MAX);
      pack_cmd(MotorIn);
      MotorOut = unpack_reply();
      Serial.println("P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));
  }
  P0_hip = MotorOut.position;
  Serial.println("Homing P0_shoulder, P0_hip complete");
  P0_midpt = fmod((P0_shoulder+P0_hip)/2,2*PI);
  while(abs(MotorOut.position - P0_midpt) <= 0.05) {
    MotorIn.p_in = P0_midpt;
    pack_cmd(MotorIn);
    MotorOut = unpack_reply();
  }
  return P0_midpt;
}

void setup() {
  //starts Serial Com
  Serial.begin(11250);
  while (!Serial) delay(10);

  //starts Can com
  SetupCan();
  delay(500);

  pinMode(POT_K1, INPUT);
  pinMode(POT_C1, INPUT);
  pinMode(POT_D1, INPUT);
  pinMode(POT_B1, INPUT);

  //button for IMU zero
  pinMode(IMU_RESET_BTN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU_RESET_BTN), isr, RISING);

  initializeIMU();
  bias_pitch = get_bias_pitch();
  bias_pitch_rate = get_bias_pitch_rate();

  //Setup motor
  EnterMotorMode();
  delay(50);

  SetZero();
  delay(50);
  homing();
  delay(50);
  SetZero();
  Serial.println("Zeroed and Homed at Midpoint of shoulder & hip");
  delay(500);
  while(MotorOut.torque< 1.1*StaticFrictionTorque){
      MotorIn.p_in = constrain(MotorIn.p_in + Step, P_MIN, P_MAX);
      pack_cmd(MotorIn);
      MotorOut = unpack_reply();
      Serial.println("P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));
  }
}



void loop() {
  float time_now = millis();
  // Zero IMU reading
  if (IMU_BTN_STATE == 1) {
    bias_pitch_rate = readgyro();
    IMU_BTN_STATE = 0;
  }

  //Read IMU
  HipAngle = (readIMU()-bias_pitch)-5; // deg
  HipVel = (readgyro()-bias_pitch_rate); // deg (positive is bending down)
  
  
  //Read POTs
  float POT_reading0 = analogRead(POT_B1);
  float G1 = map_float(POT_reading0, 0, 1023, 1,6);
  float POT_reading1 = analogRead(POT_K1); // between 0 to 1023
  float K1 = map_float(POT_reading1, 0, 1023, 2, 10); // Torque Amplitude (Proportional to Shoulder Angle)
  float POT_reading2 = analogRead(POT_C1); // between 0 to 1023
  float C1 = map_float(POT_reading2, 0, 1023, 2, 10); // Proportional to Shoulder Velocity
  float POT_reading3 = analogRead(POT_D1);
  float D1 = map_float(POT_reading3, 0, 1023, 15, 30);
  //Serial.print("    B1 = " + String(POT_reading0) + " K1 = "+String(K1)+" C1 = "+String(C1)+ " D1 =" + String(D1));
  Serial.print("    K1 = "+String(K1)+" C1 = "+String(C1)+ " D1 =" + String(D1)+ " G1 =" + String(G1));

  float AmpH = K1*(1/PI);
  float AmpS = C1*(1/PI);
  float AmpSF = D1*(1/PI);
  //float c = K2;

  //maintains position
  float As = 5; float Al = 5; float Ss = 1; float midpoint = 90;
  float shoulder = As*(1/PI)*atan((MotorOut.position*180/PI-midpoint)-10);
  float leg = Al*(1/PI)*atan((MotorOut.position*180/PI+midpoint)+20);
  float switching = -2*Ss*(1/PI)*atan(HipAngle-10);
  float test_torque = shoulder + leg + switching;
  // float motor_torque = AmpH*atan(MotorOut.position-P0_midpt)+AmpS*atan(MotorOut.position+P0_midpt)-2*AmpSF*atan(-HipAngle-10);

  MotorIn.p_in = MotorOut.position;
  float c = 1;
  MotorIn.t_in = K1*atan(K2*MotorOut.position) + C1*atan(C2*MotorOut.velocity) - D1*atan(D2*radians(HipAngle)-G1*atan(G2*radians(HipVel)));
  
  MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);
  pack_cmd(MotorIn);
  MotorOut = unpack_reply();
  Serial.println(" TORQUE_IN = " + String(MotorIn.t_in) + "TEST_TORQUE = " + String(test_torque)+ "   MEASURING:  Hip_Angle = " + String(HipAngle)+ " Hip_Vel = " + String(HipVel)+" P_out:"+String(MotorOut.position*180/PI)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));

  while (millis()-time_now < dt) {}
}