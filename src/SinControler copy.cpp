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
CAN_Rx Motor_Rx;
Joint_origines origine;
bias bias_pitch;



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

  Motor_Rx = unpack_reply();
  HomingR(Motor_Rx,1.0,Motor_Tx,origine);
  Motor_Rx = unpack_reply();
  HomingL(Motor_Rx,1.0,Motor_Tx,origine);

}




void loop() {
  unpack_reply();
  delay(1000);

}