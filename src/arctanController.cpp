#include <MotorUtilities.h>
#include <potentiometerUtilities.h>
#include <IMUutilities.h>


//////// Variable definition ////////

// intialise Motor command and Motor reply variables
MotorCommand MotorIn;
MotorReply MotorOut;
Joint_origines origines;

// intialise IMU Data
float LegAngle = 0.0;

//Mechanical constant:
const float StaticFirctionTroque = 1.0;
float K_1 = 1.5;
float TorqueAmplitude = 2.0;



void setup() {
  //starts Serial Com
  Serial.begin(11250);
  while (!Serial) delay(10);

  //starts Can com
  SetupCan();
  delay(1000);

  //Setup motor
  EnterMotorMode();
  Serial.println("motor mode");
  delay(1000);
  SetZero();
  delay(1000);

  MotorOut = unpack_reply();
  origines = Homing(MotorOut,1.0,MotorIn);
}



void loop() {
    MotorOut = unpack_reply();
    Serial.println(" origine leg: "+String(origines.leg)+ " origine shoulder: "+String(origines.shoulder)+ " current pos: "+String(MotorOut.position));
    delay(2000);
}