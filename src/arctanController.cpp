#include <MotorUtilities.h>
#include <potentiometerUtilities.h>
#include <IMUutilities.h>
#include <math.h>


//////// Variable definition ////////

// intialise Motor command and Motor reply variables
MotorCommand MotorIn;
MotorReply MotorOut;
Joint_origines origines;


//Mechanical constant:
const float StaticFirctionTroque = 1.0;
float K_1 = 1.5;
float TorqueAmplitude = 2.0;

float truncAngle;



void setup() {
  //starts Serial Com
  Serial.begin(11250);
  while (!Serial) delay(10);

  //starts Can com
  SetupCan();
  delay(1000);

  // imu
  initializeIMU();

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
    truncAngle = round(readIMU()+82);
    
    //move motor until it collides with the pulley
    MotorIn.p_in = MotorOut.position;
    MotorIn.t_in = 1.0;
    MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);
    pack_cmd(MotorIn);
    Serial.println("leg real angle = "+ String(MotorOut.position-origines.leg));
    Serial.println("leg real Leg = "+ String(MotorOut.position-origines.shoulder));

    float shoulder = TorqueAmplitude*(1/M_PI)*atan(degrees(MotorOut.position-origines.shoulder)-10)+TorqueAmplitude/2;
    float leg = TorqueAmplitude*(1/M_PI)*atan(degrees(MotorOut.position-origines.leg) +20) - TorqueAmplitude/2;
    float switching = -2*StaticFirctionTroque*(1/M_PI)*atan((truncAngle-10));

    Serial.println("shoulder T: "+String(shoulder)+ " Leg T:"+String(leg)+" imu T: "+ String(switching));

    MotorOut = unpack_reply();
    Serial.println(">>>  P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" imu"+ String(truncAngle));
  }
