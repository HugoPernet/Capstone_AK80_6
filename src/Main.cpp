#include <MotorUtilities.h>
#include <IMUutilities.h>

//////// Variable definition ////////

// intialise Motor command and Motor reply variables
MotorCommand MotorIn;
MotorReply MotorOut;
// intialise IMU Data
IMU_data IMUout;

//Mechanical constant:
const float StaticFirctionTroque = 1.0;
const float TorqueAmplitude = 2.0;


void setup() {
  //starts Serial Com
  Serial.begin(1000);
  while (!Serial) delay(10);

  //starts Can com
  SetupCan();
  delay(1000);

  //SetUp IMU
  initializeIMU();

  //Setup motor
  EnterMotorMode();
  delay(2000);
  SetZero();
  delay(2000);
  
}

float dir = 1;

void loop() {

  //move motor until it collides with the pulley
    if (abs(MotorOut.torque)<=StaticFirctionTroque){
      if (MotorIn.p_in <= P_MIN || MotorIn.p_in >= P_MAX) {
      dir *= -1;
      }
    MotorIn.p_in = constrain(MotorIn.p_in + dir*Step, P_MIN, P_MAX);
    pack_cmd(MotorIn);
    delay(10);
    MotorOut = unpack_reply();
    Serial.println("P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.velocity)+" V_out"+ String(MotorOut.torque));
    }
  //maintains position
    else{
    MotorIn.p_in = MotorOut.position;
    MotorIn.t_in = TorqueAmplitude*sin(MotorOut.position) +2.0;
    //MotorIn.t_in = constrain(MotorIn.t_in, T_MAX, T_MAX);
    pack_cmd(MotorIn);
    delay(10);
    MotorOut = unpack_reply();
    Serial.println(">>> Maintining P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));
    }
  //Read IMU
  IMUout = readIMU();
  Serial.println("IMU Pitch: "+String(IMUout.pitch) + "IMU Roll"+ String(IMUout.roll));
}