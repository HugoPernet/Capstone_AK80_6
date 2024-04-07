#include <MotorUtilities.h>
#include <potentiometerUtilities.h>
#include <IMUutilities.h>

#define POT 19 // pin number of potentiometer

//////// Variable definition ////////

// intialise Motor command and Motor reply variables
MotorCommand MotorIn;
MotorReply MotorOut;
// intialise IMU Data
float LegAngle = 0.0;

//Mechanical constant:
const float StaticFirctionTroque = 1.0;
float TorqueAmplitude = 1.0;


void setup() {
  //starts Serial Com
  Serial.begin(11250);
  while (!Serial) delay(10);

  //starts Can com
  SetupCan();
  delay(1000);

  initializeIMU();

  //Setup motor
  EnterMotorMode();
  delay(2000);
  SetZero();
  delay(2000);
  
}



void loop() {
    //Read IMU
    LegAngle = readIMU();
    Serial.println("Leg Angle : " + String(LegAngle));

    if(LegAngle >=30.0){
      Serial.println("leg mode");
      //move motor until it collides with the leg
      while (MotorOut.torque >= -StaticFirctionTroque){
        MotorIn.p_in = constrain(MotorIn.p_in - Step, P_MIN, P_MAX);
        pack_cmd(MotorIn);
        delay(5);
        MotorOut = unpack_reply();
        Serial.println("P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out "+ String(MotorOut.velocity));
      } 
      //maintains position
      MotorIn.p_in = MotorOut.position;
          MotorIn.t_in = -TorqueAmplitude*sin(radians(LegAngle)) -1.0;
      pack_cmd(MotorIn);
      delay(5);
      MotorOut = unpack_reply();
      Serial.println(">>> Maintining P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));
  }
  else{
    //move motor until it collides with the pulley
    while(abs(MotorOut.torque)<=StaticFirctionTroque){
      MotorIn.p_in = constrain(MotorIn.p_in + Step, P_MIN, P_MAX);
      pack_cmd(MotorIn);
      delay(5);
      MotorOut = unpack_reply();
      Serial.println("P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));
    }
    //maintains position
    MotorIn.p_in = MotorOut.position;
    MotorIn.t_in = TorqueAmplitude*sin(MotorOut.position) +1.2;
    //MotorIn.t_in = constrain(MotorIn.t_in, T_MAX, T_MAX);
    pack_cmd(MotorIn);
    delay(5);
    MotorOut = unpack_reply();
    Serial.println(">>> Maintining P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));
  }
}

    // read potentiometer to set torque amplitude
    //float POT_reading = analogRead(POT); // between 0 to 1023
    //TorqueAmplitude = map_float(POT_reading, 0, 1023, 0, 4);