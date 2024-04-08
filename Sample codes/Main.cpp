#include <MotorUtilities.h>
#include <potentiometerUtilities.h>
#include <IMUutilities.h>

#define POT_K1 17 // pin number of potentiometer
#define POT_K2 18
#define POT3 19 // unused

//////// Variable definition ////////

// intialise Motor command and Motor reply variables
MotorCommand MotorIn;
MotorReply MotorOut;
// intialise IMU Data
float LegAngle = 0.0;

//Mechanical constant:
const float StaticFrictionTorque = 1.0;
float TorqueAmplitude = 1.0;


void setup() {
  //starts Serial Com
  Serial.begin(11250);
  while (!Serial) delay(10);

  //starts Can com
  SetupCan();
  delay(1000);

  initializeIMU();

  pinMode(POT_K1, INPUT);
  pinMode(POT_K2, INPUT);
  pinMode(POT3, INPUT);

  //Setup motor
  EnterMotorMode();
  delay(2000);
  SetZero();
  delay(2000);
}

void slack_hip() {
  while(abs(MotorOut.torque)<=StaticFrictionTorque){
      //MotorIn.t_in = StaticFrictionTorque;
      MotorIn.p_in = constrain(MotorIn.p_in - Step, P_MIN, P_MAX);
      pack_cmd(MotorIn);
      MotorOut = unpack_reply();
      Serial.println("P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));
  }
}

void loop() {
    //Read IMU
    LegAngle = readIMU();
    Serial.println("Leg Angle : " + String(LegAngle));

    //Read POTs
    float POT_reading1 = analogRead(POT_K1); // between 0 to 1023
    float K1 = map_float(POT_reading1, 0, 1023, 4, 8);

    float POT_reading2 = analogRead(POT_K2); // between 0 to 1023
    float K2 = map_float(POT_reading2, 0, 1023, 0, 2);

    //float POT_reading3 = analogRead(POT3);
    Serial.println("    K1 = "+String(K1)+" A= "+String(K2));

    if(LegAngle >=30.0){
      Serial.println("leg mode");
      //move motor until it collides with the leg
      slack_hip;
      //maintains position
      MotorIn.p_in = MotorOut.position;
      MotorIn.t_in = -TorqueAmplitude*sin(radians(LegAngle)) -1.0;
      MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);
      pack_cmd(MotorIn);
      MotorOut = unpack_reply();
      Serial.println(">>> Maintining P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));
  }
  else{
    //move motor until it collides with the pulley
    while(abs(MotorOut.torque)<=StaticFrictionTorque){
      MotorIn.p_in = constrain(MotorIn.p_in + Step, P_MIN, P_MAX);
      pack_cmd(MotorIn);
      MotorOut = unpack_reply();
      Serial.println("P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));
    }
    //maintains position
    MotorIn.p_in = MotorOut.position;
    MotorIn.t_in = TorqueAmplitude*sin(MotorOut.position) +1.0;
    MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);
    pack_cmd(MotorIn);
    MotorOut = unpack_reply();
    Serial.println(">>> Maintining P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));
  }
}

    // read potentiometer to set torque amplitude
    //float POT_reading = analogRead(POT); // between 0 to 1023
    //TorqueAmplitude = map_float(POT_reading, 0, 1023, 0, 4);