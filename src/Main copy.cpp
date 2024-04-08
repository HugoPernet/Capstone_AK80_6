#include <MotorUtilities.h>
#include <potentiometerUtilities.h>
#include <IMUutilities.h>

#define POT_K_1 17 // pin number of potentiometer
#define POT_A 18

//////// Variable definition ////////

// intialise Motor command and Motor reply variables
MotorCommand MotorIn;
MotorReply MotorOut;
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

  initializeIMU();

  pinMode(POT_A, INPUT);
  pinMode(POT_K_1, INPUT);

  //Setup motor
  EnterMotorMode();
  delay(2000);
  SetZero();
  delay(2000);

    while(abs(MotorOut.torque)<=StaticFirctionTroque){
      MotorIn.p_in = constrain(MotorIn.p_in + Step, P_MIN, P_MAX);
      pack_cmd(MotorIn);
      MotorOut = unpack_reply();
      Serial.println("P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));
    }
    SetZero();
    delay(2000);
  
}



void loop() {
    //move motor until it collides with the pulley

    // read potentiometer to set torque amplitude
    float POT_reading1 = analogRead(POT_K_1); // between 0 to 1023
    K_1 = map_float(POT_reading1, 0, 1023, 0, 2);

    float POT_reading2 = analogRead(POT_A); // between 0 to 1023
    TorqueAmplitude = map_float(POT_reading2, 0, 1023, 4, 8);
    Serial.println("K1 = "+String(K_1)+" A= "+String(TorqueAmplitude));

    //maintains position
    MotorIn.p_in = MotorOut.position;
    MotorIn.t_in = TorqueAmplitude*sin(MotorOut.position)+ K_1*MotorOut.velocity +0.5;//+MotorOut.velocity*K_1 +0.5;
    MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);
    pack_cmd(MotorIn);
    MotorOut = unpack_reply();
    Serial.println(">>> Maintining P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));
}

