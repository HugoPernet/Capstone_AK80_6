#include <MotorUtilities.h>
#include <potentiometerUtilities.h>
#include <IMUutilities.h>

#define POT_K1 17 // pin number of potentiometer
#define POT_C1 18
#define POT_D1 19 // 

//////// Variable definition ////////

// intialise Motor command and Motor reply variables
MotorCommand MotorIn;
MotorReply MotorOut;
// intialise IMU Data
float LegVel = 0.0;
float LegAngle = 0.0;
float dt = 100;
float Leg_Vel_Initial = 0.0;

//Mechanical constant:
const float StaticFrictionTorque = 1.0;
float TorqueAmplitude = 1.0;
float K2, C2, D2 = 0.1;


void setup() {
  //starts Serial Com
  Serial.begin(11250);
  while (!Serial) delay(10);

  //starts Can com
  SetupCan();
  delay(1000);

  Leg_Vel_Initial = initializeIMU();

  pinMode(POT_K1, INPUT);
  pinMode(POT_C1, INPUT);
  pinMode(POT_D1, INPUT);

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
      Serial.print("P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));
  }
}

void loop() {
  float time_now = millis();

  //Read IMU
  LegVel = -(readIMU()-Leg_Vel_Initial);
  LegAngle = fmod(LegAngle + (LegVel*dt)*0.001*180/PI, 360);
  
  //Read POTs
  float POT_reading1 = analogRead(POT_K1); // between 0 to 1023
  float K1 = map_float(POT_reading1, 0, 1023, 2, 6); // Torque Amplitude (Proportional to Shoulder Angle)
  float POT_reading2 = analogRead(POT_C1); // between 0 to 1023
  float C1 = map_float(POT_reading2, 0, 1023, 2, 6); // Proportional to Shoulder Velocity
  float POT_reading3 = analogRead(POT_D1);
  float D1 = map_float(POT_reading3, 0, 1023, 2, 6);
  Serial.print(" K1 = "+String(K1)+" C1 = "+String(C1)+ " D1 =" + String(D1));

  //maintains position
  MotorIn.p_in = MotorOut.position;
  //MotorIn.t_in = TorqueAmplitude*sin(MotorOut.position) +1.0;
  //float c = 0.7*(K1+C1+D1)*3.14/2; // c ensures MotorIn.t_in > 0
  float c = 0;
  MotorIn.t_in = K1*atan(K2*MotorOut.position) + C1*atan(C2*MotorOut.velocity) - D1*atan(D2*LegAngle)+c;
  MotorIn.t_in = constrain(MotorIn.t_in, T_MIN, T_MAX);
  pack_cmd(MotorIn);
  MotorOut = unpack_reply();
  Serial.println("    Measuring Leg_Angle = " + String(LegAngle)+" P_out:"+String(MotorOut.position)+ " torque:"+String(MotorOut.torque)+" V_out"+ String(MotorOut.velocity));

  while (millis()-time_now < dt) {}
}