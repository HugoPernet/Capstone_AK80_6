#include <CANSAME5x.h>
#include <MotorConfig.h>

CANSAME5x CAN;


// Set values
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 2.0f;
float kd_in = 1.0f;
float t_in = 0.0f;
// measured values
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;

unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12) {
    pgg = (unsigned int) ((x-offset)*4095.0/span);
  }
  if (bits == 16) {
    pgg = (unsigned int) ((x-offset)*65535.0/span);
  }
  return pgg;
}

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  /// Converts Uint to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12) {
    pgg = ((float) x_int) * span / 4095.0 + offset;
  }
  if (bits == 16) {
    pgg = ((float) x_int) * span / 65535.0 + offset;
  }
  return pgg;
}

void EnterMotorMode() {
  /// Enable Motor ///
  unsigned char buf[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
  CAN.beginPacket(MY_PACKET_ID,dlc,false);
  for (int i =0; i<=7;i++){
    CAN.write(buf[i]);
  }
  CAN.endPacket();
}

void ExitMotorMode() {
  /// Disable the Motor ///
  unsigned char buf[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
  CAN.beginPacket(MY_PACKET_ID,dlc,false);
  for (int i =0; i<=7;i++){
    CAN.write(buf[i]);
  }
  CAN.endPacket();
}

void SetZero() {
  /// Sets current encoder position to 0 rad ///
  unsigned char buf[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};
  CAN.beginPacket(MY_PACKET_ID,dlc,false);
  for (int i =0; i<=7;i++){
    CAN.write(buf[i]);
  }
  CAN.endPacket();
}

void pack_cmd() {
  // Send command to the motor//
  float p_des = constrain(p_in, P_MIN, P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX);

  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  unsigned char buf[8] = {(p_int >> 8), (p_int & 0xff),(v_int >> 4),(((v_int & 0xf) << 4) | (kp_int >> 8)),(kp_int & 0xff),(kd_int >> 4),(((kd_int & 0xf) << 4) | (t_int >> 8)),(t_int & 0xff)};
  CAN.beginPacket(MY_PACKET_ID,dlc,false);
  
  for (int i =0; i<=7;i++){
    CAN.write(buf[i]);
  }
  CAN.endPacket();

}

void unpack_reply() {
  /// Listen to  CAN ///
  byte buf[8];

  int packetSize = CAN.parsePacket();

  if (packetSize){
      while (CAN.available()) {
        CAN.readBytes(buf,8); 
    }
  }
  
  unsigned long canId = CAN.packetId();
  unsigned int id = buf[0];
  unsigned int p_int = (buf[1] << 8) | buf[2];
  unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
  unsigned int i_int = ((buf[4] & 0xf) << 8) | buf[5];

  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
  Serial.println("P_out:"+String(p_out)+ " torque:"+String(t_out)+" V_out"+ String(v_out));
}




void setup() {
  //starts Serial Com
  Serial.begin(1000); //115200
  while (!Serial) delay(10);

  //Can PinOut
  pinMode(PIN_CAN_STANDBY, OUTPUT); 
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY 
  pinMode(PIN_CAN_BOOSTEN, OUTPUT); 
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster
  
  // start the CAN bus at 1Mbaud 
  if (!CAN.begin(1000000)) {
    Serial.println("Starting CAN failed!");
    while (1) delay(10); 
  }
  delay(1000);
  Serial.println("Starting CAN!");
  delay(1000);
  EnterMotorMode();
  SetZero();
  delay(4000);
}

float dir = 1;
void loop() {

  //move motor until it collides with the pulley
    if (abs(t_out)<=1.0){
    p_in = constrain(p_in + (dir * 0.01), P_MIN, P_MAX);
    pack_cmd();
    unpack_reply();
    delay(10);
    }
  //maintains position
    else{
    p_in = p_out;
    t_in = 6.0*sin(p_out) +2.0;
    pack_cmd();
    SERIAL_PORT_MONITOR.println("Maintaining pos arm "+String(p_out*360/2*3.14));
    delay(10);
    unpack_reply();
    }
}