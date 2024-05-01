#include <CANSAME5x.h>
#include <MotorConfig.h>

CANSAME5x CAN;

struct CAN_Rx
{
    float position = 0.0f;
    float velocity = 0.0f;
    float torque= 0.0f;
};


struct CAN_Tx
{
    float p_in_L = 0.0f;
    float v_in_L = 0.0f;
    float kp_in_L = 2.0f;
    float kd_in_L = 1.0f;
    float t_in_L= 0.0f;

    float p_in_R = 0.0f;
    float v_in_R = 0.0f;
    float kp_in_R = 2.0f;
    float kd_in_R = 1.0f;
    float t_in_R= 0.0f;
};



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

void SetupCan(){
    // initialize CAN com

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
    Serial.println("CAN started!");
}

void EnterMotorMode() {
  /// Enable Motor ///
  unsigned char buf[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
  
  if (CAN.availableForWrite()==0)
  {
  CAN.beginPacket(CAN_ID_L,dlc,false);
  for (int i =0; i<=7;i++){
    CAN.write(buf[i]);
  }
  CAN.endPacket();
  delay(5);
  CAN.beginPacket(CAN_ID_R,dlc,false);
  for (int i =0; i<=7;i++){
    CAN.write(buf[i]);
  }
  CAN.endPacket();
  Serial.println("Motor mode");
  }
}

void ExitMotorMode() {
  /// Disable the Motor ///
  unsigned char buf[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
  CAN.beginPacket(CAN_ID_R,dlc,false);
  for (int i =0; i<=7;i++){
    CAN.write(buf[i]);
  }
  CAN.endPacket();
  delay(5);
  CAN.beginPacket(CAN_ID_L,dlc,false);
  for (int i =0; i<=7;i++){
    CAN.write(buf[i]);
  }
  CAN.endPacket();
}

void SetZero() {
  /// Sets current encoder position to 0 rad ///
  unsigned char buf[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

  if(CAN.availableForWrite()==0){
    CAN.beginPacket(CAN_ID_L,dlc,false);
    for (int i =0; i<=7;i++){
      CAN.write(buf[i]);
    }
    CAN.endPacket();
    Serial.println("zero R");
    delay(5);
    CAN.beginPacket(CAN_ID_R,dlc,false);
    for (int i =0; i<=7;i++){
      CAN.write(buf[i]);
    }
    CAN.endPacket();
    Serial.println("zero L");
  }
}

void pack_cmd(CAN_Tx command) {
  // Send command to the motor///
  float p_des = constrain(command.p_in_L, P_MIN, P_MAX);
  float v_des = constrain(command.v_in_L, V_MIN, V_MAX);
  float kp = constrain(command.kp_in_L, KP_MIN, KP_MAX);
  float kd = constrain(command.kd_in_L, KD_MIN, KD_MAX);
  float t_ff = constrain(command.t_in_L, T_MIN, T_MAX);

  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  unsigned char bufL[8] = {(p_int >>8), (p_int & 0xff),(v_int >> 4),(((v_int & 0xf) << 4) | (kp_int >> 8)),(kp_int & 0xff),(kd_int >> 4),(((kd_int & 0xf) << 4) | (t_int >> 8)),(t_int & 0xff)};
  
  p_des = constrain(command.p_in_R, P_MIN, P_MAX);
  v_des = constrain(command.v_in_R, V_MIN, V_MAX);
  kp = constrain(command.kp_in_R, KP_MIN, KP_MAX);
  kd = constrain(command.kd_in_R, KD_MIN, KD_MAX);
  t_ff = constrain(command.t_in_R, T_MIN, T_MAX);

  p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  unsigned char bufR[8] = {(p_int >>8), (p_int & 0xff),(v_int >> 4),(((v_int & 0xf) << 4) | (kp_int >> 8)),(kp_int & 0xff),(kd_int >> 4),(((kd_int & 0xf) << 4) | (t_int >> 8)),(t_int & 0xff)};
  
if(CAN.availableForWrite()==0){
  CAN.beginPacket(CAN_ID_L,dlc,false);
  for (int i =0; i<=7;i++){
    CAN.write(bufL[i]);
  }
  CAN.endPacket();
  delay(5);
  CAN.beginPacket(CAN_ID_R,dlc,false);
  for (int i =0; i<=7;i++){
    CAN.write(bufR[i]);
  }
  CAN.endPacket();
  delay(5);
}
}


CAN_Rx unpack_reply_L() {
  byte bufL[8];
  CAN_Rx reply;

  unsigned int packetSize = CAN.parsePacket();
  CAN.filter(0x01,0xFF);
  /// Listen to  CAN ///
  if (packetSize){
      while (CAN.available()) {
        CAN.readBytes(bufL,8);
      }
      unsigned int p_int = (bufL[1] << 8) | bufL[2];
      unsigned int v_int = (bufL[3] << 4) | (bufL[4] >> 4);
      unsigned int i_int = ((bufL[4] & 0xf) << 8) | bufL[5];

      reply.position = uint_to_float(p_int, P_MIN, P_MAX, 16);
      reply.velocity= uint_to_float(v_int, V_MIN, V_MAX, 12); //rad/s
      reply.torque = uint_to_float(i_int, -T_MAX, T_MAX, 12); //rad
  }
  return reply;
}
CAN_Rx unpack_reply_R() {

  unsigned int packetSize = CAN.parsePacket();
  byte bufR[8];
  CAN_Rx reply;

  CAN.filter(0x02,0xFF);
  /// Listen to  CAN ///
  if (packetSize){
      while (CAN.available()) {
        CAN.readBytes(bufR,8);
      }

      unsigned int p_int = (bufR[1] << 8) | bufR[2];
      unsigned int v_int = (bufR[3] << 4) | (bufR[4] >> 4);
      unsigned int i_int = ((bufR[4] & 0xf) << 8) | bufR[5];

      reply.position = uint_to_float(p_int, P_MIN, P_MAX, 16);
      reply.velocity= uint_to_float(v_int, V_MIN, V_MAX, 12); //rad/s
      reply.torque = uint_to_float(i_int, -T_MAX, T_MAX, 12); //rad
  }
  return reply;
  }
  
struct Joint_origines
{
    float shoulder;
    float leg;
    float midpoint;

};


Joint_origines HomingL(CAN_Rx reply,float threshold,CAN_Tx command){
    Joint_origines origine;
    //goes to shoulder
    while(abs(reply.torque)<=threshold){
      command.p_in_L = constrain(command.p_in_L + Step, P_MIN, P_MAX);
      pack_cmd(command);
      reply = unpack_reply_L();
      origine.shoulder = reply.position;
    }
    delay(500);
    Serial.println("shoulder ok");
    //goes to leg
    while(reply.torque >= -threshold){
      command.p_in_L = constrain(command.p_in_L - Step, P_MIN, P_MAX);
      pack_cmd(command);
      reply = unpack_reply_L();
      origine.leg = reply.position;
    }
    delay(500);
    Serial.println("leg ok");
    origine.midpoint = abs(origine.leg-origine.shoulder)/2;
    Serial.println(origine.midpoint);
    reply = unpack_reply_L();

    //move to new motor origine
    while(abs(reply.position - (origine.leg+origine.midpoint))>=0.5){
      reply = unpack_reply_L();
      command.p_in_L = constrain(command.p_in_L + Step, P_MIN, P_MAX);
      pack_cmd(command);
      delay(10);
    }
    Serial.println("Homing L -> done");
    return origine;
}

Joint_origines HomingR(CAN_Rx reply,float threshold,CAN_Tx command){
  Joint_origines origine;
    //goes to shoulder
    while(abs(reply.torque)<=threshold){
      command.p_in_R = constrain(command.p_in_R + Step, P_MIN, P_MAX);
      pack_cmd(command);
      reply = unpack_reply_R();
      origine.leg = reply.position;
    }
    delay(500);
    Serial.println("leg ok");
    //goes to leg
    while(reply.torque >= -threshold){
      command.p_in_R = constrain(command.p_in_R - Step, P_MIN, P_MAX);
      pack_cmd(command);
      reply = unpack_reply_R();
      origine.shoulder = reply.position;
    }
    delay(500);
    Serial.println("shoulder ok");
    origine.midpoint = abs(origine.leg-origine.shoulder)/2;
    Serial.println(origine.midpoint);
    reply = unpack_reply_R();

    //move to new motor origine
    while(abs(reply.position - (origine.leg-origine.midpoint))>=0.05){
      reply = unpack_reply_R();
      command.p_in_R = constrain(command.p_in_R + Step, P_MIN, P_MAX);
      pack_cmd(command);
      delay(10);
    }
    Serial.println("Homing R -> done");
    return origine;
}



 