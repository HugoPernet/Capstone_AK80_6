#include <CANSAME5x.h>
#include <MotorConfig.h>

CANSAME5x CAN;

struct CAN_Rx
{
    float positionL = 0.0f;
    float velocityL = 0.0f;
    float torqueL= 0.0f;

    float positionR = 0.0f;
    float velocityR = 0.0f;
    float torqueR= 0.0f;
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


CAN_Rx unpack_reply() {
  byte bufL[8];
  byte bufR[8];
  struct CAN_Rx reply;

 
  unsigned int packetSize = CAN.parsePacket();
  Serial.println(packetSize);
  /// Listen to  CAN ///
  if (packetSize){
      while (CAN.available()) {
        CAN.readBytes(bufL,8);
        Serial.println("reads 1");
        Serial.print(packetSize);
      }
      unsigned int p_int = (bufL[1] << 8) | bufL[2];
      unsigned int v_int = (bufL[3] << 4) | (bufL[4] >> 4);
      unsigned int i_int = ((bufL[4] & 0xf) << 8) | bufL[5];

      reply.positionL = uint_to_float(p_int, P_MIN, P_MAX, 16);
      reply.velocityL= uint_to_float(v_int, V_MIN, V_MAX, 12); //rad/s
      reply.torqueL = uint_to_float(i_int, -T_MAX, T_MAX, 12); //rad

    Serial.println("id: "+String(bufL[0]) + " pos: "+String(reply.positionL) );
  }


  CAN.filter(0x02);
  packetSize = CAN.parsePacket();
  Serial.print(packetSize);
  /// Listen to  CAN ///
  if (packetSize){
      while (CAN.available()) {
        CAN.readBytes(bufR,8);
        Serial.println("reads 2");
        Serial.print(packetSize);
      }

      unsigned int p_int = (bufR[1] << 8) | bufR[2];
      unsigned int v_int = (bufR[3] << 4) | (bufR[4] >> 4);
      unsigned int i_int = ((bufR[4] & 0xf) << 8) | bufR[5];

      reply.positionR = uint_to_float(p_int, P_MIN, P_MAX, 16);
      reply.velocityR= uint_to_float(v_int, V_MIN, V_MAX, 12); //rad/s
      reply.torqueR = uint_to_float(i_int, -T_MAX, T_MAX, 12); //rad
    
    Serial.println("id: "+String(bufR[0]) + " pos: "+String(reply.positionR) );
  }
  return reply;
  }
  
struct Joint_origines
{
    float shoulder_L;
    float leg_L;
    float midpoint_L;

    float shoulder_R;
    float leg_R;
    float midpoint_R;

};


void HomingL(CAN_Rx reply,float threshold,CAN_Tx command,Joint_origines origine){
    
    //goes to shoulder
    while(abs(reply.torqueL)<=threshold){
      command.p_in_L = constrain(command.p_in_L + Step, P_MIN, P_MAX);
      pack_cmd(command);
      reply = unpack_reply();
      Serial.println(reply.positionL);
      origine.shoulder_L = reply.positionL;
    }
    delay(500);
    Serial.println("shoulder ok");
    //goes to leg
    while(reply.torqueL >= -threshold){
      command.p_in_L = constrain(command.p_in_L - Step, P_MIN, P_MAX);
      pack_cmd(command);
      reply = unpack_reply();
      origine.leg_L = reply.positionL;
    }
    delay(500);
    Serial.println("leg ok");
    origine.midpoint_L = abs(origine.leg_L-origine.shoulder_L)/2;
    Serial.println(origine.midpoint_L);
    reply = unpack_reply();

    //move to new motor origine
    while(abs(reply.positionL - (origine.leg_L+origine.midpoint_L))>=0.05){
      reply = unpack_reply();
      command.p_in_L = constrain(command.p_in_L + Step, P_MIN, P_MAX);
      pack_cmd(command);
      delay(10);
    }
    delay(1000);
}

Joint_origines HomingR(CAN_Rx reply,float threshold,CAN_Tx command,Joint_origines origine){
    //goes to shoulder
    while(abs(reply.torqueR)<=threshold){
      command.p_in_R = constrain(command.p_in_R + Step, P_MIN, P_MAX);
      pack_cmd(command);
      reply = unpack_reply();
      Serial.println(reply.torqueR);
      origine.leg_R = reply.positionR;
    }
    delay(500);
    Serial.println("shoulder ok");
    //goes to leg
    while(reply.torqueR >= -threshold){
      command.p_in_R = constrain(command.p_in_R - Step, P_MIN, P_MAX);
      pack_cmd(command);
      reply = unpack_reply();
      origine.shoulder_R = reply.positionR;
    }
    delay(500);
    Serial.println("leg ok");
    origine.midpoint_R = abs(origine.leg_R-origine.shoulder_R)/2;
    Serial.println(origine.midpoint_R);
    reply = unpack_reply();

    //move to new motor origine
    while(abs(reply.positionR - (origine.leg_R-origine.midpoint_R))>=0.05){
      reply = unpack_reply();
      command.p_in_R = constrain(command.p_in_R + Step, P_MIN, P_MAX);
      pack_cmd(command);
      delay(10);
    }
    delay(1000);
}



 