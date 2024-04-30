#include <CANSAME5x.h>
#include <MotorConfig.h>

CANSAME5x CAN;

struct MotorReply
{
    float position = 0.0f;
    float velocity = 0.0f;
    float torque= 0.0f;
};

struct CAN_Reply
{
    float positionL = 0.0f;
    float velocityL = 0.0f;
    float torqueL= 0.0f;
    float positionR = 0.0f;
    float velocityR = 0.0f;
    float torqueR= 0.0f;
};

struct MotorCommand
{
    float p_in = 0.0f;
    float v_in = 0.0f;
    float kp_in = 2.0f;
    float kd_in = 1.0f;
    float t_in = 0.0f;
};

struct CAN_Command
{
  MotorCommand Mot_L;
  MotorCommand Mot_R;
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
}

void ExitMotorMode() {
  /// Disable the Motor ///
  unsigned char buf[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
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
}

void SetZero() {
  /// Sets current encoder position to 0 rad ///
  unsigned char buf[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

    CAN.beginPacket(CAN_ID_R,dlc,false);
    for (int i =0; i<=7;i++){
      CAN.write(buf[i]);
    }
    CAN.endPacket();
    Serial.println("zero R");

    CAN.beginPacket(CAN_ID_L,dlc,false);
    for (int i =0; i<=7;i++){
      CAN.write(buf[i]);
    }
    CAN.endPacket();
    Serial.println("zero L");

}

void pack_cmd(float CAN_ID, struct MotorCommand command) {
  // Send command to the motor///
  float p_des = constrain(command.p_in, P_MIN, P_MAX);
  float v_des = constrain(command.v_in, V_MIN, V_MAX);
  float kp = constrain(command.kp_in, KP_MIN, KP_MAX);
  float kd = constrain(command.kd_in, KD_MIN, KD_MAX);
  float t_ff = constrain(command.t_in, T_MIN, T_MAX);

  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  unsigned char buf[8] = {(p_int >>8), (p_int & 0xff),(v_int >> 4),(((v_int & 0xf) << 4) | (kp_int >> 8)),(kp_int & 0xff),(kd_int >> 4),(((kd_int & 0xf) << 4) | (t_int >> 8)),(t_int & 0xff)};
  CAN.beginPacket(CAN_ID,dlc,false);
  
  for (int i =0; i<=7;i++){
    CAN.write(buf[i]);
  }
  CAN.endPacket();
  delay(5);

}


void unpack_reply(int packetSize) {
  byte bufL[8];
  byte bufR[8];
  struct CAN_Reply reply;

  CAN.filter(1);
  Serial.println("read 1");
  int packetSize = CAN.parsePacket();
  /// Listen to  CAN ///
  if (packetSize){
      while (CAN.available()) {
        CAN.readBytes(bufL,8);
        Serial.println("reads 2");
    }
    unsigned int p_int = (bufL[1] << 8) | bufL[2];
    unsigned int v_int = (bufL[3] << 4) | (bufR[4] >> 4);
    unsigned int i_int = ((bufL[4] & 0xf) << 8) | bufL[5];

    reply.positionL = uint_to_float(p_int, P_MIN, P_MAX, 16);
    reply.velocityL= uint_to_float(v_int, V_MIN, V_MAX, 12); //rad/s
    reply.torqueL = uint_to_float(i_int, -T_MAX, T_MAX, 12); //rad

    for (int i=0; i<=7; i++){
      Serial.println(bufR[i]);
    }
  }

  CAN.filter(2);
  Serial.println("read 2");
  packetSize = CAN.parsePacket();
  /// Listen to  CAN ///
  if (packetSize){
      while (CAN.available()) {
        CAN.readBytes(bufR,8);
        Serial.println("reads 2");
    }

    unsigned int p_int = (bufR[1] << 8) | bufR[2];
    unsigned int v_int = (bufR[3] << 4) | (bufR[4] >> 4);
    unsigned int i_int = ((bufR[4] & 0xf) << 8) | bufR[5];

    reply.positionR = uint_to_float(p_int, P_MIN, P_MAX, 16);
    reply.velocityR= uint_to_float(v_int, V_MIN, V_MAX, 12); //rad/s
    reply.torqueR = uint_to_float(i_int, -T_MAX, T_MAX, 12); //rad

    for (int i=0; i<=7; i++){
      Serial.println(bufR[i]);
    }
  }
  delay(5000);
  }
  


 struct Joint_origines
{
    float shoulder;
    float leg;
    float midpoint;

};


Joint_origines HomingL(CAN_Reply reply,float threshold,CAN_Command command){
    
    Joint_origines origine;
    
    //goes to shoulder
    while(abs(reply.torqueL)<=threshold){
      command.Mot_L.p_in = constrain(command.Mot_L.p_in + Step, P_MIN, P_MAX);
      pack_cmd(CAN_ID_L,command.Mot_L);
      reply = unpack_reply();
      origine.shoulder = reply.positionL;
    }
    delay(500);
    Serial.println("shoulder ok");
    //goes to leg
    while(reply.torqueL >= -threshold){
      command.Mot_L.p_in = constrain(command.Mot_L.p_in - Step, P_MIN, P_MAX);
      pack_cmd(CAN_ID_L,command.Mot_L);
      reply = unpack_reply();
      origine.leg = reply.positionL;
    }
    delay(500);
    Serial.println("leg ok");
    origine.midpoint = abs(origine.leg-origine.shoulder)/2;
    Serial.println(origine.midpoint);
    reply = unpack_reply();

    //move to new motor origine
    while(abs(reply.positionR - (origine.leg+origine.midpoint))>=0.05){
      reply = unpack_reply();
      command.Mot_L.p_in = constrain(command.Mot_L.p_in + Step, P_MIN, P_MAX);
      pack_cmd(CAN_ID_L,command.Mot_L);
      delay(10);
    }
    delay(1000);
    return origine;
}

Joint_origines HomingR(CAN_Reply reply,float threshold,CAN_Command command){
    
    Joint_origines origine;
    
    //goes to shoulder
    while(abs(reply.torqueR)<=threshold){
      command.Mot_R.p_in = constrain(command.Mot_R.p_in + Step, P_MIN, P_MAX);
      pack_cmd(CAN_ID_R,command.Mot_R);
      reply = unpack_reply();
      origine.leg = reply.positionR;
    }
    delay(500);
    Serial.println("shoulder ok");
    //goes to leg
    while(reply.torqueR >= -threshold){
      command.Mot_R.p_in = constrain(command.Mot_R.p_in - Step, P_MIN, P_MAX);
      pack_cmd(CAN_ID_R,command.Mot_R);
      reply = unpack_reply();
      origine.shoulder = reply.positionR;
    }
    delay(500);
    Serial.println("leg ok");
    origine.midpoint = abs(origine.leg-origine.shoulder)/2;
    Serial.println(origine.midpoint);
    reply = unpack_reply();

    //move to new motor origine
    while(abs(reply.positionR - (origine.leg-origine.midpoint))>=0.05){
      reply = unpack_reply();
      command.Mot_R.p_in = constrain(command.Mot_R.p_in + Step, P_MIN, P_MAX);
      pack_cmd(CAN_ID_R, command.Mot_R);
      delay(10);
    }
    delay(1000);
    return origine;
}

