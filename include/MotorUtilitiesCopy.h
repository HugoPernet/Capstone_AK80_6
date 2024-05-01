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
    float p_in = 0.0f;
    float v_in = 0.0f;
    float kp_in = 2.0f;
    float kd_in = 1.0f;
    float t_in= 0.0f;
};

struct rep
{
  CAN_Rx replyL;
  CAN_Rx replyR;
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
     // Set the receive callback
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
    CAN.beginPacket(CAN_ID_R,dlc,false);
    for (int i =0; i<=7;i++){
      CAN.write(buf[i]);
    }
    CAN.endPacket();
    Serial.println("zero L");
  }
}

void pack_cmd(CAN_Tx command, float id) {
  float time_now = millis();
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
  
  
if(CAN.availableForWrite()==0){
  CAN.beginPacket(id,dlc,false);
  for (int i =0; i<=7;i++){
    CAN.write(buf[i]);
  }
  CAN.endPacket();
  while (millis()-time_now < 2) {}
}
}


void onCANReceive(int packetSize) {
  if (CAN.parsePacket()) {
    Serial.print("Received CAN message with ID: 0x");
    Serial.print(CAN.packetId(), HEX);

    while (CAN.available()) {
      Serial.print(CAN.read()); // Read a byte from the CAN message
      Serial.print(" ");
    }
    Serial.println();
  }
}

rep unpack_reply() {
  byte bufL[8];
  rep reply;

  unsigned int packetSize = CAN.parsePacket();
  Serial.println(packetSize);

  /// Listen to  CAN ///
  if (packetSize){
      if (CAN.available()) {
        CAN.readBytes(bufL,8);
        
        if(CAN.packetId()==1){
          Serial.println("read1");
        unsigned int p_int = (bufL[1] << 8) | bufL[2];
        unsigned int v_int = (bufL[3] << 4) | (bufL[4] >> 4);
        unsigned int i_int = ((bufL[4] & 0xf) << 8) | bufL[5];

        reply.replyL.position = uint_to_float(p_int, P_MIN, P_MAX, 16);
        reply.replyL.velocity= uint_to_float(v_int, V_MIN, V_MAX, 12); //rad/s
        reply.replyL.torque = uint_to_float(i_int, -T_MAX, T_MAX, 12); //rad
        }
        else if (CAN.packetId()==2)
        Serial.println("read2");
        {
        unsigned int p_int = (bufL[1] << 8) | bufL[2];
        unsigned int v_int = (bufL[3] << 4) | (bufL[4] >> 4);
        unsigned int i_int = ((bufL[4] & 0xf) << 8) | bufL[5];

        reply.replyR.position = uint_to_float(p_int, P_MIN, P_MAX, 16);
        reply.replyR.velocity= uint_to_float(v_int, V_MIN, V_MAX, 12); //rad/s
        reply.replyR.torque = uint_to_float(i_int, -T_MAX, T_MAX, 12); //rad
        }
        
      }
  }
  Serial.println("L "+String(reply.replyL.position)+ " R "+String(reply.replyR.position));
  return reply;
}

CAN_Rx unpack_reply_L() {

  unsigned int packetSize = CAN.parsePacket();
  byte buf[8];
  CAN_Rx reply;

  /// Listen to  CAN ///
  if (packetSize){
      while (CAN.available()) {
        CAN.filter(0x01,0xFF);
        CAN.readBytes(buf,8);
        Serial.println(CAN.packetId());
      }

      unsigned int p_int = (buf[1] << 8) | buf[2];
      unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
      unsigned int i_int = ((buf[4] & 0xf) << 8) | buf[5];

      reply.position = uint_to_float(p_int, P_MIN, P_MAX, 16);
      reply.velocity= uint_to_float(v_int, V_MIN, V_MAX, 12); //rad/s
      reply.torque = uint_to_float(i_int, -T_MAX, T_MAX, 12); //rad
      Serial.println("L "+String(reply.position));
  }

  return reply;
  }

CAN_Rx unpack_reply_R() {

  unsigned int packetSize = CAN.parsePacket();
  byte buf[8];
  CAN_Rx reply;

  /// Listen to  CAN ///
  if (packetSize){
      while (CAN.available()) {
        CAN.filter(0x02,0xFF);
        CAN.readBytes(buf,8);
        CAN.filter(0x02,0xFF);
        Serial.println(CAN.packetId());
      }

      unsigned int p_int = (buf[1] << 8) | buf[2];
      unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
      unsigned int i_int = ((buf[4] & 0xf) << 8) | buf[5];

      reply.position = uint_to_float(p_int, P_MIN, P_MAX, 16);
      reply.velocity= uint_to_float(v_int, V_MIN, V_MAX, 12); //rad/s
      reply.torque = uint_to_float(i_int, -T_MAX, T_MAX, 12); //rad
      Serial.println("R "+String(reply.position));
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
      command.p_in = constrain(command.p_in + Step, P_MIN, P_MAX);
      pack_cmd(command,CAN_ID_L);
      reply = unpack_reply_L();
      origine.shoulder = reply.position;
    }
    delay(500);
    Serial.println("shoulder ok");
    //goes to leg
    while(reply.torque >= -threshold){
      command.p_in = constrain(command.p_in - Step, P_MIN, P_MAX);
      pack_cmd(command,CAN_ID_L);
      reply = unpack_reply_L();
      origine.leg = reply.position;
    }
    delay(500);
    Serial.println("leg ok");
    origine.midpoint = abs(origine.leg-origine.shoulder)/2;
    Serial.println(origine.midpoint);
    reply = unpack_reply_L();

    //move to new motor origine
    while(abs(reply.position - (origine.leg+origine.midpoint))>=0.05){
      reply = unpack_reply_L();
      command.p_in = constrain(command.p_in + Step, P_MIN, P_MAX);
      pack_cmd(command,CAN_ID_L);
      delay(5);
    }
    Serial.println("Homing L -> done");
    return origine;
}

Joint_origines HomingR(CAN_Rx reply,float threshold,CAN_Tx command){
  Joint_origines origine;
    //goes to shoulder
    while(abs(reply.torque)<=threshold){
      command.p_in = constrain(command.p_in + Step, P_MIN, P_MAX);
      pack_cmd(command,CAN_ID_R);
      reply = unpack_reply_R();
      origine.leg = reply.position;
    }
    delay(500);
    Serial.println("leg ok");
    //goes to leg
    while(reply.torque >= -threshold){
      command.p_in = constrain(command.p_in- Step, P_MIN, P_MAX);
      pack_cmd(command,CAN_ID_R);
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
      command.p_in = constrain(command.p_in + Step, P_MIN, P_MAX);
      pack_cmd(command,CAN_ID_R);
      delay(5);
    }
    Serial.println("Homing R -> done");
    return origine;
}

void torqueL(CAN_Rx Motor_Rx_L,float HipAngle,CAN_Tx Motor_Tx_L,Joint_origines origine_L){
  float As = 4.0;
  Motor_Rx_L = unpack_reply_L();
  float R = 5;
  ////// Left /////
  float Ts = As*sin((Motor_Rx_L.position-origine_L.shoulder)*R);
  float Tleg = -As*sin(radians(HipAngle));

  float Switch_leg = (1/PI)*atan(HipAngle -3)+0.5; 
  float Switch_shoulder = ((1/PI)*atan(degrees((Motor_Rx_L.position-origine_L.shoulder)*R) -3)+0.5);
  float Ts_switch = Ts*Switch_shoulder + 0.5;
  float Tleg_switch = Tleg*Switch_leg;
    
  Motor_Tx_L.t_in = Ts_switch+Tleg_switch;
  Motor_Tx_L.t_in = constrain(Motor_Tx_L.t_in, T_MIN, T_MAX);
  pack_cmd(Motor_Tx_L,CAN_ID_L);
}

void torqueR(CAN_Rx Motor_Rx_R,float HipAngle,CAN_Tx Motor_Tx_R, Joint_origines origine_R){
  float As = 4.0;
  Motor_Rx_R = unpack_reply_R();
  ////// Right /////
  float R = 5;
  float Ts_R = As*sin((Motor_Rx_R.position-origine_R.shoulder)*R);
  float Tleg_R = As*sin(radians(HipAngle));

  float Switch_leg_R = (1/PI)*atan(HipAngle -3)+0.5; 
  float Switch_shoulder_R = 1-((1/PI)*atan(degrees((Motor_Rx_R.position-origine_R.shoulder)*R) +10)+0.5);

  float Ts_switch_R = Ts_R*Switch_shoulder_R -0.5;
  float Tleg_switch_R = Tleg_R*Switch_leg_R;

  Motor_Tx_R.t_in = Ts_switch_R+Tleg_switch_R;
  Motor_Tx_R.t_in = constrain(Motor_Tx_R.t_in, T_MIN, T_MAX);

  pack_cmd(Motor_Tx_R,CAN_ID_R);
}
 