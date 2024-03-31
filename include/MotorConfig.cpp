// Author @HugoPernet

#include <CANSAME5x.h>
#include <MotorParameters.h>

CANSAME5x CAN;

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
  unsigned char buf[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
  
  CAN.beginPacket(CAN_ID_Motor_L,dlc,false);
  for (int i =0; i<=7;i++){
    CAN.write(buf[i]);
  }
  CAN.endPacket();
}

void ExitMotorMode() {
  unsigned char buf[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
  
  CAN.beginPacket(CAN_ID_Motor_L,dlc,false);
  for (int i =0; i<=7;i++){
    CAN.write(buf[i]);
  }
  CAN.endPacket();
}

void SetZero() {
  unsigned char buf[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};
  
  CAN.beginPacket(CAN_ID_Motor_L,dlc,false);
  for (int i =0; i<=7;i++){
    CAN.write(buf[i]);
  }
  CAN.endPacket();
}

void pack_cmd(float p_in, float v_in, float kp_in, float kd_in, float t_in) {
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
  
  CAN.beginPacket(CAN_ID_Motor_L,dlc,false);
  for (int i =0; i<=7;i++){
    CAN.write(buf[i]);
  }
  CAN.endPacket();

}

struct Motor_Out {
  float CanID;
  float p_out;
  float v_out;
  float t_out;
};

Motor_Out unpack_reply() {

  byte buf[8];
  CAN.readBytes(buf,8);

  unsigned int id = buf[0];
  unsigned int p_int = (buf[1] << 8) | buf[2];
  unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
  unsigned int i_int = ((buf[4] & 0xf) << 8) | buf[5];

  struct Motor_Out Motor_Command;

  Motor_Command.CanID = id;
  Motor_Command.p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  Motor_Command.v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  Motor_Command.t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);

  Serial.println("torque:"+String(Motor_Command.p_out)+" V out"+ String(Motor_Command.v_out));

  return Motor_Command;
}
