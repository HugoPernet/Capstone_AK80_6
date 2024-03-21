#include <Arduino.h>
#include <CANSAME5x.h>

//custom headers
#include <MotorParameters.h>

CANSAME5x CAN;
#define MY_PACKET_ID 0x968 //id is 11 bits


// intial values
float p_in = 0.0f;
float v_in = 3.0f;
float kp_in = 0.0f;
float kd_in = 3.0f;
float t_in = 0.0f;

//measured values
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;

bool dir = 0;

// define functions

unsigned int float_to_uint(float x, float x_min, float x_max, int bits)
{
  ///Converts a  float to an unsigned int, given range and number of bits///
  float span = x_max-x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if(bits==12){
    pgg = (unsigned int) ((x-offset)*4095.0/span);
  }
  if(bits==16){
    pgg = (unsigned int) ((x-offset)*65535.0/span);
  }
  return pgg;
}

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits)
{
  ///converts unsigned int to float, given range and number of bits///
  float span = x_max-x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits==12){
    pgg = ((float) x_int)*span/4095 + offset;
  }
  if (bits==16){
    pgg = ((float) x_int)*span/65535.0 + offset;
  }
  return pgg;

}

void unpack_reply() {
  /// CAB reply Oacjet structure///
  /// 16 bit position, between -4*pi and 4*pi
  /// 12 bit velocity, between -30 and +30 rad/s
  /// 12 bit current, between -40 and 40;
  /// CAN Packet is 5 8-bit words
  ///Formatted as follows. For each quantity, bit 0 is LSB
  /// 0: [position [15-8]]
  /// 1: [position[7-0]]
  /// 2: [velocity [11-4]]
  /// 3: [velocity [3-0], current [11-8]]
  /// 4: [current [7-0]]

  //read Data
  int packetSize = CAN.parsePacket();
  Serial.print(packetSize); 
  

  if (packetSize){
    uint8_t receivedData[packetSize];

    for (int i=0; i<packetSize; i++) {
    receivedData[i] = CAN.read(); 
    Serial.print("0x"); 
    Serial.print(receivedData[i], HEX); 
    Serial.print(", ");
    }
  /// unpack ints from CAN buffer ///
  unsigned int id = receivedData[0];
  unsigned int p_int = (receivedData[1] << 8) | receivedData[2];
  unsigned int v_int = (receivedData[3] << 4) | (receivedData[4] >> 4);
  unsigned int i_int = ((receivedData[4] & 0xF) << 8) | receivedData[5];
  /// convert uints to floats ///
  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
  } 
}

void EnterMotorMode(){
  //Enter Motor Mode(enable)
  CAN.beginPacket(MY_PACKET_ID);
  for (int i = 0; i<=6; i++){
    CAN.write(0xFF); // write to data segement [i]
  }
  CAN.write(0xFC);
  CAN.endPacket();
  Serial.print("motor enable");
  Serial.print(" ");

}

void ExitMotorMode(){
  CAN.beginPacket(MY_PACKET_ID);
  for (int i = 0; i<=7; i++){
    CAN.write(0xFF); // write to data segement [i]
  }
  CAN.endPacket();
}

void send_packet(){
  ///CAN Command Packet Structure///
  ///16 bit position command, between -4*pi and 4*pi
  ///12 bit velocity command, between -30 and +30rad/s
  //12 bit kp, between 0 and 500 N-m/rad
  ///12 bit kd, between 0 and 100 N-m*s/rad
  /// 12 bit feed forward toque, between -18 and 18 N-m
  
  ///CAN packet is 8x 8-bit words
  ///Formatted as follows. For each quantity, bit i0 is LSB
  ///0: [position[15-8[[ (low 8 bit)
  ///1: [position [7-0]] (high 8 bit)
  ///2: [velocity[11-4]]
  ///3: [velocity[3-0], kp[11-8]]
  ///4: [kp[7-0]]
  ///5: [kd[11-4]]
  ///6: [kd[3-0], torque [11-8]]
  ///7: [torque [7-0]]

  ///coerce data within bounds///
  float p_des = constrain(p_in, P_MIN, P_MAX); 
  float v_des = constrain(v_in, V_MIN, V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX);

 ///convert floats to unsigned ints///
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

/// pack values into the can buffer///
  CAN.beginPacket(MY_PACKET_ID);
  CAN.write(p_int >>8); //high 8 bit of p_int
  CAN.write(p_int & 0xFF); //low 8 bit of p_int
  CAN.write(v_int >> 4);// bits 4 to 11 of p_int
  CAN.write(((v_int & 0xF) <<4) | (kp_int >>8)); //bits 0 to 3 of p_int, high 8 bit of Kp
  CAN.write(kp_int & 0xFF); //low 8 bit of Kp
  CAN.write(kd_int >>4); // bits 4 to 11 of Kd
  CAN.write(((kd_int & 0xF) <<4) | (t_int >>8)); //bits 0 to 3 of kd, high 8 bit of t
  CAN.write(t_int & 0xFF); //low 8 bit of t
  CAN.endPacket();
}






void setup() {
  // init the serial monitor
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("connecting to CAN controler...");

  //define Pin Layout for Adafruit Feather M4 CAN Express with ATSAME51
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster

  // start the CAN bus at 1 Mbaud !
  if (!CAN.begin(1000000)) {
    Serial.println("Starting CAN failed!");
    while (1) delay(10);
  }
  Serial.println("CAN connection established");
}

void loop() {

   //enter motor mode
  EnterMotorMode();


  float p_step = 0.001;

  if ((p_in >= 12.5) & (dir == 1)) {
    dir = 0;
  }
  else if ((p_in <= -12.5) & (dir == 0)) {
    dir = 1;
  }

  if (dir == 1) {
    p_in = p_in + p_step;
  }
  else {
    p_in = p_in - p_step;
  }
  delay(5000);
  // finnd a way to change position


  //send CAN
  send_packet();

  //receive CAN
  Serial.print("Can available: ");
  Serial.print(" ");
  if(CAN.available())
  {
    unpack_reply();
    Serial.print(float(CAN.available()));
    Serial.print(" ");
  }
  else{
      Serial.print("False");
      Serial.print(" ");
  }

  
  //print data
  Serial.print(p_in);
  Serial.print(" ");
  Serial.print(p_out);
  Serial.print(" ");
  Serial.print(v_out);
  Serial.print(" ");
  Serial.println(t_out);
}

