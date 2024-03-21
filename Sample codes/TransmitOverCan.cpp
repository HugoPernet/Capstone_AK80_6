
#include <CANSAME5x.h>
#include <MotorParameters.h>
CANSAME5x CAN;

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

float p_des = 0.0f;

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
void ListenCan(){
     // try to parse packet
    int packetSize = CAN.parsePacket();
    if (packetSize) {
    // received a packet Serial.print("Received ");
    if (CAN.packetExtended()) { 
        Serial.print("extended ");
    }
    if (CAN.packetRtr()) {
    // Remote transmission request, packet contains no data 
    Serial.print("RTR ");
    }
    Serial.print("packet with id 0x"); 
    Serial.print(CAN.packetId(), HEX);
    if (CAN.packetRtr()) {
        Serial.print(" and requested length "); 
        Serial.println(CAN.packetDlc());
    } 
    else {
    Serial.print(" and length "); 
    Serial.println(packetSize);
    // only print packet data for non-RTR packets 
    while (CAN.available()) {
        Serial.print(CAN.read()); }
        Serial.println(); }
        Serial.println(); 
    }
}



void setup() { 
    Serial.begin(115200); 
    while (!Serial) delay(10);
    Serial.println("CAN Sender");
    pinMode(PIN_CAN_STANDBY, OUTPUT); 
    digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY 
    pinMode(PIN_CAN_BOOSTEN, OUTPUT); 
    digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster
    
    // start the CAN bus at 1Mbaud  
    if (!CAN.begin(1000000)) {
    Serial.println("Starting CAN failed!");
    while (1) delay(10); }
    Serial.println("Starting CAN!"); 
    p_des = p_in;
}

void loop() {
    // send packet: id is 11 bits, packet can contain up to 8 bytes of data 
    Serial.print("Sending packet ... ");
    Serial.print(" ");
    Serial.print("Enable Motor");
    Serial.print(" ");
    CAN.beginPacket(0x968); 
    CAN.write(0xFF);
    CAN.write(0xFF); 
    CAN.write(0xFF); 
    CAN.write(0xFF);
    CAN.write(0xFF);
    CAN.write(0xFF);
    CAN.write(0xFF);
    CAN.write(0xFC); 
    CAN.endPacket();
    Serial.println("done"); 
    delay(100);

    ///convert floats to unsigned ints///
    unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    unsigned int v_int = float_to_uint(v_in, V_MIN, V_MAX, 12);
    unsigned int kp_int = float_to_uint(kp_in, KP_MIN, KP_MAX, 12);
    unsigned int kd_int = float_to_uint(kd_in, KD_MIN, KD_MAX, 12);
    unsigned int t_int = float_to_uint(t_in, T_MIN, T_MAX, 12);

    Serial.print("move motor");
    Serial.print(" ");
    CAN.beginPacket(0x968); 
    CAN.write(p_int >>8); //high 8 bit of p_int
    CAN.write(p_int & 0xFF); //low 8 bit of p_int
    CAN.write(v_int >> 4);// bits 4 to 11 of p_int
    CAN.write(((v_int & 0xF) <<4) | (kp_int >>8)); //bits 0 to 3 of p_int, high 8 bit of Kp
    CAN.write(kp_int & 0xFF); //low 8 bit of Kp
    CAN.write(kd_int >>4); // bits 4 to 11 of Kd
    CAN.write(((kd_int & 0xF) <<4) | (t_int >>8)); //bits 0 to 3 of kd, high 8 bit of t
    CAN.write(t_int & 0xFF); //low 8 bit of t
    CAN.endPacket();
    Serial.println("done"); 
    delay(100);

    }


