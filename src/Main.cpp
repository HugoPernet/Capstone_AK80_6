/*
* Adafruit Feather M4 CAN Receiver Example */
#include <CANSAME5x.h>

//custom headers
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
#define MY_PACKET_ID 0x968



void EnterMotorMode(){
  //Enter Motor Mode(enable)
    CAN.beginExtendedPacket(MY_PACKET_ID);
    for (int i = 0; i<=6; i++){
    CAN.write(0xFF); // write to data segement [i]
    }
    CAN.write(0XFC);
    CAN.endPacket();

    // error handeling:
    if (CAN.getWriteError()){
            Serial.print("Motor Mode error");
            Serial.print(" ");  
    }
    else{
            Serial.print("Enter Motor mmode sucessfully");
            Serial.print(" ");  
    }
  
}

void ResetEncoder(){
  //Enter Motor Mode(enable)
    Serial.print("reset encoder");
    CAN.beginExtendedPacket(MY_PACKET_ID);
    for (int i = 0; i<=6; i++){
    CAN.write(0xFF); // write to data segement [i]
    }
    CAN.write(0XFE);
    CAN.endPacket();

    // error handeling:
    if (CAN.getWriteError()){
            Serial.print("Write error");
            Serial.print(" ");  
    }
    else{
            Serial.print("Write sucessfully");
            Serial.print(" ");  
    }
}

void setup() { 
    Serial.begin(115200); 
    while (!Serial) delay(10);
    Serial.println("CAN Receiver");
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
}

void loop() {
    EnterMotorMode();
    // try to parse packet
    int packetSize = CAN.parsePacket();
    
    if (packetSize) {
    // received a packet 
    Serial.print("Received ");
        if (CAN.packetExtended()) { 
            Serial.print("extended ");
        }
        Serial.print("packet with id 0x"); 
        Serial.print(CAN.packetId(), HEX);
        Serial.print(" and length "); 
        Serial.println(packetSize);
        Serial.print("Data:");

        uint8_t receivedData[packetSize];

        for (int i=0; i<packetSize; i++) {
            receivedData[i] = CAN.read(); 
            Serial.print("0x");
            Serial.print(receivedData[i], HEX); 
            Serial.print(", ");
        }
        
        Serial.println();
        Serial.print("new commit test");

    }
    delay(2000);
}