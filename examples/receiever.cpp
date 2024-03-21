/*
* Adafruit Feather M4 CAN Receiver Example */
#include <CANSAME5x.h>

CANSAME5x CAN;

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
    Serial.println("Starting CAN!"); 
}

void loop() {
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
        while (CAN.available()) {
            Serial.print(CAN.read()); 
        }
        Serial.println();
    }
    delay(1000);
}