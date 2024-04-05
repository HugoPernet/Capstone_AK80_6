#include "Arduino.h"

#define POT 19

void setup() {
    pinMode(POT, INPUT);
    Serial.begin(115200);
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void loop() {
    float POT_reading = analogRead(POT); // between 0 to 1023
    float A = map_float(POT_reading, 0, 1023, 0, 4);
    Serial.print("POT_reading: ");
    Serial.print(POT_reading);
    Serial.print(", ");
    Serial.print("A: ");
    Serial.println(A);
}