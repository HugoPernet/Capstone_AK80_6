#include <Arduino.h>


void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open
}

void loop() {
  Serial.print("Hello world");
  delay(500);
}