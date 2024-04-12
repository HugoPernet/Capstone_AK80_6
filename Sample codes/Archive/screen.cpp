// not working yet

#include "Arduino.h"
#include <IMUutilities.h>
#include <LiquidCrystal.h>

const int rs = 6, en = 5, d4 = 9, d5 = 10, d6 = 11, d7 = 12;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  Serial.begin(11250);
  while (!Serial) delay(10);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  //Wire.begin();
  Wire.begin(5,6); // SDA, SCL
  delay(2000);

  initializeIMU();
  lcd.setCursor(0,1);
  lcd.print("init IMU");
}

void loop() {
  //Read IMU
  float HipAngle = round(readIMU()); //deg
  Serial.println("HipAngle = "+ String(HipAngle));
  lcd.clear();
  lcd.print("Ang = "+String(HipAngle));
  delay(500);
}