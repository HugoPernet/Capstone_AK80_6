#include <IMUutilities.h>

float LegVel = 0;
float LegAngle = 0;
float initializetime1 = 1000;
float initializetime2 = 5000;
float dt = 50;
float Leg_Vel_Initial = 0;
int count = 1;

void setup() {
    float time_now = millis();
    Serial.begin(115200);
    while (!Serial) delay(10);
    initializeIMU();
    while (millis()-time_now < initializetime1) {
        Leg_Vel_Initial = readIMU();
    }
    time_now = millis();
    while (millis()-time_now < initializetime2) {
        count = count+1;
        Leg_Vel_Initial = Leg_Vel_Initial + readIMU();
    }
    Leg_Vel_Initial = Leg_Vel_Initial/count;
    Serial.println("Time Zeroed, count = "+String(count));
    delay(1000);
}

void loop() {
    float time_now = millis();

    //Read IMU
    LegVel = readIMU()-Leg_Vel_Initial;
    LegAngle = fmod(LegAngle + (LegVel*dt)*0.001, 2*PI);
    Serial.println("Leg Vel: " + String(LegVel)+" Leg Angle: " + String(LegAngle));

    while (millis()-time_now < dt) {}
}