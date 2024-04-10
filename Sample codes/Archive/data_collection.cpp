#include "Arduino.h"
#include <IMUutilities.h>

#define IMU_RESET_BTN 27

const int countersize = 1000;
float Angles[countersize];
float Velocities[countersize];
int run = 0;

long debounceDelay = 200;
int a = 0;


float LegVel = 0;
float LegAngle = 0;
float initializetime1 = 50; // 30 ms to 
float initializetime2 = 5000;
float dt = 10;
float Leg_Vel_Initial = 0;
int count = 1;

float meanx0 = 0;
float varx0 = 0;

void firstrun() {
    run = 1;
    for (int i = 0; i < countersize; i++) {
        float time_now = millis();
        LegVel = readIMU();
        //LegVel = readIMU()-Leg_Vel_Initial;
        //LegAngle = fmod(LegAngle + (LegVel*dt)*0.001*180/PI, 360);
        Angles[i] = LegAngle;
        Velocities[i] = LegVel; 
        Serial.println(LegVel,4);
        while (millis()-time_now < dt) {}
    }
}

float calculateMean(float arr[], int size) {
    float sum = 0;
    for (int i = 0; i < size; ++i) {
        sum += arr[i];
    }
    return sum / size;
}

// Function to calculate the variance of an array
float calculateVariance(float arr[], int size) {
    float mean = calculateMean(arr, size);
    float variance = 0;
    for (int i = 0; i < size; ++i) {
        variance += (arr[i] - mean) * (arr[i] - mean);
    }
    return variance / size;
}

void IRAM_ATTR isr() {  // the function to be called when interrupt is triggered
  long myTime=millis();
  while(millis() - myTime < debounceDelay){
  }
  a = 1;
}

void setup() {
    Serial.begin(115200);
    pinMode(IMU_RESET_BTN, INPUT);
    attachInterrupt(IMU_RESET_BTN, isr, RISING);
    float time_now = millis();
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
}


void loop() {
    float time_now = millis();
    if (run < 1) {
        firstrun();
        varx0 = calculateVariance(Velocities, countersize);
        meanx0 = calculateMean(Velocities, countersize);
        Serial.print("Variance =  " + String(varx0,4) +"Mean = "+ String(meanx0,4));
    }
    else {
        if (a == 1) {
            LegAngle = 0;
            a = 0;
        }
    //Read IMU
        LegVel = (readIMU()-meanx0)*100;
        LegVel = int(round(LegVel));
        LegVel = float(LegVel)/100;
        LegAngle = fmod(LegAngle + (LegVel*dt)*0.001*180/PI, 360);

        Serial.println("Leg Vel: " + String(LegVel,4)+" Leg Angle: " + String(LegAngle,4));
    }
    while (millis()-time_now < dt) {}
}

