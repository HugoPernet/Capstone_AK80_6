// Basic demo for accelerometer readings from Adafruit MPU6050
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h> 

Adafruit_MPU6050 mpu;

float readIMU(){
  sensors_event_t a, g, temp;
  //float pitch;
  /* Get new sensor events with the readings */
  mpu.getEvent(&a, &g, &temp);

  // /* Calculate the pitch and roll angles */
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  //float pitch = atan2(-ay, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
  float pitch = atan2(ay, az) * 180.0 / M_PI;

  return pitch;
}

float readgyro() {
  sensors_event_t a, g, temp;
  //float pitch;
  /* Get new sensor events with the readings */
  mpu.getEvent(&a, &g, &temp);

  float pitch_rate = g.gyro.x * 180/PI; //deg/s
  // float roll = g.gyro.y;
  // float yaw = g.gyro.z;
  return pitch_rate;
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

const int countersize = 1000;
float Angles[countersize];
float Velocities[countersize];

void firstrun() {
  float LegAngle = 0;
  float LegVel = 0;
  float dt = 5;
  Serial.println("Calibrating IMU...");
    for (int i = 0; i < countersize; i++) {
        long time_now = millis();
        LegAngle = readIMU();
        LegVel = readgyro();
        // LegVel = readIMU()-Leg_Vel_Initial;
        // LegAngle = fmod(LegAngle + (LegVel*dt)*0.001*180/PI, 360);
        Angles[i] = LegAngle;
        Velocities[i] = LegVel;
        //Serial.println("Leg Angle = ");
        //Serial.println(LegVel,4);
        while (millis()-time_now < dt) {}
    }
}

float initializeIMU(){
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //set accelerometer range
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  
  //set gyroscope range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  
  //set filter Bandwidth
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

  delay(100);

  // Zero-ing the sensor:
  // initialize constants
  float initializetime1 = 100; //>30ms
  
  //trash first few values
  float time_now = millis();
  while (millis()-time_now < initializetime1) {
    float Leg_Vel_Initial = readIMU();
  }
  //measure bias
  firstrun();
  float bias_pitch = calculateMean(Angles, countersize);
  float bias_pitch_rate = calculateMean(Velocities, countersize);

  Serial.println("Pitch Zeroed");
  delay(200);
  return bias_pitch, bias_pitch_rate;
}

// // Basic demo for accelerometer readings from Adafruit MPU6050
// #include <Arduino.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>
// #include <math.h> 

// Adafruit_MPU6050 mpu;

// float readIMU(){
//   sensors_event_t a, g, temp;
//   //float pitch;
//   /* Get new sensor events with the readings */
//   mpu.getEvent(&a, &g, &temp);

//   // /* Calculate the pitch and roll angles */
//   float ax = a.acceleration.x;
//   float ay = a.acceleration.y;
//   float az = a.acceleration.z;

//   float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

//   // float pitch = g.gyro.x;
//   // float roll = g.gyro.y;
//   // float yaw = g.gyro.z;

//   return pitch;
// }

// float calculateMean(float arr[], int size) {
//     float sum = 0;
//     for (int i = 0; i < size; ++i) {
//         sum += arr[i];
//     }
//     return sum / size;
// }

// // Function to calculate the variance of an array
// float calculateVariance(float arr[], int size) {
//     float mean = calculateMean(arr, size);
//     float variance = 0;
//     for (int i = 0; i < size; ++i) {
//         variance += (arr[i] - mean) * (arr[i] - mean);
//     }
//     return variance / size;
// }

// const int countersize = 1000;
// float Angles[countersize];
// float Velocities[countersize];

// void firstrun() {
//   float LegVel= 0;
//   float dt = 5;
//   Serial.println("Calibrating IMU...");
//     for (int i = 0; i < countersize; i++) {
//         long time_now = millis();
//         // LegAngle = fmod(readIMU(),360);
//         LegVel = readIMU();
//         // LegAngle = fmod(LegAngle + (LegVel*dt)*0.001*180/PI, 360);
//         // Angles[i] = LegAngle;
//         Velocities[i] = LegVel;
//         Serial.println("Leg Angle = ");
//         Serial.println(LegVel,4);
//         while (millis()-time_now < dt) {}
//     }
// }

// float initializeIMU(){
//   // Try to initialize!
//   if (!mpu.begin()) {
//     Serial.println("Failed to find MPU6050 chip");
//     while (1) {
//       delay(10);
//     }
//   }
//   Serial.println("MPU6050 Found!");

//   //set accelerometer range
//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  
//   //set gyroscope range
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  
//   //set filter Bandwidth
//   mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

//   delay(100);

//   // Zero-ing the sensor:
//   // initialize constants
//   float initializetime1 = 100; //>30ms
  
//   //trash first few values
//   float time_now = millis();
//   while (millis()-time_now < initializetime1) {
//     float Leg_Vel_Initial = readIMU();
//   }
//   //measure bias
//   firstrun();
//   float bias_pitch_rate = calculateMean(Velocities, countersize);
//   Serial.println("Pitch Zeroed");
//   delay(200);
//   return bias_pitch_rate;
// }

