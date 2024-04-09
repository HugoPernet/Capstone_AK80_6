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
  // float ax = a.acceleration.x;
  // float ay = a.acceleration.y;
  // float az = a.acceleration.z;

  float pitch = g.gyro.x;
  // float roll = g.gyro.y;
  // float yaw = g.gyro.z;

  // pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
  //Imu_readings.roll = atan2(ay, az) * 180.0 / M_PI;
  return pitch;
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
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  
  //set filter Bandwidth
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  delay(100);

  // Zero-ing the sensor:
  // initialize constants
  float initializetime1 = 1000;
  float initializetime2 = 5000;
  float Leg_Vel_Initial = 0;
  int count = 1;
  float time_now = millis();

  //trash first second of values
  while (millis()-time_now < initializetime1) {
        Leg_Vel_Initial = readIMU();
  }
  //take avg of next 5 seconds of values
  time_now = millis();
  while (millis()-time_now < initializetime2) {
      count = count+1;
      Leg_Vel_Initial = Leg_Vel_Initial + readIMU();
  }
    Leg_Vel_Initial = Leg_Vel_Initial/count;
    Serial.println("Time Zeroed, count = "+String(count));
    delay(1000);
  return Leg_Vel_Initial;
}



// void setup() {
//     float time_now = millis();
//     Serial.begin(115200);
//     while (!Serial) delay(10);
//     initializeIMU();
//     while (millis()-time_now < initializetime1) {
//         Leg_Vel_Initial = readIMU();
//     }
//     while (millis()-time_now < initializetime2) {
//         count = count+1;
//         Leg_Vel_Initial = Leg_Vel_Initial + readIMU();
//     }
//     Leg_Vel_Initial = Leg_Vel_Initial/count;
//     Serial.println("Time Zeroed, count = "+String(count));
//     delay(1000);
// }



