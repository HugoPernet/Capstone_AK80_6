// Basic demo for accelerometer readings from Adafruit MPU6050
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h> 

Adafruit_MPU6050 mpu;


void initializeIMU(){
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
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  delay(100);
}


float readIMU(){
  sensors_event_t a, g, temp;
  //float pitch;
  /* Get new sensor events with the readings */
  mpu.getEvent(&a, &g, &temp);
  delay(100);
  

  // /* Calculate the pitch and roll angles */
  // float ax = a.acceleration.x;
  // float ay = a.acceleration.y;
  // float az = a.acceleration.z;

  float pitch = g.gyro.x;
  float roll = g.gyro.y;
  float yaw = g.gyro.z;

  // pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
  //Imu_readings.roll = atan2(ay, az) * 180.0 / M_PI;

  return yaw;
}
