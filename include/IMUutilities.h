// Basic demo for accelerometer readings from Adafruit MPU6050
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h> 

Adafruit_MPU6050 mpu;

struct IMU_data{
  float pitch = 0.0f;
  float roll = 0.0f;
};

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
  Serial.print("Accelerometer range set to: ");
  
  //set gyroscope range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  
  //set filter Bandwidth
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");

  delay(100);
}


struct IMU_data readIMU(){
  struct IMU_data Imu_readings;
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Calculate the pitch and roll angles */
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  Imu_readings.pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
  Imu_readings.roll = atan2(ay, az) * 180.0 / M_PI;

  /* Print out the values */
  Serial.print("Pitch:");
  Serial.print(Imu_readings.pitch);
  Serial.print(" degrees, Roll:");
  Serial.print(Imu_readings.roll);
  Serial.println(" degrees");

  return Imu_readings;
}
