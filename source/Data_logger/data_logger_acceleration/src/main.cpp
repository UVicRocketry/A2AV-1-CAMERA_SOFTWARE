#include "I2Cdev.h"
#include "MPU6050.h"
#include "accelerometer_filter_test.h"
#include <Arduino.h>

#define FILTER_WIDTH 100
#define SAMPLE_PERIOD_MS 1
MPU6050 mpu;
accelerometer_SMA<FILTER_WIDTH> filter{};
unsigned long timer{};
unsigned long samples{};
void sample_accelerometer() {
  float acceleration{};
  acceleration = mpu.getAccelerationZ();
  float filtered{filter(acceleration)};
  //print raw and filtered accelerations in separate columns
  if(samples > 1000){
    Serial.print(acceleration/2048.0 - 0.14); //scale to g's
    Serial.print(",");
    Serial.print(filtered - 0.14); //still a 0.14 offset after calibration
    Serial.print(",");
    Serial.println(samples);
  }
  samples++;
}

void setup() {
  Serial.begin(9600);
  delay(20000);
  /*--Start I2C interface--*/
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  /*Initialize device and check connection*/
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  if (mpu.testConnection() == false) {
    Serial.println("mpu connection bad");
  }

  /* Use the code below to change accel/gyro offset values. Use MPU6050_Zero
   * to obtain the recommended offsets */
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setXAccelOffset(mpu.getAccelXSelfTestFactoryTrim()); // Set your accelerometer offset for axis X
  mpu.setYAccelOffset(mpu.getAccelYSelfTestFactoryTrim()); // Set your accelerometer offset for axis Y
  mpu.setZAccelOffset(mpu.getAccelZSelfTestFactoryTrim()); // Set your accelerometer offset for axis Z
  mpu.setXGyroOffset(mpu.getGyroXSelfTestFactoryTrim());  // Set your gyro offset for axis X
  mpu.setYGyroOffset(mpu.getGyroYSelfTestFactoryTrim());  // Set your gyro offset for axis Y
  mpu.setZGyroOffset(mpu.getGyroZSelfTestFactoryTrim());  // Set your gyro offset for axis Z


  //print .csv file headers
  Serial.println("");
  Serial.println("Raw Accel,Filtered Accel,sample");
  timer = millis();
}


void loop() {
  // put your main code here, to run repeatedly:
  if(millis() - timer > SAMPLE_PERIOD_MS){
    timer += SAMPLE_PERIOD_MS;
    sample_accelerometer();
  }
  
}
