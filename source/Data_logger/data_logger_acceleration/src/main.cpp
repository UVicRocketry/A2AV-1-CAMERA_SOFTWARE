#include "I2Cdev.h"
#include "MPU6050.h"
#include "accelerometer_filter_test.h"
#include <Arduino.h>

#define FILTER_WIDTH 20

MPU6050 mpu;
int16_t range = 0;
accelerometer_SMA<FILTER_WIDTH> filter{};

void test_accelerometer() {
  int acceleration{};
  acceleration = mpu.getAccelerationZ();
  int filtered{filter(acceleration)};
  Serial.print("Raw Accel: ");
  Serial.println(acceleration);
  Serial.print("Filtered Accel: ");
  Serial.println(filtered);
}

void setup() {
  Serial.begin(9600);
  /*--Start I2C interface--*/
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  /*Initialize device and check connection*/
  Serial.println("Initializing MPU...");
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  Serial.println("Testing MPU6050 connection...");
  if (mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  } else {
    Serial.println("MPU6050 connection successful");
  }

  /* Use the code below to change accel/gyro offset values. Use MPU6050_Zero
   * to obtain the recommended offsets */
  Serial.println("Updating internal sensor offsets...\n");
  mpu.setXAccelOffset(0); // Set your accelerometer offset for axis X
  mpu.setYAccelOffset(0); // Set your accelerometer offset for axis Y
  mpu.setZAccelOffset(0); // Set your accelerometer offset for axis Z
  mpu.setXGyroOffset(0);  // Set your gyro offset for axis X
  mpu.setYGyroOffset(0);  // Set your gyro offset for axis Y
  mpu.setZGyroOffset(0);  // Set your gyro offset for axis Z
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
}

void loop() {
  // put your main code here, to run repeatedly:
  test_accelerometer();
}
