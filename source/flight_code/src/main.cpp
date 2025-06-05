// https://github.com/ElectronicCats/mpu6050/wiki/1.-How-does-MPU6050-work%3F

#include "I2Cdev.h"
#include "MPU6050.h"
#include "UART_messages.h"
#include "accelerometer_filter.h"
#include <Arduino.h>
#include <Stream.h>

#define MAXCONNECTIONATTEMPTS 3
#define SIXHOURSINMILLISECOND 21600000
#define BUFF_SIZE 20
#define INTERRUPT_PIN 2
#define SAMPLING_PERIOD_MICROS 100000

int8_t ACCELERATION_Z_THRESHOLD_Gs = 2;
uint8_t connection_attempts{};
typedef enum {
  ON_PAD,
  LAUNCHED,
  TIMER_EXPIRED

} camera_state_t;

static MPU6050 mpu{};

static camera_state_t cameraState{};

void init_UART() {
  Serial.begin(115200);
  while (!Serial)
    ;
}

void init_I2C() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having
                         // compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

void init_mpu() {

  Serial.println(F("Testing MPU6050 connection..."));
  while (mpu.testConnection() == false) {
    if (++connection_attempts >= MAXCONNECTIONATTEMPTS) {
      Serial.println("MPU6050 connection failed");
      // skip launch detection if unable to connect to mpu
      cameraState = LAUNCHED;
      ACCELERATION_Z_THRESHOLD_Gs = -1;
    }
  }

  Serial.println("MPU6050 connection successful");

  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
}

void setup() {
  init_UART();
  init_I2C();
  init_mpu();
}

void loop() {
  static accelerometer_SMA<20> filter;
  static Camera camera{Serial};
  static unsigned long prevMicros = micros();
  switch (cameraState) {
  case ON_PAD: 
    if (micros() - prevMicros >= SAMPLING_PERIOD_MICROS) {
      int rawValue = mpu.getAccelerationZ();
      int filteredValue = filter(rawValue);
      Serial.print(rawValue);
      Serial.print('\t');
      Serial.println(filteredValue);
      prevMicros += SAMPLING_PERIOD_MICROS;
      if (filteredValue > ACCELERATION_Z_THRESHOLD_Gs) {
        camera.start_timer();
        cameraState = LAUNCHED;
      }
    }
    break;
  
  case LAUNCHED:
    if (camera.get_timer() > SIXHOURSINMILLISECOND) {
      camera.stop_timer();
      camera.ToggleRecording();
      cameraState = TIMER_EXPIRED;
    }

    break;
  case TIMER_EXPIRED:
    while (true)
      ;

    break;

  default:
    break;
  }
}

