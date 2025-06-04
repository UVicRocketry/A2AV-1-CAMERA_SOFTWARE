#include "I2Cdev.h"
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "UART_messages.h"
#include "accelerometer_filter.h"
#include <Arduino.h>

#define MAXCONNECTIONATTEMPTS (3)
#define SIXHOURSINMILLISECOND (21600000)
#define BUFF_SIZE (20)
#define INTERRUPT_PIN (2)

typedef enum {
  ON_PAD,
  LAUNCHED,
  TIMER_EXPIRED

} camera_state_t;

// Indicates whether MPU6050 interrupt pin has gone high
volatile bool MPUInterrupt = false;
bool DMPReady = false;

static unsigned long timer{};

static MPU6050 mpu{};

static uint8_t txBuf[BUFF_SIZE], rxBuf[BUFF_SIZE];

uint8_t FIFOBuffer[64];

uint8_t MPUIntStatus;

Quaternion q; // [w, x, y, z]         Quaternion container

VectorInt16 aa; // [x, y, z]            Accel sensor measurements

VectorInt16 aaReal;

VectorFloat gravity;

uint8_t devStatus; // Return status after each device operation (0 = success, !0
                   // = error)

static camera_state_t cameraState{};

void DMPDataReady() { MPUInterrupt = true; }

void start_timer() { timer = millis(); }

void stop_timer() { timer = 0; }

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
  if (mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  } else {
    Serial.println("MPU6050 connection successful");
  }

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
}

void init_dmp() {
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP...")); // Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(
        F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to
     * use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;

  } else {
    Serial.print(F("DMP Initialization failed (code ")); // Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
}

void setup() {
  init_UART();
  init_I2C();
  init_mpu();
  init_dmp();
}

void loop() {
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpConvertToWorldFrame(&aaReal, &aa, &q);
    // at this line aaReal.z should contain an accurate z acceleration in g
  }

  // Todo: implement logic for state transistions
