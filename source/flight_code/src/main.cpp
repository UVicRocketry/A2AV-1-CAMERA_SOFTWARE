// https://github.com/ElectronicCats/mpu6050/wiki/1.-How-does-MPU6050-work%3F

#include "I2Cdev.h"
#include "MPU6050.h"
#include "UART_messages.h"
#include "accelerometer_filter.h"
#include <Arduino.h>
#include <Stream.h>
#include <buzzer.h>

//the width of the simple moving average filter
#define FILTER_WIDTH 100

#define MAXCONNECTIONATTEMPTS 3

//this is how long the camera will record for in milliseconds
//#define ONEHOURINMILLISECOND 3600000
#define ONEHOURINMILLISECOND 600000 //uncomment this #define directive for testing
//size of the UART transmission bufferk
#define BUFF_SIZE 20

#define SAMPLING_PERIOD_MICROS 1000 //(1 millisecond, sampling frequency = 1000Hz)
//note: since the filter width is 100 samples (the current sample depends on the 100 previous samples) there is 99 ms of latency or (m-1) samples

/*The offset required in the raw ADC output of the MPU 6050 with a 16g full scale reading
such that when at rest the vertical acceleration measured (z) is 1g*/
#define CALIBRATION_FACTOR 250

// The threshold at which launch is considered to have been detected
//According to the RasAero simulations, we will reach acceleration of 300 ft / s^2 in the first 0.04 seconds of launch
//corresponds to:
/*
              1 m          1g
300 ft/s^2 * --------  *  --------
             3.28 ft      9.8 m/s^2
*/
//float ACCELERATION_Z_THRESHOLD_Gs = 9.33;
float ACCELERATION_Y_THRESHOLD_Gs = 1.2; //uncomment this for testing

uint8_t connection_attempts{};

typedef enum {
    ON_PAD,
    LAUNCHED,
    TIMER_EXPIRED
} camera_state_t;

static MPU6050 mpu{};

static camera_state_t cameraState{ON_PAD};

void init_UART() {
    Serial.begin(9600);
    while (!Serial);
}

void init_I2C() {
    Wire.begin();
    Wire.setClock(400000);
}

void init_mpu() {
    Serial.println(F("Testing MPU6050 connection..."));
    while (mpu.testConnection() == false) {
        if (++connection_attempts >= MAXCONNECTIONATTEMPTS) {
            Serial.println("MPU6050 connection failed");
            // skip launch detection if unable to connect to mpu
            cameraState = LAUNCHED;
            ACCELERATION_Y_THRESHOLD_Gs = -1;
        }
    }

    Serial.println("MPU6050 connection successful");

    mpu.initialize();
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
}

void setup() {
    init_UART();
    init_I2C();
    init_mpu();
    Serial.end();
    Serial.begin(115200);

    //Buzzer Logic
    buzzer_init();
    play_buzzer();
}

void loop() {
    static accelerometer_SMA<FILTER_WIDTH> filter;
    static Camera camera{Serial};
    static unsigned long prevMicros = micros();
    switch (cameraState) {
        case ON_PAD:
            if (micros() - prevMicros >= SAMPLING_PERIOD_MICROS) {
                float rawValue = -mpu.getAccelerationY() + 2048.0 - 250.0;
                float filteredValue = filter(rawValue);
                prevMicros += SAMPLING_PERIOD_MICROS;
                if (filteredValue > ACCELERATION_Y_THRESHOLD_Gs) {
                    camera.start_timer();
                    cameraState = LAUNCHED;
                }
            }
            break;

        case LAUNCHED:
            if (millis() - camera.get_timer() > ONEHOURINMILLISECOND) {
                camera.stop_timer();
                camera.ToggleRecording();
                cameraState = TIMER_EXPIRED;
            }

            break;
        case TIMER_EXPIRED:
            break;

        default:
            break;
    }
}
