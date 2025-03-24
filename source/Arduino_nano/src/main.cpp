#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"




/*
--------------------GLOBAL VARIABLE DECLARATIONS-----------------------------
*/
MPU6050 mpu;
int16_t range = 0;
int16_t az;
//Buffers for transmitting UART signals
#define BUFF_SIZE   20
uint8_t 
    txBuf[BUFF_SIZE],
    rxBuf[BUFF_SIZE];

long int timer;


//Function Declarations
uint8_t crc8_calc( uint8_t crc, unsigned char a, uint8_t poly );
uint8_t calcCrc( uint8_t *buf, uint8_t numBytes );
void GetDeviceInfo();
bool Calibrate_Accelerometer_MPU6050(int threshold );
void start_timer();
void stop_timer();
void changeMode();

/*
PARAMS: uint8_t crc - the current value of the crc byte after processing
                      all bytes prior to current one
        unsigned char a - The current byte being processed
        uint8_t poly ?

DESC: Helper function for running the crc algorithm. Updates crc based on result from 
      running formula on "a"
RETURNS: The modified crc after processing "a"
*/
uint8_t crc8_calc( uint8_t crc, unsigned char a, uint8_t poly )
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80)
            crc = (crc << 1) ^ poly;
        else
            crc = crc << 1;
    }
    return crc;
    
}

/*
PARAMS: uint8_t* buf - The buffer to run the crc algorithm on
        uint8_t numBytes - The size of the buffer 
DESC: Runs the cyclical redundancy check algorithm on the buffer
      Uses the polynomial 0xd5

RETURNS: The crc byte to be appended to the buffer before transmitting
*/
uint8_t calcCrc( uint8_t *buf, uint8_t numBytes ){
    uint8_t crc = 0;
    for( uint8_t i=0; i<numBytes; i++ )
        crc = crc8_calc( crc, *(buf+i), 0xd5 );

    return crc;
}

/*
PARAMS: none
DESC: sends GetDeviceInfo command to RunCam. Instructs the RunCam to send packet containing details about capabilities, current status etc. 
RETURNS: void
*/
void GetDeviceInfo(){
    //Send command to get device info
    txBuf[0] = 0xCC;
    txBuf[1] = 0x00; 
    txBuf[2] = calcCrc(txBuf,2);
    Serial.write(txBuf,3);
}

/*
PARAMS: none
DESC: sends ToggleRecording command to RunCam. Instructs the RunCam to Toggle its status from actively capturing footage to not (or vice versa). 
RETURNS: void
*/
void ToggleRecording(){
    //Send command to turn camera on
     txBuf[0] = 0xCC;
     txBuf[1] = 0x01; 
     txBuf[2] = 0x01;
     txBuf[3] = calcCrc(txBuf,3);
     Serial.write(txBuf, 4);
}

/*
PARAMS: none
DESC: polls the MPU 6050 accelerometer for the acceleration readings in each direction
RETURNS: pointers, max (first index) and min (second index) acceleration reading (raw output from ADC)
*/
 void Calibrate_Accelerometer__MPU6050(int* max, int* min){
    long calibration_timer = millis();
    Serial.print(calibration_timer);
    az = mpu.getAccelerationZ();
    while(calibration_timer+10000>millis()){
        if(az > *max){
            *max = az;
            Serial.println(*max);
        }
        if(az < *min){
            *min = az;
            Serial.println(*min);
        }
        az = mpu.getAccelerationZ();
    }
    
 }

//timer determines when to stop recording so footage is not overwritten
void start_timer(){
    timer = millis();
}

void stop_timer(){
    Serial.print("Time: ");
    
    if(timer != 0){
        Serial.println(millis()-timer);
        timer = 0;
    }else{
        Serial.println("Timer not started");
    }
}
//Implement functionality to change recording quality
void changeMode(){
    txBuf[0] = 0xCC;
    txBuf[1] = 0x01; 
    txBuf[2] = 0x02;
    txBuf[3] = calcCrc(txBuf,3);
    Serial.write(txBuf, 4);
}

void setup(){
    //Initialize serial interface at 115200 baud
    Serial.begin(115200);
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
  if(mpu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else{
    Serial.println("MPU6050 connection successful");
  }

   /* Use the code below to change accel/gyro offset values. Use MPU6050_Zero to obtain the recommended offsets */ 
  Serial.println("Updating internal sensor offsets...\n");
  mpu.setXAccelOffset(0); //Set your accelerometer offset for axis X
  mpu.setYAccelOffset(0); //Set your accelerometer offset for axis Y
  mpu.setZAccelOffset(0); //Set your accelerometer offset for axis Z
  mpu.setXGyroOffset(0);  //Set your gyro offset for axis X
  mpu.setYGyroOffset(0);  //Set your gyro offset for axis Y
  mpu.setZGyroOffset(0);  //Set your gyro offset for axis Z
  Serial.println("Accel Range");
  

 
}
   

//TO DO:
//pipe out data to a file and analyze accelerometer data to determine a threshold
//Hysteresis (maybe)
void loop(){
    String input {};
    while(Serial.available()>0){
        input = Serial.readString();
        Serial.println(input);
        if(input == "ToggleRecording"){
            ToggleRecording();
        }
        if(input == "GetDeviceInfo"){
            GetDeviceInfo();
        }
        if(input == "ChangeMode"){
            changeMode();
        }
        if(input == "ReadAccelerometer"){
            int min = 10000; //larger than max ADC reading at +- 16g sensitivity
            int max = -10000; //smaller than min ADC reading at +- 16g sensitivity
            Calibrate_Accelerometer__MPU6050(&max,&min);
            Serial.print("Acceleration: ");
            Serial.println("max (raw ADC): ");
            Serial.println(max);
            Serial.println("min(raw ADC): ");
            Serial.println(min);
        }

        if(input == "StartTimer"){
            start_timer();
        }
        if(input == "StopTimer"){
            stop_timer();
        }
    }
    
   
}