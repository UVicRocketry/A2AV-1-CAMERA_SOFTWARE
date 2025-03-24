#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#define BUFF_SIZE   20
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
//Buffers for transmitting UART signals
uint8_t 
    txBuf[BUFF_SIZE],
    rxBuf[BUFF_SIZE];

long int timer;

uint8_t crc8_calc( uint8_t crc, unsigned char a, uint8_t poly );
uint8_t calcCrc( uint8_t *buf, uint8_t numBytes );
void GetDeviceInfo();
bool process_Accelerometer_data_MPU6050(int threshold );
void start_timer();
void stop_timer();
void changeMode();
//TODO:

//Implement crc functionality
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
    }//for
    return crc;
    
}//crc8_calc

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
}//calcCrcTx
//Implement functionality to get device info
void GetDeviceInfo(){
    //Send command to get device info
    txBuf[0] = 0xCC;
    txBuf[1] = 0x00; 
    txBuf[2] = calcCrc(txBuf,2);
    Serial.write(txBuf,3);
}
//Implement functionality to turn camera on

void ToggleRecording(){
    //Send command to turn camera on
     txBuf[0] = 0xCC;
     txBuf[1] = 0x01; 
     txBuf[2] = 0x01;
     txBuf[3] = calcCrc(txBuf,3);
     Serial.write(txBuf, 4);
}


//Gets data from accelerometer and returns true if it experiences acceleration greater than the given threshold

float process_Accelerometer_data_MPU6050(){
    /* Get a new sensor event */
    mpu.getAcceleration(&ax, &ay, &az);
    while(az < 10000){
        Serial.print("acceleration: ");
        Serial.println(az);
        mpu.getAcceleration(&ax, &ay, &az);
    }
    return az;

}

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

 
}
   


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
            while(1){
                float acceleration = process_Accelerometer_data_MPU6050();
                if(true ){
                    Serial.print("Acceleration: ");
                    Serial.print(acceleration-17);
                    Serial.println(" m/s^2");
                    break;
                }
            }
        }

        if(input == "StartTimer"){
            start_timer();
        }
        if(input == "StopTimer"){
            stop_timer();
        }
    }
    
   
}