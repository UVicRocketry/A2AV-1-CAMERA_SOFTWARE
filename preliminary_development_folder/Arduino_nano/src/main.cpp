#include <Arduino.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define BUFF_SIZE   20
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
//Buffers for transmitting UART signals
uint8_t 
    txBuf[BUFF_SIZE],
    rxBuf[BUFF_SIZE];

long int timer;

uint8_t crc8_calc( uint8_t crc, unsigned char a, uint8_t poly );
uint8_t calcCrc( uint8_t *buf, uint8_t numBytes );
void GetDeviceInfo();
bool process_Accelerometer_data_ADXL375(int threshold );
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

float process_Accelerometer_data_ADXL375(){
    /* Get a new sensor event */
    sensors_event_t event;
    accel.getEvent(&event);
    return event.acceleration.z;

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
    if(!accel.begin())
  {
    /* There was a problem detecting the ADXL375 ... check your connections */
    Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
    while(1);
  }

  // Range is fixed at +-200g

 
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
                float acceleration = process_Accelerometer_data_ADXL375();
                if(acceleration - 17 > 7 ){
                    Serial.println("Acceleration greater than 7 m/s^2");
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