/*
-Camera_Simulator.cpp-
-Khephren Gould 12/13/2024-

This code simulates the behavior of the camera used to capture footage onboard
UVic Rocketry's Anduril 2 Rocket
*/
#include <Arduino.h>
#include <SoftwareSerial.h>

//Max size of UART buffers (Rx and Tx)
#define BUFF_SIZE   20

/*
--------------------GLOBAL VARIABLE DECLARATIONS-----------------------------
*/
//Class to interface with accelerometer
//Note: only z-axis acceleration should be used with this sensor to simulate adafruit adxl375

//boolean used for launch and landing detection
bool LAUNCHED = false;
bool LANDED = false;

//Used to output data over software serial
//Nano only has one hardware serial interface
//Do not want to corrupt it by writing un-needed data on it
SoftwareSerial Serial1 (4,5);

//Buffers for transmitting UART signals
uint8_t 
    txBuf[BUFF_SIZE],
    rxBuf[BUFF_SIZE];

/*
idx: the current index of the receive (Rx buffer)
crc: byte resulting from running cyclical redundancy check algorithm on buffer
exp_crc: the crc byte we expected to receive based on the other bytes in the packet
ch: stores a single byte read from the uart interface
*/
uint8_t 
    idx,
    crc,
    exp_crc,
    ch;

//the timer threshold used to determine when to stop recording (in MILLISECONDS)
//I repeat: IN MILLISECONDS
long long MAX_FOOTAGE_LENGTH_MS = 20000;

//The acceleration in m/s that triggers launch detection (in METERS/SECOND)
//I repeat: IN METERS/SECOND
const float THRESHOLD = 9.81;

//variable to track what millis() value the timer starts at
int start_time = 0;
//variable to track the current millis() value
long long current_time = 0;

//enumerator used for states when receiving packets
enum eRXStates
{
    RX_HEADER=0,
    RX_PACKET
};

//tracks the type of byte the last byte read was
static uint8_t
        state = RX_HEADER;


/*
--------------------FUNCTION DECLARATIONS---------------------------
*/
/*
PARAMS: uint8_t* buf - The buffer to run the crc algorithm on
        uint8_t numBytes - The size of the buffer 
DESC: Runs the cyclical redundancy check algorithm on the buffer
      Uses the polynomial 0xd5

RETURNS: The crc byte to be appended to the buffer before transmitting
*/
uint8_t calcCrc( uint8_t* buf, uint8_t numBytes );

/*
PARAMS: uint8_t crc - the current value of the crc byte after processing
                      all bytes prior to current one
        unsigned char a - The current byte being processed
        uint8_t poly ?

DESC: Helper function for running the crc algorithm. Updates crc based on result from 
      running formula on "a"
RETURNS: The modified crc after processing "a"
*/
uint8_t crc8_calc( uint8_t crc, unsigned char a, uint8_t poly );

/*
PARAMS: NONE
DESC: Polls uart interface between RunCam and Nano, appends all bytes received to the rxBuf
RETURNS: None
*/
void process_RunCam_data();



/*
--------------------Main Program----------------------------
*/
void setup( void )
{
    Serial.begin(115200); 
    //give the runcam time to send whatever it does out of reset
    delay(5000); 
    //flush any random chars from receive buffer
    while( Serial.available() > 0 )
        Serial.read();
   
    //send turn camera on command, begin recording comand
    // and ask device to send info. keep prompting device to turn on until info received.
    //Basically, keep trying to turn the camera on until successfull

      txBuf[0] = 0xCC;
      txBuf[1] = 0x01; 
      txBuf[2] = 0x01;
      txBuf[3] = calcCrc(txBuf,3);
      Serial.write(txBuf, 4);

      delay(500);
      //begin recording
      txBuf[0] = 0xCC;
      txBuf[1] = 0x01; 
      txBuf[2] = 0x03;
      txBuf[3] = calcCrc(txBuf,3);
      Serial.write(txBuf, 4);

      delay(500);
      //send device info
      txBuf[0] = 0xCC;
      txBuf[1] = 0x00; 
      txBuf[2] = calcCrc(txBuf,2);
      Serial.write(txBuf,3);
      delay(500);
    
}//setup

void process_RunCam_data(){
   if( Serial.available() > 0 )
    {
      //read a single byte
        ch = Serial.read();        
        switch( state )
        {
            case RX_HEADER:
                //we're looking for the header (0xcc) byte
                if( ch == 0xCC ){
                    //found it; place it in the rxBuffer and move to data state
                    idx = 0;
                    rxBuf[idx++] = ch;
                    state = RX_PACKET;
                }
            break;

            case RX_PACKET:
                //just collect bytes into the receive buffer
                rxBuf[idx++] = ch;
                //don't run off the end of the buffer
                if( idx == BUFF_SIZE )
                    idx--;
            break;    
        }//switch
        
    }//if
}//process_RunCam_data


void loop( void ){
    //poll uart interface
    process_RunCam_data();
    //update the current time
    current_time = millis();
    //poll I2C interface
    bool launch_detected = current_time > 10000;
    //if launch detected, increase footage quality
    if(launch_detected && !LAUNCHED){
      txBuf[0]=0xCC;
      txBuf[1]=0x01;
      txBuf[2]=0x02;
      txBuf[3]= calcCrc(txBuf,3);
      Serial.write(txBuf,4);
      start_time = millis();
      LAUNCHED=true;
    }
    //if we have detected launch and the timer has expired, stop recording.
    if((current_time - start_time > MAX_FOOTAGE_LENGTH_MS ) && LAUNCHED && !LANDED){
      txBuf[0]=0xCC;
      txBuf[1]=0x01;
      txBuf[2]=0x04;
      txBuf[3]= calcCrc(txBuf,3);
      Serial.write(txBuf,4);
      LANDED = true;
    }
}//loop



uint8_t calcCrc( uint8_t *buf, uint8_t numBytes ){
    uint8_t crc = 0;
    for( uint8_t i=0; i<numBytes; i++ )
        crc = crc8_calc( crc, *(buf+i), 0xd5 );

    return crc;
}//calcCrcTx

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