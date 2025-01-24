#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
//For ADXL375 accelerometer
#include <Adafruit_ADXL375.h>

#include "CameraFunctions.h"

#define BUFF_SIZE   20
//Note: only z-axis acceleration should be used with this sensor to simulate adafruit adxl375
Adafruit_MPU6050 mpu;
const int mpuAddress = 0x68;          // I2C address of the MPU-6050
bool LAUNCHED;
//Used to output data over software serial
//Nano only has one hardware serial interface
//Do not want to corrupt it by writing un-needed data on it
SoftwareSerial Serial1 (4,5);
char
    szStr[30];

//Buffers for transmitting UART signals.
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
long long MAX_FOOTAGE_LENGTH_MS = 6000;

//Will be false if the accelerometer fails to begin. 
bool haveAccelerometer; 


//The acceleration in m/s that triggers launch detection (in METERS/SECOND)
//I repeat: IN METERS/SECOND
const float THRESHOLD = 10;

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
-----------------ACCELEROMETER THINGS---------------
*/
//Primarily using information from Greyson's algorithm
//TODO determine if need another call to Signal.begin(baud rate)

//Optional tag that is added used by accelerometer to tag events
int32_t sensorID = 12345; 

//Accelerometer type from Adafruit_ADXL375.h 
//Documentation found in: https://github.com/adafruit/Adafruit_ADXL375/blob/master/Adafruit_ADXL375.cpp
Adafruit_ADXL375 accel = Adafruit_ADXL375(sensorID);
 

void calibrateAccelerometer(); //From Greyson
//Note to self, NOT presently using G's, instead everything will remain in m/s^2


/*
----------------- END OF ACCELEROMETER THINGS---------------
*/

/*
----------------- functions taken from main--------------------
*/

// void getDeviceInfo(){
//   //get device info command
//     txBuf[0] = 0xCC;
//     txBuf[1] = 0x00; 
//     txBuf[2] = calcCrc( txBuf, 2 );  //compute the CRC    
//     Serial.write(txBuf, 3);
// }

// void increaseFootageQuality(){
//       txBuf[0]=0xCC;
//       txBuf[1]=0x01;
//       txBuf[2]=0x02;
//       txBuf[3]= calcCrc(txBuf,3);
//       Serial.write(txBuf,4);
// }

// void stopRecording(){
//   txBuf[0]=0xCC;
//       txBuf[1]=0x01;
//       txBuf[2]=0x04;
//       txBuf[3]= calcCrc(txBuf,3);
//       Serial.write(txBuf,4);
// }

void setup( void )
{
    mpu.begin();
    Serial.begin(115200); 
    Serial1.begin(9600);
/* Initialise the sensor */
  if(!accel.begin()) {
    /* There was a problem detecting the ADXL375 ... check your connections */
    Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
    haveAccelerometer = false; 

    // while(1);
    /*
    TODO add a flow to the algorithm that will start recording only in high definition, and 
    will record until the end of space/battery/limiting factor with no consideration for flight/landing/other more complex algos
    */
    
  }else{
    haveAccelerometer = true; 
  }
  
  if(haveAccelerometer){ 
    calibrateAccelerometer();
  }



    //give the runcam time to send whatever it does out of reset
    delay(3000); 
    //flush any spurious chars from receive buffer
    while( Serial.available() > 0 )
        Serial.read();
    //get device info command
    getDeviceInfo(); 
    // txBuf[0] = 0xCC;
    // txBuf[1] = 0x00; 
    // txBuf[2] = calcCrc( txBuf, 2 );  //compute the CRC    
    // Serial.write(txBuf, 3);
    
}//setup

/*
A function to process incoming data from the RunCam
Data is recieved from the hardware serial interface.
Debugging messages are sent to the user through a software serial interface
This interface uses pin 3 as Tx and has no Rx

*/
void process_RunCam_data(){
   if( Serial.available() > 0 )
    {
        ch = Serial.read();        
        switch( state )
        {
            case    RX_HEADER:
                //we're looking for the header (0xcc) byte
                if( ch == 0xCC )
                {
                    //found it; place it in the rxBuffer and move to data state
                    idx = 0;
                    rxBuf[idx++] = ch;
                    state = RX_PACKET;
                }//if
                
            break;

            case    RX_PACKET:
                //just collect bytes into the receive buffer
                rxBuf[idx++] = ch;
                //don't run off the end of the buffer
                if( idx == BUFF_SIZE )
                    idx--;
            break;
            
        }//switch
        
    }//if
}

// /*
// PARAMS: NONE
// DESC: Polls the I2C interface between the arduino nano and the accelerometer. Checks
//       the result against THRESHOLD to see if its been reached
// RETURNS: boolean set to true if THRESHOLD exceeded and false if not
// NOTE THIS ONE USES THE MPU6050!!! 
// */
// bool process_Accelerometer_data(){
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);
//   if(a.acceleration.z > THRESHOLD){
//     return true;
//   }
//   return false;
// }

// bool process_Accelerometer_data_ADXL375(){
//   sensors_event_t event; 
//   acccel.getEvent(&event);
//   if(event.acceleration.z > THRESHOLD){
//     return true;
//   }
//   return false;
// }


void loop( void )
{
    
    //if a byte available?
    delay(500);
    process_RunCam_data();
    current_time = millis();
    bool launch_detected = process_Accelerometer_data(THRESHOLD);
    //if launch detected, increase footage quality

    if(launch_detected && !LAUNCHED){
      // txBuf[0]=0xCC;
      // txBuf[1]=0x01;
      // txBuf[2]=0x02;
      // txBuf[3]= calcCrc(txBuf,3);
      // Serial.write(txBuf,4);
      increaseFootageQuality(); 

      start_time = millis();
      LAUNCHED=true;
    }
    //if we have detected launch and the timer has expired, stop recording.
    if(current_time - start_time > MAX_FOOTAGE_LENGTH_MS && LAUNCHED){
      // txBuf[0]=0xCC;
      // txBuf[1]=0x01;
      // txBuf[2]=0x04;
      // txBuf[3]= calcCrc(txBuf,3);
      // Serial.write(txBuf,4);
    stopRecording(); 
    }
    
}//loop

uint8_t calcCrc( uint8_t *buf, uint8_t numBytes )
{
    uint8_t crc = 0;
    for( uint8_t i=0; i<numBytes; i++ )
        crc = crc8_calc( crc, *(buf+i), 0xd5 );
        
    return crc;
    
}//calcCrcTx

uint8_t crc8_calc( uint8_t crc, unsigned char a, uint8_t poly )
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) 
    {
        if (crc & 0x80)
            crc = (crc << 1) ^ poly;
        else
            crc = crc << 1;
            
    }//for
    
    return crc;
    
}//crc8_calc

//From Greysons's algorithm, for calibrating the accelerometer
void calibrateAccelerometer() {
  delay(2000); // Delay for stable readings

  sensors_event_t event;
  accel.getEvent(&event);

  x_zero = event.acceleration.x;
  y_zero = event.acceleration.y;
  z_zero = event.acceleration.z;

  //This should possibly be commented out for actual code running. (I THINK)

  Serial.println("Zero calibration complete");
  Serial.print("x_zero: "); Serial.println(x_zero);
  Serial.print("y_zero: "); Serial.println(y_zero);
  Serial.print("z_zero: "); Serial.println(z_zero); Serial.println("");
}
