#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define BUFF_SIZE   20
//Note: only z-axis acceleration should be used with this sensor to simulate adafruit adxl375
Adafruit_MPU6050 mpu;
const int mpuAddress = 0x68;          // I2C address of the MPU-6050
bool LAUNCHED;
SoftwareSerial Serial1 (4,5);
char
    szStr[30];
uint8_t 
    txBuf[BUFF_SIZE],
    rxBuf[BUFF_SIZE];
    
uint8_t 
    idx,
    crc,
    exp_crc,
    ch;
const float THRESHOLD = 10;
int start_time = 0;
long long current_time = 0;
enum eRXStates
{
    RX_HEADER=0,
    RX_PACKET
};
static uint8_t
        state = RX_HEADER;
void setup( void )
{
    mpu.begin();
    Serial.begin(115200); 
    Serial1.begin(9600);
    //give the runcam time to send whatever it does out of reset
    delay(3000); 
    //flush any spurious chars from receive buffer
    while( Serial.available() > 0 )
        Serial.read();
    //get device info command
    txBuf[0] = 0xCC;
    txBuf[1] = 0x00; 
    txBuf[2] = calcCrc( txBuf, 2 );  //compute the CRC    
    Serial.write(txBuf, 3);
    
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
bool process_Accelerometer_data(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  if(a.acceleration.z > 9.81){
    return true;
  }
  return false;
}
void loop( void )
{
    
    //if a byte available?
    delay(500);
    process_RunCam_data();
    current_time = millis();
    bool launch_detected = process_Accelerometer_data();
    if(launch_detected && !LAUNCHED){
      txBuf[0]=0xCC;
      txBuf[1]=0x01;
      txBuf[2]=0x02;
      txBuf[3]= calcCrc(txBuf,3);
      Serial.write(txBuf,4);
      start_time = millis();
      LAUNCHED=true;
    }
    if(current_time - start_time > 6000 && LAUNCHED){
      txBuf[0]=0xCC;
      txBuf[1]=0x01;
      txBuf[2]=0x04;
      txBuf[3]= calcCrc(txBuf,3);
      Serial.write(txBuf,4);
    
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
