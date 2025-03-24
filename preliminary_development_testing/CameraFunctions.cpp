#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
//For ADXL375 accelerometer
#include <Adafruit_ADXL375.h>

void getDeviceInfo(){
  //get device info command
    txBuf[0] = 0xCC;
    txBuf[1] = 0x00; 
    txBuf[2] = calcCrc( txBuf, 2 );  //compute the CRC    
    Serial.write(txBuf, 3);
}


//sends message to camera to increase camera quality
void increaseFootageQuality(){
      txBuf[0]=0xCC;
      txBuf[1]=0x01;
      txBuf[2]=0x02;
      txBuf[3]= calcCrc(txBuf,3);
      Serial.write(txBuf,4);
}


//Sends message  to camera to stop running 
void stopRecording(){
  txBuf[0]=0xCC;
      txBuf[1]=0x01;
      txBuf[2]=0x04;
      txBuf[3]= calcCrc(txBuf,3);
      Serial.write(txBuf,4);
}

/*
PARAMS: NONE
DESC: Polls the I2C interface between the arduino nano and the accelerometer. Checks
      the result against THRESHOLD to see if its been reached
RETURNS: boolean set to true if THRESHOLD exceeded and false if not
NOTE THIS ONE USES THE MPU6050!!! 
*/
bool process_Accelerometer_data(int threshold){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  if(a.acceleration.z > threshold){
    return true;
  }
  return false;
}


//Gets data from accelerometer and returns true if it experiences acceleration greater than the given threshold
bool process_Accelerometer_data_ADXL375(int threshold ){
  sensors_event_t event; 
  acccel.getEvent(&event);
  if(event.acceleration.z > threshold){
    return true;
  }
  return false;
}
