/*
-runcam.chip.c-
-Khephren Gould 12/10/2024-
This code is a configuration file defining the behavior of a custom chip
The goal of this custom chip is to simulate the behavior of the RunCam Split 4 V2

*/
#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//maximum size of both tx and rx buffers
#define BUFF_SIZE 20

/*
------------GLOBAL VARIABLE DECLARATIONS------------------
*/
//string used for formatting bytes for printing
char szStr[30];
//enumerate a few states to use in the main switch case block
enum RXstates{
  RXHEADER = 0,
  RXPACKET,
  RXFULL
};

//string to store the current quality of footage being recorded
char* Quality = "1080P, 30fps";
//a bool to compare crc8 values
bool NO_ERRORS=false;
//bool to detect if camera is powered on
bool ON = false;
/*
bytes: index for interating over buffers
crc: byte resulting from running cyclical redundancy check algorithm on buffer
exp_crc: the crc byte we expected to receive based on the other bytes in the packet
*/
uint8_t bytes, crc, exp_crc;
//tracks how many bytes written over uart as part of current packet
int count = 1;

//Buffers for reading and writing data
uint8_t
  txBuf[BUFF_SIZE],
  rxBuf[BUFF_SIZE];

//struct required by wokwi uart api
typedef struct {
  uart_dev_t uart0;
} chip_state_t;
/*
----------------------FUNCTION DECLARATIONS---------------------
*/
/*
PARAMS: user_data - a void pointer, byte - the data recieved
DESC: callback required by wokwi uart api, runs upon receiving UART data
RETURNS: void
*/
static void on_uart_rx_data(void* user_data, uint8_t byte);

/*
PARAMS: user_data - a void pointer
DESC: callback required by wokwi uart api, runs after each byte sent
over uart
RETURNS: void
*/
static void on_uart_write_done(void* user_data);

/*
PARAMS: uint8_t *buf - array containing contents of tx/rx buffers
DESC: main function for computing crc8 error check byte
RETRUNS: uint8_t cyclic redundancy check byte
*/
uint8_t calcCrc( uint8_t *buf, uint8_t numBytes );


/*
PARAMS: uint8_t crc - the current crc, unsigned char a, current byte
uint8_t poly - the polynomical being used for the calculation (0xd5)
DESC: function to run crc8 algorithm on a single byte
RETURNS: the updated value for the crc
*/
uint8_t crc8_calc( uint8_t crc, unsigned char a, uint8_t poly );

/*
PARAMS: uint8 data - the single byte that should be written over UART
DESC:writes a single byte of data over UART
RETURNS: void
*/
static void write_data(uint8_t data,chip_state_t* chip);

/*
PARAMS: uint8_t input (a single byte received over UART)
DESC: basically the main loop. performs all the logic functions for the runcam
RETURNS: void
*/
void process_data(uint8_t input, chip_state_t* chip);

/*
----------------------MAIN PROGRAM--------------------
*/
void chip_init(void){
  chip_state_t* chip = malloc(sizeof(chip_state_t));
  const uart_config_t uart_config = {
  .tx = pin_init("TX", INPUT_PULLUP),
  .rx = pin_init("RX", INPUT),
  .baud_rate = 115200,
  .rx_data = on_uart_rx_data,
  .write_done = on_uart_write_done,
  .user_data = chip,
  };
  chip->uart0 = uart_init(&uart_config);
  printf("RunCam initialized!\n");
}

void process_data(uint8_t input, chip_state_t* chip){
  static uint8_t state = RXHEADER;
  switch(state){
    case RXHEADER:
      if(input == 0xCC){
        bytes=0;
        rxBuf[bytes++] = input;
        printf("got: %02X\n", input);
        state = RXPACKET;
      }//if
      break;
    case RXPACKET:
      rxBuf[bytes++] = input;
      printf("got: %02X\n", input);
      if(bytes == BUFF_SIZE){
        bytes--;
      }
      if((bytes==3 && rxBuf[1]==0x00)||bytes==4){
        state = RXFULL;
        crc = calcCrc(rxBuf,bytes - 1);
        exp_crc = rxBuf[bytes - 1];
        if(crc==exp_crc){
          NO_ERRORS=true;
        }else{
          NO_ERRORS = false;
          printf("crc8 error :(");
        }
        sprintf(szStr, "CRC 0x%02X (0x%02X)\n", crc, exp_crc);
        printf("%s",szStr);
        printf("Message recieved:");
        for(char ch = 0; ch<bytes ; ch++){
          sprintf(szStr,"%02X",rxBuf[ch]);
          printf("%s", szStr);
        }
        printf("\n");
      }
      if(state==RXFULL){
        if(bytes==3){
          printf("Received Command: Send Device Info\n");
          printf("Sending info:\n");
          if(rxBuf[1]==0x00 && NO_ERRORS){
            txBuf[0]=0xCC;
            printf("sent %02X\n", txBuf[0]);
            txBuf[1]=0b11010111;
            printf("Sent %02X\n", txBuf[1]);
            txBuf[2]=0x0A;
            printf("sent %02X\n", txBuf[2]);
            txBuf[3]= calcCrc(rxBuf, 2);
            printf("sent %02X\n", txBuf[3]);

            write_data(txBuf[0], chip);
          }
        }else if(rxBuf[1]==0x01 && NO_ERRORS){
          printf("received control command\n");
          if(rxBuf[2]==0x00 && ON){
            printf("Detected Wifi button press\n");
            printf("Activating Wifi mode\n");
          }
          if(rxBuf[2]==0x01){
            printf("Detected Power button press\n");
            printf("Camera powering on\n");
            ON=true;
          }
          if(rxBuf[2]==0x02 && ON){
            printf("Detected toggle mode command\n");
            if(strcmp(Quality, "1080P, 30fps")==0){
              Quality = "2.7k, 50fps";
            }else{
              Quality = "1080P, 30fps";
            }
            printf("Camera mode switched to: %s\n", Quality);
          }
          if(rxBuf[2]==0x03 && ON){
            printf("Detected Start Recording command\n");
            printf("begin recording footage at %s\n", Quality);
          }
          if(rxBuf[2]==0x04 && ON){
            printf("Detected Stop Recording command\n");
            printf("Recording stopped\n");
          }
        }else{
          if(!NO_ERRORS){
            printf("CRC Error :(");
          }else{
            printf("unknown error");
          }
        }
        state = RXHEADER;
      }
    break;
  }//switch
}//process_data

static void write_data(uint8_t data, chip_state_t* chip){
  uart_write(chip->uart0, &data, sizeof(data));
}

static void on_uart_rx_data(void* user_data, uint8_t byte){
  chip_state_t* chip = (chip_state_t*)user_data;
  process_data(byte, chip);
}

static void on_uart_write_done(void* user_data){
  chip_state_t *chip = (chip_state_t*) user_data;
  if(count<4){
    write_data(txBuf[count], chip);
    count++;
  }
}

uint8_t calcCrc( uint8_t *buf, uint8_t numBytes ){
    uint8_t crc = 0;
    for( uint8_t i=0; i<numBytes; i++ )
        crc = crc8_calc( crc, *(buf+i), 0xd5 );
        
    return crc; 
}//calcCrcTx

uint8_t crc8_calc( uint8_t crc, unsigned char a, uint8_t poly ){
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80)
            crc = (crc << 1) ^ poly;
        else
            crc = crc << 1; 
    }//for
    return crc;
}//crc8_calc