#include <Arduino.h>

// put function declarations here:
#define BUFF_SIZE   20

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

enum eRXStates
{
    RX_HEADER=0,
    RX_PACKET
};
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
uint8_t calcCrc( uint8_t *buf, uint8_t numBytes )
{
    uint8_t crc = 0;
    for( uint8_t i=0; i<numBytes; i++ )
        crc = crc8_calc( crc, *(buf+i), 0xd5 );
        
    return crc;
    
}//calcCrcTx


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(10000);
}

void loop() {
    static uint8_t
        state = RX_HEADER;

    //if a byte available?
    if( Serial.available() > 0 )
    {
        ch = Serial.read();        
        switch( state )
        {
            case    RX_HEADER:
                //we're looking for the header (0xcc) byte
                if( ch == 0xcc )
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
                
                if( idx == 5 )
                {
                    //serial monitor would have showed any bytes received; move to a new line
                    Serial.println();

                    //we know for the dev info command we get the header, a 3-byte info packet and a CRC giving 5 bytes
                    
                    //compute the CRC and extract the one from the message                    
                    crc = calcCrc( rxBuf, 4 );
                    exp_crc = rxBuf[4];
                    sprintf( szStr, "CRC 0x%02X (0x%02X)\n", crc, exp_crc );
                    Serial.print( szStr );
                    
                    Serial.print( F("Message received: ") );
                    //print the human-readable HEX output
                    for( ch=0; ch<5; ch++ )
                    {
                        sprintf( szStr, "%02X ", rxBuf[ch] );                        
                        Serial.print( szStr );
                        
                    }//for
                    
                    Serial.println();
                    
                    //after this, halt and catch fire
                    while(1);
                                
                }//if                
                
            break;
            
        }//switch
        
    }//if
    
}

// put function definitions here:
