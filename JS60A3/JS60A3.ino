#include "Arduino.h"

#define BUF_SIZE  40

bool bSend[2];
int rcvLen[2], rcvIdx[2], tmCnt[2];
unsigned char tmr, RsBuf[2][BUF_SIZE];
uint8_t TxBuf[8] = {0xFC, 0xA5, 0, 0, 0, 0, 0xA1, 0x01};
//uint8_t TxBuf[8] = {'A', 'B', 'C', 'D', 'A', 'D', 'B', 'C'};

void setup()
{
  int i;

  for(i = 0; i < 2; i++)
  {
    bSend[i] = true;
    rcvLen[i] = rcvIdx[i] = tmCnt[i] = 0;
  }

  Serial.begin(460800);  
  delay(1000);
}

void AddRxData(int port, unsigned char data)
{
  unsigned short i, rxCRC, calCRC = 0;
  RsBuf[port][rcvLen[port]++] = data & 0xFF;

  if( rcvLen[port] >= 10 + rcvIdx[port] )
  {
    for(i = 0; i < rcvLen[port]-1; i++)
    {
      if( RsBuf[port][i] == 0x35 && RsBuf[port][i+1] == 0x2B )
      {
        rcvIdx[port] = i;
        break;
      }
    }

    if( rcvLen[port] >= 10 + rcvIdx[port] )
    {
      rxCRC = RsBuf[port][rcvIdx[port]+9];
      rxCRC = (rxCRC << 8) | RsBuf[port][rcvIdx[port]+8];
      
      for(; i < 8 + rcvIdx[port]; i++) calCRC += RsBuf[port][i];
  
      if( rxCRC == calCRC )
      {
        rcvLen[port] = rcvIdx[port] = 0;
        bSend[port] = true;
        delayMicroseconds(200);
      }
      else
      {
        /*Serial.print(rcvLen[port]);
        Serial.print(':');
        Serial.print(rcvIdx[port]);
        Serial.print(':');
        for(i = rcvIdx[port]; i < 10 + rcvIdx[port]; i++)
        {
          Serial.print(RsBuf[port][i], HEX); Serial.print(',');
        }
        Serial.print(calCRC, HEX);
        Serial.print(',');
        Serial.print(port);
        Serial.println(" error!!!");
        rcvLen[port] = rcvIdx[port] = 0;*/
      }
    }
  }
}

void loop()
{
  unsigned char val;

  while( Serial.available() > 0 )
  {
    //val = Serial.read();
    //Serial.println(val);
    AddRxData(0, Serial.read());
  }

  if( bSend[0] )
  {
    Serial.write(TxBuf, 8);
    bSend[0] = false;
    tmCnt[0] = 0;
  }
  else
  {
    if( ++tmCnt[0] > 1000 )
    {
      tmCnt[0] = 0;
      bSend[0] = true;
    }
  }
  
  delayMicroseconds(200);
}

