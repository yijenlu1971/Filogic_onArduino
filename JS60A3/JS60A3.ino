#include "hal_uart.h"

#define BUF_SIZE  40

bool bSend[2];
int rcvLen[2], rcvIdx[2], tmCnt[2];
unsigned char tmr, RsBuf[2][BUF_SIZE];
unsigned char TxBuf[8] = {0xFC, 0xA5, 0, 0, 0, 0, 0xA1, 0x01};

static volatile uint32_t send_notice = 1;
uint8_t rx_vfifo_buffer[32];
uint8_t tx_vfifo_buffer[32];

void user_uart_callback(hal_uart_callback_event_t status, void *user_data)
{
  if( status == HAL_UART_EVENT_READY_TO_WRITE )
    send_notice = 1;
}

void setup()
{
  int i;
  hal_uart_config_t uart_config;

  Serial.begin(115200);

  // Configure UART PORT1
  uart_config.baudrate = HAL_UART_BAUDRATE_115200;
  uart_config.parity = HAL_UART_PARITY_NONE;
  uart_config.stop_bit = HAL_UART_STOP_BIT_1;
  uart_config.word_length = HAL_UART_WORD_LENGTH_8;
  hal_uart_init(HAL_UART_0, &uart_config);
  hal_uart_init(HAL_UART_2, &uart_config);

  for(i = 0; i < 2; i++)
  {
    tmCnt[i] = rcvLen[i] = rcvIdx[i] = 0;
    bSend[i] = true;
  }

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
      }
      /*else
      {
        Serial.print(rcvLen[port]);
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
        rcvLen[port] = rcvIdx[port] = 0;
      }*/
    }
  }
}
void loop()
{
  unsigned short i, len;
  hal_uart_port_t uPort[2] = {HAL_UART_0, HAL_UART_2};

  hal_uart_send_polling(HAL_UART_1, TxBuf, 8);
  hal_uart_send_polling(HAL_UART_2, TxBuf, 8);

  for(i = 0; i < 2; i++)
  {
    len = hal_uart_receive_polling(uPort[i], RsBuf[i], BUF_SIZE);
    
  }

  Serial.println("123");
  delay(1);
}

