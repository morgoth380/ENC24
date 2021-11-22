#ifndef _UART_H_
#define _UART_H_

#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_usart.h"


#define RX1_PIN          GPIO_Pin_10       //���������
#define TX1_PIN          GPIO_Pin_6        //���������
#define RX1_SRC          GPIO_PinSource10  //���������
#define TX1_SRC          GPIO_PinSource6   //���������

#define USART1_RDR_ADDRESS      USART1_BASE+0x024 // RDR
#define USART1_TDR_ADDRESS      USART1_BASE+0x028 // TDR

#define USART1_TX_DMA_CHANNEL            DMA1_Channel4
#define USART1_TX_DMA_FLAG_TC            DMA1_FLAG_TC4
#define USART1_TX_DMA_FLAG_GL            DMA1_FLAG_GL4
#define USART1_RX_DMA_CHANNEL            DMA1_Channel5
#define USART1_RX_DMA_FLAG_TC            DMA1_FLAG_TC5
#define USART1_RX_DMA_FLAG_GL            DMA1_FLAG_GL5

#define BR115200         115200
#define BR1000000        1000000
#define BR2000000        2000000
#define BR2500000        2500000
#define BR4000000        4000000

extern  u8 UartBuf1[255];            // ����� ����� UART1 
extern  u8 UartTxInd;

//!���, �������� ������ ������ �����������
typedef struct
{
  uint32_t ThetaMECH; //!���� �������� ����
  uint32_t ThetaPhase;//!������������� ����
  uint32_t instSpeed; //!���������� ��������
  uint32_t averSpeed; //!������� ����������
  uint8_t encoErr;    //!��� ������
  uint8_t errCnt;     //!������� ������
  uint8_t encoCrc ;   //!����������� �����
}UART1TxBff_type;

//!���, �������� ������ ������ ���������

typedef struct{
  uint8_t cmd;         //!��� �������
  uint8_t encoType;    //!��� ��������
  uint8_t EndatVer;    //!������ ��������� EnDat
  uint32_t resol;      //!���������� �������� �� ������
  uint32_t offset;     //!�������� ������� ������� �������� ��� ���������� �� ���� �������
  uint8_t PolePairs;    //!���������� ��� ������� ���������
  uint8_t crc8;        //!����������� �����
}UART1RxBff_type;

typedef struct{
    uint8_t RxBuffer[255];
    uint8_t TxBuffer[255];
    uint8_t StateMB;
    uint8_t RxLength;
    uint16_t TxLength;
    u8      TxInd;
    u8      RxInd; 
}MBuart_type;


extern MBuart_type MBuart;
void InitUartMB(void);
void InitUart1(void);
void UartTransmiteDma (MBuart_type *MBuart);
void UartRecieveDma (MBuart_type *MBuart);


void USART3_IRQHandler(void);
void USART1_IRQHandler(void);
#endif //_UART_H_