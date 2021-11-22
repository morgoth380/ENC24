#ifndef _UART_H_
#define _UART_H_

#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_usart.h"


#define RX1_PIN          GPIO_Pin_10       //ПРОВЕРИТЬ
#define TX1_PIN          GPIO_Pin_6        //ПРОВЕРИТЬ
#define RX1_SRC          GPIO_PinSource10  //ПРОВЕРИТЬ
#define TX1_SRC          GPIO_PinSource6   //ПРОВЕРИТЬ

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

extern  u8 UartBuf1[255];            // буфер порта UART1 
extern  u8 UartTxInd;

//!Тип, задающий формат буфера передатчика
typedef struct
{
  uint32_t ThetaMECH; //!угол поворота вала
  uint32_t ThetaPhase;//!электрический угол
  uint32_t instSpeed; //!мгновенная скорость
  uint32_t averSpeed; //!среднее напряжение
  uint8_t encoErr;    //!код ошибки
  uint8_t errCnt;     //!счетчик ошибок
  uint8_t encoCrc ;   //!контрольная сумма
}UART1TxBff_type;

//!Тип, задающий формат буфера приемника

typedef struct{
  uint8_t cmd;         //!тип запроса
  uint8_t encoType;    //!тип энкодера
  uint8_t EndatVer;    //!версия протокола EnDat
  uint32_t resol;      //!количество отсчетов на оборот
  uint32_t offset;     //!смещение нулевой позиции энкодера для ориентации по полю вектора
  uint8_t PolePairs;    //!количество пар полюсов двигателя
  uint8_t crc8;        //!Контрольная сумма
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