#include "uart.h"
#include "stm32f30x.h"
#include "crc.h"
#include "string.h"
#include "enco.h"
#include "math.h"

#define START_SOFT 0x08000000L //!стартовый адрес FLASH
#define END_SOFT   /*0x08010000L*/0x0800F7FF //!конечный адрес FLASH
#define PERIOD_FACTOR 2 //коэффициент, учитывающий реальный период обработки данных энкодера
#define TRANSMIT_EN 0
#define TRANSMIT_DIS 1

MBuart_type MBuart;
uint8_t UartBuf1[255];   //буфер передатчика порта UART1
uint8_t UartRxBuf1[255]; //буфер приемника порта UART1

// Глобальная структура DMA для организации обмена.
DMA_InitTypeDef  DMA_InitStructure;

extern __IO uint16_t *ADC_SIN1, *ADC_COS1, *ADC_SIN2, *ADC_COS2; //!Для отладки
u16 glowErrState;

void Enco_TIM_Config(u16 request_time_mks); // Время передаем в мкс
u16 phasingDoneFlgDef(encoBlockStatus *encoBlockPnt);
void processingPeriodTimerCalc(u16 processingPeriod);
void debugTimeMeasTimerInit(void);
void errIndGlowOff(void);
extBlockModeType extBlockTypeDef(encoBlockStatus *encoBlockPnt);
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
void USART1_IRQHandler(void)
{  
  u16 Crc, CntRxDma, CrcRx;
  static u16 softCrc = 0; //!CRC FLASH-памяти программы
  float electricSpd;
  encoBlockStatus *encoBlockPnt;
  static telegramModeType readEncoDataState = EXT_BLOCK_IDENT;
  
  identDataType  * idRxDataPnt;
  RxEncoDataType * RxDataPnt;
  TxEncoDataType * TxDataPnt;
  identAnswType  * idTxDataPnt;
  u16 telegramType;
  u16 TxDataLen;
  u16 phasingDoneFlg;
  u16 dataLength;
  u16 idCrc, idCrcRx;
 
  encoResolModeType resolutionMode;
  
  idRxDataPnt = (identDataType *)&MBuart.RxBuffer[0];
  RxDataPnt = (RxEncoDataType *)&MBuart.RxBuffer[0];
  TxDataPnt = (TxEncoDataType *)&MBuart.TxBuffer[0];
  
  encoBlockPnt = &encoBlock;
  if (USART_GetITStatus(USART1,USART_IT_TC) == SET){  // После завершения передачи данных отключаем DMA     
    DMA_Cmd(USART1_TX_DMA_CHANNEL, DISABLE);                                       
    USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);        
    USART_ITConfig(USART1, USART_IT_TC, DISABLE); 
  }
  
  
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){ 
    // INTERRUPT RECIEVE    
    MBuart.RxLength ++; 
  }
  
  if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET){ // Выявлено прерывание по состоянию IDLE
    
    // DMA RECIEVE
    CntRxDma = (u8)(~(DMA_GetCurrDataCounter(USART1_RX_DMA_CHANNEL)));
    DMA_Cmd(USART1_RX_DMA_CHANNEL, DISABLE);         // Отключаем DMA на прием                                   
    USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);  // Отключаем DMA на прием
    USART_ClearITPendingBit(USART1, USART_IT_IDLE);  // сброс флага IDLE         
    USART_ITConfig (USART1, USART_IT_IDLE, DISABLE); // Enable Idle interrupt 
     
    //Контроль минимальной длины принятых данных
    if (CntRxDma < sizeof(identDataType)){
      MBuart.RxLength = sizeof(identDataType);
      UartRecieveDma(&MBuart); // Запускаем DMA на прием 
      return;
    }
    
    
    Crc = GetCrc(RxDataPnt, sizeof(RxEncoDataType) - sizeof(RxDataPnt->crc));
    CrcRx = RxDataPnt->crc;
    
    dataLength = sizeof(identDataType) - sizeof(idRxDataPnt->crc);
    idCrc = GetCrc(idRxDataPnt, dataLength);
    idCrcRx = idRxDataPnt->crc;
    
    switch(readEncoDataState){
    case EXT_BLOCK_IDENT: //Состояние ожидания запроса идентификации
      idRxDataPnt = (identDataType *)&MBuart.RxBuffer[0];
      if(idCrcRx == idCrc){
        telegramType = idRxDataPnt->header.bits.telegramType;
        errIndGlowOff(); //Отключение индикации отсутствия запросов от верхнего уровня
        if(telegramType == IDENT_TELEGRAM){
          idTxDataPnt = (identAnswType *)&MBuart.TxBuffer[0];
          idTxDataPnt->extBlock = extBlockTypeDef(encoBlockPnt); //формируем код блока расширения
          TxDataLen = sizeof(identAnswType) - sizeof(idTxDataPnt->crc);
          idTxDataPnt->crc = GetCrc((unsigned char *)(&MBuart.TxBuffer[0]), TxDataLen);
          MBuart.TxLength = sizeof(identAnswType);
          UartTransmiteDma(&MBuart); 
        }else{
          readEncoDataState = EXT_BLOCK_EXCHANGE;
        }
      }else if(CrcRx == Crc){ //Поступила телеграмма данных энкодеров
        errIndGlowOff(); //Отключение индикации отсутствия запросов от верхнего уровня
        telegramType = RxDataPnt->header.bits.telegramType;
        if(telegramType == EXCHANGE_TELEGRAM){
          readEncoDataState = EXT_BLOCK_EXCHANGE;
        }
      }
      break;
    case EXT_BLOCK_EXCHANGE: //Состояние обмена
      if(idCrcRx == idCrc){
        errIndGlowOff(); //Отключение индикации отсутствия запросов от верхнего уровня
        telegramType = idRxDataPnt->header.bits.telegramType;
        if(telegramType == IDENT_TELEGRAM){
          readEncoDataState = EXT_BLOCK_IDENT;
        }
      }else if(CrcRx == Crc){
                
        errIndGlowOff(); //Отключение индикации отсутствия запросов от верхнего уровня
        
        encoBlockPnt->baseEncoMotorData.serialMode = RxDataPnt->header.bits.serialMode; //Тип последовательного порта
        resolutionMode = RxDataPnt->header.bits.encoResolMode; //Тип разрешения
        encoBlockPnt->baseEncoMotorData.encoResolutionType = resolutionMode;
        if(resolutionMode == BITS_PER_TURN){
          encoBlockPnt->baseEncoMotorData.pulseResolution = (u16)((u16)1 << RxDataPnt->bitResolution);
          encoBlockPnt->baseEncoMotorData.bitResolution  = RxDataPnt->bitResolution;
        }else{
          encoBlockPnt->baseEncoMotorData.pulseResolution = RxDataPnt->pulseResolution;
          encoBlockPnt->baseEncoMotorData.bitResolution = encoBlockPnt->baseEncoMotorData.bitResolution;
        }
        encoBlockPnt->thetaOffset = RxDataPnt->encoAngleShift;
        encoBlockPnt->baseEncoMotorData.polePairsNum = RxDataPnt->motorPolePairsNum; //Число пар полюсов 
        encoBlockPnt->encoProcessingPeriod = /*RxDataPnt->processingPeriod * PERIOD_FACTOR*/400;
        encoBlockPnt->baseEncoMotorData.fltStrgLen = encoBlockPnt->fltStrgLen = RxDataPnt->angleFltBufNum;  //Размер буфера фильтрации фазы
        encoBlockPnt->baseEncoMotorData.encoSpdFltrTime = encoBlockPnt->encoSpdFltrTime = RxDataPnt->spdFltTime * 100;         //Время фильтрации скорости
        encoBlockPnt->spdPhasingParam = RxDataPnt->header.bits.spdPhasingSign; //Фазировка энкодера
        encoBlockPnt->incrSpdPhasingSign = RxDataPnt->header.bits.fastSpdSign; //Фазировка быстрой скорости
        encoBlockPnt->baseEncoMotorData.fastSpdUse = (fastSpdUseType)RxDataPnt->header.bits.fastSpdUse;  //Использование аналоговой инкрементной скорости
        encoBlockPnt->PWMOn = RxDataPnt->header.bits.PWM_On;                   //Флаг включенного ШИМ
        encoBlockPnt->autoPhasingOn = RxDataPnt->header.bits.autoPhasing;      //Флаг включенного режима автофазирования
        encoBlockPnt->drvMode = (drvModeType)RxDataPnt->header.bits.drvType;                //Режим управления: Вектор / Скаляр
        encoBlockPnt->ADC_Amplitude = RxDataPnt->ADC_Amplitude;                //Амплитудный уровень аналоговых сигналов, в кодах АЦП
        encoBlockPnt->encoEmulMode = (encoEmulModeType)RxDataPnt->header.bits.encoEmulMode;      //Режим эмуляции энкодера
        encoBlockPnt->baseEncoMotorData.encoEmulResol = RxDataPnt->encoEmulResol; //Разрешение эмулятора энкодера 
      
        processingPeriodTimerCalc(encoBlockPnt->encoProcessingPeriod);
        phasingDoneFlg = phasingDoneFlgDef(encoBlockPnt); //Проверка завершения фазирования по R-сигналу
      }
      //!Ответные данные
      TxDataPnt->TxHeader.bits.encoErr = encoBlockPnt->encoErr;   //Авария энкодера
      TxDataPnt->TxHeader.bits.Rsygnal = encoBlockPnt->RsignalFlg;//R-сигнал для метки
      TxDataPnt->TxHeader.bits.spdCalcMode = ANALOG_FAST_MODE;    //Режим измерения скорости: аналоговый или цифровой
      TxDataPnt->TxHeader.bits.R_PhasingFlg = phasingDoneFlg; //флаг завершения автофазирования по R-сигналу 
      TxDataPnt->electricTheta = encoBlockPnt->calculatedData.electricTheta;                   //Электрическая позиция энкодера
      electricSpd = encoBlockPnt->calculatedData.electricSpd;
      electricSpd = (fabsf(electricSpd) < MINIMAL_SPD) ? 0.0F : electricSpd;
      TxDataPnt->electricSpd = electricSpd;                     //Электрическая скорость энкодера
      TxDataPnt->softVersion.bits.version = SOFT_VERSION;       //Версия ПО
      TxDataPnt->softVersion.bits.subVersion = SOFT_SUBVERSION; //Подверсия ПО
      softCrc = (!softCrc) ? GetCrc((void*)((u32)START_SOFT), (u16)(END_SOFT - START_SOFT)) : softCrc; //!CRC ПО расчитывается один раз в начале программы
      TxDataPnt->softCrc = softCrc; //CRC ПО
      
      TxDataLen = sizeof(TxEncoDataType) - sizeof(TxDataPnt->dataCRC); 
      TxDataPnt->dataCRC =  GetCrc(&TxDataPnt->TxHeader.word, TxDataLen); //CRC передаваемого пакета                
      MBuart.TxLength = sizeof(TxEncoDataType); 
      UartTransmiteDma(&MBuart);
      
      break;
    }  
    UartRecieveDma(&MBuart); // Запускаем DMA на прием 
    MBuart.RxInd ++; // Счетчик пакетов      
  } 
}
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
// Запуск передачи данных по UART через DMA канал
void UartTransmiteDma (MBuart_type *MBuart)
{    
  
    // Disable the DMA channels 
    DMA_Cmd(USART1_TX_DMA_CHANNEL, DISABLE);     
  // Disable the USART Tx/Rx DMA request 
  USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
  
  // DMA channel Tx of USART Configuration 
  DMA_DeInit(USART1_TX_DMA_CHANNEL);
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_TDR_ADDRESS;
  
  DMA_InitStructure.DMA_BufferSize = (u16)MBuart->TxLength; // Кол-во передаваемых байт
  //DMA_InitStructure.DMA_BufferSize = (u16)12;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)MBuart->TxBuffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; 
  DMA_Init(USART1_TX_DMA_CHANNEL, &DMA_InitStructure);
  
  
  
  // Enable the USART Tx/Rx DMA request 
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE); 
  // Разрешаем прерывание по окончанию передачи
  USART_ClearFlag(USART1, USART_FLAG_TC); // Transmite complete flag clear      
  USART_ITConfig (USART1, USART_IT_TC, ENABLE); // Enable Transmite complete interrupt                               
  // Enable the DMA channel 
  DMA_Cmd(USART1_TX_DMA_CHANNEL, ENABLE);
  
  
}
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
// Запуск приема данных по UART через DMA канал
void UartRecieveDma (MBuart_type *MBuart)
{     
      // Disable the DMA channels 
      DMA_Cmd(USART1_RX_DMA_CHANNEL, DISABLE);     
      // Disable the USART Tx/Rx DMA request 
      USART_DMACmd(USART1,  USART_DMAReq_Rx, DISABLE);       
      // DMA channel Rx of USART Configuration 
      DMA_DeInit(USART1_RX_DMA_CHANNEL);
      DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_RDR_ADDRESS;
      
      DMA_InitStructure.DMA_BufferSize = sizeof(MBuart->RxBuffer); // Сколько принимаем 
      DMA_InitStructure.DMA_MemoryBaseAddr = (u32)MBuart->RxBuffer;
      DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
      DMA_Init(USART1_RX_DMA_CHANNEL, &DMA_InitStructure);       
      // Enable the USART Tx/Rx DMA request 
      USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);      
      
      USART_ClearFlag(USART1, USART_FLAG_IDLE); // Idle flag clear      
      USART_ITConfig (USART1, USART_IT_IDLE, ENABLE); // Enable Idle interrupt       
      // Enable the DMA channel 
      DMA_Cmd(USART1_RX_DMA_CHANNEL, ENABLE);  
      
}     
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
// Настройка порта UART1 для связи с главным МК
// PA10 - Rx, PB6 - Tx

void initUart1(void)
{
    USART_InitTypeDef USART_InitStructure; 
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    USART_DeInit(USART1);    
    
    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);    //!Включение тактирования порта A и B
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // Тактируем USART1
    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1, ENABLE); 
    
    GPIO_PinAFConfig(GPIOA, RX1_SRC, GPIO_AF_7); // Настройка 10-го пина в качестве входа порта UART1
    GPIO_PinAFConfig(GPIOB, TX1_SRC, GPIO_AF_7); // Настройка 6-го пина в качестве выхода порта UART1   
    
    GPIO_InitStructure.GPIO_Pin   = RX1_PIN;            // Формируем маску настраиваемых пинов
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;       // Настройка порта на альтернативный режим
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // Настройка порта на ввод
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;       // Включение подтяжки
    GPIO_Init(GPIOA, &GPIO_InitStructure);              // Инициализация порта A
    
    GPIO_InitStructure.GPIO_Pin   = TX1_PIN;            // Формируем маску настраиваемых пинов
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;   //Нет подтяжки
    GPIO_Init(GPIOB, &GPIO_InitStructure);              // Инициализация порта B
    
    USART_OverSampling8Cmd(USART1, ENABLE);
    USART_InitStructure.USART_BaudRate = 3000000U; //реальная частота передачи данных от верхнего уровня
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStructure);
    USART_OverrunDetectionConfig(USART1, USART_OVRDetection_Disable); 
    
    // Общая инициализация глобального DMA как для Rx так и для Tx
    // Базовые параметры
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
        
    // Enable the USART1 Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);       
    
    /*
    // INTERRUPT RECIEVE
    USART_ClearFlag(USART1, USART_FLAG_IDLE | USART_FLAG_RXNE); // Idle flag clear  
    USART_ITConfig (USART1, USART_IT_RXNE, ENABLE); 
    USART_ITConfig (USART1, USART_IT_IDLE, ENABLE);     
    */    
    
    USART_Cmd(USART1, ENABLE); 
    
    UartRecieveDma(&MBuart); // Запускаем DMA на прием
    
}


void Enco_TIM_Config(u16 request_time_mks)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  uint16_t PrescalerValue, PeriodValue;
    
  // Определяем величину периода и прескалера 
  PrescalerValue = (u16)( request_time_mks / 100) - 1; // Приводим значение к кратности 0,1 мс и назначаем прескалеру 
  PeriodValue = (u16)(SystemCoreClock/10000); // вычисляем значение для периода - 0.1 мс
  
  /* TIM6 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  /* Enable the TIM6 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn; //TIM6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = PeriodValue; // Задает интервал 0,1 мс
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue; // Задает кол-во (0,1 мс)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
  
  /*Selects the TIMx Trigger Output Mode*/
  TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update); //!разрешение формирования события обновления таймера 6 для запуска ADC1/ADC2

  /* Init TIM_OCInitStructure */
  TIM_OCStructInit(&TIM_OCInitStructure);
     
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
  
  TIM_Cmd(TIM6, ENABLE); 
}

/**
  * @brief  Проверка выполненого фазирования по референтной метке
  * @param encoBlockPnt - указатель на объект-описатель энкодера 
  * @retval флаг завершение фазирования
  */
u16 phasingDoneFlgDef(encoBlockStatus *encoBlockPnt)
{
  u16 phasingDone;
  if(encoBlockPnt->baseEncoMotorData.blockType != SIN_COS){
    phasingDone = 1; 
  }else {
    //Фазирование выполнено, если обнаружен R-сигнал и выполнено синхронизация с инкрементными сигналами
    phasingDone = (encoBlockPnt->RsignalFlg && (encoBlockPnt->incrPosPhasingDone != NOT_DETECT)) ? 1 : 0; 
  }
  return(phasingDone);
}


/**
  * @brief  Проверка необходимости переинициализации таймера, задающего период обработки данных
  * @param processingPeriod - требуемый период обработки данных энкодера
  * @retval
  */
void processingPeriodTimerCalc(u16 processingPeriod)
{
  static u16 prevEncoPeriod = 0;
  
  if (processingPeriod != prevEncoPeriod){ //!если период запросов от внешнего МК изменился, изменим время отсчета TIM6
    prevEncoPeriod = processingPeriod;   //!сохраняем новое значение 
    Enco_TIM_Config (processingPeriod);  //!Переинициализация таймера, формирующего выдержку перед опросом энкодера
  }
}

/**
  * @brief  Отключение светодиодной индикации аварии обмена
  * @param
  * @retval
  */
void errIndGlowOff(void)
{
  TIM_SetCounter(TIM17, 0);           //Сброс таймера аварии обмена
  glowErrState = RESET;               //Сброс индикации аварии обмена
  GPIO_ResetBits(GPIOC, GPIO_Pin_15); //Сброс индикации аварии обмена
}


/**
  * @brief  Формирует код, уникальный для всех блоков расширения
  * @param  encoBlockPnt - Указатель на объект-описатель блока энкодеров
  * @retval Уникальный код текущего блока расширения
  */
extBlockModeType extBlockTypeDef(encoBlockStatus *encoBlockPnt){
  extBlockModeType extBlockType; //Глобальный код блока расширения
  encoBlockType blockType;       //Текущий внутренний код блока обработки энкодеров
  
  blockType = encoBlockPnt->baseEncoMotorData.blockType;
  
  switch(blockType){
  case SIN_COS:
    extBlockType = SIN_COS_EXT_BLOCK;
    break;
  case INCREMENTAL:
    extBlockType = INCREMENT_EXT_BLOCK;
    break;
  case SERIAL:
    extBlockType = SERIAL_EXT_BLOCK;
    break;
  default:
    break;
  }
  return(extBlockType);
}


/**
  * @brief  Настройка таймера вычисления длительности выполнения кода
  * @param  
  * @retval 
  */
/*
void debugTimeMeasTimerInit(void)
{
  u16 PrescalerValue;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);                      //!тактирование таймера TIM2
  TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFFUL; //!0xFFFFFFFF - значение после достижения которого будет прерывание по обновлению
  TIM_TimeBaseStructure.TIM_Prescaler = 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  PrescalerValue = 1;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

  // Prescaler configuration //
  TIM_PrescalerConfig(TIM7, PrescalerValue, TIM_PSCReloadMode_Immediate);
  TIM_Cmd(TIM7, ENABLE);
}*/


