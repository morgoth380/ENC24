#include "stm32f30x.h"
#include "enco.h"
#include "flashDrv.h"
#include "IQmathLib.h"
#include "math.h"
#include "flashDrv.h"
#include "time.h"
#include "string.h"
#define pi 3.14159265358979F
#define F_ZERO         0.02F         //!частота при уменьшении до которой и ниже ее показания измеренной скрости будут нулевые
#define ADC_SIZE_BUFFER    2
#define ADC34_SIZE_BUFFER  2

#define MODE1 0
#define MODE2 1
#define PREC  1
#define FAST_SPD_STRG_LEN  2
#define R_SIGNAL_HOLD_ITIME 0.02F //Время удержания флага R-сигнала для логгера

ENDAT_SPI_BUFFER EndatSpiData;
SSI_SPI_BUFFER SSI_SpiData;

__IO uint32_t calibration_value_1 = 0, calibration_value_2 = 0;
__IO uint32_t calibration_value_3 = 0, calibration_value_4 = 0;

__IO uint32_t ADC1_ValueTab[ADC_SIZE_BUFFER];
__IO uint32_t ADC34_ValueTab[ADC_SIZE_BUFFER];

__IO uint16_t *ADC_SIN1, *ADC_COS1, *ADC_SIN2, *ADC_COS2;
__IO uint16_t *ADC_SIN3, *ADC_COS3, *ADC_SIN4, *ADC_COS4;

u16 R_signalFlg;

void Delay(__IO uint32_t nTime);
void compareRegCalc(float electricSpeed, TIM_TypeDef* TIMx, u32 ARR_val);

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 

encoFlashMemDataType encoFlashMemData;
encoBlockStatus encoBlock = ENCO_BLOCK_STATUS_DEFAULTS;

//==============================================================================

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
u16 CrcEnDat(u16 clocks, u16 endat, u16 alarm1, u16 alarm2, uint32_t highpos, uint32_t lowpos)
{
  u16 ff[5];
  u16 code [65];
  u16 ex;
  u16 crc = 0;
  int16_t i;
  for(i = 0; i < 5; i++)
  {
    ff[i] = 1;
  }
  
  if(endat == ECN1313){ //!EnDat2.1 Расчет совпадает с оригинальной методикой
    code[0] = alarm1;
    for(i = 1; i < 33; i++)
    {
      code[i] = (lowpos & 0x00000001L)? 1 : 0;
      lowpos >>=1;
    }
    for(i = 33; i < 65; i++)
    {
      code[i] = (highpos & 0x00000001L) ? 1 : 0;
      highpos >>=1;
    }
    for(i = 0; i <= clocks; i++)
    {
      ex = ff[4] ^ code[i];
      ff[4] = ff[3];
      ff[3] = ff[2] ^ ex;
      ff[2] = ff[1];
      ff[1] = ff[0] ^ ex;
      ff[0] = ex;
    }
    for(i = 4; i >= 0; i--)
    {
      ff[i] = ff[i] ? 0 : 1;
      crc <<= 1;
      crc |= ff[i];
    }
  }else{ //!Если EnDat2.2 Отличается от EnDat2.1 только еще одним битом аварии
    code[0] = alarm1; //!!!Может наоборот
    code[1] = alarm2; //!!!Может наоборот
    for(i = 2; i < 34; i++) //!первые два бита заняты битами ошибок
    {
      code[i] = (lowpos & 0x00000001L)? 1 : 0; //!добавляем к битам ошибок код позиции
      lowpos >>=1;
    }
    for(i = 34; i < 65; i++)
    {
      code[i] = (highpos & 0x00000001L) ? 1 : 0; //!добавляем к младшему слову позиции старшее слово позиции
      highpos >>=1;
    }
    for(i = 0; i <= clocks + 1; i++)
    {
      ex = ff[4] ^ code[i];
      ff[4] = ff[3];
      ff[3] = ff[2] ^ ex;
      ff[2] = ff[1];
      ff[1] = ff[0] ^ ex;
      ff[0] = ex;
    }
    for(i = 4; i >= 0; i--)
    {
      ff[i] = ff[i] ? 0 : 1;
      crc <<= 1;
      crc |= ff[i];
    }
  }
  
  return crc;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
// Настройка аппаратного SPI для работы с EnDat
// SPI_MISO - PB4 (SPI1_MISO), PB5 (SPI1_MOSI)
// SPI_MOSI - PA7 (SPI1_MOSI)
// SPI_CLK  - PA5 (SPI1_SCK), PA2 
// SPI_CS   - PA4 (SPI1_NSS)
void EndatSpiInit (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure;

  EndatSpiData.EndatSpiState = StopMode; // Текущее состояние 
  
  /* Enable the SPI 1 peripheral */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  
  /* Enable SCK, MOSI, MISO and NSS GPIO clocks */
  RCC_AHBPeriphClockCmd(SPI1_SCK_GPIO_CLK | SPI1_MISO_GPIO_CLK | SPI1_MOSI_GPIO_CLK |
                        SPI1_NSS_GPIO_CLK , ENABLE);
    
  /* SPI pin mappings */
  GPIO_PinAFConfig(SPI1_SCK_GPIO_PORT,  SPI1_SCK_SOURCE,  SPI1_SCK_AF);
  GPIO_PinAFConfig(SPI1_MOSI_GPIO_PORT, SPI1_MOSI_SOURCE, SPI1_MOSI_AF);
  GPIO_PinAFConfig(SPI1_MISO_GPIO_PORT, SPI1_MISO_SOURCE, SPI1_MISO_AF);
  
  // General setting for GPIO pins
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN;
  GPIO_Init(SPI1_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  SPI1_MOSI_PIN;
  GPIO_Init(SPI1_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPI1_MISO_PIN;
  GPIO_Init(SPI1_MISO_GPIO_PORT, &GPIO_InitStructure);
  
  /* SPI NSS pin configuration */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // Работает как выход
  GPIO_InitStructure.GPIO_Pin = SPI1_NSS_PIN;
  GPIO_Init(SPI1_NSS_GPIO_PORT, &GPIO_InitStructure);
  
  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPI1);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_10b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //исходный уровень на линии: 1
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //прием вторым фронтом
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // 32
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(SPI1, &SPI_InitStructure);
  
  /* Configure the SPI interrupt priority */
  
  NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the SPI peripheral */
  SPI_Cmd(SPI1, ENABLE);  
}
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
void Delay(__IO uint32_t nTime)
{ 
  __IO uint32_t TimingDelay;
  
  TimingDelay = nTime;

  while(TimingDelay != 0) {
    TimingDelay++;
    TimingDelay--;
    TimingDelay--;
  }
}
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
// Настройка ADC для работы с синусно-косинусным энкодером
// Работа в режиме DMA по всем 4-м каналам
void encoderAdcInit (encoBlockStatus *encoBlockPnt)
{
  GPIO_InitTypeDef      GPIO_InitStructure;
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;  
  DMA_InitTypeDef       DMA_InitStructure;
  NVIC_InitTypeDef      NVIC_InitStructure;
  
  
  /* Configure the ADC12 clock */
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2); 
  
  /* Enable ADC12 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
  
  
 /* Enable GPIO clocks */
  RCC_AHBPeriphClockCmd(ADCx_INPUT_SIN1_GPIO_CLK | ADCx_INPUT_COS1_GPIO_CLK | ADCx_INPUT_SIN2_GPIO_CLK | ADCx_INPUT_COS2_GPIO_CLK , ENABLE);  
  
  /* Enable DMA Clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  
  
  /* Configure ADC1 Channe1 as analog input */
  GPIO_InitStructure.GPIO_Pin = ADCx_INPUT_SIN1_PIN;  //!PA0 - вход сигнала А  абсолютной позиции (медленная синусоида)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; //аналоговый вход
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(ADCx_INPUT_SIN1_GPIO_PORT, &GPIO_InitStructure);  

  /* Configure ADC12 Channe12 as analog input */
  GPIO_InitStructure.GPIO_Pin = ADCx_INPUT_COS1_PIN;  //!PB2 - вход сигнала B (COS) абсолютной позиции
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(ADCx_INPUT_COS1_GPIO_PORT, &GPIO_InitStructure);  
  
    /* Configure ADC1 Channe2 as analog input */
  GPIO_InitStructure.GPIO_Pin = ADCx_INPUT_SIN2_PIN; //!PA1 сигнал A инкрементального сигнала sin/cos (быстрая синусоида)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(ADCx_INPUT_SIN2_GPIO_PORT, &GPIO_InitStructure);  

  /* Configure ADC12 Channe3 as analog input */
  GPIO_InitStructure.GPIO_Pin = ADCx_INPUT_COS2_PIN; //!PA6 сигнал B инкрементального сигнала sin/cos
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(ADCx_INPUT_COS2_GPIO_PORT, &GPIO_InitStructure);
  
  // Указатели на переменные данных ADC ---------------------------------------  
  ADC_SIN1 = (__IO uint16_t *)(&ADC1_ValueTab[0]);         //!медленная синусоида   
  ADC_COS1 = ((__IO uint16_t *)(&ADC1_ValueTab[0])) + 1;   //!медленная косинусоида 
  ADC_SIN2 = (__IO uint16_t *)(&ADC1_ValueTab[1]);         //!быстрая синусоида
  ADC_COS2 = ((__IO uint16_t *)(&ADC1_ValueTab[1])) + 1;   //!быстрая косинусоида
  
#warning разобраться нужно ли менять DISCEN (стр 362 мануала к 303CBT6)
  
  // DMA1 channel1 configuration ---------------------------------------------- 
  DMA_DeInit(ADC1_DMA_CHANNEL);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC_CDR_ADDRESS;//(uint32_t)&ADC1->DR; //!источник данных
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC1_ValueTab; //!приемник данных для DMA
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = ADC_SIZE_BUFFER; // размер буфера приемника данных DMA
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word; 
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(ADC1_DMA_CHANNEL, &DMA_InitStructure);

  //DMA_ITConfig(ADC1_DMA_CHANNEL, DMA_IT_TC, ENABLE);  // Тестовое включение прерывания для оценки скорости оцифровки
  DMA_Cmd(ADC1_DMA_CHANNEL, ENABLE); //Enable the DMA1 - Channel1
    
  // Enable DMA1 Channel1
  DMA_Cmd(ADC1_DMA_CHANNEL, ENABLE);
 
  //configure ADC1 parameters
  ADC_StructInit(&ADC_InitStructure);  
  
  Delay(100000);
  
  
  //Calibration procedure 
  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
  ADC_VoltageRegulatorCmd(ADC2, ENABLE);
  
  //Insert delay equal to 10 µs
    Delay(10);
  
   
  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC1);

  ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC2);
  
  while(ADC_GetCalibrationStatus(ADC1) != RESET );
  calibration_value_1 = ADC_GetCalibrationValue(ADC1);

  while(ADC_GetCalibrationStatus(ADC2) != RESET );
  calibration_value_2 = ADC_GetCalibrationValue(ADC2); 
  
  
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_RegSimul;                                                            
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv2;          
  //ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;    //!DMA будет считывать регистр данных-common_data_reg только при заершении конвертирования обоих АЦП. Это из-за того, что используются каналы с различными скоростями конвертирования 
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 3;          
  ADC_CommonInit(ADCx_INPUT_SIN1_ADC, &ADC_CommonInitStructure);       // Работает для ADC1 & ADC2

  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;         //1 проход по всем каналам при 1-м триг. событии    //ADC_ContinuousConvMode_Enable
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_13;        //!тригер оцифровки от TIM6_TRGO     //ADC_ExternalTrigConvEvent_0 //сигнал начала ицифровки - выход таймера TIM6   
  
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;  //!По положительному фронту от таймера
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
  ADC_InitStructure.ADC_NbrOfRegChannel = 2;
  ADC_Init(ADCx_INPUT_SIN1_ADC, &ADC_InitStructure); // ADC1
  ADC_Init(ADCx_INPUT_COS1_ADC, &ADC_InitStructure);  // ADC2
  
  
  
  /*------------------------------------------------------------------------------------*/
  /*                    Настройка порядка оцифровки АЦП №1, АЦП №2                                      */
  /*-------------------------------------------------------------------------------------*/
  /* ADC1 regular channel7 configuration */ 
  ADC_RegularChannelConfig(ADCx_INPUT_SIN1_ADC, ADCx_INPUT_SIN1_CHANNEL, 1, ADC_SampleTime_7Cycles5); // ADC1
  ADC_RegularChannelConfig(ADCx_INPUT_COS1_ADC, ADCx_INPUT_COS1_CHANNEL, 1, ADC_SampleTime_7Cycles5); // ADC2
  ADC_RegularChannelConfig(ADCx_INPUT_SIN2_ADC, ADCx_INPUT_SIN2_CHANNEL, 2, ADC_SampleTime_7Cycles5); // ADC1
  ADC_RegularChannelConfig(ADCx_INPUT_COS2_ADC, ADCx_INPUT_COS2_CHANNEL, 2, ADC_SampleTime_7Cycles5); // ADC2
  
  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);  
   
  ADC_TempSensorCmd (ADC1, ENABLE);
  ADC_TempSensorCmd (ADC2, ENABLE);  
  
  /* wait for ADRDY */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
  /* wait for ADC2 ADRDY */
  while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));  
    
  ADC_DMACmd(ADC1, ENABLE);  
  ADC_DMAConfig(ADC1, ADC_DMAMode_Circular); // !*!*!*!*!*!
  
  /* Start ADC1 Software Conversion */ 
  ADC_StartConversion(ADC1); 
}
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
//!Настройка модуля обработки инкрементального энкодера

void incrementalModeInit(encoBlockStatus *encoBlockPnt)
{ 
  GPIO_InitTypeDef      GPIO_InitStructure;            //!Структура для настройки портов
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      //!структура для настройки таймеров
  TIM_ICInitTypeDef  TIM_ICInitStructure;         //!Структура для настройки таймера
  NVIC_InitTypeDef NVIC_InitStructure;
  uint16_t PeriodValue;
  uint16_t pulseResolution;
  uint16_t PrescalerValue = 0;
  
  pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
  TIM_Cmd(TIM2, DISABLE); //Останов таймера TIM2
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  // тактирование порта А
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);  // тактирование порта B

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //!тактирование таймера TIM2
   
  //!Настройка пина PA15 в качестве счетного входа 1 таймера TIM2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //!Альтернативная функция
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //!Подтяжка
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  //!Подтяжка к нулю
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;       //!PA15
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  GPIO_PinAFConfig(GPIOA,  GPIO_PinSource15,  GPIO_AF_1); 
  
 
  // Настройка пина PB3 в качестве счетного входа 2 таймера TIM2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //!Альтернативная функция
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //!Подтяжка
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  //!Подтяжка к нулю
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;       //!PB3
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource3,  GPIO_AF_1); 
  

  //!Настройка пина PB8 в качестве входа для сигнала референтной метки
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    //!
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //!Подтяжка
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; //!Подтяжка к нулю
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;       //!PB8
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource8,  GPIO_AF_1);
  
  /*Настройка режима захвата таймера TIM2 для контроля наличия сигналов A и B*/
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // Принимаем каждый захват
  TIM_ICInitStructure.TIM_ICFilter = /*10*/10;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  
  /*Настройка таймера TIM2 на режим модуля энкодера*/
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //Включение режима энкодера
  TIM_SetAutoreload(TIM2, (RESOLUTION_FACTOR * pulseResolution - 1)); //Настройка значения обновления таймера-счетчика модуля энкодера
  
  
  /*настройка таймера контроля R-сигнала**/ 
  /*Таймер нужен только для того чтоб вызвать прерывание по R-сигналу*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);   //!тактирование таймера TIM16
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  PrescalerValue = 4;     //!делим частоту работы таймера на 4
  PeriodValue = (uint16_t)(SystemCoreClock * PrescalerValue / encoBlockPnt->encoProcessingPeriod) - 1;
  
  TIM_TimeBaseStructure.TIM_Period  = PeriodValue;           //!значение переполнения
  TIM_TimeBaseStructure.TIM_Prescaler  = PrescalerValue - 1; //! делитель 4
  TIM_TimeBaseStructure.TIM_ClockDivision  = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);
  
  /*Настройка режима захвата таймера TIM2 для обработки АВАРИЙ обрыва инкрементных линий*/
  /*
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //Канал №1:
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter = 5;    
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
  
  //Канал №2
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter = 5;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
  */
  /*Настройка режима захвата таймера TIM16*/
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // Принимаем каждый захват
  TIM_ICInitStructure.TIM_ICFilter = 5;   
  TIM_ICInit(TIM16, &TIM_ICInitStructure);
  
  TIM_ITConfig(TIM16, TIM_IT_CC1, ENABLE);
  TIM_Cmd(TIM16, ENABLE); //Запуск таймера
  TIM_Cmd(TIM2, ENABLE); //Запуск таймера 
}


/**
  * @brief  Расчет скорости инкрементального энкодера
  * @param  encoBlockPnt - указатель на объект-описатель блока энкодеров
  * @retval
  */
void incrementDataProcessing(encoBlockStatus *encoBlockPnt)
{
  float fltDiscrSpeed, ThetaElec;
  static u16 holdTimer = 0;
  u16 polePairsNum;
  float ThetaMechPU;
  
  ThetaMechPU = getDiscrThetaMechPU(encoBlockPnt);                       // механическая фаза по инкрементным сигналам
  polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
  ThetaElec = ThetaMechPU * polePairsNum;                                //Электрический угол по инкрементным сигналам
  ThetaElec = getActualElectrPhase(ThetaElec, encoBlockPnt);             //Устранение лишней фазы
  ThetaElec = encoBlockPnt->spdPhasingParam ? (-ThetaElec) : ThetaElec;  //учет фазировки энкодера в фазе
  fltDiscrSpeed = discrSpdIncrCalc(ThetaMechPU, encoBlockPnt);           //скорость по дискретной фазе
  fltDiscrSpeed = encoBlockPnt->spdPhasingParam ? (-fltDiscrSpeed) : fltDiscrSpeed;  //учет фазировки энкодера в скорости
    
  if(R_signalFlg == 1){
    //TIM_SetCounter(TIM2, 0);
    R_signalFlg = 0;
    encoBlockPnt->RsignalFlg = 1;
    holdTimer = (u16)(R_SIGNAL_HOLD_ITIME * 1000000.0F / encoBlockPnt->encoProcessingPeriod + 0.5F);
  }
  if(holdTimer > 0){
    holdTimer--;
    encoBlockPnt->RsignalFlg = (holdTimer == 0) ? 0 : encoBlockPnt->RsignalFlg;
  }
  
  encoBlockPnt->calculatedData.shadowSpeedElectric = fltDiscrSpeed;
  encoBlockPnt->calculatedData.electricTheta = ThetaElec;
}


/**
  * @brief  Расчет скорости и фазы для энкодера с  последовательным интерфейсом
  * @param  encoBlockPnt - указатель на объект-описатель блока энкодеров
  * @retval
  */
void serialDataProcessing(unsigned long long position, encoBlockStatus *encoBlockPnt){
   u16 digitSpdSign;
   volatile u16   encoderCRC, crcCalc; //Проверка CRC отключена
   u32   encoderPosition;
   float fltSpd;
   float endatFltSpd, fltFastSpd;
   float discrThetaMechPU;
   float fltDiscrSpeed;
   float ThetaMechPU, ThetaElec;

  
   encoderPosition = getEnDatEncoderPosition(encoBlockPnt, position);
   encoderCRC = getEnDatEncoderCRC(position); 
   crcCalc = EnDatEncoderCRCcalc(encoBlockPnt, encoderPosition, position);
   if(encoderCRC != crcCalc){
     crcCalc++;
   }
   ThetaMechPU = EnDatEncoderMechPosCalc(encoBlockPnt, encoderPosition); //механическоий угол по enDat
   ThetaElec = EnDatEncoderElecPosCalc(encoBlockPnt, ThetaMechPU);       //электрический угол по enDat
   ThetaElec = getActualElectrPhase(ThetaElec, encoBlockPnt);            //электрический угол по enDat
   ThetaElec = encoBlockPnt->spdPhasingParam ? (-ThetaElec) : ThetaElec; //Учет фазировки энкодера
   endatFltSpd = endatEncoSpdCalc(ThetaMechPU, encoBlockPnt);            //Расчет скорости по цифровой фазе
   discrThetaMechPU = getDiscrThetaMechPU(encoBlockPnt);                 //текущий угол из модуля инкрементного энкодера
   fltDiscrSpeed = incrAngleSpdCalc(discrThetaMechPU, encoBlockPnt);     //Скорость по фазе модуля инкрементного энкодера
   digitSpdSign = digitalPhaseInversion(encoBlockPnt, endatFltSpd, fltDiscrSpeed, &encoFlashMemData); //Синхронизация знаков скоростей endatFltSpd и fltDiscrSpeed
   endatFltSpd = encoBlockPnt->spdPhasingParam ? (-endatFltSpd) : endatFltSpd;   //Учет фазировки энкодера в значении скорости
   fltFastSpd = fastSinCosSignalSpdCalcForEnDat(encoBlockPnt);             //Расчет скорости по аналоговым быстрым синусоидам
   fltFastSpd*= encoFlashMemData.fastSpdSignSetting;                  //Учет текущей настройки знака быстрой аналоговой скорости
   fastSinCosSpdSignPhasing(encoBlockPnt, endatFltSpd, fltFastSpd, &encoFlashMemData, -1); //синхронизация быстрой аналоговой скорости и цифровой
   fastSpdSignReadFromFlash(encoBlockPnt->drvMode);
   
   if (encoBlockPnt->baseEncoMotorData.fastSpdUse == USE){
     fltSpd = spdCalcModeValSelect(encoBlockPnt, endatFltSpd, fltFastSpd); //выбор одной из двух скоростей: цифровой или аналоговой
   }else{
     fltSpd = endatFltSpd;
   }
   encoBlockPnt->encoErr = enDatEncoErrDetect(encoBlockPnt, position);  //Контроль аварий энкодера
   //!Запись расчитанных значений в глобальную структуру энкодера
   encoBlockPnt->calculatedData.incrementModulSpd = fltDiscrSpeed;
   encoBlockPnt->incrModulSpdPhasingStatus = digitSpdSign;
   encoBlockPnt->calculatedData.incrementModulAngle = discrThetaMechPU;
   encoBlockPnt->calculatedData.ThetaMechPU = ThetaMechPU;
   encoBlockPnt->calculatedData.ThetaMechFinePU = 0; 
   encoBlockPnt->calculatedData.ThetaMechCoarsePU = ThetaMechPU;
   encoBlockPnt->calculatedData.electricTheta = ThetaElec;                      
   encoBlockPnt->calculatedData.shadowSpeedElectric = fltSpd; // Скорость заносим в теневую переменную для организации общей фильтрации в main.c                             
   EndatSpiData.PosState = WaitPacket; //!ждем новый пакет  

}

/**
  * @brief  Расчет скорости и фазы для энкодера типа Sin/Cos
  * @param encoBlockPnt - указатель на объект-описатель обработчика энкодеров
  * @retval
  */
void sinCosDataProcessing(encoBlockStatus *encoBlockPnt){
    u16 digitSpdSign;
    u16 polePairsNum;
    float   ThetaMechPU, ThetaElec;
    float fltDiscrSpeed;
    float fltAnalogSpeed;
    float analogThetaMechPU;
    float fltFastSpd;
    float fltSpd;
    float thetaOffset;
       
    setBasePhasingDataBeforeStart(encoBlockPnt); //считывание параметров автофазирования из flash-памяти, установка стартового значения счетчика инкрементной фазы
    ThetaMechPU = getDiscrThetaMechPU(encoBlockPnt);            // механическая фаза по инкрементным сигналам
    polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
    ThetaElec = ThetaMechPU * polePairsNum;           //Электрический угол по инкрементным сигналам
    thetaOffset = (float)encoBlockPnt->thetaOffset / 3600.0F;
    ThetaElec += thetaOffset;                                   //Прибавка смещения 
    ThetaElec = getActualElectrPhase(ThetaElec, encoBlockPnt);  //Устранение лишней фазы после прибавки
    ThetaElec = encoBlockPnt->spdPhasingParam ? (-ThetaElec) : ThetaElec;  //учет фазировки энкодера в фазе
    fltDiscrSpeed = discrSpdCalc(ThetaMechPU, encoBlockPnt);         //скорость по дискретной фазе
    analogThetaMechPU = getAnalogThetaMechPU(encoBlockPnt);          //механическая фаза по аналоговым сигналам SIN и COS
    fltAnalogSpeed = analogSpdCalc(analogThetaMechPU, encoBlockPnt); //скорость по аналоговой фазе
    fltFastSpd = fastSinCosSignalSpdCalc(encoBlockPnt);              //Расчет скорости по аналоговым быстрым синусоидам
    fltFastSpd*= fastSpdSignDef(encoBlockPnt, encoFlashMemData.fastSpdSignSetting); //учет знака аналоговой инкрементной скорости
    digitSpdSign = digitalPhaseInversion(encoBlockPnt, fltAnalogSpeed, fltDiscrSpeed, &encoFlashMemData);
    encoBlockPnt->incrPosPhasingDone = digitSpdSign;
    encoBlockPnt->incrPosPhasingDone = (encoBlockPnt->PWMOn == 0) ? NOT_DETECT : encoBlockPnt->incrPosPhasingDone;
    fastSinCosSpdSignPhasing(encoBlockPnt, fltDiscrSpeed, fltFastSpd, &encoFlashMemData, digitSpdSign); //синхронизация быстрой аналоговой скорости и цифровой 
    phaseShiftDetect(encoBlockPnt, &encoFlashMemData);  //устранение фазового сдвига между аналоговой фазой и цифровой
     if(encoBlockPnt->baseEncoMotorData.fastSpdUse == NOT_USE){ //Если аналоговая инкрементная скорость не используется
       fltSpd = fltDiscrSpeed;      //Работаем только с цифровой скростью
    }else{
       fltSpd = spdCalcModeValSelect(encoBlockPnt, fltDiscrSpeed, fltFastSpd); //выбор одной из двух скоростей: цифровой или аналоговой 
    }
    fltSpd = encoBlockPnt->spdPhasingParam ? (-fltSpd) : fltSpd;  //учет фазировки энкодера в скорости
    //fltSpd = velocityMedianFilter(fltSpd);
    
    encoBlockPnt->encoErr = sinCosEncoErrDetect(encoBlockPnt, fltSpd, encoFlashMemData.phaseShift); //Проверка аварий энкодера типа sin/cos 
    encoBlockPnt->calculatedData.shadowSpeedElectric = fltSpd;
    encoBlockPnt->calculatedData.ThetaMechPU = ThetaMechPU;
    encoBlockPnt->calculatedData.electricTheta = ThetaElec;
}



/**
  * @brief  Считывание базовых значений из Flash-памяти для настройки аппаратного инкрементного 
  *         модуля энкодера при подаче питания или смене режима управления. Установка стартового значения счетчика
  *         инкрементного модуля энкодера.
  * @param  encoBlockPnt - указатель на объект-описатель обработчика энкодеров
  * @retval
  */
void setBasePhasingDataBeforeStart(encoBlockStatus *encoBlockPnt){
  u16 pulseResolution;
  float ThetaMechPU;
  s16 incrPosStartVal;
  u32 cntState;
  static u16 baseDataDoneInScalar = 0;
  static u16 baseDataDoneInVect   = 0;
  
  pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution; //Разрешение, импульсы / оборот
  baseDataDoneInVect = (encoBlockPnt->encoErr != ENCO_OK)? 0 : baseDataDoneInVect;
  baseDataDoneInScalar = (encoBlockPnt->encoErr != ENCO_OK)? 0 : baseDataDoneInScalar;
  
  switch(encoBlockPnt->drvMode){
  case SCALAR_MODE:
       baseDataDoneInVect = 0;
       if(!baseDataDoneInScalar){ //настройки из flash-памяти читаем один раз
          encoFlashMemData.encoInputSetting = getEncoInputSettFromFlash(); //Настройки цифрового входа аппаратного модуля энкодера
          encoFlashMemData.phaseShift = corrPhaseRead(BASE_PAGE_ADDR); //текущее значение из FLASH; //Текущая добавка к фазе, рассчитываемой по абсолютным сигналам sin/Cos
          encoFlashMemData.fastSpdSignSetting  = getFastSpdSignFromFlash(); //текущая настройка знака быстрой аналоговой скорости
          ThetaMechPU = getAnalogThetaMechPU(encoBlockPnt); //текущая фаза по абсолютным сигналам sin/cos
          ThetaMechPU += encoFlashMemData.phaseShift;  //корректирующий угол
          ThetaMechPU = removeExcessPhase(ThetaMechPU);//контроль допустимого диапазона фазы после прибавки смещения
          incrPosStartVal = (s16)(ThetaMechPU * (RESOLUTION_FACTOR * pulseResolution) + 0.5F); //базовое значение инкрементной фазы
          while(TIM1->CR1 & TIM_CR1_CEN == 0){ //ждем запуск таймера
            ;
          }
          TIM1->CR1 &= ~TIM_CR1_CEN; //Останов таймера
          while(TIM1->CR1 & TIM_CR1_CEN == 1){ //ждем останов таймера
            ;
          }
          TIM_SetCounter(TIM1, incrPosStartVal); //установка счетчика модуля энкодера в начальное значение
          cntState = TIM_GetCounter(TIM1);       //проверка на успешность установки
          while(cntState != incrPosStartVal){ //ждем установку таймера
            ;
          }
          
          TIM1->CR1 |= TIM_CR1_CEN; //Запуск таймера
          baseDataDoneInScalar = 1;//((cntState == incrPosStartVal) && (encoBlockPnt->encoErr == ENCO_OK)) ? 1 : baseDataDoneInScalar;
       }
    break;
  case VECTOR_MODE:
       baseDataDoneInScalar = 0;
       if(!baseDataDoneInVect){ //настройки входа из flash-памяти читаем один раз
          encoFlashMemData.encoInputSetting = getEncoInputSettFromFlash();  //Настройки цифрового входа аппаратного модуля энкодера
          encoFlashMemData.phaseShift = corrPhaseRead(BASE_PAGE_ADDR);           //Смещение, полученное при автофазировке
          encoFlashMemData.fastSpdSignSetting  = getFastSpdSignFromFlash(); //текущая настройка знака инкрементной аналоговой скорости
          ThetaMechPU = getAnalogThetaMechPU(encoBlockPnt); //текущая исходная механическая фаза по SIN и COS
          ThetaMechPU += encoFlashMemData.phaseShift;  //текущая исходная механическая фаза + корректирующее смещение
          ThetaMechPU = removeExcessPhase(ThetaMechPU);//контроль допустимого диапазона фазы после прибавки смещения
          incrPosStartVal = (s16)(ThetaMechPU * (RESOLUTION_FACTOR * pulseResolution) + 0.5F); //исходное значение инкрементной фазы
          TIM_SetCounter(TIM1, incrPosStartVal); //установка счетчика в начальное значение
          cntState = TIM_GetCounter(TIM1);       //проверка на успешность установки
          baseDataDoneInVect = ((cntState == incrPosStartVal) && (encoBlockPnt->encoErr == ENCO_OK))? 1 : baseDataDoneInVect;   
       }
    break;
  }
}



/**
  * @brief  Однократное считывание знака быстрой скорости из FLASH-памяти при смене режима управления ПЧ
  * @param  drvMode - текущий режим управления ПЧ
  * @param  scalarPhasingDataReadFlg - указатель на флаг считанных данных в скалярном режиме
  * @param  vectorPhasingDataReadFlg - указатель на флаг считанных данных в векторном режиме
  * @retval
  */
void fastSpdSignReadFromFlash(drvModeType drvMode){
  static u16 scalarPhasingDataReadFlg = 0;
  static u16 vectorPhasingDataReadFlg = 0;
  
  switch(drvMode){
  case SCALAR_MODE:
    vectorPhasingDataReadFlg = 0;
    if(scalarPhasingDataReadFlg == 0){ //настройки входа из flash-памяти читаем один раз
      encoFlashMemData.fastSpdSignSetting  = getFastSpdSignFromFlash(); //текущая настройка знака быстрой аналоговой скорости из Flash-памяти
      scalarPhasingDataReadFlg = 1;
    }
    break;
  case VECTOR_MODE:
    scalarPhasingDataReadFlg = 0;
    if(vectorPhasingDataReadFlg == 0){ //настройки знака быстрой скорости из flash-памяти читаем один раз
        encoFlashMemData.fastSpdSignSetting  = getFastSpdSignFromFlash(); //текущая настройка знака быстрой аналоговой скорости
        vectorPhasingDataReadFlg = 1;
    }
    break;
  }
}


// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 


// = = = = = = СВАЛКА = = = = = = = = = = = = = = = = =
// = = = = = = СВАЛКА = = = = = = = = = = = = = = = = =
// = = = = = = СВАЛКА = = = = = = = = = = = = = = = = =
// = = = = = = СВАЛКА = = = = = = = = = = = = = = = = =
// Настройка аппаратного SPI для работы с 
// SPI_MISO - PB4 (SPI1_MISO), PB5 (SPI1_MISO)
// SPI_MOSI - PA7 (SPI1_MOSI)
// SPI_CLK  - PA5 (TIM2_CH1), PA2 (TIM2_CH3)  !!!!
// SPI_CS   - PA4 (SPI1_NSS)
void EndatTimInit (void)
{ // Таймер для опроса Endat
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
    
#define PERIOD 72*2
  
  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA and GPIOB clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
  
  // Основные ноги настриваем на ручное управление
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;

  // РА7 - выход данных
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  // РА4 - Направление передачи
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  // РB4 - вход данных
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  
  // Настройка Клока
  /* GPIOA Configuration: TIM2 CH1 (PA5 - CLK) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
    
  /* Connect TIM Channels to AF */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_1); // TIM2

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Init TIM_OCInitStructure */
  TIM_OCStructInit(&TIM_OCInitStructure);
  
  /* Output Compare Toggle Mode configuration: Channel1 */
  
  TIM_OCInitStructure.TIM_OCMode = TIM_ForcedAction_Active; //TIM_OCMode_Toggle; // Изначально ставим в 1.
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = PERIOD/2;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
  
  /* TIM enable counter */
//  TIM_Cmd(TIM2, ENABLE);

  /* TIM IT enable */
//  TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_Update, ENABLE);  
}


/**
  * @brief  Настройка модуля инкрементного энкодера для обработки sin/cos энкодера
  * @param  locEncoder: структура с данными энкодера
  * @retval 
  */


void sinCosDigitModeInit(encoBlockStatus *encoBlockPnt)
{
  GPIO_InitTypeDef      GPIO_InitStructure;            //!Структура для настройки портов
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      //!структура для настройки таймеров
  TIM_ICInitTypeDef  TIM_ICInitStructure;         //!Структура для настройки таймера
  NVIC_InitTypeDef NVIC_InitStructure;
  uint16_t encoInputSetting = 0;
  uint16_t PeriodValue;
  uint16_t pulseResolution;
  uint16_t PrescalerValue = 0;
  
  pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
  TIM_Cmd(TIM1, DISABLE); //Останов таймера TIM1
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  // тактирование порта А
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //!тактирование таймера TIM1
   
  //!Настройка пина PA8 в качестве счетного входа 1 таймера TIM1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //!Альтернативная функция
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //!Подтяжка
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  //!Подтяжка к нулю
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;       //!PA8
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  GPIO_PinAFConfig(GPIOA,  GPIO_PinSource8,  GPIO_AF_6); 
  
 
  /*Настройка пина PA9 в качестве счетного входа 2 таймера TIM1*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //!Альтернативная функция
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //!Подтяжка
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  //!Подтяжка к нулю
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;       //!PA9
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  GPIO_PinAFConfig(GPIOA,  GPIO_PinSource9,  GPIO_AF_6); 
  
  
   /*Вход PB7 используем для R-сигнала энкодера*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    //!Альтернативная функция
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //!Подтяжка
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; //!Подтяжка вниз
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;      //!PB7
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource7,  GPIO_AF_10); 
  
  /*Настройка режима захвата таймера TIM1 для контроля наличия сигналов A и B*/
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // Принимаем каждый захват
  TIM_ICInitStructure.TIM_ICFilter = /*10*/10;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  
  /*Настройка таймера TIM1 на режим модуля энкодера*/
  encoInputSetting = getEncoInputSettFromFlash(); //Настройка фазировки счетного входа из FLASH-памяти
  encoInputSetting = (encoInputSetting == 0) ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
  TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, encoInputSetting, TIM_ICPolarity_Rising); //Включение режима энкодера
  TIM_SetAutoreload(TIM1, (RESOLUTION_FACTOR * pulseResolution - 1)); //Настройка значения обновления таймера-счетчика модуля энкодера
  
  
  /*настройка таймера контроля R-сигнала**/ 
  /*Таймер нужен только для того чтоб вызвать прерывание по R-сигналу*/
  
  /*Глобальное разрешение прерывания от таймера TIM3*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);   //!тактирование таймера TIM3
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  PrescalerValue = 4;     //!делим частоту работы таймера на 4
  PeriodValue = (uint16_t)(SystemCoreClock * PrescalerValue / encoBlockPnt->encoProcessingPeriod) - 1;
  
  TIM_TimeBaseStructure.TIM_Period  = PeriodValue;           //!значение переполнения
  TIM_TimeBaseStructure.TIM_Prescaler  = PrescalerValue - 1; //! делитель 4
  TIM_TimeBaseStructure.TIM_ClockDivision  = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  
  /*Настройка режима захвата таймера TIM1 для обработки АВАРИЙ обрыва инкрементных линий*/
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //Канал №1:
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = encoInputSetting;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter = 5;    
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  TIM_ITConfig(TIM1, TIM_IT_CC1, /*ENABLE*/DISABLE);
  
  //Канал №2
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter = 5;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  TIM_ITConfig(TIM1, TIM_IT_CC2, /*ENABLE*/DISABLE);
  
  /*Настройка режима захвата таймера TIM3*/
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // Принимаем каждый захват
  TIM_ICInitStructure.TIM_ICFilter = 5;   
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  
  TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
  TIM_Cmd(TIM3, ENABLE); //Запуск таймера TIM3
  TIM_Cmd(TIM1, ENABLE); //Запуск таймера TIM1 
 
}


/**
  * @brief  PLL фильтрация фазы энкодера типа sin/cos
  * @param  Входной параметр - нефильтрованный рассчитанный угол
  * @retval Выходной параметр - фильтрованный угол
  */
#define F_PLL  15.0F
#define FLT_DIVIDE_CONST2  1 
#define DAMPING_COEF    0.9F 

float PLLphaseFilter(encoBlockStatus *encoBlockPnt, float ThetaElec){
  static float prevFltPhase = 0.0F;
  static double integrator1Output = 0;
  static double integrator2Output = 0;
  static double prevDivide2 = 0;
  double gain;
  double gainedPhase;
  double divide1;
  double divide2;
  double dThetaElec;
  double integrator2input;
  static float err = 0;
  static double prevIntegrator2input = 0;
  err = ThetaElec - prevFltPhase;
  err = (err > 0.5F) ? (err - 1.0F) : err;
  err = (err < -0.5F) ? (err + 1.0F) : err;
  gain = 2 * pi * F_PLL;
  gain = pow(gain, 2);
  dThetaElec = (double)err;
  gainedPhase = dThetaElec * gain;
  divide1 = gainedPhase * 2 * DAMPING_COEF / (2 * pi * F_PLL);
  divide2 = gainedPhase * FLT_DIVIDE_CONST2;
  integrator1Output+= (divide2 + prevDivide2) * encoBlockPnt->encoProcessingPeriod / 1000000 / 2;
  prevDivide2 = divide2;
  integrator2input = integrator1Output + divide1;
  integrator2Output += (integrator2input + prevIntegrator2input) * encoBlockPnt->encoProcessingPeriod / 1000000 / 2;
  prevIntegrator2input = integrator2input;
  integrator2Output = (integrator2Output > 0.5F) ? (integrator2Output - 1.0F) : integrator2Output;
  integrator2Output = (integrator2Output < -0.5F) ? (integrator2Output + 1.0F) : integrator2Output;
  prevFltPhase = integrator2Output + 0.5F;
  return(integrator2Output + 0.5F);
}

/**
  * @brief  Расчет механической фазы по абсолютным сигналам sin и cos
  * @param  encoBlockPnt - указатель на структуру энкодера с текущими значениями SIN и COS в отчетах АЦП
  * @retval ThetaMechPU - механическая фаза, полученная по сигналам SIN и COS
  */
float getAnalogThetaMechPU(encoBlockStatus *encoBlockPnt){
  float fPrecTheta;
  float fThetaMechCoarse;
  float fThetaMechCoarsePU;
  float minResolution;
  float fThetaMechFine;
  float fThetaMechFinePU;
  s16 fastSin, fastCos;
  s16 slowSin, slowCos;
  u16    numFinePeriod;    //!Целое число периодов инкрементных синусоид
  float ThetaMechPU;
  u16 encoResolution;
  
  fastSin = encoBlockPnt->analogSignals.fastSin;
  fastCos = encoBlockPnt->analogSignals.fastCos;
  slowSin = encoBlockPnt->analogSignals.slowSin;
  slowCos = encoBlockPnt->analogSignals.slowCos;
  encoResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
  
  //Расчет фазы внутри инкрементной (быстрой) синусоиды
   if(fastCos >= 0)
   {
     if (fastCos > 0)
     {
       fPrecTheta = atan((float)fastSin/fastCos);
     }
     else if ((fastCos == 0) && (fastSin > 0))
     {
       fPrecTheta = pi/2;
     }
     else if ((fastCos == 0) && (fastSin < 0))
     {
       fPrecTheta = -pi/2;
     }
     fPrecTheta += pi/2;
   }
   else
   {
     fPrecTheta = atan((float)fastSin/fastCos) + 3*pi/2;
   }
  
   //Расчет фазы внутри медленной синусоиды (расчет абсолютной механической позиции)//
   if(slowCos >= 0)
   {
     if (slowCos > 0)
     {
       fThetaMechCoarse = atan((float)slowSin/slowCos);
     }
     else if ((slowCos == 0) && (slowSin > 0))
     {
       fThetaMechCoarse = pi/2;
     }
     else if ((slowCos == 0) && (slowSin < 0))
     {
       fThetaMechCoarse = -pi/2;
     }
     fThetaMechCoarse += pi/2;
   }
   else
   {
     fThetaMechCoarse = atan((float)slowSin/slowCos) + 3*pi/2;
   }
   
   fThetaMechCoarsePU = fThetaMechCoarse/(2*pi);              //! Относительная грубая  составляющая угла в double
   minResolution = 1.0F/(encoResolution);                      //!часть оборота, соотв. одной полной инкрементной синусоиде
   numFinePeriod = (u16)(fThetaMechCoarsePU / minResolution); //!текущее целое количество быстрых синусоид
   fThetaMechCoarsePU = numFinePeriod * minResolution;        //!Относит. угол проворота, соотв. зафиксированному целому кол-ву инкрементных синусоид   
   fThetaMechFine = fPrecTheta/(encoResolution);              //!Абсолютная точная составляющая угла (рад.) в float (1 << encoder.BitResolution - количество быстрых синусоид на оборот)
   fThetaMechFinePU = fThetaMechFine/(2*pi);                  //!Относительная точная составляющая угла в float  
   ThetaMechPU = fThetaMechCoarsePU + fThetaMechFinePU;       //!Итоговое значение механической позиции
   return ThetaMechPU;
}

/**
  * @brief  Получение актуальной электрической фазы с учетом числа пар полюсов
  * @param  sPnt - указатель на структуру энкодера
  * @retval ThetaElec - электрическая фаза с учетом числа пар полюсов
  */

float getActualElectrPhase(float ThetaElec, encoBlockStatus *encoBlockPnt){
   uint8_t i;
   
   i = encoBlockPnt->baseEncoMotorData.polePairsNum; //Число пар полюсов
   while(i)                                          // Скручиваем лишнюю фазу
   {
      if ( ThetaElec > 1.0F){
        ThetaElec -= 1.0F; 
      }
      if (ThetaElec < -1.0F){
        ThetaElec += 1.0F;
      }
      i--;     
   }
   return ThetaElec;
}


/**
  * @brief  Медианный фильтр скорости
  * @param  velocity - текущая скорость
  * @retval fltVelocity - фильтрованная скорость
  */
#define CAPT_STRG_LEN 4                  //размер хранилища медианного фильтра
float velocityMedianFilter(float velocity){
   volatile u16 debugVal;
   u16 pnt;
   u16 pnt2;
   float tmp, fltVelocity;
   static float captureStrg[CAPT_STRG_LEN] = {0}; //хранилище значений таймера захвата
   static float sortedValStrg[CAPT_STRG_LEN] = {0};      //хранилище значений таймера захвата
   static SINCOSBUF sinCosIncr = {captureStrg, sortedValStrg, CAPT_STRG_LEN, 0}; //структура управления кольцевым буфером
   
  //Сохраняем в кольцевой буфер
   *(sinCosIncr.captureStrg + sinCosIncr.storagePos)= velocity;               //!текущее захваченное значение в кольцевой буфер
   sinCosIncr.storagePos=(sinCosIncr.storagePos+1)&(sinCosIncr.storageLen-1);  //!новое значение позиции в хранилище
   
   //Копируем захваченные значения в массив для сортировки
   memcpy(&sinCosIncr.sortedCaptStrg[0], &sinCosIncr.captureStrg[0], sizeof(sortedValStrg));
   
   //Пузырьковая сортировка массива значений скорости
   for(pnt = 0; pnt < sinCosIncr.storageLen; pnt++){
     for(pnt2 = 0; pnt2 < sinCosIncr.storageLen - 1 - pnt; pnt2++){
       if(sinCosIncr.sortedCaptStrg[pnt2] > sinCosIncr.sortedCaptStrg[pnt2+1]){
         tmp = sinCosIncr.sortedCaptStrg[pnt2];
         sinCosIncr.sortedCaptStrg[pnt2] = sinCosIncr.sortedCaptStrg[pnt2+1];
         sinCosIncr.sortedCaptStrg[pnt2+1] = tmp;
       }
     }
   }
   
   //фильтрованное значение таймера захвата
   fltVelocity = sinCosIncr.sortedCaptStrg[sinCosIncr.storageLen/2] + sinCosIncr.sortedCaptStrg[sinCosIncr.storageLen/2 - 1];
   fltVelocity =  fltVelocity / 2.0F;
   return fltVelocity;
}

/**
  * @brief  Инверсия счетного входа интерфейса энкодера
  * @param  
  * @retval 0 - если направление счета счетчика прямое, 1 - если обратное
  */

uint8_t encoderInterfacePhaseShift(void){
   uint32_t CCERState;
   CCERState = TIM1->CCER; //текущее состояние
   //TIM_Cmd(TIM1, DISABLE); //Останов таймера TIM1 
   if (CCERState & TIM_CCER_CC1P){//если инверсия входа включена, отключаем ее
     TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
     TIM_Cmd(TIM1, ENABLE); //Запуск таймера TIM1 
     return(0);
   }else{ //если инверсия входа отключена, включаем ее
     TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Rising);
     //TIM_Cmd(TIM1, ENABLE); //Запуск таймера TIM1 
     return(1);
   } 
}


/**
  * @brief  Функция расчета скорости по аналоговой относительной фазе
  * @param  analogThetaMechPU - текущее значение механической фазы
  * @param  encoBlockPnt - указатель на структуру энкодера
  * @retval 
  */
float analogSpdCalc(float analogThetaMechPU, encoBlockStatus *encoBlockPnt){
   u16 polePairsNum;
   u16 encoProcessingPeriod;
   float angleDiff, K;
   static float encoFltStrg[64]={0};           //!буфер для фильтрации  аналоговой фазы
   static ENCOFLT encoFlt = {encoFltStrg, 32, 0, 0, 0};  //!структура с данными для расчета скорости по аналоговой фазе
   
   encoProcessingPeriod = encoBlockPnt->encoProcessingPeriod;
   polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
   encoFlt.storageLen = 64/*encoder->EncoFlt_N*/;
   angleDiff = analogThetaMechPU - (*(encoFlt.thetaStrg + encoFlt.storagePos)); //!Приращение аналогового угла
   *(encoFlt.thetaStrg + encoFlt.storagePos)= analogThetaMechPU;            //!сохраняем вычисленный угол
   encoFlt.storagePos=(encoFlt.storagePos+1)&(encoFlt.storageLen-1);        //!Новое значение позиции в хранилище аналоговой фазы
   // пересчет перехода через единицу
   if(angleDiff < -0.8F) {
     angleDiff += 1.0F;
   }
   if(angleDiff > 0.8F) {//при реверсе
      angleDiff -= 1.0F;
   }
   encoFlt.TmpFltr = encoFlt.TmpFltr + ((angleDiff - encoFlt.TmpFltr) * 0.125F);
     // Рассчитываем электрическую скорость с учетом пар полюсов.
   K = (float)polePairsNum * 1000000.0F / (encoProcessingPeriod * FREQ_BASE * encoFlt.storageLen); 
   encoFlt.fltSpeed = K * encoFlt.TmpFltr;
   return(encoFlt.fltSpeed); //скорость по аналоговой фазе
}


/**
  * @brief  Расчет скорости по дискретной относительной фазе
  * @param  текущее значение механической дискретной фазы
  * @param  указатель на структуру энкодера
  * @retval скорость по инкрементной фазе
  */
float discrSpdCalc(float discrThetaMechPU, encoBlockStatus *encoBlockPnt){
   float angleDiff;
   u16 polePairsNum;
   u16 encoProcessingPeriod;
   static float K = 0;
   static float encoFltStrg[64]={0};           //!буфер для фильтрации  аналоговой фазы
   static ENCOFLT encoFlt = {encoFltStrg, 32, 0, 0, 0};  //!структура с данными для расчета скорости по аналоговой фазе
   
   encoProcessingPeriod = encoBlockPnt->encoProcessingPeriod;
   polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
   encoFlt.storageLen = encoBlockPnt->fltStrgLen;                              //!Размер хранилища фильтра фазы
   angleDiff = discrThetaMechPU - (*(encoFlt.thetaStrg + encoFlt.storagePos)); //!Приращение угла
   *(encoFlt.thetaStrg + encoFlt.storagePos)= discrThetaMechPU;                //!сохраняем вычисленный угол
   encoFlt.storagePos=(encoFlt.storagePos+1)&(encoFlt.storageLen-1);           //!Новое значение угла в хранилище фазы
   // пересчет перехода через единицу
   if(angleDiff < -0.8F) {
     angleDiff += 1.0F;
   }
   if(angleDiff > 0.8F) {//при реверсе
      angleDiff -= 1.0F;
   }
   encoFlt.TmpFltr = encoFlt.TmpFltr +((angleDiff - encoFlt.TmpFltr) * 0.125F);
     // Рассчитываем электрическую скорость с учетом пар полюсов.
   if(encoBlockPnt->PWMOn == 0){
     K = (float)polePairsNum * 1000000.0F / (encoProcessingPeriod * FREQ_BASE * encoFlt.storageLen); 
   }
   encoFlt.fltSpeed = K * encoFlt.TmpFltr;
   return(encoFlt.fltSpeed); //скорость по дискретной фазе
}

/**
  * @brief  Расчет скорости по дискретной относительной фазе для инкрементального энкодера
  * @param  текущее значение механической дискретной фазы
  * @param  указатель на структуру энкодера
  * @retval скорость по инкрементной фазе
  */
float discrSpdIncrCalc(float discrThetaMechPU, encoBlockStatus *encoBlockPnt)
{
   float angleDiff;
   u16 polePairsNum;
   u16 encoProcessingPeriod;
   static float K = 0;
   static float encoFltStrg[64]={0};           //!буфер для фильтрации  аналоговой фазы
   static ENCOFLT encoFlt = {encoFltStrg, 32, 0, 0, 0};  //!структура с данными для расчета скорости по аналоговой фазе
   
   encoProcessingPeriod = encoBlockPnt->encoProcessingPeriod;                  //Период расчета данных энкодера
   polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
   encoFlt.storageLen = encoBlockPnt->fltStrgLen;                              //Размер хранилища фильтра фазы
   angleDiff = discrThetaMechPU - (*(encoFlt.thetaStrg + encoFlt.storagePos)); //Приращение угла
   *(encoFlt.thetaStrg + encoFlt.storagePos)= discrThetaMechPU;                //сохраняем вычисленный угол
   encoFlt.storagePos=(encoFlt.storagePos+1)&(encoFlt.storageLen-1);           //Новое значение угла в хранилище фазы
   // пересчет перехода через единицу
   if(angleDiff < -0.8F) {
     angleDiff += 1.0F;
   }
   if(angleDiff > 0.8F) {//при реверсе
      angleDiff -= 1.0F;
   }
   encoFlt.TmpFltr = encoFlt.TmpFltr +((angleDiff - encoFlt.TmpFltr) * 0.125F);
     // Рассчитываем электрическую скорость с учетом пар полюсов.
   if(encoBlockPnt->PWMOn == 0){
     K = (float)polePairsNum * 1000000.0F / (encoProcessingPeriod * FREQ_BASE * encoFlt.storageLen); 
   }
   encoFlt.fltSpeed = K * encoFlt.TmpFltr;
   return(encoFlt.fltSpeed); //скорость по дискретной фазе  
}

/**
  * @brief  Расчет фазы по инкрементным сигналам
  * @param  указатель на структуру энкодера
  * @retval текущая дискретная (инкрементная)фаза
  */
float getDiscrThetaMechPU(encoBlockStatus *encoBlockPnt)
{
  uint32_t incrementPosCnt;
  u16 encoderResolution;
  encoBlockType blockType;
  float ThetaMechPU;

  blockType = encoBlockPnt->baseEncoMotorData.blockType;
  if(blockType == SIN_COS){
    encoderResolution = RESOLUTION_FACTOR * encoBlockPnt->baseEncoMotorData.pulseResolution;
    incrementPosCnt = encoBlockPnt->incrEncoPos; //текущее значение счетчика модуля энкодера
    incrementPosCnt &= (encoderResolution - 1); //маскирование счетчика
  }else if(blockType == INCREMENTAL){
    encoderResolution = RESOLUTION_FACTOR * encoBlockPnt->baseEncoMotorData.pulseResolution;
    incrementPosCnt = encoBlockPnt->incrEncoPos; //текущее значение счетчика модуля энкодера
    incrementPosCnt = (incrementPosCnt > (encoderResolution - 1)) ? (encoderResolution - 1) : incrementPosCnt;
  }else{ //Serial
    encoderResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
    incrementPosCnt = encoBlockPnt->incrEncoPos; //текущее значение счетчика модуля энкодера
    incrementPosCnt &= (encoderResolution - 1); //маскирование счетчика
  }
  
  ThetaMechPU = (float)incrementPosCnt / encoderResolution; //расчет мех. угла по значению счетчика
  return(ThetaMechPU);
}



/**
  * @brief  Проверка события прерывания от R-сигнала
  * @param  
  * @retval флаг поступления референтного сигнала
  */
extern u16 R_Event; //Флаг события R-сигнала
u16 RsignalEventPresent(){
  return R_Event;
}

/**
  * @brief  Сброс флага события прерывания от R-сигнала
  * @param  
  * @retval
  */
void RsignalEventAck(){
  R_Event = 0;
}

/**
  * @brief  Устранение переполнения фазы из-за прибавления смещения
  * @param  shiftPhase - текущий угол со смещением
  * @retval Фазовый угол в диапазоне 0...2pi
  */
float removeExcessPhase(float phaseShift)
{
     while(phaseShift < 0.0F){
       phaseShift += 1.0F;
     }
  
     while(phaseShift >= 1.0F){  
       phaseShift -= 1.0F;
     }
     
   return phaseShift;
}


/**
  * @brief  Проверка текущего аналогового угла на соответствие диапазону 
  * @param shiftPhase текущий проверяемый фазовый угол
  * @param compareAngle значение для сравнения
  * @retval результат проверки
  */
/*
#define DELTA_PHASE 46603L //IQ(1/360) - максимально возможное отклонение вычисленной фазы
uint8_t phaseRangCheck(_iq shiftPhase, _iq compareAngle){
  if((shiftPhase >= (compareAngle - DELTA_PHASE)) && (shiftPhase <= (compareAngle + DELTA_PHASE)))
}
*/

uint8_t getEncoInputSettFromFlash(){
  _iq inputSetting;
  inputSetting = flashInputSet_read(BASE_PAGE_ADDR); //текущее значение из FLASH
  if((inputSetting == NO_INIT_VAL) || (inputSetting == 0)){ //если FLASH-память не инициализирована..
    return(0); 
  }else{
    return(1);
  }
}



/**
  * @brief  Проверка отсутствия предварительной фазировки энкодера и коррекция счетчика цифровой фазы в режиме векторного управления
  * @param encoStatus - текущий статус аварии энкодера
  * @param encoder - указатель на структуру описателя энкодеров
  * @param phaseAdd - корректирующий фазовый сдвиг, полученный во время низкоуровнего автофазирования 
  * @retval флаг наличия аварии "Нет фазировки энкодера"
  */

#define UNDER_VAL  0.98F
#define OVER_VAL   0.02F
#define ALLOW_R_PERIOD (8 * 1000U / encoBlockPnt->encoProcessingPeriod) //8 мс
#define R_HOLD_TIME    (12 * 1000U / encoBlockPnt->encoProcessingPeriod) //12 мс

uint8_t checkSinCosPhasingMiss(uint16_t encoStatus, encoBlockStatus *encoBlockPnt, float phaseAdd){
  static u16 RtimeEn = 0;
  static u16 RStateCnt = 0;
  static s16 TIM1Val = 0;
  u16 underCntVal, overCntVal;
  s16 tmp;
  u16 dif;
  uint16_t errStatus = encoStatus;
  static uint16_t R_SignalErrStatus = 0;
  float shiftPhase;
  static float phaseVal1 = 0;
  static float phaseVal2 = 0;
  u16 pulseResolution;
  
  pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
   /*-------------------------------------------------------------------------------*/
   /*                Контроль фазового сдвига в момент поступления R-метки          */
   /*-------------------------------------------------------------------------------*/
   phaseVal1 = (encoBlockPnt->PWMOn == 0) ? UNDER_VAL : phaseVal1;
   phaseVal2 = (encoBlockPnt->PWMOn == 0) ? OVER_VAL : phaseVal2;
   R_SignalErrStatus = (encoBlockPnt->PWMOn == 0) ? ENCO_OK : R_SignalErrStatus;
  
   underCntVal = (u16)(UNDER_VAL * RESOLUTION_FACTOR * pulseResolution); //допустимое отклонение счетчика слева от 360 град (8192 имп)
   overCntVal  = (u16)(OVER_VAL * RESOLUTION_FACTOR * pulseResolution);  //допустимое отклонение счетчика справа от 360 град (8192 имп)
  
   RtimeEn = (RtimeEn > 0) ? (RtimeEn - 1) : RtimeEn; //таймер задающий допустимое время повторного поступления R-сигнала

   
   if(RsignalEventPresent()){//если получен R-сигнал
     if(RtimeEn == 0){ //если R-сигнал поступил через допустимое время
       RtimeEn = ALLOW_R_PERIOD;
       RStateCnt = R_HOLD_TIME;
       encoBlockPnt->RsignalFlg = 1; //для логгера
       RsignalEventAck(); //Сброс флага прерывания от референтного сигнала  
       TIM1Val = TIM_GetCounter(TIM1); //Текущее значение счетчика в момент поступления R-метки
       shiftPhase = getAnalogThetaMechPU(encoBlockPnt); //текущий фазовый сдвиг в момент R-прерывания
       shiftPhase += phaseAdd; //учет дополнительного смещения
       shiftPhase = removeExcessPhase(shiftPhase);//контроль допустимого диапазона фазы после прибавки смещения
       if(!(((shiftPhase >= phaseVal1) && (shiftPhase < 1.0F)) || ((shiftPhase >= 0.0F) && (shiftPhase < phaseVal2)))){
         if((shiftPhase >= (0.5F - DELTA_PHASE)) && (shiftPhase <= (0.5F + DELTA_PHASE))){ //сдвиг на +180 градусов
            R_SignalErrStatus = INCORRECT_ANGLE_AT_R_ERR;
         }else if ((shiftPhase >= (0.75F - DELTA_PHASE)) && (shiftPhase <= (0.75F+ DELTA_PHASE))){ //сдвиг на 90 град
            R_SignalErrStatus = INCORRECT_ANGLE_AT_R_ERR;
         }else if ((shiftPhase >= (0.25F - DELTA_PHASE)) && (shiftPhase <= (0.25F + DELTA_PHASE))) {  //сдвиг на 90 град
            R_SignalErrStatus = INCORRECT_ANGLE_AT_R_ERR;
         }  
       }
     }else{
       RsignalEventAck();
     }
   
     //-----------Контроль зашумленности инкрементных сигналов------------//
     if((TIM1Val >= underCntVal) && (TIM1Val < RESOLUTION_FACTOR * pulseResolution)){ //Допустимый недосчет счетчика
        ;
     }else if((TIM1Val >= 0) && (TIM1Val < overCntVal)){                                    //Допустимый пересчет счетчика
       ;
     }else{ //значение цифровой фазы сильно отклонено от абсолютного нуля
       R_SignalErrStatus = (R_SignalErrStatus != INCORRECT_ANGLE_AT_R_ERR) ? INCORRECT_INCR_ABS_PHASE_DIFF_ERR : R_SignalErrStatus;
     } 
     
   }else{
     RStateCnt = (RStateCnt != 0) ? (RStateCnt - 1) : RStateCnt; //выдержка перед обнулением флага R-сигнала
     encoBlockPnt->RsignalFlg = (RStateCnt == 0) ? 0 : encoBlockPnt->RsignalFlg;
   }
   
   /*-------------------------------------------------------------------------------*/
   /*                Коррекция счетчика цифровой фазы                               */
   /*-------------------------------------------------------------------------------*/
   if((!encoBlockPnt->PWMOn) && (TIM1Val != 0)){ //ШИМ отключен и была сдвижка счетчика в момент R-метки
     if((TIM1Val >= underCntVal) && (TIM1Val < RESOLUTION_FACTOR * pulseResolution)){ //если недосчет счетчика
        dif = RESOLUTION_FACTOR * pulseResolution - TIM1Val; //8192 - TIM1Val
        tmp = TIM_GetCounter(TIM1);
        tmp = tmp + dif;
        tmp &= (RESOLUTION_FACTOR * pulseResolution - 1);
        TIM_SetCounter(TIM1, tmp);  
        TIM1Val = 0; //Сдвижка учтена
     }else if((TIM1Val > 0) && (TIM1Val < overCntVal)){ //если пересчет счетчика
        tmp = TIM_GetCounter(TIM1);
        tmp -= TIM1Val;
        tmp = (tmp < 0) ? tmp + RESOLUTION_FACTOR * pulseResolution : tmp;
        tmp &= (RESOLUTION_FACTOR * pulseResolution - 1);
        TIM_SetCounter(TIM1, tmp);
        TIM1Val = 0; //Сдвижка учтена
     }
   }
   errStatus = (R_SignalErrStatus != 0) ? R_SignalErrStatus : errStatus;
   return errStatus; 
}


/**
  * @brief  Проверка наличия противофазы между цифровой и аналоговой фазами,
  *         инверсия цифрового сигнала инкрементной синусоиды.
  * @param fltAnalogSpeed - скорость, рассчитанная по абсолютным сигналам sin и cos
  * @param fltDiscrSpeed - скорость, рассчитанная по цифровому сигналу от инкременных синусоид
  * @param pLocNvData - указатель на объект с энергонезависимыми данными низкоуровнего фазирования
  * @retval флаг завершенной синхронизации скоростей
  */


#define MIN_PHASING_SPD (0.2F / FREQ_BASE)
#define ANALOG_SPD_FLT_TIME  ((uint16_t)(2 * 100000UL / encoBlockPnt->encoProcessingPeriod)) // 0.2 сек

u16 digitalPhaseInversion(encoBlockStatus *encoBlockPnt, float fltAnalogSpeed, float fltDiscrSpeed, encoFlashMemDataType *pLocNvData)
{
  
   static u16 spdSynhrCnt1 = 0; //счетчик состояния fltAnalogSpeed > 0 и fltDiscrSpeed < 0
   static u16 spdSynhrCnt2 = 0; //счетчик состояния fltAnalogSpeed < 0 и fltDiscrSpeed > 0
   static u16 spdSynhrCnt3 = 0; //счетчик состояния fltAnalogSpeed < 0 и fltDiscrSpeed < 0
   static u16 spdSynhrCnt4 = 0; //счетчик состояния fltAnalogSpeed > 0 и fltDiscrSpeed > 0
   u16 digitSpdSign = NOT_DETECT;
   
   if(encoBlockPnt->drvMode == VECTOR_MODE){
     return NOT_DETECT; //В векторном режиме автофазирование не выполняется
   }
   /*Сброс счетчиков состояний при выключенном ШИМ*/
   spdSynhrCnt1 = (encoBlockPnt->PWMOn == 0) ? 0 : spdSynhrCnt1;
   spdSynhrCnt2 = (encoBlockPnt->PWMOn == 0) ? 0 : spdSynhrCnt2;
   spdSynhrCnt3 = (encoBlockPnt->PWMOn == 0) ? 0 : spdSynhrCnt3;
   spdSynhrCnt4 = (encoBlockPnt->PWMOn == 0) ? 0 : spdSynhrCnt4;
   
  if((fabsf(fltAnalogSpeed) >= MIN_PHASING_SPD) && (fabsf(fltDiscrSpeed) >= MIN_PHASING_SPD)){ //Проверка знаков после 0,2 Гц
       if((fltAnalogSpeed > 0) && (fltDiscrSpeed < 0)){ //если знаки аналоговой и цифровой скорости отличаются
           spdSynhrCnt2 = (spdSynhrCnt2 > 0) ? --spdSynhrCnt2 : spdSynhrCnt2;
           spdSynhrCnt3 = (spdSynhrCnt3 > 0) ? --spdSynhrCnt3 : spdSynhrCnt3;
           spdSynhrCnt4 = (spdSynhrCnt4 > 0) ? --spdSynhrCnt4 : spdSynhrCnt4;
           if(spdSynhrCnt1 >= ANALOG_SPD_FLT_TIME){
             pLocNvData->encoInputSetting = encoderInterfacePhaseShift(); //инвертируем вход интерфейса энкодера
             nonVolatileDataFlashWrite(pLocNvData); //Настройки инкрементного входа пишем во FLASH
           }else{
             spdSynhrCnt1++;
           }
        }else if((fltAnalogSpeed < 0)&&(fltDiscrSpeed > 0)){
           spdSynhrCnt1 = (spdSynhrCnt1 > 0) ? --spdSynhrCnt1 : spdSynhrCnt1;
           spdSynhrCnt3 = (spdSynhrCnt3 > 0) ? --spdSynhrCnt3 : spdSynhrCnt3;
           spdSynhrCnt4 = (spdSynhrCnt4 > 0) ? --spdSynhrCnt4 : spdSynhrCnt4;
           if(spdSynhrCnt2 >= ANALOG_SPD_FLT_TIME){
             pLocNvData->encoInputSetting = encoderInterfacePhaseShift(); //инвертируем вход интерфейса энкодера
             nonVolatileDataFlashWrite(pLocNvData);
           }else{
             spdSynhrCnt2++;
           }
        }else if((fltAnalogSpeed < 0) && (fltDiscrSpeed < 0)){
           spdSynhrCnt1 = (spdSynhrCnt1 > 0) ? --spdSynhrCnt1 : spdSynhrCnt1;
           spdSynhrCnt2 = (spdSynhrCnt2 > 0) ? --spdSynhrCnt2 : spdSynhrCnt2;
           spdSynhrCnt4 = (spdSynhrCnt4 > 0) ? --spdSynhrCnt4 : spdSynhrCnt4;
           if(spdSynhrCnt3 >= ANALOG_SPD_FLT_TIME){
             digitSpdSign = NEGATIVE;
           }else{
             spdSynhrCnt3++;
           }
        }else if((fltAnalogSpeed > 0) && (fltDiscrSpeed > 0)){
           spdSynhrCnt1 = (spdSynhrCnt1 > 0) ? --spdSynhrCnt1 : spdSynhrCnt1;
           spdSynhrCnt2 = (spdSynhrCnt2 > 0) ? --spdSynhrCnt2 : spdSynhrCnt2;
           spdSynhrCnt3 = (spdSynhrCnt3 > 0) ? --spdSynhrCnt3 : spdSynhrCnt3;
           if(spdSynhrCnt4 >= ANALOG_SPD_FLT_TIME){
             digitSpdSign = POSITIVE;
           }else{
             spdSynhrCnt4++;
           }
        }
  }else{
    digitSpdSign = NOT_DETECT;
  }
  return digitSpdSign;
}


/**
  * @brief  Запись корректирующего фазового сдвига в FLASH-память
  * @param 
  * @retval
  */
void nonVolatileDataFlashWrite(encoFlashMemDataType *encoFlashMemData){
  flash_unlock();//разблокирование FLASH-памяти перед записью
  flash_erase_page(BASE_PAGE_ADDR); //стирание страницы памяти перед записью
  flash_write(BASE_PAGE_ADDR, encoFlashMemData); //запись
  flash_lock(); //блокировка памяти
}


/**
  * @brief  Проверка наличия фазового сдвига между фазой, рассчитанной по сигналам C и D, и R-импульсом.
  *         Определение корректирующего фазового сдвига для рассчитанной фазы.   
  * @param pLocNvData - указатель на структуру энергонезависимых данных
  * @retval 
  */
void phaseShiftDetect(encoBlockStatus *encoBlockPnt, encoFlashMemDataType *pLocNvData){
   static u16  RtimeEn  = 0;
   static u16 RStateCnt = 0;
   static u16 RSignalDelayCnt = 0;
   u16 RSignalDelay;  //задержка на обработку прерывания по R-сигналу при подаче питания
   u16 pulseResolution;
   float shiftPhase;
   uint16_t tmpcr1;
   volatile uint16_t cntVal; //отладочная переменная. Текущее значение счетчика фазы
   
   pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
   if(encoBlockPnt->drvMode == VECTOR_MODE){
     return; //В векторном режиме синхронизация фаз не выполняется
   }
   
   //Задержка на обработку R-сигнала при подаче питания
   RSignalDelay = (u16)(R_PROCESSING_DELAY * 1000000.0F / encoBlockPnt->encoProcessingPeriod + 0.5F);
   if(RSignalDelayCnt < RSignalDelay){
     RSignalDelayCnt++;
     RsignalEventAck();   //Сброс флага прерывания от референтного сигнала
     return;
   }
     
     //угол берем из прерывания по R-сигналу
   if(RtimeEn > 0) {RtimeEn--;} //счетчик тактов. Задает интервал, в течении которого R-сигнал игнорируется
   if(RsignalEventPresent()){//если получен R-сигнал
     if(RtimeEn == 0){ //если R-сигнал поступил через допустимое время
       RtimeEn = 20;   //повторное поступление R-сигнала игнрируется в течении 20 тактов
       RStateCnt = 500;     //R-сигнал удерживается для логгера 500 тактов
       encoBlockPnt->RsignalFlg = 1; //для логгера
       RsignalEventAck();   //Сброс флага прерывания от референтного сигнала
       cntVal = TIM_GetCounter(TIM1);
       
       //проверяем направление счета
       tmpcr1 = TIM1->CR1;
       if(tmpcr1 & (1 << DIR_SIGN)){ //если направление отрицательное
       TIM_SetCounter(TIM1, RESOLUTION_FACTOR * pulseResolution - 1);
       }else{  //если направление положительное
         TIM_SetCounter(TIM1, 0);
       }
       shiftPhase = getAnalogThetaMechPU(encoBlockPnt); //текущий фазовый сдвиг в момент R-прерывания
       shiftPhase += pLocNvData->phaseShift; //учет дополнительного корректирующего смещения
       shiftPhase = removeExcessPhase(shiftPhase);//контроль допустимого диапазона фазы после прибавки корректирующего смещения
       if((shiftPhase >= (0.5F - DELTA_PHASE)) && (shiftPhase <= (0.5F + DELTA_PHASE))){ //сдвиг на +180 градусов
         pLocNvData->phaseShift += 0.5F; //сдвиг корректирующей фазы
         pLocNvData->phaseShift = removeExcessPhase(pLocNvData->phaseShift);//контроль допустимого диапазона фазы
         nonVolatileDataFlashWrite(pLocNvData);
       }else if ((shiftPhase >= (0.75F - DELTA_PHASE)) && (shiftPhase <= (0.75F + DELTA_PHASE))){ //сдвиг на 90 град
         pLocNvData->phaseShift += 0.25F;
         pLocNvData->phaseShift = removeExcessPhase(pLocNvData->phaseShift);//контроль допустимого диапазона фазы
         nonVolatileDataFlashWrite(pLocNvData);
       }else if ((shiftPhase >= (0.25F - DELTA_PHASE)) && (shiftPhase <= (0.25F + DELTA_PHASE))) {  //сдвиг на 90 град
         pLocNvData->phaseShift += 0.75F;
         pLocNvData->phaseShift = removeExcessPhase(pLocNvData->phaseShift);//контроль допустимого диапазона фазы
         nonVolatileDataFlashWrite(pLocNvData);
       }else{ //иначе фазового сдвига нет, все норм
         //encoder.phasingByRSignal = 1; //флаг завершения этапа фазировки по референтной метке
       }
     }else{
       RsignalEventAck();
     }

   }else{
     if(RStateCnt != 0){
       RStateCnt--; //задержка на обнуление R-сигнала для логгера
     }else{
       encoBlockPnt->RsignalFlg = 0; // обнуляем флаг R-сигнала
     }
   }
}


float  fastSinCosSignalSpdCalc(encoBlockStatus *encoBlockPnt){
     u16 polePairsNum;
     u16 encoProcessingPeriod;
     u16 pulseResolution;
     float relThetaMechFine;
     static float K1 = 0.0F;
     float angleDiff;
     float fastSinSpd;
     float fPrecTheta;
     float fRelThetaMechFine;
     static float encoFltStrgFastSin[64]={0};
     static ENCOFLT encoFltFastSin = {&encoFltStrgFastSin[0], 32, 0, 0, 0};  //!структура с данными для расчета скорости

     polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
     encoProcessingPeriod = encoBlockPnt->encoProcessingPeriod;
     pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
     encoFltFastSin.storageLen = FAST_SPD_STRG_LEN; // Буфер только на 2 отсчета
     fPrecTheta = precThetaCalc(encoBlockPnt);      // Расчет позиции внутри быстрой синусоиды
     fRelThetaMechFine = fPrecTheta / (2*pi);       // Относительный угол внутри быстрой синусоиды
     relThetaMechFine = fRelThetaMechFine;          // Относительный угол внутри быстрой синусоиды
     
     angleDiff = relThetaMechFine - (*(encoFltFastSin.thetaStrg + encoFltFastSin.storagePos));   //!Приращение угла внутри быстрой синусоиды
     *(encoFltFastSin.thetaStrg + encoFltFastSin.storagePos)= relThetaMechFine;              //!сохраняем вычисленный угол внутри быстрой синусоиды
     encoFltFastSin.storagePos=(encoFltFastSin.storagePos+1)&(encoFltFastSin.storageLen-1);  //!Формируем новое значение позиции в хранилище

     // пересчет перехода через единицу
     if(angleDiff < -0.75F) {
          angleDiff += 1.0F;
     }
     if(angleDiff > 0.75F) {
          angleDiff -= 1.0F;
     }
     
     if(encoBlockPnt->PWMOn == 0){
       K1 = 1000000.0F * (float)(polePairsNum) / encoProcessingPeriod / encoFltFastSin.storageLen / pulseResolution / FREQ_BASE;
     }
     // Подфильтровать приращение мехнической фазы перед расчетом скорости
     encoFltFastSin.TmpFltr = angleDiff; //encoFltFastSin.TmpFltr +((angleDiff - encoFltFastSin.TmpFltr) * 0.125F);
     fastSinSpd = K1 * encoFltFastSin.TmpFltr;   //скорость внутри быстрой синусоиды
     encoFltFastSin.fltSpeed = fastSinSpd;
     
     return(encoFltFastSin.fltSpeed);
}

/**
  * @brief  Расчет позиции внутри быстрой синусоиды
  * @param 
  * @retval 
  */
double precThetaCalc(encoBlockStatus *encoBlockPnt)
{
  
  double fPrecTheta;
  s16 fastSin;
  s16 fastCos;
  fastSin = encoBlockPnt->analogSignals.fastSin;
  fastCos = encoBlockPnt->analogSignals.fastCos;
    
   if(fastCos >= 0){
     if (fastCos > 0){
       fPrecTheta = atan((double)fastSin/fastCos);
     }
     else if ((fastCos == 0) && (fastSin > 0)){
       fPrecTheta = pi/2;
     }
     else if ((fastCos == 0) && (fastSin < 0)){
       fPrecTheta = -pi/2;
     }
     fPrecTheta += pi/2;
   }else{
     fPrecTheta = atan((double)fastSin/fastCos) + 3*pi/2;
   }
   return fPrecTheta; //угол внутри быстрой синусоиды
}


/**
  * @brief  Считывание из Flash-памяти текущего знака быстрой аналоговой скорости
  * @param  
  * @retval текущее значение фазового сдвига
  */
s32 getFastSpdSignFromFlash(){
  s32 fastSpdSign;
  fastSpdSign = flash_FastSpdSignRead( BASE_PAGE_ADDR ); //текущее значение из FLASH
  if ((fastSpdSign == -1) || (fastSpdSign == 1)){
     return(fastSpdSign);
  }else{
    return(1);
  }
}

/**
  * @brief  Синхронизация скорости, вычисленной по аналоговым сигналам с цифровой скоростью
  * @param digatalSpd - скорость измеренная по инкрементным синусоидам, преобразованным в меандр (по пиле)
  * @param fastAnalogSpd - скорость, измеренная по аналоговым инкрементым синусоидам
  * @param digitalInputInitFlg - флаг завершенной синхронизации аналоговой пилы и цифровой
  * @retval  
  */
void fastSinCosSpdSignPhasing(encoBlockStatus *encoBlockPnt, float digatalSpd, float fastAnalogSpd, encoFlashMemDataType *pLocNvData, s16 digitSpdSignDetectFlg){
  static float spdModeSwitchVal = 0;
  static float minSpdModeSwitchVal;
  static u16 state1Cnt = 0;
  static u16 state2Cnt = 0;
  
  if(encoBlockPnt->drvMode == VECTOR_MODE){ //синхронизация быстрой и цифровой скоростей в векторном режиме не выполняется. Пока что
    return;
  }
  
  if(encoBlockPnt->PWMOn == 0){
    spdModeSwitchVal = spdModeValCalc(encoBlockPnt);
    minSpdModeSwitchVal = 0.1F / FREQ_BASE;
    state1Cnt = 0;
    state2Cnt = 0;
  }else{
      if(digitSpdSignDetectFlg == NOT_DETECT){ //digitSpdSignDetectFlg = -1 для энкодера типа enDat
        return;
      }else if ((fabsf(digatalSpd) > minSpdModeSwitchVal) && (fabsf(digatalSpd) < spdModeSwitchVal)){ //синхронизация знаков быстрой аналоговой скорости и цифровой скорости
        if( (digatalSpd > 0) && (fastAnalogSpd < 0) ){
          state2Cnt = (state2Cnt != 0) ? (state2Cnt - 1) : state2Cnt;
          
          ++state1Cnt;
          if(state1Cnt == 800){
            pLocNvData->fastSpdSignSetting *= -1; //Приводим быструю аналоговую и цифровую скорости к одному знаку
            nonVolatileDataFlashWrite(pLocNvData); //запись знака быстрой аналоговой скорости в Flash
          }
        }else if ( (digatalSpd < 0) && (fastAnalogSpd > 0) ){
          state1Cnt = (state1Cnt != 0) ? (state1Cnt - 1) : state1Cnt;
          
          ++state2Cnt;
          if(state2Cnt == 800){
             pLocNvData->fastSpdSignSetting *= -1; //Приводим быструю аналоговую и цифровую скорости к одному знаку
             nonVolatileDataFlashWrite(pLocNvData); //запись знака быстрой аналоговой скорости в Flash
          }
        }else{
          pLocNvData->fastSpdSignSetting *= 1;  //Знаки цифровой и быстрой аналоговой скорости совпадают. Оставляем коэф. без изменения
        }
      }else{
        state1Cnt = (state1Cnt != 0) ? (state1Cnt - 1) : state1Cnt;
        state2Cnt = (state2Cnt != 0) ? (state2Cnt - 1) : state2Cnt;
      }
  }
}


/**
  * @brief  Переключение между режимами измерения скорости
  * @param encoder - указатель на структуру описателя энкодера
  * @param disrSpd - скорость, измеренная по цифровой фазе
  * @param fastAnalogSpd - скрость измеренная по аналоговым инкрементым сигналам
  * @retval скорость, соответствующая активному режиму измерения скорости 
  */
#define SPD_MODE_HYST (0.2F/FREQ_BASE) //гистерезис при переходе из второго режима в первый
float spdCalcModeValSelect(encoBlockStatus *encoBlockPnt, float disrSpd, float fastAnalogSpd )
{
    float spdRetVal;
    float mode2toMode1Val;
    static float spdModeSwitchVal = 0.0F;
    static spdCalcModeType  spdCalcMode = ANALOG_FAST_MODE; //Изначально режим быстрых синусоид
    
    if(encoBlockPnt->PWMOn == RESET){ //Расчет частоты переключения режимов скорости
      spdModeSwitchVal = spdModeValCalc(encoBlockPnt);
      mode2toMode1Val = spdModeSwitchVal - SPD_MODE_HYST; //частота возврата в первый режим с учетом гистерезиса
    }

    switch (spdCalcMode){
    case ANALOG_FAST_MODE:
      spdRetVal = fastAnalogSpd; 
      if ( fabsf(disrSpd) >= spdModeSwitchVal ) {
          spdCalcMode = DIGITAL_MODE;                                         
      }
      break;
    default: //DIGITAL_MODE
      spdRetVal = disrSpd; 
      if ( fabsf(disrSpd) <= mode2toMode1Val ) {
         spdCalcMode = ANALOG_FAST_MODE;                                         
      }
      break;
    }
   
    return(spdRetVal);
}


/**
  * @brief  Вычисление частоты переключения режимов измерения скорости
  * @param  encoBlockPnt - указатель на структуру описания энкодера
  * @retval величина частоты переключения режимов 
  */
float spdModeValCalc(encoBlockStatus *encoBlockPnt){
  u16 polePairsNum;
  u16 encoResolution;
  u16 encoProcessingPeriod;
  encoBlockType blockType;
  float spdModeSwitchVal;
 
  polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
  encoResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
  blockType = encoBlockPnt->baseEncoMotorData.blockType;
  encoProcessingPeriod = encoBlockPnt->encoProcessingPeriod;
  if(blockType == SIN_COS){
    spdModeSwitchVal = (float)polePairsNum * 1000000.0F / FREQ_BASE / encoResolution / RESOLUTION_FACTOR / encoProcessingPeriod / FAST_SPD_STRG_LEN; 
  }else{
    spdModeSwitchVal = (float)polePairsNum * 1000000.0F / FREQ_BASE / encoResolution / encoProcessingPeriod / FAST_SPD_STRG_LEN; 
  }
  spdModeSwitchVal *= 0.7F; //Значение скорости, при достижении которой переключаются режимы
  return(spdModeSwitchVal);
}


/**
  * @brief  Расчет скорости по быстрым аналоговым сигналам для энкодера типа EnFat
  * @param  sEnco - указатель на структуру описания энкодера
  * @retval Скорость по быстрым аналоговым сигналам
  */
float  fastSinCosSignalSpdCalcForEnDat(encoBlockStatus *encoBlockPnt)
{
     u16 polePairsNum;
     u16 encoProcessingPeroid;
     u16 pulseResolution;
     float fRelThetaMechFine;
     float relThetaMechFine;
     static float K1 = 0;
     float angleDiff;
     float fastSinSpd;
     double fPrecTheta;
     static float encoFltStrgFastSin[64]={0};
     static ENCOFLT encoFltFastSin = {&encoFltStrgFastSin[0], 32, 0, 0, 0};  //!структура с данными для расчета скорости

     polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
     encoProcessingPeroid = encoBlockPnt->encoProcessingPeriod;
     pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
     encoFltFastSin.storageLen = FAST_SPD_STRG_LEN; // Буфер только на 2 отсчета
     fPrecTheta = precThetaCalc(encoBlockPnt);      // Расчет позиции внутри быстрой синусоиды
     fRelThetaMechFine = fPrecTheta/(2*pi);         // Относительный угол внутри быстрой синусоиды
     relThetaMechFine = fRelThetaMechFine;     // Относительный угол внутри быстрой синусоиды
     
     angleDiff = relThetaMechFine - (*(encoFltFastSin.thetaStrg + encoFltFastSin.storagePos));   //!Приращение угла внутри быстрой синусоиды
     *(encoFltFastSin.thetaStrg + encoFltFastSin.storagePos)= relThetaMechFine;              //!сохраняем вычисленный угол внутри быстрой синусоиды
     encoFltFastSin.storagePos=(encoFltFastSin.storagePos+1)&(encoFltFastSin.storageLen-1);  //!Формируем новое значение позиции в хранилище

     // пересчет перехода через единицу
     if(angleDiff < -0.75F) {
          angleDiff += 1.0F;
     }
     if(angleDiff > 0.75F) {
          angleDiff -= 1.0F;
     }
     
     if(encoBlockPnt->PWMOn == 0){
       K1 = 1000000.0F *(float)polePairsNum * RESOLUTION_FACTOR / (encoProcessingPeroid * encoFltFastSin.storageLen)/pulseResolution / FREQ_BASE;
     }
     // Подфильтровать приращение мехнической фазы перед расчетом скорости
     encoFltFastSin.TmpFltr = encoFltFastSin.TmpFltr +((angleDiff - encoFltFastSin.TmpFltr) * 0.125F);
     fastSinSpd = K1 * encoFltFastSin.TmpFltr; //скорость внутри быстрой синусоиды
     encoFltFastSin.fltSpeed = fastSinSpd;
     
     return(encoFltFastSin.fltSpeed);
}



/**
  * @brief  Синхронизация скорости, вычисленной по аналоговым сигналам с цифровой скоростью для энкодера EnDat
  * @param digatalSpd - скорость измеренная по абсолютной позиции
  * @param fastAnalogSpd - скорость, измеренная по аналоговым инкрементым синусоидам
  * @retval  
  */
void fastSinCosSpdSignPhasingForEnDat(encoBlockStatus *encoBlockPnt, float digatalSpd, float fastAnalogSpd, encoFlashMemDataType *pLocNvData){
  static float spdModeSwitchVal = 0;
  
  if(encoBlockPnt->PWMOn == 0){
    spdModeSwitchVal = spdModeValCalc(encoBlockPnt);
  }
  
  if(fabsf(digatalSpd) < spdModeSwitchVal ){ //синхронизация знаков быстрой аналоговой скорости и цифровой скорости
    if( (digatalSpd > 0) && (fastAnalogSpd < 0) ){
      pLocNvData->fastSpdSignSetting *= -1; //Приводим быструю аналоговую и цифровую скорости к одному знаку
    }else if ( (digatalSpd < 0) && (fastAnalogSpd > 0) ){
      pLocNvData->fastSpdSignSetting *= -1; //Приводим быструю аналоговую и цифровую скорости к одному знаку
    }else{
      pLocNvData->fastSpdSignSetting *= 1;  //Знаки цифровой и быстрой аналоговой скорости совпадают. Оставляем коэф. без изменения
    }
    nonVolatileDataFlashWrite(pLocNvData); //запись знака быстрой аналоговой скорости в Flash
  }
}



/**
  * @brief  Получение позиции энкодера по коду от Endat
  * @param
  * @param
  * @retval  Код позиции
  */
u32 getEnDatEncoderPosition(encoBlockStatus *encoBlockPnt, unsigned long long position){
   u32 encoderPosition;
   u32 tmpPos;
   u16 mask;
   u16 i;
   u16 bitResolution;
   bitResolution = encoBlockPnt->baseEncoMotorData.bitResolution; 
   //получаем маску на CRC
    mask = (1 << NUM_CRC_BITS) - 1;
    // Перекидываем биты старшим вперед. Изначально приходит положение младшим вперед
    mask = (1 << bitResolution) - 1; 
    tmpPos = (u32)((position >> NUM_CRC_BITS) & mask) ;
    encoderPosition = 0;
    for (i = 0; i < bitResolution; i++){      
      encoderPosition <<= 1;
      encoderPosition |= (tmpPos & 0x01) ? 1 : 0;
      tmpPos >>= 1;    
    }
    return(encoderPosition);
}

/**
  * @brief  Извлечение CRC из принятого от EnDat-энкодера пакета
  * @param
  * @param
  * @retval  Полученное CRC пакета
  */
u16 getEnDatEncoderCRC(unsigned long long position){
   u16 encoderCRC;
   u16 mask;
   mask = (1<<NUM_CRC_BITS) - 1;
   encoderCRC = (u16)(position & mask); // CRC в младших битах
   return(encoderCRC);
}



/**
  * @brief  Расчет CRC принятого от EnDat-энкодера пакета
  * @param
  * @param
  * @retval  Рассчитанное CRC пакета
  */
u16 EnDatEncoderCRCcalc(encoBlockStatus *encoBlockPnt, u32   encoderPosition, unsigned long long position){
    u16 alarm1, alarm2;
    u16 EndatModeCrc;
    u16 crcCalc;
    u16 bitResolution;
    u32   highpos, lowpos;
    volatile u16 encoErr;

    bitResolution = encoBlockPnt->baseEncoMotorData.bitResolution;
   // Находим биты ошибки
    switch (encoBlockPnt->baseEncoMotorData.serialMode)
    {
    case ECN1313:
        alarm1 = (position >> (bitResolution + NUM_CRC_BITS)) & 0x01;//!бит ошибки err1
        alarm2 = 0;     //!бит ошибки err2    
        encoErr = alarm1;     
        EndatModeCrc = ECN1313;
      break;

    case ECN1325:
        alarm2 = (position >> (bitResolution + NUM_CRC_BITS)) & 0x01;//!бит ошибки err2
        alarm1 = (position >> (bitResolution + NUM_CRC_BITS + 1)) & 0x01;//!бит ошибки err1
        encoErr = alarm1 + (alarm2 << 1);
        EndatModeCrc = ECN1325;
      break;    
    }   
    highpos = ((uint64_t)encoderPosition >> 32) & 0xFFFFFFFFL; //!выделяем старшие 32 бита позиции
    lowpos = encoderPosition & 0xFFFFFFFFL;          //!выделяем младшие 32 бита позиции 
    crcCalc = CrcEnDat(bitResolution, EndatModeCrc, alarm1, alarm2, highpos, lowpos); //!вычисляем crc 
    return(crcCalc);
}


/**
  * @brief  Расчет электрического угла
  * @param  encoder - указатель на структуру  описателя энкодера
  * @param  ThetaMechPU - механический угол
  * @retval электрический угол
  */
float EnDatEncoderElecPosCalc(encoBlockStatus *encoBlockPnt, float ThetaMechPU){
  float ThetaElec;
  float thetaOffset;
  u16 polePairsNum;
  
  polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum; //Число пар полюсов
  thetaOffset = encoBlockPnt->thetaOffset;
    
  ThetaElec = ThetaMechPU * polePairsNum;         //!Электрический угол
  ThetaElec += (1.0F/3600) * thetaOffset;     //!Электрический угол с учетом смещения
  return(ThetaElec);
}



/**
  * @brief  Расчет механического угла по полученной от Endat-энкодера позиции
  * @param  encoder - указатель на структуру  описателя энкодера
  * @param  encoderPosition - код абсолютной позиции энкодера
  * @retval механический угол
  */
float EnDatEncoderMechPosCalc(encoBlockStatus *encoBlockPnt, u32 encoderPosition){
  float fThetaMech;
  float fThetaMechPU;
  u16 bitResolution;
  
  bitResolution = encoBlockPnt->baseEncoMotorData.bitResolution;
  fThetaMech = (2*pi/(1 << bitResolution)) * (encoderPosition);//!Абсолютный итоговый угол (в рад.) в float            
  fThetaMechPU = fThetaMech/(2*pi);                            //!Относительный итоговый угол в float-формате
 
  return(fThetaMechPU);
}


/**
  * @brief  Расчет скорости для endat-энкодера
  * @param  encoder - указатель на структуру  описателя энкодера
  * @retval текущая скорость по endat-энкодеру
  */
float endatEncoSpdCalc(float discrThetaMechPU, encoBlockStatus *encoBlockPnt)
{
   static u16 skipCnt = 0;
   u16 skipTime;
   u16 polePairsNum;
   u16 encoProcessingPeriod;
   skipTime = (u16)(0.1F * 1000000.0F / encoBlockPnt->encoProcessingPeriod + 0.5F);
   float angleDiff;
   static float K = 0;
   static float encoFltStrg[64]={0};                       //!буфер для фильтрации  фазы
   static ENCOFLT encoFlt = {encoFltStrg, 32, 0, 0, 0};  //!структура с данными для расчета скорости
   
   polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
   encoProcessingPeriod = encoBlockPnt->encoProcessingPeriod;
   encoFlt.storageLen = encoBlockPnt->fltStrgLen;        //Размер хранилища фильтра фазы
   angleDiff = discrThetaMechPU - (*(encoFlt.thetaStrg + encoFlt.storagePos));   //!Приращение  угла
   *(encoFlt.thetaStrg + encoFlt.storagePos)= discrThetaMechPU;              //!по текущей позиции буфера фильтрации сохраняем вычисленный угол
   encoFlt.storagePos=(encoFlt.storagePos+1)&(encoFlt.storageLen-1);         //!Новое значение позиции в хранилище аналоговой фазы
   // пересчет перехода через единицу
   if(angleDiff < -0.8F) {
     angleDiff += 1.0F;
   }
   if(angleDiff > 0.8F) {//при реверсе
      angleDiff -= 1.0F;
   }
   encoFlt.TmpFltr = encoFlt.TmpFltr +((angleDiff - encoFlt.TmpFltr) * 0.125F);
     // Рассчитываем электрическую скорость с учетом пар полюсов.
   if(encoBlockPnt->PWMOn == 0){
     K = (float)polePairsNum * 1000000.0F / (encoProcessingPeriod * FREQ_BASE * encoFlt.storageLen); 
   }
   encoFlt.fltSpeed = K * encoFlt.TmpFltr;
   
   skipCnt++;
   if(skipCnt < skipTime){
     return(0.0F);
   }
   return(encoFlt.fltSpeed);
}

/**
  * @brief  Проверка обрыва R-сигнала
  * @param  encoStatus - текущий статус аварии
  * @param  encoder - указатель на структуру  описателя энкодера
  * @param  spdVal  - текущая скорость ротора
  * @retval статус аварии
  */

#define SPD_SIGN_STATE_CNT  (1000000/encoBlockPnt->encoProcessingPeriod) //время стабильного движения в одном направлении - 2 сек
#define SPD_VAL_STATE_CNT  (1000000/encoBlockPnt->encoProcessingPeriod)  //время стабильного движения со коростью контроля - 1 сек
#define DELTA_TIME ((float)encoBlockPnt->encoProcessingPeriod/1000000.0F)       //время такта анализа скорости
#define FULL_ROTATION 1.0F
#define DELTA_ROTATION 0.25F

uint16_t sinCosRSignalBreakCheck( uint16_t encoStatus, encoBlockStatus *encoBlockPnt, float spdVal){
  uint16_t errStatus = encoStatus;
  static uint16_t RSignalErrStatus = ENCO_OK;
  static RSignalControlStateType RSignalControlState = PWM_OFF;
  static uint16_t state1Cnt = 0;
  static uint16_t state2Cnt = 0;
  static uint16_t spdValStateCnt = 0;
  static float fltSpd = 0;
  float absEncoSpd = 0;
  static float rotateAngle = 0;
  static float  debugMaxRotateAngle = 0;
  u16 polePairsNum;
  
  polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
  fltSpd = f_Filtr(fltSpd, spdVal, 8);
  
  switch(RSignalControlState){
  case PWM_OFF: //исходное состояние
    RSignalErrStatus = 0;
    encoBlockPnt->phasingByRSignal = 0;
    rotateAngle = 0;
    state1Cnt = state2Cnt = spdValStateCnt = 0; //сброс счетчиков
    RSignalControlState = (encoBlockPnt->PWMOn == 1) ? STAB_SPD_SIGN_WAIT : RSignalControlState; //ждем запуск ПЧ
    break;
  case STAB_SPD_SIGN_WAIT: //Состояние ожидания стабильного движения в одном направлении
    if(spdVal > 0){                        //если скорость положительная
      state2Cnt = (state2Cnt != 0) ? --state2Cnt : state2Cnt; //декремент счетчика состояния отрицательной скорости
      ++state1Cnt; //счетчик состояния положительной скорости
      RSignalControlState = (state1Cnt == SPD_SIGN_STATE_CNT) ? STAB_SPD_VAL_WAIT : RSignalControlState; //если направление устоялось переход в проверку величины скорости
      state2Cnt = (state1Cnt == SPD_SIGN_STATE_CNT) ? 0 : state2Cnt; //сброс счетчиков при достижении интервала и смене состояния
      state1Cnt = (state1Cnt == SPD_SIGN_STATE_CNT) ? 0 : state1Cnt; //сброс счетчиков при достижении интервала и смене состояния
    }else if (spdVal < 0){                 //если скорость отрицательная
      state1Cnt = (state1Cnt != 0) ? --state1Cnt : state1Cnt; //декремент счетчика состояния положительной скорости
      ++state2Cnt; //счетчик состояния отрицательной скорости
      RSignalControlState = (state2Cnt == SPD_SIGN_STATE_CNT) ? STAB_SPD_VAL_WAIT : RSignalControlState; //если направление устоялось переход в проверку величины скорости
      state1Cnt = (state2Cnt == SPD_SIGN_STATE_CNT) ? 0 : state1Cnt; //сброс счетчиков при достижении интервала и смене состояния
      state2Cnt = (state2Cnt == SPD_SIGN_STATE_CNT) ? 0 : state2Cnt; //сброс счетчиков при достижении интервала и смене состояния
    }
    RSignalControlState = (encoBlockPnt->PWMOn == 0) ? PWM_OFF : RSignalControlState; //проверка ШИМ
    break;
  case STAB_SPD_VAL_WAIT: //состояние ожидания движения на стабильной скорости
    if(fabsf(spdVal) >= (1.0F / FREQ_BASE)){ //если достигли скорости контроля
      spdValStateCnt++;
      RSignalControlState = (spdValStateCnt == SPD_VAL_STATE_CNT) ? R_SIGNAL_WAIT : RSignalControlState; //переход в состояние ожидания R-сигнала если есть стабильное движение
      spdValStateCnt = (spdValStateCnt == SPD_VAL_STATE_CNT) ? 0 : spdValStateCnt; //сброс счетчика
    }else{
      spdValStateCnt = (spdValStateCnt != 0) ? spdValStateCnt - 1 : spdValStateCnt;
      RSignalControlState = STAB_SPD_SIGN_WAIT;
    }
    RSignalControlState = (encoBlockPnt->PWMOn == 0) ? PWM_OFF : RSignalControlState; //проверка ШИМ
    break;
  case R_SIGNAL_WAIT: //состояние ожидания R-сигнала
    absEncoSpd = fabsf(fltSpd); //модуль скорости энкодера
    absEncoSpd = (absEncoSpd / polePairsNum) * FREQ_BASE; //частота вращения ротора
    rotateAngle = rotateAngle + absEncoSpd * DELTA_TIME; //текущий проворот вала, в отн. ед. (интеграл vdt)
    
    debugMaxRotateAngle = (rotateAngle > debugMaxRotateAngle) ? rotateAngle : debugMaxRotateAngle;
    
    RSignalControlState = (rotateAngle >= FULL_ROTATION + DELTA_ROTATION) ? R_SIGNAL_MISS : RSignalControlState; //если был сделан проворот, но R-сигнал не поступил - авария
    rotateAngle =  (encoBlockPnt->RsignalFlg == 1) ? 0 : rotateAngle; //Сброс накопленного проворота при R-сигнале
    encoBlockPnt->phasingByRSignal = (encoBlockPnt->RsignalFlg == 1) ? 1 : encoBlockPnt->phasingByRSignal; //флаг найденного R-сигнала для передачи на верхний уровень
    RSignalControlState = ((encoBlockPnt->autoPhasingOn == 1) && (encoBlockPnt->RsignalFlg == 1)) ? PWM_OFF_WAIT : RSignalControlState; //В режиме автофазировки R-сигнал контролируется один раз за пуск
    RSignalControlState = (encoBlockPnt->PWMOn == 0) ? PWM_OFF : RSignalControlState; //проверка ШИМ
     
    break;
  case R_SIGNAL_MISS:      //Состояние отсутствия R-сигнала
    RSignalErrStatus = (encoBlockPnt->autoPhasingOn == 1) ? R_MISS_IN_TUNE_ERR : R_MISS_IN_RUNNIG_ERR;
    RSignalControlState = (encoBlockPnt->PWMOn == 0) ? PWM_OFF : RSignalControlState;
    //RSignalErrStatus = (encoder->PWMOn == 0) ? ENCO_OK : RSignalErrStatus;
    break;
  case PWM_OFF_WAIT: //Ожидание отключения ШИМ после обнаруженного R-сигнала в режиме автофазирования
    RSignalControlState = (encoBlockPnt->PWMOn == 0) ? PWM_OFF : RSignalControlState; //проверка ШИМ
    break;
  }
  errStatus = (RSignalErrStatus != ENCO_OK) ? RSignalErrStatus : errStatus;
  return(errStatus);
}



/**
  * @brief  Проверка аварий энкодера типа enDat
  * @param encoder - указатель на структуру описателя энкодеров
  * @param position - считанный с энкодера код
  * @retval статус аварии
  */
uint16_t enDatEncoErrDetect(encoBlockStatus *encoBlockPnt, unsigned long long position){
  uint16_t errStatus = ENCO_OK;
  
  errStatus = enDatDataExchangeErrDetect(errStatus, encoBlockPnt, position); // Проверка отсутствия обмена по протоколу enDat
  if(encoBlockPnt->baseEncoMotorData.fastSpdUse == USE){
    errStatus = enDatEncoderIncrSignalBreakCheck(errStatus, encoBlockPnt); //Обрыв аналоговых сигналов
    //С кодом ниже разобраться. Нужно ли это, если есть проверка квадратурности
    /*
    errStatus = encoderIncrSignalBreakCheck(errStatus, encoBlockPnt);      //Контроль меандра
    errStatus = fastSinCosSignalFaultDetect(errStatus, encoBlockPnt);
    */
    errStatus = encoderIncrSignalLowSpdBreakCheck(errStatus, encoBlockPnt); //Проверка квадратурности
    
  }
  return(errStatus);
}


/**
  * @brief  Проверка аварий энкодера типа sinCos
  * @param encoder - указатель на структуру описателя энкодеров
  * @param  spdVal - текущая скорость энкодера
  * @retval статус аварии
  */
uint16_t sinCosEncoErrDetect(encoBlockStatus *encoBlockPnt, float spdVal, float phaseAdd){
  uint16_t errStatus = ENCO_OK;
  errStatus = sinCosEncoderCablesBreakCheck(errStatus, encoBlockPnt);     //Проверка обрыва кабеля энкодера
  errStatus = sinCosEncoderAbsSignalBreakCheck(errStatus, encoBlockPnt);  //Проверка обрыва отдельных сигналов позиции: D и C
  //errStatus = encoderIncrSignalBreakCheck(errStatus, encoBlockPnt);       //Контроль обрыва инкрементных аналоговых сигналов
  errStatus = sinCosRSignalBreakCheck(errStatus, encoBlockPnt, spdVal);   //Проверка отсутствия R-сигнала
  errStatus = encoderIncrSignalLowSpdBreakCheck(errStatus, encoBlockPnt);
  errStatus = (encoBlockPnt->drvMode == VECTOR_MODE) ? checkSinCosPhasingMiss(errStatus, encoBlockPnt, phaseAdd) : errStatus; //Проверка невыполненного низкоуровнего фазирования. Проверка выполняется в векторном режиме
  return(errStatus);
}


/**
  * @brief  Проверка обрыва кабеля энкодера типа SIN/COS
  * @param  encoErrStatus - текущий статус аварии энкодера
  * @param  encoder - указатель на структуру описателя энкодеров
  * @retval статус аварии
  */

#define MIN_ADC_ZERO -100 //отклонение нулевого уровня кода АЦП
#define MAX_ADC_ZERO 100  //отклонение нулевого уровня кода АЦП
#define MIN_ERR_VAL -8   //минимальное аварийное значение уровня нуля инкрементаного сигнала
#define MAX_ERR_VAL  8   //максимальное аварийное значение уровня нуля инкрементаного сигнала 
#define CABLE_BREAK_CNT  2*ONE_SEC_TACT_CNT


uint16_t sinCosEncoderCablesBreakCheck(uint16_t encoErrStatus, encoBlockStatus *encoBlockPnt){
    static uint16_t CD_ErrStatus     = ENCO_OK;
    static uint16_t AB_ErrStatus     = ENCO_OK;
    uint16_t sinCosErrStatus  = encoErrStatus;
    static uint16_t D_C_missCnt = 0;
    static uint16_t A_B_missCnt = 0;
    s16 fastSin, fastCos, slowSin, slowCos;
    
    fastSin = encoBlockPnt->analogSignals.fastSin;
    fastCos = encoBlockPnt->analogSignals.fastCos;
    slowSin = encoBlockPnt->analogSignals.slowSin;
    slowCos = encoBlockPnt->analogSignals.slowCos;
    
    if ((slowSin >= MIN_ERR_VAL) && (slowSin <= MAX_ERR_VAL) && (slowCos >= MIN_ERR_VAL) && (slowCos <= MAX_ERR_VAL)){ //проверка аварийной ситуации
      D_C_missCnt = (D_C_missCnt < CABLE_BREAK_CNT) ? (D_C_missCnt + 1) : D_C_missCnt;
      CD_ErrStatus = (D_C_missCnt == CABLE_BREAK_CNT) ? C_OR_D_MISS : CD_ErrStatus; //Статус - нет позиционных сигналов: D и С
    }else{
      D_C_missCnt = (D_C_missCnt > 0) ? (D_C_missCnt - 1) : D_C_missCnt;
      CD_ErrStatus = (D_C_missCnt == 0) ? ENCO_OK : CD_ErrStatus;
    }
     
    if ((fastSin >= MIN_ERR_VAL) && (fastSin <= MAX_ERR_VAL) && (fastCos >= MIN_ERR_VAL) && (fastCos <= MAX_ERR_VAL)){ //проверка аварийной ситуации
      A_B_missCnt = (A_B_missCnt < CABLE_BREAK_CNT) ? (A_B_missCnt + 1) : A_B_missCnt;
      AB_ErrStatus = (A_B_missCnt == CABLE_BREAK_CNT) ? A_OR_B_MISS : AB_ErrStatus; //Статус - нет инкрементных аналоговых сигналов A и B
    }else{
      A_B_missCnt = (A_B_missCnt > 0) ? (A_B_missCnt - 1) : A_B_missCnt;
      AB_ErrStatus = (A_B_missCnt == 0) ? ENCO_OK : AB_ErrStatus;
    }
    
    sinCosErrStatus = ((CD_ErrStatus == C_OR_D_MISS) && (AB_ErrStatus == A_OR_B_MISS)) ? CABLE_BREAK_ERR : sinCosErrStatus; //Обрыв кабеля
    sinCosErrStatus = ((CD_ErrStatus == C_OR_D_MISS) && (AB_ErrStatus == ENCO_OK)) ? C_OR_D_MISS : sinCosErrStatus;     //Нет D и С
    sinCosErrStatus = ((CD_ErrStatus == ENCO_OK) && (AB_ErrStatus == A_OR_B_MISS)) ? A_OR_B_MISS : sinCosErrStatus;     //Нет A и B
    
    //конвертирование внутренних кодов аварий в код для передачи на верхний уровень
    if(encoBlockPnt->autoPhasingOn == 1){ //режим автофазирования
      sinCosErrStatus = (sinCosErrStatus == C_OR_D_MISS) ? C_D_MISS_IN_TUNE_ERR : sinCosErrStatus; //меняем внутренний код аварии на внешний
      sinCosErrStatus = (sinCosErrStatus == A_OR_B_MISS) ? A_B_MISS_IN_TUNE_ERR : sinCosErrStatus; //меняем внутренний код аварии на внешний
    }else{ //рабочий режим
      sinCosErrStatus = (sinCosErrStatus == C_OR_D_MISS) ? C_D_MISS_IN_RUNNING_ERR : sinCosErrStatus; //меняем внутренний код аварии на внешний
      sinCosErrStatus = (sinCosErrStatus == A_OR_B_MISS) ? A_B_MISS_IN_RUNNING_ERR : sinCosErrStatus; //меняем внутренний код аварии на внешний
    }
  
    return sinCosErrStatus;
  
}


/**
  * @brief  Проверка одновременного обрыва инкрементных аналоговых сигналов энкодера типа enDat
  * @param  encoErrStatus - текщий статус аварии энкодера
  * @param  encoder - указатель на структуру-описатель энкодера
  * @retval статус аварии
  */

uint16_t enDatEncoderIncrSignalBreakCheck(uint16_t encoErrStatus, encoBlockStatus *encoBlockPnt){
    uint16_t sinCosErrStatus  = encoErrStatus;
    static uint16_t AB_ErrStatus = ENCO_OK;
    static uint16_t A_B_missCnt = 0;
    s16 fastSin, fastCos;
    
    fastSin = encoBlockPnt->analogSignals.fastSin;
    fastCos = encoBlockPnt->analogSignals.fastCos;
  
    if ((fastSin >= MIN_ERR_VAL) && (fastSin <= MAX_ERR_VAL) && (fastCos >= MIN_ERR_VAL) && (fastCos <= MAX_ERR_VAL)){ //проверка аварийной ситуации
      A_B_missCnt++;
      AB_ErrStatus = (A_B_missCnt == CABLE_BREAK_CNT) ? /*A_OR_B_MISS*/A_B_MISS_IN_RUNNING_ERR : AB_ErrStatus; //Статус - нет инкрементных аналоговых сигналов A и B
    }else{
      A_B_missCnt = (A_B_missCnt > 0) ? --A_B_missCnt : A_B_missCnt;
      AB_ErrStatus = (A_B_missCnt == 0) ? ENCO_OK : AB_ErrStatus;
    }
    
    sinCosErrStatus = (AB_ErrStatus != ENCO_OK) ? AB_ErrStatus : sinCosErrStatus;
    return sinCosErrStatus;
}

/**
  * @brief  Проверка отсутствия обмена по протоколу edDat
  * @param  encoStatus - текущий статус аварии энкодера 
  * @param  encoder - указатель на структуру  описателя энкодера
  * @param  position - считанный с энкодера код
  * @retval статус аварии
  */
u16 enDatDataExchangeErrDetect(uint16_t encoStatus, encoBlockStatus *encoBlockPnt, unsigned long long position){
    uint16_t bitResolution;
    uint16_t errStatus, tmp;
    uint16_t alarm1;
    uint16_t alarm2;
    static u16 fltState = 0;
    static uint16_t errPresentCnt = 0;
    static uint16_t enDatErrStatus = 0;
    errStatus = encoStatus;
    tmp = 0;
    bitResolution = encoBlockPnt->baseEncoMotorData.bitResolution;
    switch (encoBlockPnt->baseEncoMotorData.serialMode){
    case ECN1313:
        alarm1 = (position >> (bitResolution + NUM_CRC_BITS)) & 0x01;//!бит ошибки err1
        alarm2 = 0;     //!бит ошибки err2        
         break;
    case ECN1325:
        alarm2 = (position >> (bitResolution + NUM_CRC_BITS)) & 0x01;//!бит ошибки err2
        alarm1 = (position >> (bitResolution + NUM_CRC_BITS + 1)) & 0x01;//!бит ошибки err
        break;    
    }
    tmp = ((alarm1 == 1) || (alarm2 == 1)) ? 1 : 0; //Авария обмена по протоколу enDat
    //-----Фильтрация аварии-------//
    switch(fltState){
    case 0: //Состояние "Не аварии"
      enDatErrStatus = 0;
      if(tmp != 0){
        if(errPresentCnt >= ENDAT_ERR_TIME){
          fltState = 1;
        }else{
          errPresentCnt++;
        }
      }else{
        errPresentCnt = (errPresentCnt > 0) ? (errPresentCnt - 1) : errPresentCnt;
      }
      break;
    case 1: //Состояние аварии "Нет обмена"
      enDatErrStatus = ENDAT_EXCHANGE_MISS_ERR;
      if(tmp == 0){
        if(errPresentCnt <= 0){
          fltState = 0;
        }else{
          errPresentCnt--;
        }
      }else{
        errPresentCnt = (errPresentCnt < ENDAT_ERR_TIME) ? (errPresentCnt + 1) : errPresentCnt;
      }
      break;
    }
       
    errStatus = enDatErrStatus ? enDatErrStatus : errStatus; 
    return errStatus;
}

/**
  * @brief  Проверка обрыва инкрементных аналоговых сигналов энкодера типа enDat
  * @param  encoErrStatus - текущий статус аварии энкодеров
  * @param  encoder - указатель на структуру описателя энкодеров
  * @retval статус аварии
  */

#define ERR_TIME  2        //20 мс - время ожидания установления
#define SLOW_SIGNAL_ERR_TIME 10
#define INCR_SIGNAL_ERR_TIME 2
#define ERR_WAIT 0
#define ERR_HOLD 1

uint16_t encoderIncrSignalBreakCheck(uint16_t encoErrStatus, encoBlockStatus *encoBlockPnt){
    uint16_t errStatus;
    static uint16_t errDetectState = ERR_WAIT;
    static uint16_t heldErr = ENCO_OK;
    static uint16_t errHoldCnt = 0;
    
    switch(errDetectState){
    case ERR_WAIT: //Состояние ожидания авварии
      errStatus = encoErrStatus;
      if(incrSignalErr != ENCO_OK){ //если обнаружена авария 
        errStatus = incrSignalErr;
        errDetectState = ERR_HOLD; //переход в состояние удержания аварии
        heldErr = incrSignalErr;
      }
      break;
    case ERR_HOLD: //Состояние удержания аварии
      errStatus = heldErr;
      errHoldCnt++;
      errDetectState = (errHoldCnt == ONE_SEC_TACT_CNT) ? ERR_WAIT : errDetectState; //если выдержали время - возврат в состояние ожидания аварии
      errHoldCnt = (errHoldCnt == ONE_SEC_TACT_CNT ) ? 0 : errHoldCnt; //сброс счетчика
      break;
    }
    return errStatus;
}


/**
  * @brief  Проверка обрыва сигналов D и C энкодера типа sinCos
  * @param  encoStatus  - текущий статус аварии энкодеров
  * @param  encoder  - указатель на структуру описателя энкодеров
  * @retval статус аварии
  */

#define MIN_SQRT_ALLOW_VAL 0.81F //минимально допустимое значение корня из суммы квадратов sin и cos
#define MAX_SQRT_ALLOW_VAL 1.50F //максимально допустимое значение корня из суммы квадратов sin и cos

#define MIN_SQRT_ALLOW_VAL_INCR 0.5F //минимально допустимое значение корня из суммы квадратов sin и cos
#define MAX_SQRT_ALLOW_VAL_INCR 1.50F //максимально допустимое значение корня из суммы квадратов sin и cos

uint16_t sinCosEncoderAbsSignalBreakCheck(int16_t encoStatus, encoBlockStatus *encoBlockPnt){
    uint16_t sinCosErrStatus  = encoStatus;
    uint16_t absPosSignalErr;
    uint16_t absPosSignalD_Err;
    uint16_t absPosSignalC_Err;
    float normalizeSin;
    float normalizeCos;
    float tmp;
    static uint16_t errCnt = 0;
    float sqrtVal;
    s16 slowSin, slowCos;
    static u16 maxSin = 0;
    static u16 maxCos = 0;
    
    slowSin = encoBlockPnt->analogSignals.slowSin;
    slowCos = encoBlockPnt->analogSignals.slowCos;
    
    maxSin = (abs(slowSin) > maxSin) ? abs(slowSin) : maxSin;
    maxCos = (abs(slowCos) > maxCos) ? abs(slowCos) : maxCos;
    absPosSignalErr = ENCO_OK;
    normalizeSin = (float)slowSin / /*encoBlockPnt->ADC_Amplitude*/SIN_COS_ADC_AMPL; //нормализованный sin
    normalizeCos = (float)slowCos / /*encoBlockPnt->ADC_Amplitude*/SIN_COS_ADC_AMPL; //нормализованный cos
    tmp = powf(normalizeSin, 2) + powf(normalizeCos, 2); //сумма  квадратов абсолютных сигналов sin и cos
    sqrtVal = sqrtf(tmp);

    if(!((sqrtVal >= MIN_SQRT_ALLOW_VAL) && (sqrtVal <= MAX_SQRT_ALLOW_VAL))){ //нормальное значение суммы квадратов - в пределах 0,7...1,2
        errCnt++;
        if(errCnt >= SLOW_SIGNAL_ERR_TIME){
          absPosSignalD_Err = ((slowSin >= MIN_ERR_VAL) && (slowSin <= MAX_ERR_VAL)) ? D_MISS_IN_WORK : ENCO_OK;     //Авария D
          absPosSignalC_Err = ((slowCos >= MIN_ERR_VAL) && (slowCos <= MAX_ERR_VAL)) ? C_MISS_IN_WORK : ENCO_OK;     //Авария C
          absPosSignalErr = ((absPosSignalD_Err == D_MISS_IN_WORK) && (absPosSignalC_Err == ENCO_OK))? D_MISS_IN_WORK : absPosSignalErr;       //Если D в аварии, C норм
          absPosSignalErr = ((absPosSignalC_Err == C_MISS_IN_WORK) && (absPosSignalD_Err == ENCO_OK))? C_MISS_IN_WORK : absPosSignalErr;       //Если С в аварии, D норм
          absPosSignalErr = ((absPosSignalC_Err == C_MISS_IN_WORK) && (absPosSignalD_Err == D_MISS_IN_WORK))? C_OR_D_MISS : absPosSignalErr;   //Если D и C в аварии
          absPosSignalErr = ((absPosSignalC_Err == ENCO_OK) && (absPosSignalD_Err == ENCO_OK))? C_OR_D_MISS : absPosSignalErr; //Если D и C не в нулевом уровне - дребезг
          
          //преобразуем внутренний код аварии во внешний
          absPosSignalErr = ((absPosSignalErr != ENCO_OK) && (encoBlockPnt->autoPhasingOn == 1)) ?  C_D_MISS_IN_TUNE_ERR : absPosSignalErr;
          absPosSignalErr = ((absPosSignalErr != ENCO_OK) && (encoBlockPnt->autoPhasingOn == 0)) ?  C_D_MISS_IN_RUNNING_ERR : absPosSignalErr;
          
        }
     }else{
       errCnt = (errCnt > 0) ? --errCnt : errCnt;
       absPosSignalErr = (errCnt == 0) ? ENCO_OK : absPosSignalErr;
     }
    sinCosErrStatus = ((absPosSignalErr != ENCO_OK) && (encoStatus != CABLE_BREAK_ERR)) ? absPosSignalErr : sinCosErrStatus;
    return sinCosErrStatus;  
}


/**
  * @brief  Настройка модуля энкодера процессора для обработки инкрементных сигналов A и B
  * @param  encoBlockPnt  - указатель на структуру описателя энкодеров
  * @retval
  */
void enDat_SSI_Encoder_A_B_Init(encoBlockStatus *encoBlockPnt){
  GPIO_InitTypeDef      GPIO_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  uint16_t encoInputSetting = 0;
  uint16_t pulseResolution;
  
  pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
  TIM_Cmd(TIM1, DISABLE); //Останов таймера TIM1
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //!тактирование таймера TIM1
  
  //!Настройка пина PA8 в качестве счетного входа 1 таймера TIM1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //!Альтернативная функция
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //!Подтяжка
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  //!Подтяжка к нулю
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;       //!PA8
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  GPIO_PinAFConfig(GPIOA,  GPIO_PinSource8,  GPIO_AF_6); 
  
 
  //!Настройка пина PA9 в качестве счетного входа 2 таймера TIM1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //!Альтернативная функция
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //!Подтяжка
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  //!Подтяжка к нулю
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;       //!PA9
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  GPIO_PinAFConfig(GPIOA,  GPIO_PinSource9,  GPIO_AF_6);
  
  /*настройка таймера TIM1 на режим модуля энкодера*/
  encoInputSetting = getEncoInputSettFromFlash(); //Выбор активного фронта инкрементного сигнала
  encoInputSetting = (encoInputSetting == 0) ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
  TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, encoInputSetting, TIM_ICPolarity_Rising); //Включение режима энкодера
  TIM_SetAutoreload(TIM1, (pulseResolution - 1)); //Настройка значения обновления таймера-счетчика модуля энкодера

 /*Настройка режима захвата таймера TIM1 для обработки АВАРИЙ обрыва инкрементных линий*/
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //Канал №1:
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = encoInputSetting;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8; 
  TIM_ICInitStructure.TIM_ICFilter = 5;     
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE); //разрешение прерывания таймера TIM1 по захвату от канала 1
  
  //Канал №2
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter = 5;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE); //разрешение прерывания  таймера TIM1 по захвату от канала 2 
  
  TIM_Cmd(TIM1, ENABLE); //Запуск таймера TIM1 

}

/**
  * @brief  Настройка модуля SPI для энкодеров типа SSI
  * @param  encoder  - структура-описатель энкодера
  * @retval 
  */
void SSISpiInit(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure;

  EndatSpiData.EndatSpiState = StopMode; // Текущее состояние 
  
  /* Enable the SPI 1 peripheral */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  
  /* Enable SCK, MOSI, MISO and NSS GPIO clocks */
  RCC_AHBPeriphClockCmd(SPI1_SCK_GPIO_CLK | SPI1_MISO_GPIO_CLK | SPI1_MOSI_GPIO_CLK |
                        SPI1_NSS_GPIO_CLK , ENABLE);
    
  /* SPI pin mappings */
  GPIO_PinAFConfig(SPI1_SCK_GPIO_PORT,  SPI1_SCK_SOURCE,  SPI1_SCK_AF);
  GPIO_PinAFConfig(SPI1_MOSI_GPIO_PORT, SPI1_MOSI_SOURCE, SPI1_MOSI_AF);
  GPIO_PinAFConfig(SPI1_MISO_GPIO_PORT, SPI1_MISO_SOURCE, SPI1_MISO_AF);
  
  // General setting for GPIO pins
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN;
  GPIO_Init(SPI1_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  SPI1_MOSI_PIN;
  GPIO_Init(SPI1_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPI1_MISO_PIN;
  GPIO_Init(SPI1_MISO_GPIO_PORT, &GPIO_InitStructure);
  
  /* SPI NSS pin configuration */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // Работает как выход
  GPIO_InitStructure.GPIO_Pin = SPI1_NSS_PIN;
  GPIO_Init(SPI1_NSS_GPIO_PORT, &GPIO_InitStructure);
  
  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPI1);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_10b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // 32
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(SPI1, &SPI_InitStructure);
  
  /* Configure the SPI interrupt priority */
  
  NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the SPI peripheral */
  SPI_Cmd(SPI1, ENABLE);
}



/**
  * @brief  Проверка текущего угла на соответствие диапазону
  * @param  endatAngle - текущий угол, вычисленный по endat
  * @param  incrEncoModulAngle - текущий угол модуля инкрементного энкодера
  * @retval флаг аварии
  */
u16 fastSinCosSignalErrDetectForEnDat(u16 encoErr, float endatAngle, float incrEncoModulAngle, u16 pwmStatus){
    static u16 err = ENCO_OK;
    static u16 errCnt = 0;
    u16 encoErrStatus;
    
    err = (pwmStatus == 0) ? ENCO_OK : err;
    errCnt = (pwmStatus == 0) ? 0 : errCnt;
    encoErrStatus = encoErr;
    
    if(fabsf(endatAngle - incrEncoModulAngle) >= 10.0F / 360.0F){
      errCnt++;
      err = (errCnt >= 2500) ? INCORRECT_INCR_ABS_PHASE_DIFF_ERR : err;
    }else{
      errCnt = (errCnt > 0) ? (errCnt - 1) : errCnt;
      err = (errCnt == 0) ? ENCO_OK : err;
    }
    encoErrStatus = (err != ENCO_OK) ? err : encoErrStatus;
    return(encoErrStatus);
}



/**
  * @brief  Расчет скорости по дискретной фазе модуля инкрементного энкодера
  * @param  discrThetaMechPU - текущий угол, вычисленный по значению из модуля инкрементного энкодера
  * @param  encoder - указатель на структуру энкодера
  * @retval скорость по инкрементной фазе
  */
float incrAngleSpdCalc(float discrThetaMechPU, encoBlockStatus *encoBlockPnt)
{
   u16 polePairsNum;
   u16 encoProcessingPeriod;
   float angleDiff;
   static float K = 0;
   static float encoFltStrg[64]={0};                              //!буфер для фильтрации  аналоговой фазы
   static ENCOFLT encoFlt = {encoFltStrg, 32, 0, 0, 0};         //!структура с данными для расчета скорости по аналоговой фазе
   
   polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum; // число пар полюсов
   encoProcessingPeriod = encoBlockPnt->encoProcessingPeriod;   //Период обработки данных энкодера
   encoFlt.storageLen = encoBlockPnt->fltStrgLen;
   angleDiff = discrThetaMechPU - (*(encoFlt.thetaStrg + encoFlt.storagePos));   //!Приращение аналогового угла
   *(encoFlt.thetaStrg + encoFlt.storagePos)= discrThetaMechPU;              //!по текущей позиции буфера фильтрации сохраняем вычисленный угол
   encoFlt.storagePos=(encoFlt.storagePos+1)&(encoFlt.storageLen-1); //!Новое значение позиции в хранилище аналоговой фазы
   // пересчет перехода через единицу
   if(angleDiff < -0.8F) {
     angleDiff += 1.0F;
   }
   if(angleDiff > 0.8F) {//при реверсе
      angleDiff -= 1.0F;
   }
   encoFlt.TmpFltr = encoFlt.TmpFltr +((angleDiff - encoFlt.TmpFltr) * 0.125F);
     // Рассчитываем электрическую скорость с учетом пар полюсов.
   if(encoBlockPnt->PWMOn == 0){
     K = (float)polePairsNum * 1000000.0F / (encoProcessingPeriod * FREQ_BASE * encoFlt.storageLen); 
   }
   encoFlt.fltSpeed = K * encoFlt.TmpFltr;
   return(encoFlt.fltSpeed); //скорость по дискретной фазе
}

/**
  * @brief  Установка исходного значения таймера-счетчика модуля энкодера по значению фазы, полученной по протоколу EnDat
            и проверка успешности установки
  * @param  encoBlockPnt - Указатель на структуру энкодера
  * @param  encoErrStatus - текущая авария энкодера
  * @retval статус аварии
  */
u16 fastSinCosSignalFaultDetect(u16 encoErrStatus, encoBlockStatus *encoBlockPnt){
   static absIncrementPhasingStateType absIncrementPhasingState = SPD_SIGN_SYNHRO;
   s16 baseIncrementCntVal;
   u32 cntState;
   u16 encoErr = 0;
   u16 pulseResolution;
   float mechThetaPU;
   float incrementModulAngle;
   
   pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
   mechThetaPU = encoBlockPnt->calculatedData.ThetaMechPU;
   incrementModulAngle = encoBlockPnt->calculatedData.incrementModulAngle;
   absIncrementPhasingState = (encoBlockPnt->PWMOn == 0) ? SPD_SIGN_SYNHRO : absIncrementPhasingState;
   encoErr = (encoBlockPnt->PWMOn == 0) ? fastSinCosSignalErrDetectForEnDat(encoErr, mechThetaPU, incrementModulAngle, encoBlockPnt->PWMOn) : encoErr;
   encoErr = encoErrStatus;
   switch(absIncrementPhasingState){
   case SPD_SIGN_SYNHRO:
     if(encoBlockPnt->incrModulSpdPhasingStatus != NOT_DETECT){
        baseIncrementCntVal = (s16)(mechThetaPU * pulseResolution + 0.5F); //исходное значение инкрементной фазы
        TIM_SetCounter(TIM1, baseIncrementCntVal); //установка счетчика в начальное значение
        cntState = TIM_GetCounter(TIM1);           //проверка на успешность установки
        if(cntState == baseIncrementCntVal){       //если счетчик установился в нужное значение..
          absIncrementPhasingState = ERR_CHECKING;
        }
     }
     break;
   case ERR_CHECKING:
     encoErr = fastSinCosSignalErrDetectForEnDat(encoErr, mechThetaPU, incrementModulAngle, encoBlockPnt->PWMOn);
     break;
   }
   return encoErr;
}

/**
  * @brief  Контроль обрыва инкрементных сигналов на малой скорости
  * @param encoErrStatus - текущий статус аварии
  * @param encoder - указатель на структуру-описатель энкодера
  * @retval статус аварии
  */
uint16_t encoderIncrSignalLowSpdBreakCheck(uint16_t encoErrStatus, encoBlockStatus *encoBlockPnt){
    uint16_t sinCosErrStatus  = encoErrStatus;
    uint16_t incrPosSignalErr;
    uint16_t incrPosSignalA_Err;
    uint16_t incrPosSignalB_Err;
    float normalizeSin;
    float normalizeCos;
    float tmp;
    static uint16_t errCnt = 0;
    float sqrtVal;
    float maxSpd;
    s16 fastSin, fastCos;
    static float maxSqrtVal = 0.0F;
    static float minSqrtVal = 1.0F;
    static s16 maxSin = 0;
    static s16 maxCos = 0;
    
    
    fastSin = encoBlockPnt->analogSignals.fastSin;
    fastCos = encoBlockPnt->analogSignals.fastCos;
    
    maxSin = (fastSin > maxSin)? fastSin : maxSin;
    maxCos = (fastCos > maxCos)? fastCos : maxCos;
    
    maxSpd = spdModeValCalc(encoBlockPnt);
    if(fabsf(encoBlockPnt->calculatedData.electricSpd) > maxSpd){   
      return(sinCosErrStatus);
    }
    
    incrPosSignalErr = ENCO_OK;
    normalizeSin = (float)fastSin / SIN_COS_ADC_AMPL; //нормализованный sin
    normalizeCos = (float)fastCos / SIN_COS_ADC_AMPL; //нормализованный cos
    tmp = powf(normalizeSin, 2) + powf(normalizeCos, 2); //сумма  квадратов инкрементных сигналов sin и cos
    sqrtVal = sqrtf(tmp);
    
    
    maxSqrtVal = (sqrtVal > maxSqrtVal) ? sqrtVal : maxSqrtVal;
    minSqrtVal = (sqrtVal < minSqrtVal) ? sqrtVal : minSqrtVal;

    if(!((sqrtVal >= MIN_SQRT_ALLOW_VAL_INCR) && (sqrtVal <= MAX_SQRT_ALLOW_VAL_INCR))){ //допустимое значение суммы квадратов - в пределах 0,5...1,5
        errCnt++;
        if(errCnt >= INCR_SIGNAL_ERR_TIME){
          incrPosSignalA_Err = ((fastSin >= MIN_ERR_VAL) && (fastSin <= MAX_ERR_VAL)) ? A_MISS_IN_WORK : ENCO_OK;  
          incrPosSignalB_Err = ((fastCos >= MIN_ERR_VAL) && (fastCos <= MAX_ERR_VAL)) ? B_MISS_IN_WORK : ENCO_OK;    
          incrPosSignalErr = ((incrPosSignalA_Err == A_MISS_IN_WORK) && (incrPosSignalB_Err == ENCO_OK))? A_MISS_IN_WORK : incrPosSignalErr; 
          incrPosSignalErr = ((incrPosSignalB_Err == B_MISS_IN_WORK) && (incrPosSignalA_Err == ENCO_OK))? B_MISS_IN_WORK : incrPosSignalErr;   
          incrPosSignalErr = ((incrPosSignalB_Err == A_MISS_IN_WORK) && (incrPosSignalA_Err == B_MISS_IN_WORK))? A_OR_B_MISS : incrPosSignalErr; 
          incrPosSignalErr = ((incrPosSignalB_Err == ENCO_OK) && (incrPosSignalA_Err == ENCO_OK))? A_OR_B_MISS : incrPosSignalErr;
          
          //преобразуем внутренний код аварии во внешний
          incrPosSignalErr = ((incrPosSignalErr != ENCO_OK) && (encoBlockPnt->autoPhasingOn == 1)) ?  A_B_MISS_IN_TUNE_ERR : incrPosSignalErr;
          incrPosSignalErr = ((incrPosSignalErr != ENCO_OK) && (encoBlockPnt->autoPhasingOn == 0)) ?  A_B_MISS_IN_RUNNING_ERR : incrPosSignalErr;
          
        }
     }else{
       errCnt = (errCnt > 0) ? (errCnt - 1) : errCnt;
       incrPosSignalErr = (errCnt == 0) ? ENCO_OK : incrPosSignalErr;
     }
    sinCosErrStatus = ((incrPosSignalErr != ENCO_OK) && (incrPosSignalErr != CABLE_BREAK_ERR)) ? incrPosSignalErr : sinCosErrStatus;
    return sinCosErrStatus;

}


/**
  * @brief  Выбор знака быстрой инкрементной скорости
  * @param autoFastSpdSign - выбор знака аналоговой инкрементной скорости
  * @param encoBlockPnt - указатель на структуру-описатель энкодера
  * @retval знак инкрементной скорости: 1 или -1
  */
int32_t fastSpdSignDef(encoBlockStatus *encoBlockPnt, int32_t autoFastSpdSign){
  int32_t fastSpdSign;
  u16 incrSpdPhasingSign;
  
  fastSpdSign = autoFastSpdSign;
  incrSpdPhasingSign = encoBlockPnt->incrSpdPhasingSign;
  switch(incrSpdPhasingSign){
  case AUTO_DEF:
    fastSpdSign = autoFastSpdSign;
    break;
  case POSITIVE:
    fastSpdSign = 1L;
    break;
  case NEGATIVE:
    fastSpdSign = -1L;
    break;
  }
  return(fastSpdSign);
}

/**
  * @brief  Настройка эмулятора энкодера
  * @param
  * @param
  * @retval
  */
#define MAX_DIVIDE_VAL 65535U    //максимально возможное значение коэффициента деления частоты шины
#define MAX_ARR_VAL    65536U    //максимально возможное значение регистра перезагрузки
#define MIN_REF_FREQ    0.05F    //минимальное задание частоты
#define MAX_REF_FREQ  80
#define MIN_ARR_VAL   50
#define PWM_FREQ_FACTOR     2    //коэффициент, учитывающий зависимость частоты генерируемого сигнала от частоты "пилы" 

void encoEmulInit(encoBlockStatus *encoBlockPnt){
  static u16 BDTRflg = 0;
  GPIO_InitTypeDef GPIO_InitStructure; //!Структура для настройки портов
  TIM_OCInitTypeDef TIM_OCInitStruct;
  TIM_TypeDef* encoEmulTIM = TIM15;
  TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); //!тактирование порта B
  
  //Настройка пина PB14 Выход А эмулятора энкодера
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //!Альтернативная функция
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //!Подтяжка
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;     //!Подтяжка к нулю
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;          //!PB14
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource14,  GPIO_AF_1); //!PB14 задействован для таймера TIM15
  
  //Настройка пина PB15 Выход B эмулятора энкодера
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //!Альтернативная функция
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //!Подтяжка
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;     //!Подтяжка к нулю
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;          //!PB15
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource15,  GPIO_AF_1); //!PB15 задействован для таймера TIM15
  
  //Настройка выхода 1 TIM15
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE); //Тактирование таймера 15
  
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_Toggle; //Пин в режиме выхода, переключение по совпадению
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //Разрешение выхода (CC1E = 0)
  TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable; //комплиментарный режим запрещен
  TIM_OCInitStruct.TIM_Pulse = 0; //Значение регистра сравнения
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;       //Исходное состояние до совпадения
  TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCPolarity_High;     //Исходное состояние до совпадения
  TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;   //Не исп.
  TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset; //Не исп.
  TIM_OC1Init(encoEmulTIM, &TIM_OCInitStruct); //Настройка выхода 1 таймера TIM15
  
  //Настройка выхода 2 TIM15
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_Toggle; //Пин в режиме выхода, переключение по совпадению
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //Разрешение выхода (CC2E = 0)
  TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable; //комплиментарный режим запрещен
  TIM_OCInitStruct.TIM_Pulse = 0; //Значение регистра сравнения
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;       //Исходное состояние до совпадения
  TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCPolarity_High;     //Исходное состояние до совпадения
  TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;   //Не исп.
  TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset; //Не исп.
  TIM_OC2Init(encoEmulTIM, &TIM_OCInitStruct); //Настройка выхода 2 таймера TIM15
  
  //Базовая настройка таймера
  TIM_InternalClockConfig(encoEmulTIM);               //Таймер тактируется от внутреннего источника
  encoEmulStartSettigsSet(encoEmulTIM, encoBlockPnt); //Базовая настройка таймера эмулятора
  if(BDTRflg == 0){
    BDTRflg = 1;
    TIM_CtrlPWMOutputs(encoEmulTIM, ENABLE);            //Глобальное разрешение выхода сравнения (MOE = 1)
    TIM_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Disable;
    TIM_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStruct.TIM_DeadTime = 0x00;
    TIM_BDTRInitStruct.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStruct.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM15, &TIM_BDTRInitStruct);
  }
}

/**
  * @brief  Расчет регистра перезагрузки таймера TIM15 эмулятора энкодера
  * @param
  * @param
  * @retval
  */
void encoEmulCalc(encoBlockStatus *encoBlockPnt){
  static u16 firstResetting = 0;
  static u16 skipCnt = 0;
  TIM_OCInitTypeDef TIM_OCInitStruct;
  u16 skipTime;
  u16 encoEmulResol;
  u32 ARR_val;
  float realTimFreq;
  float minEncoEmulRef;
  float electricSpeed;
  encoEmulModeType encoEmulMode;
  u16 polePairsNum;
  volatile u16 timerState;
  TIM_TypeDef* TIMx = TIM15; //таймер, используемый для формирования сигнала эмулятора энкодера
  
  skipTime = (u16)(0.1F * 1000000.0F / encoBlockPnt->encoProcessingPeriod + 0.5F);
  
  
  if(skipCnt < skipTime){
    skipCnt++;
    return;
  }
  
  if(isnan(encoBlockPnt->calculatedData.electricSpd)){
    return;
  }
  
  electricSpeed = fabsf(encoBlockPnt->calculatedData.electricSpd) * FREQ_BASE;
  minEncoEmulRef = encoBlockPnt->minRefFreq;
  encoEmulMode = encoBlockPnt->encoEmulMode;

  if((fabsf(electricSpeed) < minEncoEmulRef) || (encoEmulMode == ENCO_EMUL_OFF)){   //Скорость равна 0 или эмулятор отключен
    ARR_val = MAX_ARR_VAL;
    TIM_SetAutoreload(TIMx, ARR_val / 2);
    TIM_SetCompare1(TIMx, ARR_val - 1);
    TIM_SetCompare2(TIMx, ARR_val - 1);
    return;
  }
  firstResetting = 0;
  //Расчет параметров модуля сравнения для обеспечения требуемой частоты сигнала эмулятора
  realTimFreq = encoBlockPnt->realTimClockFreq;                         //Текущая частота тактирования таймера
  polePairsNum  = encoBlockPnt->baseEncoMotorData.polePairsNum;         //Число пар полюсов
  encoEmulResol = 1 << encoBlockPnt->baseEncoMotorData.encoEmulResol;   //Разрешение эмулятора
  ARR_val = (u32)(realTimFreq * polePairsNum / PWM_FREQ_FACTOR / electricSpeed / encoEmulResol + 0.5F) - 1;
  ARR_val = (ARR_val > MAX_ARR_VAL) ? MAX_ARR_VAL : ARR_val;   //проверка на превышение предела
  TIM_SetAutoreload(TIMx, ARR_val);      //Настройка частоты сигнала эмулятора
  compareRegCalc(encoBlockPnt->calculatedData.electricSpd, TIMx, ARR_val); //Расчет значений регистров сравнения
  timerState = TIM_GetCR1State(TIMx);
  encoEmulOutputEnable(TIMx); //Разрешение выходов эмулятора
  if((timerState & TIM_CR1_CEN) == 0){ //Запустить таймер если остановлен
     TIM_Cmd(TIMx, ENABLE);
  }
}


/**
  * @brief  Расчет регистров сравнения таймера, формируещего сигнал эмулятора энкодера
  * @param electricSpeed  скорость вращения энкодера
  * @param ARR_val текущее значение регистра обновления таймера
  * @retval
  */
void compareRegCalc(float electricSpeed, TIM_TypeDef* TIMx, u32 ARR_val){
  u32 compare1;
  u32 compare2;
  u32 compareVal;
  s16 spdSign;
  static s16 spdSignFltState = 0;

  spdSign = (electricSpeed < 0.0F) ? NEGATIVE_SPD : POSITIVE_SPD;
  compareVal = (u32)((float)ARR_val / 2 + 0.5F);
  switch(spdSign){
  case POSITIVE_SPD:
    compare1 = 0;
    compare2 = compareVal;
    break;
  case NEGATIVE_SPD:
    compare1 = compareVal;
    compare2 = 0;
    break;
  }
  TIM_SetCompare1(TIMx, compare1);
  TIM_SetCompare2(TIMx, compare2);
}

/**
  * @brief  Установка выходов эмулятора энкодера в низкий уровень
  * @param TIMx - указатель на таймер, используемый для формирования
  *               сигнала эмулятора
  * @retval
  */
void encoEmulOutputDesable(TIM_TypeDef* TIMx){
  TIMx->CCER &= (uint32_t)~TIM_CCER_CC1E;
  TIMx->CCER &= (uint32_t)~TIM_CCER_CC2E;
}

/**
  * @brief  Разрешение выходов эмулятора энкодера
  * @param TIMx - указатель на таймер, используемый для формирования
  *               сигнала эмулятора
  * @retval
  */
void encoEmulOutputEnable(TIM_TypeDef* TIMx)
{
  if((TIMx->CCER & TIM_CCER_CC1E) == 0){
    TIMx->CCER |= TIM_CCER_CC1E;
  }
  if((TIMx->CCER & TIM_CCER_CC2E) == 0){
     TIMx->CCER |= TIM_CCER_CC2E;
  }
}


/**
  * @brief  Установка настроек эмулятора в начальный уровень
  * @param TIMx - указатель на таймер, используемый для формирования эмуляции
  * @retval
  */
void encoEmulStartSettigsSet(TIM_TypeDef* TIMx, encoBlockStatus *encoBlockPnt){
  u16 encoEmulResol;
  u16 polePairsNum;
  u32 prescalerVal;
  u32 emulPeriodReg;
  u32 prescalerRegVal;
  u16 periodRegVal;
  float timClockFreq;
  float realTimClockFreq;
  float minRefFreq;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; //!структура для базовой настройки таймера
  NVIC_InitTypeDef NVIC_InitStructure;
  
  encoEmulResol = 1 << encoBlockPnt->baseEncoMotorData.encoEmulResol; //Разрешение эмулятора энкодера
  polePairsNum =  encoBlockPnt->baseEncoMotorData.polePairsNum;  //Число пар полюсов двигателя
  
  if(polePairsNum == 0){
    return; //если данные от верхнего уровня не приняты, настройку не выполняем
  }
  
  
  //Расчитаем частоту тактирования таймера, при которой на максимально возможной частоте задания
  //80 Гц в регистре перезагрузки будет загружено минимальное значение 50.
  //Значение 50 соответствует минимальному значению, ниже которого погрешность
  //формирования периода эмуляции начнет превышать 1%
  
  timClockFreq = PWM_FREQ_FACTOR * MIN_ARR_VAL * MAX_REF_FREQ * encoEmulResol / polePairsNum; //Частота тактирования
  prescalerVal = (u16)((float)APB2Clock / timClockFreq + 0.5F);                                  //Предделитель частоты таймера
  prescalerVal = (prescalerVal == 0) ? 1 : prescalerVal;
  realTimClockFreq = (float)APB2Clock / prescalerVal;                                     //Реальная частота тактирования таймера TIM15
  encoBlockPnt->realTimClockFreq = realTimClockFreq;
  
  //Расчет значения регистра перезагрузки, при которой, при рассчитанной частоте тактирования
  //возможно будет сформировать сигнал эмуляции на минимальном задании 0,05 Гц
  emulPeriodReg = (u32)(realTimClockFreq * polePairsNum / PWM_FREQ_FACTOR / MIN_REF_FREQ / encoEmulResol + 0.5F);
  emulPeriodReg = (emulPeriodReg > MAX_ARR_VAL) ? MAX_ARR_VAL : emulPeriodReg;
  minRefFreq = realTimClockFreq * polePairsNum / PWM_FREQ_FACTOR / emulPeriodReg / encoEmulResol; //Реальная минимальное задание, для которого возможно формирование сигнала эмуляции
  encoBlockPnt->minRefFreq = minRefFreq;
  prescalerRegVal = prescalerVal - 1;
  periodRegVal = emulPeriodReg - 1;
  
  TIM_Cmd(TIMx, DISABLE);
  TIM_TimeBaseStructure.TIM_Period = periodRegVal;
  TIM_TimeBaseStructure.TIM_Prescaler = prescalerRegVal;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x00;
  TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
  TIM_PrescalerConfig(TIMx, prescalerRegVal, TIM_PSCReloadMode_Update);
  TIM_ARRPreloadConfig(TIMx, ENABLE);
  TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
  
  //Настройка прерывания
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM15_IRQn; //TIM6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE; //Прерывания запрещены
  NVIC_Init(&NVIC_InitStructure);
  
  TIM_ITConfig(TIMx, TIM_IT_CC1, DISABLE); //Прерывания запрещены
  TIM_ITConfig(TIMx, TIM_IT_CC2, DISABLE); //Прерывания запрещены
}

