#include "stm32f30x.h"
#include "enco.h"
#include "flashDrv.h"
#include "IQmathLib.h"
#include "math.h"
#include "flashDrv.h"
#include "time.h"
#include "string.h"
#define pi 3.14159265358979F
#define F_ZERO         0.02F         //!������� ��� ���������� �� ������� � ���� �� ��������� ���������� ������� ����� �������
#define ADC_SIZE_BUFFER    2
#define ADC34_SIZE_BUFFER  2

#define MODE1 0
#define MODE2 1
#define PREC  1
#define FAST_SPD_STRG_LEN  2
#define R_SIGNAL_HOLD_ITIME 0.02F //����� ��������� ����� R-������� ��� �������

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
  
  if(endat == ECN1313){ //!EnDat2.1 ������ ��������� � ������������ ���������
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
  }else{ //!���� EnDat2.2 ���������� �� EnDat2.1 ������ ��� ����� ����� ������
    code[0] = alarm1; //!!!����� ��������
    code[1] = alarm2; //!!!����� ��������
    for(i = 2; i < 34; i++) //!������ ��� ���� ������ ������ ������
    {
      code[i] = (lowpos & 0x00000001L)? 1 : 0; //!��������� � ����� ������ ��� �������
      lowpos >>=1;
    }
    for(i = 34; i < 65; i++)
    {
      code[i] = (highpos & 0x00000001L) ? 1 : 0; //!��������� � �������� ����� ������� ������� ����� �������
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
// ��������� ����������� SPI ��� ������ � EnDat
// SPI_MISO - PB4 (SPI1_MISO), PB5 (SPI1_MOSI)
// SPI_MOSI - PA7 (SPI1_MOSI)
// SPI_CLK  - PA5 (SPI1_SCK), PA2 
// SPI_CS   - PA4 (SPI1_NSS)
void EndatSpiInit (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure;

  EndatSpiData.EndatSpiState = StopMode; // ������� ��������� 
  
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
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // �������� ��� �����
  GPIO_InitStructure.GPIO_Pin = SPI1_NSS_PIN;
  GPIO_Init(SPI1_NSS_GPIO_PORT, &GPIO_InitStructure);
  
  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPI1);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_10b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //�������� ������� �� �����: 1
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //����� ������ �������
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
// ��������� ADC ��� ������ � �������-���������� ���������
// ������ � ������ DMA �� ���� 4-� �������
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
  GPIO_InitStructure.GPIO_Pin = ADCx_INPUT_SIN1_PIN;  //!PA0 - ���� ������� �  ���������� ������� (��������� ���������)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; //���������� ����
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(ADCx_INPUT_SIN1_GPIO_PORT, &GPIO_InitStructure);  

  /* Configure ADC12 Channe12 as analog input */
  GPIO_InitStructure.GPIO_Pin = ADCx_INPUT_COS1_PIN;  //!PB2 - ���� ������� B (COS) ���������� �������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(ADCx_INPUT_COS1_GPIO_PORT, &GPIO_InitStructure);  
  
    /* Configure ADC1 Channe2 as analog input */
  GPIO_InitStructure.GPIO_Pin = ADCx_INPUT_SIN2_PIN; //!PA1 ������ A ���������������� ������� sin/cos (������� ���������)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(ADCx_INPUT_SIN2_GPIO_PORT, &GPIO_InitStructure);  

  /* Configure ADC12 Channe3 as analog input */
  GPIO_InitStructure.GPIO_Pin = ADCx_INPUT_COS2_PIN; //!PA6 ������ B ���������������� ������� sin/cos
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(ADCx_INPUT_COS2_GPIO_PORT, &GPIO_InitStructure);
  
  // ��������� �� ���������� ������ ADC ---------------------------------------  
  ADC_SIN1 = (__IO uint16_t *)(&ADC1_ValueTab[0]);         //!��������� ���������   
  ADC_COS1 = ((__IO uint16_t *)(&ADC1_ValueTab[0])) + 1;   //!��������� ����������� 
  ADC_SIN2 = (__IO uint16_t *)(&ADC1_ValueTab[1]);         //!������� ���������
  ADC_COS2 = ((__IO uint16_t *)(&ADC1_ValueTab[1])) + 1;   //!������� �����������
  
#warning ����������� ����� �� ������ DISCEN (��� 362 ������� � 303CBT6)
  
  // DMA1 channel1 configuration ---------------------------------------------- 
  DMA_DeInit(ADC1_DMA_CHANNEL);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC_CDR_ADDRESS;//(uint32_t)&ADC1->DR; //!�������� ������
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC1_ValueTab; //!�������� ������ ��� DMA
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = ADC_SIZE_BUFFER; // ������ ������ ��������� ������ DMA
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word; 
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(ADC1_DMA_CHANNEL, &DMA_InitStructure);

  //DMA_ITConfig(ADC1_DMA_CHANNEL, DMA_IT_TC, ENABLE);  // �������� ��������� ���������� ��� ������ �������� ���������
  DMA_Cmd(ADC1_DMA_CHANNEL, ENABLE); //Enable the DMA1 - Channel1
    
  // Enable DMA1 Channel1
  DMA_Cmd(ADC1_DMA_CHANNEL, ENABLE);
 
  //configure ADC1 parameters
  ADC_StructInit(&ADC_InitStructure);  
  
  Delay(100000);
  
  
  //Calibration procedure 
  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
  ADC_VoltageRegulatorCmd(ADC2, ENABLE);
  
  //Insert delay equal to 10 �s
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
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;    //!DMA ����� ��������� ������� ������-common_data_reg ������ ��� ��������� ��������������� ����� ���. ��� ��-�� ����, ��� ������������ ������ � ���������� ���������� ��������������� 
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 3;          
  ADC_CommonInit(ADCx_INPUT_SIN1_ADC, &ADC_CommonInitStructure);       // �������� ��� ADC1 & ADC2

  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;         //1 ������ �� ���� ������� ��� 1-� ����. �������    //ADC_ContinuousConvMode_Enable
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_13;        //!������ ��������� �� TIM6_TRGO     //ADC_ExternalTrigConvEvent_0 //������ ������ ��������� - ����� ������� TIM6   
  
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;  //!�� �������������� ������ �� �������
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
  ADC_InitStructure.ADC_NbrOfRegChannel = 2;
  ADC_Init(ADCx_INPUT_SIN1_ADC, &ADC_InitStructure); // ADC1
  ADC_Init(ADCx_INPUT_COS1_ADC, &ADC_InitStructure);  // ADC2
  
  
  
  /*------------------------------------------------------------------------------------*/
  /*                    ��������� ������� ��������� ��� �1, ��� �2                                      */
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
//!��������� ������ ��������� ���������������� ��������

void incrementalModeInit(encoBlockStatus *encoBlockPnt)
{ 
  GPIO_InitTypeDef      GPIO_InitStructure;            //!��������� ��� ��������� ������
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      //!��������� ��� ��������� ��������
  TIM_ICInitTypeDef  TIM_ICInitStructure;         //!��������� ��� ��������� �������
  NVIC_InitTypeDef NVIC_InitStructure;
  uint16_t PeriodValue;
  uint16_t pulseResolution;
  uint16_t PrescalerValue = 0;
  
  pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
  TIM_Cmd(TIM2, DISABLE); //������� ������� TIM2
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  // ������������ ����� �
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);  // ������������ ����� B

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //!������������ ������� TIM2
   
  //!��������� ���� PA15 � �������� �������� ����� 1 ������� TIM2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //!�������������� �������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //!��������
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  //!�������� � ����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;       //!PA15
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  GPIO_PinAFConfig(GPIOA,  GPIO_PinSource15,  GPIO_AF_1); 
  
 
  // ��������� ���� PB3 � �������� �������� ����� 2 ������� TIM2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //!�������������� �������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //!��������
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  //!�������� � ����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;       //!PB3
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource3,  GPIO_AF_1); 
  

  //!��������� ���� PB8 � �������� ����� ��� ������� ����������� �����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    //!
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //!��������
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; //!�������� � ����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;       //!PB8
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource8,  GPIO_AF_1);
  
  /*��������� ������ ������� ������� TIM2 ��� �������� ������� �������� A � B*/
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // ��������� ������ ������
  TIM_ICInitStructure.TIM_ICFilter = /*10*/10;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  
  /*��������� ������� TIM2 �� ����� ������ ��������*/
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //��������� ������ ��������
  TIM_SetAutoreload(TIM2, (RESOLUTION_FACTOR * pulseResolution - 1)); //��������� �������� ���������� �������-�������� ������ ��������
  
  
  /*��������� ������� �������� R-�������**/ 
  /*������ ����� ������ ��� ���� ���� ������� ���������� �� R-�������*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);   //!������������ ������� TIM16
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  PrescalerValue = 4;     //!����� ������� ������ ������� �� 4
  PeriodValue = (uint16_t)(SystemCoreClock * PrescalerValue / encoBlockPnt->encoProcessingPeriod) - 1;
  
  TIM_TimeBaseStructure.TIM_Period  = PeriodValue;           //!�������� ������������
  TIM_TimeBaseStructure.TIM_Prescaler  = PrescalerValue - 1; //! �������� 4
  TIM_TimeBaseStructure.TIM_ClockDivision  = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);
  
  /*��������� ������ ������� ������� TIM2 ��� ��������� ������ ������ ������������ �����*/
  /*
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //����� �1:
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter = 5;    
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
  
  //����� �2
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter = 5;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
  */
  /*��������� ������ ������� ������� TIM16*/
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // ��������� ������ ������
  TIM_ICInitStructure.TIM_ICFilter = 5;   
  TIM_ICInit(TIM16, &TIM_ICInitStructure);
  
  TIM_ITConfig(TIM16, TIM_IT_CC1, ENABLE);
  TIM_Cmd(TIM16, ENABLE); //������ �������
  TIM_Cmd(TIM2, ENABLE); //������ ������� 
}


/**
  * @brief  ������ �������� ���������������� ��������
  * @param  encoBlockPnt - ��������� �� ������-��������� ����� ���������
  * @retval
  */
void incrementDataProcessing(encoBlockStatus *encoBlockPnt)
{
  float fltDiscrSpeed, ThetaElec;
  static u16 holdTimer = 0;
  u16 polePairsNum;
  float ThetaMechPU;
  
  ThetaMechPU = getDiscrThetaMechPU(encoBlockPnt);                       // ������������ ���� �� ������������ ��������
  polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
  ThetaElec = ThetaMechPU * polePairsNum;                                //������������� ���� �� ������������ ��������
  ThetaElec = getActualElectrPhase(ThetaElec, encoBlockPnt);             //���������� ������ ����
  ThetaElec = encoBlockPnt->spdPhasingParam ? (-ThetaElec) : ThetaElec;  //���� ��������� �������� � ����
  fltDiscrSpeed = discrSpdIncrCalc(ThetaMechPU, encoBlockPnt);           //�������� �� ���������� ����
  fltDiscrSpeed = encoBlockPnt->spdPhasingParam ? (-fltDiscrSpeed) : fltDiscrSpeed;  //���� ��������� �������� � ��������
    
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
  * @brief  ������ �������� � ���� ��� �������� �  ���������������� �����������
  * @param  encoBlockPnt - ��������� �� ������-��������� ����� ���������
  * @retval
  */
void serialDataProcessing(unsigned long long position, encoBlockStatus *encoBlockPnt){
   u16 digitSpdSign;
   volatile u16   encoderCRC, crcCalc; //�������� CRC ���������
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
   ThetaMechPU = EnDatEncoderMechPosCalc(encoBlockPnt, encoderPosition); //������������� ���� �� enDat
   ThetaElec = EnDatEncoderElecPosCalc(encoBlockPnt, ThetaMechPU);       //������������� ���� �� enDat
   ThetaElec = getActualElectrPhase(ThetaElec, encoBlockPnt);            //������������� ���� �� enDat
   ThetaElec = encoBlockPnt->spdPhasingParam ? (-ThetaElec) : ThetaElec; //���� ��������� ��������
   endatFltSpd = endatEncoSpdCalc(ThetaMechPU, encoBlockPnt);            //������ �������� �� �������� ����
   discrThetaMechPU = getDiscrThetaMechPU(encoBlockPnt);                 //������� ���� �� ������ ������������� ��������
   fltDiscrSpeed = incrAngleSpdCalc(discrThetaMechPU, encoBlockPnt);     //�������� �� ���� ������ ������������� ��������
   digitSpdSign = digitalPhaseInversion(encoBlockPnt, endatFltSpd, fltDiscrSpeed, &encoFlashMemData); //������������� ������ ��������� endatFltSpd � fltDiscrSpeed
   endatFltSpd = encoBlockPnt->spdPhasingParam ? (-endatFltSpd) : endatFltSpd;   //���� ��������� �������� � �������� ��������
   fltFastSpd = fastSinCosSignalSpdCalcForEnDat(encoBlockPnt);             //������ �������� �� ���������� ������� ����������
   fltFastSpd*= encoFlashMemData.fastSpdSignSetting;                  //���� ������� ��������� ����� ������� ���������� ��������
   fastSinCosSpdSignPhasing(encoBlockPnt, endatFltSpd, fltFastSpd, &encoFlashMemData, -1); //������������� ������� ���������� �������� � ��������
   fastSpdSignReadFromFlash(encoBlockPnt->drvMode);
   
   if (encoBlockPnt->baseEncoMotorData.fastSpdUse == USE){
     fltSpd = spdCalcModeValSelect(encoBlockPnt, endatFltSpd, fltFastSpd); //����� ����� �� ���� ���������: �������� ��� ����������
   }else{
     fltSpd = endatFltSpd;
   }
   encoBlockPnt->encoErr = enDatEncoErrDetect(encoBlockPnt, position);  //�������� ������ ��������
   //!������ ����������� �������� � ���������� ��������� ��������
   encoBlockPnt->calculatedData.incrementModulSpd = fltDiscrSpeed;
   encoBlockPnt->incrModulSpdPhasingStatus = digitSpdSign;
   encoBlockPnt->calculatedData.incrementModulAngle = discrThetaMechPU;
   encoBlockPnt->calculatedData.ThetaMechPU = ThetaMechPU;
   encoBlockPnt->calculatedData.ThetaMechFinePU = 0; 
   encoBlockPnt->calculatedData.ThetaMechCoarsePU = ThetaMechPU;
   encoBlockPnt->calculatedData.electricTheta = ThetaElec;                      
   encoBlockPnt->calculatedData.shadowSpeedElectric = fltSpd; // �������� ������� � ������� ���������� ��� ����������� ����� ���������� � main.c                             
   EndatSpiData.PosState = WaitPacket; //!���� ����� �����  

}

/**
  * @brief  ������ �������� � ���� ��� �������� ���� Sin/Cos
  * @param encoBlockPnt - ��������� �� ������-��������� ����������� ���������
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
       
    setBasePhasingDataBeforeStart(encoBlockPnt); //���������� ���������� ��������������� �� flash-������, ��������� ���������� �������� �������� ������������ ����
    ThetaMechPU = getDiscrThetaMechPU(encoBlockPnt);            // ������������ ���� �� ������������ ��������
    polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
    ThetaElec = ThetaMechPU * polePairsNum;           //������������� ���� �� ������������ ��������
    thetaOffset = (float)encoBlockPnt->thetaOffset / 3600.0F;
    ThetaElec += thetaOffset;                                   //�������� �������� 
    ThetaElec = getActualElectrPhase(ThetaElec, encoBlockPnt);  //���������� ������ ���� ����� ��������
    ThetaElec = encoBlockPnt->spdPhasingParam ? (-ThetaElec) : ThetaElec;  //���� ��������� �������� � ����
    fltDiscrSpeed = discrSpdCalc(ThetaMechPU, encoBlockPnt);         //�������� �� ���������� ����
    analogThetaMechPU = getAnalogThetaMechPU(encoBlockPnt);          //������������ ���� �� ���������� �������� SIN � COS
    fltAnalogSpeed = analogSpdCalc(analogThetaMechPU, encoBlockPnt); //�������� �� ���������� ����
    fltFastSpd = fastSinCosSignalSpdCalc(encoBlockPnt);              //������ �������� �� ���������� ������� ����������
    fltFastSpd*= fastSpdSignDef(encoBlockPnt, encoFlashMemData.fastSpdSignSetting); //���� ����� ���������� ������������ ��������
    digitSpdSign = digitalPhaseInversion(encoBlockPnt, fltAnalogSpeed, fltDiscrSpeed, &encoFlashMemData);
    encoBlockPnt->incrPosPhasingDone = digitSpdSign;
    encoBlockPnt->incrPosPhasingDone = (encoBlockPnt->PWMOn == 0) ? NOT_DETECT : encoBlockPnt->incrPosPhasingDone;
    fastSinCosSpdSignPhasing(encoBlockPnt, fltDiscrSpeed, fltFastSpd, &encoFlashMemData, digitSpdSign); //������������� ������� ���������� �������� � �������� 
    phaseShiftDetect(encoBlockPnt, &encoFlashMemData);  //���������� �������� ������ ����� ���������� ����� � ��������
     if(encoBlockPnt->baseEncoMotorData.fastSpdUse == NOT_USE){ //���� ���������� ������������ �������� �� ������������
       fltSpd = fltDiscrSpeed;      //�������� ������ � �������� ��������
    }else{
       fltSpd = spdCalcModeValSelect(encoBlockPnt, fltDiscrSpeed, fltFastSpd); //����� ����� �� ���� ���������: �������� ��� ���������� 
    }
    fltSpd = encoBlockPnt->spdPhasingParam ? (-fltSpd) : fltSpd;  //���� ��������� �������� � ��������
    //fltSpd = velocityMedianFilter(fltSpd);
    
    encoBlockPnt->encoErr = sinCosEncoErrDetect(encoBlockPnt, fltSpd, encoFlashMemData.phaseShift); //�������� ������ �������� ���� sin/cos 
    encoBlockPnt->calculatedData.shadowSpeedElectric = fltSpd;
    encoBlockPnt->calculatedData.ThetaMechPU = ThetaMechPU;
    encoBlockPnt->calculatedData.electricTheta = ThetaElec;
}



/**
  * @brief  ���������� ������� �������� �� Flash-������ ��� ��������� ����������� ������������� 
  *         ������ �������� ��� ������ ������� ��� ����� ������ ����������. ��������� ���������� �������� ��������
  *         ������������� ������ ��������.
  * @param  encoBlockPnt - ��������� �� ������-��������� ����������� ���������
  * @retval
  */
void setBasePhasingDataBeforeStart(encoBlockStatus *encoBlockPnt){
  u16 pulseResolution;
  float ThetaMechPU;
  s16 incrPosStartVal;
  u32 cntState;
  static u16 baseDataDoneInScalar = 0;
  static u16 baseDataDoneInVect   = 0;
  
  pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution; //����������, �������� / ������
  baseDataDoneInVect = (encoBlockPnt->encoErr != ENCO_OK)? 0 : baseDataDoneInVect;
  baseDataDoneInScalar = (encoBlockPnt->encoErr != ENCO_OK)? 0 : baseDataDoneInScalar;
  
  switch(encoBlockPnt->drvMode){
  case SCALAR_MODE:
       baseDataDoneInVect = 0;
       if(!baseDataDoneInScalar){ //��������� �� flash-������ ������ ���� ���
          encoFlashMemData.encoInputSetting = getEncoInputSettFromFlash(); //��������� ��������� ����� ����������� ������ ��������
          encoFlashMemData.phaseShift = corrPhaseRead(BASE_PAGE_ADDR); //������� �������� �� FLASH; //������� ������� � ����, �������������� �� ���������� �������� sin/Cos
          encoFlashMemData.fastSpdSignSetting  = getFastSpdSignFromFlash(); //������� ��������� ����� ������� ���������� ��������
          ThetaMechPU = getAnalogThetaMechPU(encoBlockPnt); //������� ���� �� ���������� �������� sin/cos
          ThetaMechPU += encoFlashMemData.phaseShift;  //�������������� ����
          ThetaMechPU = removeExcessPhase(ThetaMechPU);//�������� ����������� ��������� ���� ����� �������� ��������
          incrPosStartVal = (s16)(ThetaMechPU * (RESOLUTION_FACTOR * pulseResolution) + 0.5F); //������� �������� ������������ ����
          while(TIM1->CR1 & TIM_CR1_CEN == 0){ //���� ������ �������
            ;
          }
          TIM1->CR1 &= ~TIM_CR1_CEN; //������� �������
          while(TIM1->CR1 & TIM_CR1_CEN == 1){ //���� ������� �������
            ;
          }
          TIM_SetCounter(TIM1, incrPosStartVal); //��������� �������� ������ �������� � ��������� ��������
          cntState = TIM_GetCounter(TIM1);       //�������� �� ���������� ���������
          while(cntState != incrPosStartVal){ //���� ��������� �������
            ;
          }
          
          TIM1->CR1 |= TIM_CR1_CEN; //������ �������
          baseDataDoneInScalar = 1;//((cntState == incrPosStartVal) && (encoBlockPnt->encoErr == ENCO_OK)) ? 1 : baseDataDoneInScalar;
       }
    break;
  case VECTOR_MODE:
       baseDataDoneInScalar = 0;
       if(!baseDataDoneInVect){ //��������� ����� �� flash-������ ������ ���� ���
          encoFlashMemData.encoInputSetting = getEncoInputSettFromFlash();  //��������� ��������� ����� ����������� ������ ��������
          encoFlashMemData.phaseShift = corrPhaseRead(BASE_PAGE_ADDR);           //��������, ���������� ��� �������������
          encoFlashMemData.fastSpdSignSetting  = getFastSpdSignFromFlash(); //������� ��������� ����� ������������ ���������� ��������
          ThetaMechPU = getAnalogThetaMechPU(encoBlockPnt); //������� �������� ������������ ���� �� SIN � COS
          ThetaMechPU += encoFlashMemData.phaseShift;  //������� �������� ������������ ���� + �������������� ��������
          ThetaMechPU = removeExcessPhase(ThetaMechPU);//�������� ����������� ��������� ���� ����� �������� ��������
          incrPosStartVal = (s16)(ThetaMechPU * (RESOLUTION_FACTOR * pulseResolution) + 0.5F); //�������� �������� ������������ ����
          TIM_SetCounter(TIM1, incrPosStartVal); //��������� �������� � ��������� ��������
          cntState = TIM_GetCounter(TIM1);       //�������� �� ���������� ���������
          baseDataDoneInVect = ((cntState == incrPosStartVal) && (encoBlockPnt->encoErr == ENCO_OK))? 1 : baseDataDoneInVect;   
       }
    break;
  }
}



/**
  * @brief  ����������� ���������� ����� ������� �������� �� FLASH-������ ��� ����� ������ ���������� ��
  * @param  drvMode - ������� ����� ���������� ��
  * @param  scalarPhasingDataReadFlg - ��������� �� ���� ��������� ������ � ��������� ������
  * @param  vectorPhasingDataReadFlg - ��������� �� ���� ��������� ������ � ��������� ������
  * @retval
  */
void fastSpdSignReadFromFlash(drvModeType drvMode){
  static u16 scalarPhasingDataReadFlg = 0;
  static u16 vectorPhasingDataReadFlg = 0;
  
  switch(drvMode){
  case SCALAR_MODE:
    vectorPhasingDataReadFlg = 0;
    if(scalarPhasingDataReadFlg == 0){ //��������� ����� �� flash-������ ������ ���� ���
      encoFlashMemData.fastSpdSignSetting  = getFastSpdSignFromFlash(); //������� ��������� ����� ������� ���������� �������� �� Flash-������
      scalarPhasingDataReadFlg = 1;
    }
    break;
  case VECTOR_MODE:
    scalarPhasingDataReadFlg = 0;
    if(vectorPhasingDataReadFlg == 0){ //��������� ����� ������� �������� �� flash-������ ������ ���� ���
        encoFlashMemData.fastSpdSignSetting  = getFastSpdSignFromFlash(); //������� ��������� ����� ������� ���������� ��������
        vectorPhasingDataReadFlg = 1;
    }
    break;
  }
}


// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 


// = = = = = = ������ = = = = = = = = = = = = = = = = =
// = = = = = = ������ = = = = = = = = = = = = = = = = =
// = = = = = = ������ = = = = = = = = = = = = = = = = =
// = = = = = = ������ = = = = = = = = = = = = = = = = =
// ��������� ����������� SPI ��� ������ � 
// SPI_MISO - PB4 (SPI1_MISO), PB5 (SPI1_MISO)
// SPI_MOSI - PA7 (SPI1_MOSI)
// SPI_CLK  - PA5 (TIM2_CH1), PA2 (TIM2_CH3)  !!!!
// SPI_CS   - PA4 (SPI1_NSS)
void EndatTimInit (void)
{ // ������ ��� ������ Endat
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
    
#define PERIOD 72*2
  
  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA and GPIOB clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
  
  // �������� ���� ���������� �� ������ ����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;

  // ��7 - ����� ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  // ��4 - ����������� ��������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  // �B4 - ���� ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  
  // ��������� �����
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
  
  TIM_OCInitStructure.TIM_OCMode = TIM_ForcedAction_Active; //TIM_OCMode_Toggle; // ���������� ������ � 1.
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
  * @brief  ��������� ������ ������������� �������� ��� ��������� sin/cos ��������
  * @param  locEncoder: ��������� � ������� ��������
  * @retval 
  */


void sinCosDigitModeInit(encoBlockStatus *encoBlockPnt)
{
  GPIO_InitTypeDef      GPIO_InitStructure;            //!��������� ��� ��������� ������
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      //!��������� ��� ��������� ��������
  TIM_ICInitTypeDef  TIM_ICInitStructure;         //!��������� ��� ��������� �������
  NVIC_InitTypeDef NVIC_InitStructure;
  uint16_t encoInputSetting = 0;
  uint16_t PeriodValue;
  uint16_t pulseResolution;
  uint16_t PrescalerValue = 0;
  
  pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
  TIM_Cmd(TIM1, DISABLE); //������� ������� TIM1
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  // ������������ ����� �
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //!������������ ������� TIM1
   
  //!��������� ���� PA8 � �������� �������� ����� 1 ������� TIM1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //!�������������� �������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //!��������
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  //!�������� � ����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;       //!PA8
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  GPIO_PinAFConfig(GPIOA,  GPIO_PinSource8,  GPIO_AF_6); 
  
 
  /*��������� ���� PA9 � �������� �������� ����� 2 ������� TIM1*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //!�������������� �������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //!��������
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  //!�������� � ����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;       //!PA9
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  GPIO_PinAFConfig(GPIOA,  GPIO_PinSource9,  GPIO_AF_6); 
  
  
   /*���� PB7 ���������� ��� R-������� ��������*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    //!�������������� �������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //!��������
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; //!�������� ����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;      //!PB7
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource7,  GPIO_AF_10); 
  
  /*��������� ������ ������� ������� TIM1 ��� �������� ������� �������� A � B*/
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // ��������� ������ ������
  TIM_ICInitStructure.TIM_ICFilter = /*10*/10;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  
  /*��������� ������� TIM1 �� ����� ������ ��������*/
  encoInputSetting = getEncoInputSettFromFlash(); //��������� ��������� �������� ����� �� FLASH-������
  encoInputSetting = (encoInputSetting == 0) ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
  TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, encoInputSetting, TIM_ICPolarity_Rising); //��������� ������ ��������
  TIM_SetAutoreload(TIM1, (RESOLUTION_FACTOR * pulseResolution - 1)); //��������� �������� ���������� �������-�������� ������ ��������
  
  
  /*��������� ������� �������� R-�������**/ 
  /*������ ����� ������ ��� ���� ���� ������� ���������� �� R-�������*/
  
  /*���������� ���������� ���������� �� ������� TIM3*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);   //!������������ ������� TIM3
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  PrescalerValue = 4;     //!����� ������� ������ ������� �� 4
  PeriodValue = (uint16_t)(SystemCoreClock * PrescalerValue / encoBlockPnt->encoProcessingPeriod) - 1;
  
  TIM_TimeBaseStructure.TIM_Period  = PeriodValue;           //!�������� ������������
  TIM_TimeBaseStructure.TIM_Prescaler  = PrescalerValue - 1; //! �������� 4
  TIM_TimeBaseStructure.TIM_ClockDivision  = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  
  /*��������� ������ ������� ������� TIM1 ��� ��������� ������ ������ ������������ �����*/
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //����� �1:
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = encoInputSetting;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter = 5;    
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  TIM_ITConfig(TIM1, TIM_IT_CC1, /*ENABLE*/DISABLE);
  
  //����� �2
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter = 5;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  TIM_ITConfig(TIM1, TIM_IT_CC2, /*ENABLE*/DISABLE);
  
  /*��������� ������ ������� ������� TIM3*/
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // ��������� ������ ������
  TIM_ICInitStructure.TIM_ICFilter = 5;   
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  
  TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
  TIM_Cmd(TIM3, ENABLE); //������ ������� TIM3
  TIM_Cmd(TIM1, ENABLE); //������ ������� TIM1 
 
}


/**
  * @brief  PLL ���������� ���� �������� ���� sin/cos
  * @param  ������� �������� - ��������������� ������������ ����
  * @retval �������� �������� - ������������� ����
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
  * @brief  ������ ������������ ���� �� ���������� �������� sin � cos
  * @param  encoBlockPnt - ��������� �� ��������� �������� � �������� ���������� SIN � COS � ������� ���
  * @retval ThetaMechPU - ������������ ����, ���������� �� �������� SIN � COS
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
  u16    numFinePeriod;    //!����� ����� �������� ������������ ��������
  float ThetaMechPU;
  u16 encoResolution;
  
  fastSin = encoBlockPnt->analogSignals.fastSin;
  fastCos = encoBlockPnt->analogSignals.fastCos;
  slowSin = encoBlockPnt->analogSignals.slowSin;
  slowCos = encoBlockPnt->analogSignals.slowCos;
  encoResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
  
  //������ ���� ������ ������������ (�������) ���������
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
  
   //������ ���� ������ ��������� ��������� (������ ���������� ������������ �������)//
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
   
   fThetaMechCoarsePU = fThetaMechCoarse/(2*pi);              //! ������������� ������  ������������ ���� � double
   minResolution = 1.0F/(encoResolution);                      //!����� �������, �����. ����� ������ ������������ ���������
   numFinePeriod = (u16)(fThetaMechCoarsePU / minResolution); //!������� ����� ���������� ������� ��������
   fThetaMechCoarsePU = numFinePeriod * minResolution;        //!�������. ���� ���������, �����. ���������������� ������ ���-�� ������������ ��������   
   fThetaMechFine = fPrecTheta/(encoResolution);              //!���������� ������ ������������ ���� (���.) � float (1 << encoder.BitResolution - ���������� ������� �������� �� ������)
   fThetaMechFinePU = fThetaMechFine/(2*pi);                  //!������������� ������ ������������ ���� � float  
   ThetaMechPU = fThetaMechCoarsePU + fThetaMechFinePU;       //!�������� �������� ������������ �������
   return ThetaMechPU;
}

/**
  * @brief  ��������� ���������� ������������� ���� � ������ ����� ��� �������
  * @param  sPnt - ��������� �� ��������� ��������
  * @retval ThetaElec - ������������� ���� � ������ ����� ��� �������
  */

float getActualElectrPhase(float ThetaElec, encoBlockStatus *encoBlockPnt){
   uint8_t i;
   
   i = encoBlockPnt->baseEncoMotorData.polePairsNum; //����� ��� �������
   while(i)                                          // ���������� ������ ����
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
  * @brief  ��������� ������ ��������
  * @param  velocity - ������� ��������
  * @retval fltVelocity - ������������� ��������
  */
#define CAPT_STRG_LEN 4                  //������ ��������� ���������� �������
float velocityMedianFilter(float velocity){
   volatile u16 debugVal;
   u16 pnt;
   u16 pnt2;
   float tmp, fltVelocity;
   static float captureStrg[CAPT_STRG_LEN] = {0}; //��������� �������� ������� �������
   static float sortedValStrg[CAPT_STRG_LEN] = {0};      //��������� �������� ������� �������
   static SINCOSBUF sinCosIncr = {captureStrg, sortedValStrg, CAPT_STRG_LEN, 0}; //��������� ���������� ��������� �������
   
  //��������� � ��������� �����
   *(sinCosIncr.captureStrg + sinCosIncr.storagePos)= velocity;               //!������� ����������� �������� � ��������� �����
   sinCosIncr.storagePos=(sinCosIncr.storagePos+1)&(sinCosIncr.storageLen-1);  //!����� �������� ������� � ���������
   
   //�������� ����������� �������� � ������ ��� ����������
   memcpy(&sinCosIncr.sortedCaptStrg[0], &sinCosIncr.captureStrg[0], sizeof(sortedValStrg));
   
   //����������� ���������� ������� �������� ��������
   for(pnt = 0; pnt < sinCosIncr.storageLen; pnt++){
     for(pnt2 = 0; pnt2 < sinCosIncr.storageLen - 1 - pnt; pnt2++){
       if(sinCosIncr.sortedCaptStrg[pnt2] > sinCosIncr.sortedCaptStrg[pnt2+1]){
         tmp = sinCosIncr.sortedCaptStrg[pnt2];
         sinCosIncr.sortedCaptStrg[pnt2] = sinCosIncr.sortedCaptStrg[pnt2+1];
         sinCosIncr.sortedCaptStrg[pnt2+1] = tmp;
       }
     }
   }
   
   //������������� �������� ������� �������
   fltVelocity = sinCosIncr.sortedCaptStrg[sinCosIncr.storageLen/2] + sinCosIncr.sortedCaptStrg[sinCosIncr.storageLen/2 - 1];
   fltVelocity =  fltVelocity / 2.0F;
   return fltVelocity;
}

/**
  * @brief  �������� �������� ����� ���������� ��������
  * @param  
  * @retval 0 - ���� ����������� ����� �������� ������, 1 - ���� ��������
  */

uint8_t encoderInterfacePhaseShift(void){
   uint32_t CCERState;
   CCERState = TIM1->CCER; //������� ���������
   //TIM_Cmd(TIM1, DISABLE); //������� ������� TIM1 
   if (CCERState & TIM_CCER_CC1P){//���� �������� ����� ��������, ��������� ��
     TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
     TIM_Cmd(TIM1, ENABLE); //������ ������� TIM1 
     return(0);
   }else{ //���� �������� ����� ���������, �������� ��
     TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Rising);
     //TIM_Cmd(TIM1, ENABLE); //������ ������� TIM1 
     return(1);
   } 
}


/**
  * @brief  ������� ������� �������� �� ���������� ������������� ����
  * @param  analogThetaMechPU - ������� �������� ������������ ����
  * @param  encoBlockPnt - ��������� �� ��������� ��������
  * @retval 
  */
float analogSpdCalc(float analogThetaMechPU, encoBlockStatus *encoBlockPnt){
   u16 polePairsNum;
   u16 encoProcessingPeriod;
   float angleDiff, K;
   static float encoFltStrg[64]={0};           //!����� ��� ����������  ���������� ����
   static ENCOFLT encoFlt = {encoFltStrg, 32, 0, 0, 0};  //!��������� � ������� ��� ������� �������� �� ���������� ����
   
   encoProcessingPeriod = encoBlockPnt->encoProcessingPeriod;
   polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
   encoFlt.storageLen = 64/*encoder->EncoFlt_N*/;
   angleDiff = analogThetaMechPU - (*(encoFlt.thetaStrg + encoFlt.storagePos)); //!���������� ����������� ����
   *(encoFlt.thetaStrg + encoFlt.storagePos)= analogThetaMechPU;            //!��������� ����������� ����
   encoFlt.storagePos=(encoFlt.storagePos+1)&(encoFlt.storageLen-1);        //!����� �������� ������� � ��������� ���������� ����
   // �������� �������� ����� �������
   if(angleDiff < -0.8F) {
     angleDiff += 1.0F;
   }
   if(angleDiff > 0.8F) {//��� �������
      angleDiff -= 1.0F;
   }
   encoFlt.TmpFltr = encoFlt.TmpFltr + ((angleDiff - encoFlt.TmpFltr) * 0.125F);
     // ������������ ������������� �������� � ������ ��� �������.
   K = (float)polePairsNum * 1000000.0F / (encoProcessingPeriod * FREQ_BASE * encoFlt.storageLen); 
   encoFlt.fltSpeed = K * encoFlt.TmpFltr;
   return(encoFlt.fltSpeed); //�������� �� ���������� ����
}


/**
  * @brief  ������ �������� �� ���������� ������������� ����
  * @param  ������� �������� ������������ ���������� ����
  * @param  ��������� �� ��������� ��������
  * @retval �������� �� ������������ ����
  */
float discrSpdCalc(float discrThetaMechPU, encoBlockStatus *encoBlockPnt){
   float angleDiff;
   u16 polePairsNum;
   u16 encoProcessingPeriod;
   static float K = 0;
   static float encoFltStrg[64]={0};           //!����� ��� ����������  ���������� ����
   static ENCOFLT encoFlt = {encoFltStrg, 32, 0, 0, 0};  //!��������� � ������� ��� ������� �������� �� ���������� ����
   
   encoProcessingPeriod = encoBlockPnt->encoProcessingPeriod;
   polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
   encoFlt.storageLen = encoBlockPnt->fltStrgLen;                              //!������ ��������� ������� ����
   angleDiff = discrThetaMechPU - (*(encoFlt.thetaStrg + encoFlt.storagePos)); //!���������� ����
   *(encoFlt.thetaStrg + encoFlt.storagePos)= discrThetaMechPU;                //!��������� ����������� ����
   encoFlt.storagePos=(encoFlt.storagePos+1)&(encoFlt.storageLen-1);           //!����� �������� ���� � ��������� ����
   // �������� �������� ����� �������
   if(angleDiff < -0.8F) {
     angleDiff += 1.0F;
   }
   if(angleDiff > 0.8F) {//��� �������
      angleDiff -= 1.0F;
   }
   encoFlt.TmpFltr = encoFlt.TmpFltr +((angleDiff - encoFlt.TmpFltr) * 0.125F);
     // ������������ ������������� �������� � ������ ��� �������.
   if(encoBlockPnt->PWMOn == 0){
     K = (float)polePairsNum * 1000000.0F / (encoProcessingPeriod * FREQ_BASE * encoFlt.storageLen); 
   }
   encoFlt.fltSpeed = K * encoFlt.TmpFltr;
   return(encoFlt.fltSpeed); //�������� �� ���������� ����
}

/**
  * @brief  ������ �������� �� ���������� ������������� ���� ��� ���������������� ��������
  * @param  ������� �������� ������������ ���������� ����
  * @param  ��������� �� ��������� ��������
  * @retval �������� �� ������������ ����
  */
float discrSpdIncrCalc(float discrThetaMechPU, encoBlockStatus *encoBlockPnt)
{
   float angleDiff;
   u16 polePairsNum;
   u16 encoProcessingPeriod;
   static float K = 0;
   static float encoFltStrg[64]={0};           //!����� ��� ����������  ���������� ����
   static ENCOFLT encoFlt = {encoFltStrg, 32, 0, 0, 0};  //!��������� � ������� ��� ������� �������� �� ���������� ����
   
   encoProcessingPeriod = encoBlockPnt->encoProcessingPeriod;                  //������ ������� ������ ��������
   polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
   encoFlt.storageLen = encoBlockPnt->fltStrgLen;                              //������ ��������� ������� ����
   angleDiff = discrThetaMechPU - (*(encoFlt.thetaStrg + encoFlt.storagePos)); //���������� ����
   *(encoFlt.thetaStrg + encoFlt.storagePos)= discrThetaMechPU;                //��������� ����������� ����
   encoFlt.storagePos=(encoFlt.storagePos+1)&(encoFlt.storageLen-1);           //����� �������� ���� � ��������� ����
   // �������� �������� ����� �������
   if(angleDiff < -0.8F) {
     angleDiff += 1.0F;
   }
   if(angleDiff > 0.8F) {//��� �������
      angleDiff -= 1.0F;
   }
   encoFlt.TmpFltr = encoFlt.TmpFltr +((angleDiff - encoFlt.TmpFltr) * 0.125F);
     // ������������ ������������� �������� � ������ ��� �������.
   if(encoBlockPnt->PWMOn == 0){
     K = (float)polePairsNum * 1000000.0F / (encoProcessingPeriod * FREQ_BASE * encoFlt.storageLen); 
   }
   encoFlt.fltSpeed = K * encoFlt.TmpFltr;
   return(encoFlt.fltSpeed); //�������� �� ���������� ����  
}

/**
  * @brief  ������ ���� �� ������������ ��������
  * @param  ��������� �� ��������� ��������
  * @retval ������� ���������� (������������)����
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
    incrementPosCnt = encoBlockPnt->incrEncoPos; //������� �������� �������� ������ ��������
    incrementPosCnt &= (encoderResolution - 1); //������������ ��������
  }else if(blockType == INCREMENTAL){
    encoderResolution = RESOLUTION_FACTOR * encoBlockPnt->baseEncoMotorData.pulseResolution;
    incrementPosCnt = encoBlockPnt->incrEncoPos; //������� �������� �������� ������ ��������
    incrementPosCnt = (incrementPosCnt > (encoderResolution - 1)) ? (encoderResolution - 1) : incrementPosCnt;
  }else{ //Serial
    encoderResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
    incrementPosCnt = encoBlockPnt->incrEncoPos; //������� �������� �������� ������ ��������
    incrementPosCnt &= (encoderResolution - 1); //������������ ��������
  }
  
  ThetaMechPU = (float)incrementPosCnt / encoderResolution; //������ ���. ���� �� �������� ��������
  return(ThetaMechPU);
}



/**
  * @brief  �������� ������� ���������� �� R-�������
  * @param  
  * @retval ���� ����������� ������������ �������
  */
extern u16 R_Event; //���� ������� R-�������
u16 RsignalEventPresent(){
  return R_Event;
}

/**
  * @brief  ����� ����� ������� ���������� �� R-�������
  * @param  
  * @retval
  */
void RsignalEventAck(){
  R_Event = 0;
}

/**
  * @brief  ���������� ������������ ���� ��-�� ����������� ��������
  * @param  shiftPhase - ������� ���� �� ���������
  * @retval ������� ���� � ��������� 0...2pi
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
  * @brief  �������� �������� ����������� ���� �� ������������ ��������� 
  * @param shiftPhase ������� ����������� ������� ����
  * @param compareAngle �������� ��� ���������
  * @retval ��������� ��������
  */
/*
#define DELTA_PHASE 46603L //IQ(1/360) - ����������� ��������� ���������� ����������� ����
uint8_t phaseRangCheck(_iq shiftPhase, _iq compareAngle){
  if((shiftPhase >= (compareAngle - DELTA_PHASE)) && (shiftPhase <= (compareAngle + DELTA_PHASE)))
}
*/

uint8_t getEncoInputSettFromFlash(){
  _iq inputSetting;
  inputSetting = flashInputSet_read(BASE_PAGE_ADDR); //������� �������� �� FLASH
  if((inputSetting == NO_INIT_VAL) || (inputSetting == 0)){ //���� FLASH-������ �� ����������������..
    return(0); 
  }else{
    return(1);
  }
}



/**
  * @brief  �������� ���������� ��������������� ��������� �������� � ��������� �������� �������� ���� � ������ ���������� ����������
  * @param encoStatus - ������� ������ ������ ��������
  * @param encoder - ��������� �� ��������� ��������� ���������
  * @param phaseAdd - �������������� ������� �����, ���������� �� ����� ������������� ��������������� 
  * @retval ���� ������� ������ "��� ��������� ��������"
  */

#define UNDER_VAL  0.98F
#define OVER_VAL   0.02F
#define ALLOW_R_PERIOD (8 * 1000U / encoBlockPnt->encoProcessingPeriod) //8 ��
#define R_HOLD_TIME    (12 * 1000U / encoBlockPnt->encoProcessingPeriod) //12 ��

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
   /*                �������� �������� ������ � ������ ����������� R-�����          */
   /*-------------------------------------------------------------------------------*/
   phaseVal1 = (encoBlockPnt->PWMOn == 0) ? UNDER_VAL : phaseVal1;
   phaseVal2 = (encoBlockPnt->PWMOn == 0) ? OVER_VAL : phaseVal2;
   R_SignalErrStatus = (encoBlockPnt->PWMOn == 0) ? ENCO_OK : R_SignalErrStatus;
  
   underCntVal = (u16)(UNDER_VAL * RESOLUTION_FACTOR * pulseResolution); //���������� ���������� �������� ����� �� 360 ���� (8192 ���)
   overCntVal  = (u16)(OVER_VAL * RESOLUTION_FACTOR * pulseResolution);  //���������� ���������� �������� ������ �� 360 ���� (8192 ���)
  
   RtimeEn = (RtimeEn > 0) ? (RtimeEn - 1) : RtimeEn; //������ �������� ���������� ����� ���������� ����������� R-�������

   
   if(RsignalEventPresent()){//���� ������� R-������
     if(RtimeEn == 0){ //���� R-������ �������� ����� ���������� �����
       RtimeEn = ALLOW_R_PERIOD;
       RStateCnt = R_HOLD_TIME;
       encoBlockPnt->RsignalFlg = 1; //��� �������
       RsignalEventAck(); //����� ����� ���������� �� ������������ �������  
       TIM1Val = TIM_GetCounter(TIM1); //������� �������� �������� � ������ ����������� R-�����
       shiftPhase = getAnalogThetaMechPU(encoBlockPnt); //������� ������� ����� � ������ R-����������
       shiftPhase += phaseAdd; //���� ��������������� ��������
       shiftPhase = removeExcessPhase(shiftPhase);//�������� ����������� ��������� ���� ����� �������� ��������
       if(!(((shiftPhase >= phaseVal1) && (shiftPhase < 1.0F)) || ((shiftPhase >= 0.0F) && (shiftPhase < phaseVal2)))){
         if((shiftPhase >= (0.5F - DELTA_PHASE)) && (shiftPhase <= (0.5F + DELTA_PHASE))){ //����� �� +180 ��������
            R_SignalErrStatus = INCORRECT_ANGLE_AT_R_ERR;
         }else if ((shiftPhase >= (0.75F - DELTA_PHASE)) && (shiftPhase <= (0.75F+ DELTA_PHASE))){ //����� �� 90 ����
            R_SignalErrStatus = INCORRECT_ANGLE_AT_R_ERR;
         }else if ((shiftPhase >= (0.25F - DELTA_PHASE)) && (shiftPhase <= (0.25F + DELTA_PHASE))) {  //����� �� 90 ����
            R_SignalErrStatus = INCORRECT_ANGLE_AT_R_ERR;
         }  
       }
     }else{
       RsignalEventAck();
     }
   
     //-----------�������� ������������� ������������ ��������------------//
     if((TIM1Val >= underCntVal) && (TIM1Val < RESOLUTION_FACTOR * pulseResolution)){ //���������� �������� ��������
        ;
     }else if((TIM1Val >= 0) && (TIM1Val < overCntVal)){                                    //���������� �������� ��������
       ;
     }else{ //�������� �������� ���� ������ ��������� �� ����������� ����
       R_SignalErrStatus = (R_SignalErrStatus != INCORRECT_ANGLE_AT_R_ERR) ? INCORRECT_INCR_ABS_PHASE_DIFF_ERR : R_SignalErrStatus;
     } 
     
   }else{
     RStateCnt = (RStateCnt != 0) ? (RStateCnt - 1) : RStateCnt; //�������� ����� ���������� ����� R-�������
     encoBlockPnt->RsignalFlg = (RStateCnt == 0) ? 0 : encoBlockPnt->RsignalFlg;
   }
   
   /*-------------------------------------------------------------------------------*/
   /*                ��������� �������� �������� ����                               */
   /*-------------------------------------------------------------------------------*/
   if((!encoBlockPnt->PWMOn) && (TIM1Val != 0)){ //��� �������� � ���� ������� �������� � ������ R-�����
     if((TIM1Val >= underCntVal) && (TIM1Val < RESOLUTION_FACTOR * pulseResolution)){ //���� �������� ��������
        dif = RESOLUTION_FACTOR * pulseResolution - TIM1Val; //8192 - TIM1Val
        tmp = TIM_GetCounter(TIM1);
        tmp = tmp + dif;
        tmp &= (RESOLUTION_FACTOR * pulseResolution - 1);
        TIM_SetCounter(TIM1, tmp);  
        TIM1Val = 0; //������� ������
     }else if((TIM1Val > 0) && (TIM1Val < overCntVal)){ //���� �������� ��������
        tmp = TIM_GetCounter(TIM1);
        tmp -= TIM1Val;
        tmp = (tmp < 0) ? tmp + RESOLUTION_FACTOR * pulseResolution : tmp;
        tmp &= (RESOLUTION_FACTOR * pulseResolution - 1);
        TIM_SetCounter(TIM1, tmp);
        TIM1Val = 0; //������� ������
     }
   }
   errStatus = (R_SignalErrStatus != 0) ? R_SignalErrStatus : errStatus;
   return errStatus; 
}


/**
  * @brief  �������� ������� ����������� ����� �������� � ���������� ������,
  *         �������� ��������� ������� ������������ ���������.
  * @param fltAnalogSpeed - ��������, ������������ �� ���������� �������� sin � cos
  * @param fltDiscrSpeed - ��������, ������������ �� ��������� ������� �� ����������� ��������
  * @param pLocNvData - ��������� �� ������ � ������������������ ������� ������������� �����������
  * @retval ���� ����������� ������������� ���������
  */


#define MIN_PHASING_SPD (0.2F / FREQ_BASE)
#define ANALOG_SPD_FLT_TIME  ((uint16_t)(2 * 100000UL / encoBlockPnt->encoProcessingPeriod)) // 0.2 ���

u16 digitalPhaseInversion(encoBlockStatus *encoBlockPnt, float fltAnalogSpeed, float fltDiscrSpeed, encoFlashMemDataType *pLocNvData)
{
  
   static u16 spdSynhrCnt1 = 0; //������� ��������� fltAnalogSpeed > 0 � fltDiscrSpeed < 0
   static u16 spdSynhrCnt2 = 0; //������� ��������� fltAnalogSpeed < 0 � fltDiscrSpeed > 0
   static u16 spdSynhrCnt3 = 0; //������� ��������� fltAnalogSpeed < 0 � fltDiscrSpeed < 0
   static u16 spdSynhrCnt4 = 0; //������� ��������� fltAnalogSpeed > 0 � fltDiscrSpeed > 0
   u16 digitSpdSign = NOT_DETECT;
   
   if(encoBlockPnt->drvMode == VECTOR_MODE){
     return NOT_DETECT; //� ��������� ������ ��������������� �� �����������
   }
   /*����� ��������� ��������� ��� ����������� ���*/
   spdSynhrCnt1 = (encoBlockPnt->PWMOn == 0) ? 0 : spdSynhrCnt1;
   spdSynhrCnt2 = (encoBlockPnt->PWMOn == 0) ? 0 : spdSynhrCnt2;
   spdSynhrCnt3 = (encoBlockPnt->PWMOn == 0) ? 0 : spdSynhrCnt3;
   spdSynhrCnt4 = (encoBlockPnt->PWMOn == 0) ? 0 : spdSynhrCnt4;
   
  if((fabsf(fltAnalogSpeed) >= MIN_PHASING_SPD) && (fabsf(fltDiscrSpeed) >= MIN_PHASING_SPD)){ //�������� ������ ����� 0,2 ��
       if((fltAnalogSpeed > 0) && (fltDiscrSpeed < 0)){ //���� ����� ���������� � �������� �������� ����������
           spdSynhrCnt2 = (spdSynhrCnt2 > 0) ? --spdSynhrCnt2 : spdSynhrCnt2;
           spdSynhrCnt3 = (spdSynhrCnt3 > 0) ? --spdSynhrCnt3 : spdSynhrCnt3;
           spdSynhrCnt4 = (spdSynhrCnt4 > 0) ? --spdSynhrCnt4 : spdSynhrCnt4;
           if(spdSynhrCnt1 >= ANALOG_SPD_FLT_TIME){
             pLocNvData->encoInputSetting = encoderInterfacePhaseShift(); //����������� ���� ���������� ��������
             nonVolatileDataFlashWrite(pLocNvData); //��������� ������������� ����� ����� �� FLASH
           }else{
             spdSynhrCnt1++;
           }
        }else if((fltAnalogSpeed < 0)&&(fltDiscrSpeed > 0)){
           spdSynhrCnt1 = (spdSynhrCnt1 > 0) ? --spdSynhrCnt1 : spdSynhrCnt1;
           spdSynhrCnt3 = (spdSynhrCnt3 > 0) ? --spdSynhrCnt3 : spdSynhrCnt3;
           spdSynhrCnt4 = (spdSynhrCnt4 > 0) ? --spdSynhrCnt4 : spdSynhrCnt4;
           if(spdSynhrCnt2 >= ANALOG_SPD_FLT_TIME){
             pLocNvData->encoInputSetting = encoderInterfacePhaseShift(); //����������� ���� ���������� ��������
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
  * @brief  ������ ��������������� �������� ������ � FLASH-������
  * @param 
  * @retval
  */
void nonVolatileDataFlashWrite(encoFlashMemDataType *encoFlashMemData){
  flash_unlock();//��������������� FLASH-������ ����� �������
  flash_erase_page(BASE_PAGE_ADDR); //�������� �������� ������ ����� �������
  flash_write(BASE_PAGE_ADDR, encoFlashMemData); //������
  flash_lock(); //���������� ������
}


/**
  * @brief  �������� ������� �������� ������ ����� �����, ������������ �� �������� C � D, � R-���������.
  *         ����������� ��������������� �������� ������ ��� ������������ ����.   
  * @param pLocNvData - ��������� �� ��������� ����������������� ������
  * @retval 
  */
void phaseShiftDetect(encoBlockStatus *encoBlockPnt, encoFlashMemDataType *pLocNvData){
   static u16  RtimeEn  = 0;
   static u16 RStateCnt = 0;
   static u16 RSignalDelayCnt = 0;
   u16 RSignalDelay;  //�������� �� ��������� ���������� �� R-������� ��� ������ �������
   u16 pulseResolution;
   float shiftPhase;
   uint16_t tmpcr1;
   volatile uint16_t cntVal; //���������� ����������. ������� �������� �������� ����
   
   pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
   if(encoBlockPnt->drvMode == VECTOR_MODE){
     return; //� ��������� ������ ������������� ��� �� �����������
   }
   
   //�������� �� ��������� R-������� ��� ������ �������
   RSignalDelay = (u16)(R_PROCESSING_DELAY * 1000000.0F / encoBlockPnt->encoProcessingPeriod + 0.5F);
   if(RSignalDelayCnt < RSignalDelay){
     RSignalDelayCnt++;
     RsignalEventAck();   //����� ����� ���������� �� ������������ �������
     return;
   }
     
     //���� ����� �� ���������� �� R-�������
   if(RtimeEn > 0) {RtimeEn--;} //������� ������. ������ ��������, � ������� �������� R-������ ������������
   if(RsignalEventPresent()){//���� ������� R-������
     if(RtimeEn == 0){ //���� R-������ �������� ����� ���������� �����
       RtimeEn = 20;   //��������� ����������� R-������� ����������� � ������� 20 ������
       RStateCnt = 500;     //R-������ ������������ ��� ������� 500 ������
       encoBlockPnt->RsignalFlg = 1; //��� �������
       RsignalEventAck();   //����� ����� ���������� �� ������������ �������
       cntVal = TIM_GetCounter(TIM1);
       
       //��������� ����������� �����
       tmpcr1 = TIM1->CR1;
       if(tmpcr1 & (1 << DIR_SIGN)){ //���� ����������� �������������
       TIM_SetCounter(TIM1, RESOLUTION_FACTOR * pulseResolution - 1);
       }else{  //���� ����������� �������������
         TIM_SetCounter(TIM1, 0);
       }
       shiftPhase = getAnalogThetaMechPU(encoBlockPnt); //������� ������� ����� � ������ R-����������
       shiftPhase += pLocNvData->phaseShift; //���� ��������������� ��������������� ��������
       shiftPhase = removeExcessPhase(shiftPhase);//�������� ����������� ��������� ���� ����� �������� ��������������� ��������
       if((shiftPhase >= (0.5F - DELTA_PHASE)) && (shiftPhase <= (0.5F + DELTA_PHASE))){ //����� �� +180 ��������
         pLocNvData->phaseShift += 0.5F; //����� �������������� ����
         pLocNvData->phaseShift = removeExcessPhase(pLocNvData->phaseShift);//�������� ����������� ��������� ����
         nonVolatileDataFlashWrite(pLocNvData);
       }else if ((shiftPhase >= (0.75F - DELTA_PHASE)) && (shiftPhase <= (0.75F + DELTA_PHASE))){ //����� �� 90 ����
         pLocNvData->phaseShift += 0.25F;
         pLocNvData->phaseShift = removeExcessPhase(pLocNvData->phaseShift);//�������� ����������� ��������� ����
         nonVolatileDataFlashWrite(pLocNvData);
       }else if ((shiftPhase >= (0.25F - DELTA_PHASE)) && (shiftPhase <= (0.25F + DELTA_PHASE))) {  //����� �� 90 ����
         pLocNvData->phaseShift += 0.75F;
         pLocNvData->phaseShift = removeExcessPhase(pLocNvData->phaseShift);//�������� ����������� ��������� ����
         nonVolatileDataFlashWrite(pLocNvData);
       }else{ //����� �������� ������ ���, ��� ����
         //encoder.phasingByRSignal = 1; //���� ���������� ����� ��������� �� ����������� �����
       }
     }else{
       RsignalEventAck();
     }

   }else{
     if(RStateCnt != 0){
       RStateCnt--; //�������� �� ��������� R-������� ��� �������
     }else{
       encoBlockPnt->RsignalFlg = 0; // �������� ���� R-�������
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
     static ENCOFLT encoFltFastSin = {&encoFltStrgFastSin[0], 32, 0, 0, 0};  //!��������� � ������� ��� ������� ��������

     polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
     encoProcessingPeriod = encoBlockPnt->encoProcessingPeriod;
     pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
     encoFltFastSin.storageLen = FAST_SPD_STRG_LEN; // ����� ������ �� 2 �������
     fPrecTheta = precThetaCalc(encoBlockPnt);      // ������ ������� ������ ������� ���������
     fRelThetaMechFine = fPrecTheta / (2*pi);       // ������������� ���� ������ ������� ���������
     relThetaMechFine = fRelThetaMechFine;          // ������������� ���� ������ ������� ���������
     
     angleDiff = relThetaMechFine - (*(encoFltFastSin.thetaStrg + encoFltFastSin.storagePos));   //!���������� ���� ������ ������� ���������
     *(encoFltFastSin.thetaStrg + encoFltFastSin.storagePos)= relThetaMechFine;              //!��������� ����������� ���� ������ ������� ���������
     encoFltFastSin.storagePos=(encoFltFastSin.storagePos+1)&(encoFltFastSin.storageLen-1);  //!��������� ����� �������� ������� � ���������

     // �������� �������� ����� �������
     if(angleDiff < -0.75F) {
          angleDiff += 1.0F;
     }
     if(angleDiff > 0.75F) {
          angleDiff -= 1.0F;
     }
     
     if(encoBlockPnt->PWMOn == 0){
       K1 = 1000000.0F * (float)(polePairsNum) / encoProcessingPeriod / encoFltFastSin.storageLen / pulseResolution / FREQ_BASE;
     }
     // �������������� ���������� ����������� ���� ����� �������� ��������
     encoFltFastSin.TmpFltr = angleDiff; //encoFltFastSin.TmpFltr +((angleDiff - encoFltFastSin.TmpFltr) * 0.125F);
     fastSinSpd = K1 * encoFltFastSin.TmpFltr;   //�������� ������ ������� ���������
     encoFltFastSin.fltSpeed = fastSinSpd;
     
     return(encoFltFastSin.fltSpeed);
}

/**
  * @brief  ������ ������� ������ ������� ���������
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
   return fPrecTheta; //���� ������ ������� ���������
}


/**
  * @brief  ���������� �� Flash-������ �������� ����� ������� ���������� ��������
  * @param  
  * @retval ������� �������� �������� ������
  */
s32 getFastSpdSignFromFlash(){
  s32 fastSpdSign;
  fastSpdSign = flash_FastSpdSignRead( BASE_PAGE_ADDR ); //������� �������� �� FLASH
  if ((fastSpdSign == -1) || (fastSpdSign == 1)){
     return(fastSpdSign);
  }else{
    return(1);
  }
}

/**
  * @brief  ������������� ��������, ����������� �� ���������� �������� � �������� ���������
  * @param digatalSpd - �������� ���������� �� ������������ ����������, ��������������� � ������ (�� ����)
  * @param fastAnalogSpd - ��������, ���������� �� ���������� ����������� ����������
  * @param digitalInputInitFlg - ���� ����������� ������������� ���������� ���� � ��������
  * @retval  
  */
void fastSinCosSpdSignPhasing(encoBlockStatus *encoBlockPnt, float digatalSpd, float fastAnalogSpd, encoFlashMemDataType *pLocNvData, s16 digitSpdSignDetectFlg){
  static float spdModeSwitchVal = 0;
  static float minSpdModeSwitchVal;
  static u16 state1Cnt = 0;
  static u16 state2Cnt = 0;
  
  if(encoBlockPnt->drvMode == VECTOR_MODE){ //������������� ������� � �������� ��������� � ��������� ������ �� �����������. ���� ���
    return;
  }
  
  if(encoBlockPnt->PWMOn == 0){
    spdModeSwitchVal = spdModeValCalc(encoBlockPnt);
    minSpdModeSwitchVal = 0.1F / FREQ_BASE;
    state1Cnt = 0;
    state2Cnt = 0;
  }else{
      if(digitSpdSignDetectFlg == NOT_DETECT){ //digitSpdSignDetectFlg = -1 ��� �������� ���� enDat
        return;
      }else if ((fabsf(digatalSpd) > minSpdModeSwitchVal) && (fabsf(digatalSpd) < spdModeSwitchVal)){ //������������� ������ ������� ���������� �������� � �������� ��������
        if( (digatalSpd > 0) && (fastAnalogSpd < 0) ){
          state2Cnt = (state2Cnt != 0) ? (state2Cnt - 1) : state2Cnt;
          
          ++state1Cnt;
          if(state1Cnt == 800){
            pLocNvData->fastSpdSignSetting *= -1; //�������� ������� ���������� � �������� �������� � ������ �����
            nonVolatileDataFlashWrite(pLocNvData); //������ ����� ������� ���������� �������� � Flash
          }
        }else if ( (digatalSpd < 0) && (fastAnalogSpd > 0) ){
          state1Cnt = (state1Cnt != 0) ? (state1Cnt - 1) : state1Cnt;
          
          ++state2Cnt;
          if(state2Cnt == 800){
             pLocNvData->fastSpdSignSetting *= -1; //�������� ������� ���������� � �������� �������� � ������ �����
             nonVolatileDataFlashWrite(pLocNvData); //������ ����� ������� ���������� �������� � Flash
          }
        }else{
          pLocNvData->fastSpdSignSetting *= 1;  //����� �������� � ������� ���������� �������� ���������. ��������� ����. ��� ���������
        }
      }else{
        state1Cnt = (state1Cnt != 0) ? (state1Cnt - 1) : state1Cnt;
        state2Cnt = (state2Cnt != 0) ? (state2Cnt - 1) : state2Cnt;
      }
  }
}


/**
  * @brief  ������������ ����� �������� ��������� ��������
  * @param encoder - ��������� �� ��������� ��������� ��������
  * @param disrSpd - ��������, ���������� �� �������� ����
  * @param fastAnalogSpd - ������� ���������� �� ���������� ����������� ��������
  * @retval ��������, ��������������� ��������� ������ ��������� �������� 
  */
#define SPD_MODE_HYST (0.2F/FREQ_BASE) //���������� ��� �������� �� ������� ������ � ������
float spdCalcModeValSelect(encoBlockStatus *encoBlockPnt, float disrSpd, float fastAnalogSpd )
{
    float spdRetVal;
    float mode2toMode1Val;
    static float spdModeSwitchVal = 0.0F;
    static spdCalcModeType  spdCalcMode = ANALOG_FAST_MODE; //���������� ����� ������� ��������
    
    if(encoBlockPnt->PWMOn == RESET){ //������ ������� ������������ ������� ��������
      spdModeSwitchVal = spdModeValCalc(encoBlockPnt);
      mode2toMode1Val = spdModeSwitchVal - SPD_MODE_HYST; //������� �������� � ������ ����� � ������ �����������
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
  * @brief  ���������� ������� ������������ ������� ��������� ��������
  * @param  encoBlockPnt - ��������� �� ��������� �������� ��������
  * @retval �������� ������� ������������ ������� 
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
  spdModeSwitchVal *= 0.7F; //�������� ��������, ��� ���������� ������� ������������� ������
  return(spdModeSwitchVal);
}


/**
  * @brief  ������ �������� �� ������� ���������� �������� ��� �������� ���� EnFat
  * @param  sEnco - ��������� �� ��������� �������� ��������
  * @retval �������� �� ������� ���������� ��������
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
     static ENCOFLT encoFltFastSin = {&encoFltStrgFastSin[0], 32, 0, 0, 0};  //!��������� � ������� ��� ������� ��������

     polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
     encoProcessingPeroid = encoBlockPnt->encoProcessingPeriod;
     pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
     encoFltFastSin.storageLen = FAST_SPD_STRG_LEN; // ����� ������ �� 2 �������
     fPrecTheta = precThetaCalc(encoBlockPnt);      // ������ ������� ������ ������� ���������
     fRelThetaMechFine = fPrecTheta/(2*pi);         // ������������� ���� ������ ������� ���������
     relThetaMechFine = fRelThetaMechFine;     // ������������� ���� ������ ������� ���������
     
     angleDiff = relThetaMechFine - (*(encoFltFastSin.thetaStrg + encoFltFastSin.storagePos));   //!���������� ���� ������ ������� ���������
     *(encoFltFastSin.thetaStrg + encoFltFastSin.storagePos)= relThetaMechFine;              //!��������� ����������� ���� ������ ������� ���������
     encoFltFastSin.storagePos=(encoFltFastSin.storagePos+1)&(encoFltFastSin.storageLen-1);  //!��������� ����� �������� ������� � ���������

     // �������� �������� ����� �������
     if(angleDiff < -0.75F) {
          angleDiff += 1.0F;
     }
     if(angleDiff > 0.75F) {
          angleDiff -= 1.0F;
     }
     
     if(encoBlockPnt->PWMOn == 0){
       K1 = 1000000.0F *(float)polePairsNum * RESOLUTION_FACTOR / (encoProcessingPeroid * encoFltFastSin.storageLen)/pulseResolution / FREQ_BASE;
     }
     // �������������� ���������� ����������� ���� ����� �������� ��������
     encoFltFastSin.TmpFltr = encoFltFastSin.TmpFltr +((angleDiff - encoFltFastSin.TmpFltr) * 0.125F);
     fastSinSpd = K1 * encoFltFastSin.TmpFltr; //�������� ������ ������� ���������
     encoFltFastSin.fltSpeed = fastSinSpd;
     
     return(encoFltFastSin.fltSpeed);
}



/**
  * @brief  ������������� ��������, ����������� �� ���������� �������� � �������� ��������� ��� �������� EnDat
  * @param digatalSpd - �������� ���������� �� ���������� �������
  * @param fastAnalogSpd - ��������, ���������� �� ���������� ����������� ����������
  * @retval  
  */
void fastSinCosSpdSignPhasingForEnDat(encoBlockStatus *encoBlockPnt, float digatalSpd, float fastAnalogSpd, encoFlashMemDataType *pLocNvData){
  static float spdModeSwitchVal = 0;
  
  if(encoBlockPnt->PWMOn == 0){
    spdModeSwitchVal = spdModeValCalc(encoBlockPnt);
  }
  
  if(fabsf(digatalSpd) < spdModeSwitchVal ){ //������������� ������ ������� ���������� �������� � �������� ��������
    if( (digatalSpd > 0) && (fastAnalogSpd < 0) ){
      pLocNvData->fastSpdSignSetting *= -1; //�������� ������� ���������� � �������� �������� � ������ �����
    }else if ( (digatalSpd < 0) && (fastAnalogSpd > 0) ){
      pLocNvData->fastSpdSignSetting *= -1; //�������� ������� ���������� � �������� �������� � ������ �����
    }else{
      pLocNvData->fastSpdSignSetting *= 1;  //����� �������� � ������� ���������� �������� ���������. ��������� ����. ��� ���������
    }
    nonVolatileDataFlashWrite(pLocNvData); //������ ����� ������� ���������� �������� � Flash
  }
}



/**
  * @brief  ��������� ������� �������� �� ���� �� Endat
  * @param
  * @param
  * @retval  ��� �������
  */
u32 getEnDatEncoderPosition(encoBlockStatus *encoBlockPnt, unsigned long long position){
   u32 encoderPosition;
   u32 tmpPos;
   u16 mask;
   u16 i;
   u16 bitResolution;
   bitResolution = encoBlockPnt->baseEncoMotorData.bitResolution; 
   //�������� ����� �� CRC
    mask = (1 << NUM_CRC_BITS) - 1;
    // ������������ ���� ������� ������. ���������� �������� ��������� ������� ������
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
  * @brief  ���������� CRC �� ��������� �� EnDat-�������� ������
  * @param
  * @param
  * @retval  ���������� CRC ������
  */
u16 getEnDatEncoderCRC(unsigned long long position){
   u16 encoderCRC;
   u16 mask;
   mask = (1<<NUM_CRC_BITS) - 1;
   encoderCRC = (u16)(position & mask); // CRC � ������� �����
   return(encoderCRC);
}



/**
  * @brief  ������ CRC ��������� �� EnDat-�������� ������
  * @param
  * @param
  * @retval  ������������ CRC ������
  */
u16 EnDatEncoderCRCcalc(encoBlockStatus *encoBlockPnt, u32   encoderPosition, unsigned long long position){
    u16 alarm1, alarm2;
    u16 EndatModeCrc;
    u16 crcCalc;
    u16 bitResolution;
    u32   highpos, lowpos;
    volatile u16 encoErr;

    bitResolution = encoBlockPnt->baseEncoMotorData.bitResolution;
   // ������� ���� ������
    switch (encoBlockPnt->baseEncoMotorData.serialMode)
    {
    case ECN1313:
        alarm1 = (position >> (bitResolution + NUM_CRC_BITS)) & 0x01;//!��� ������ err1
        alarm2 = 0;     //!��� ������ err2    
        encoErr = alarm1;     
        EndatModeCrc = ECN1313;
      break;

    case ECN1325:
        alarm2 = (position >> (bitResolution + NUM_CRC_BITS)) & 0x01;//!��� ������ err2
        alarm1 = (position >> (bitResolution + NUM_CRC_BITS + 1)) & 0x01;//!��� ������ err1
        encoErr = alarm1 + (alarm2 << 1);
        EndatModeCrc = ECN1325;
      break;    
    }   
    highpos = ((uint64_t)encoderPosition >> 32) & 0xFFFFFFFFL; //!�������� ������� 32 ���� �������
    lowpos = encoderPosition & 0xFFFFFFFFL;          //!�������� ������� 32 ���� ������� 
    crcCalc = CrcEnDat(bitResolution, EndatModeCrc, alarm1, alarm2, highpos, lowpos); //!��������� crc 
    return(crcCalc);
}


/**
  * @brief  ������ �������������� ����
  * @param  encoder - ��������� �� ���������  ��������� ��������
  * @param  ThetaMechPU - ������������ ����
  * @retval ������������� ����
  */
float EnDatEncoderElecPosCalc(encoBlockStatus *encoBlockPnt, float ThetaMechPU){
  float ThetaElec;
  float thetaOffset;
  u16 polePairsNum;
  
  polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum; //����� ��� �������
  thetaOffset = encoBlockPnt->thetaOffset;
    
  ThetaElec = ThetaMechPU * polePairsNum;         //!������������� ����
  ThetaElec += (1.0F/3600) * thetaOffset;     //!������������� ���� � ������ ��������
  return(ThetaElec);
}



/**
  * @brief  ������ ������������� ���� �� ���������� �� Endat-�������� �������
  * @param  encoder - ��������� �� ���������  ��������� ��������
  * @param  encoderPosition - ��� ���������� ������� ��������
  * @retval ������������ ����
  */
float EnDatEncoderMechPosCalc(encoBlockStatus *encoBlockPnt, u32 encoderPosition){
  float fThetaMech;
  float fThetaMechPU;
  u16 bitResolution;
  
  bitResolution = encoBlockPnt->baseEncoMotorData.bitResolution;
  fThetaMech = (2*pi/(1 << bitResolution)) * (encoderPosition);//!���������� �������� ���� (� ���.) � float            
  fThetaMechPU = fThetaMech/(2*pi);                            //!������������� �������� ���� � float-�������
 
  return(fThetaMechPU);
}


/**
  * @brief  ������ �������� ��� endat-��������
  * @param  encoder - ��������� �� ���������  ��������� ��������
  * @retval ������� �������� �� endat-��������
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
   static float encoFltStrg[64]={0};                       //!����� ��� ����������  ����
   static ENCOFLT encoFlt = {encoFltStrg, 32, 0, 0, 0};  //!��������� � ������� ��� ������� ��������
   
   polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum;
   encoProcessingPeriod = encoBlockPnt->encoProcessingPeriod;
   encoFlt.storageLen = encoBlockPnt->fltStrgLen;        //������ ��������� ������� ����
   angleDiff = discrThetaMechPU - (*(encoFlt.thetaStrg + encoFlt.storagePos));   //!����������  ����
   *(encoFlt.thetaStrg + encoFlt.storagePos)= discrThetaMechPU;              //!�� ������� ������� ������ ���������� ��������� ����������� ����
   encoFlt.storagePos=(encoFlt.storagePos+1)&(encoFlt.storageLen-1);         //!����� �������� ������� � ��������� ���������� ����
   // �������� �������� ����� �������
   if(angleDiff < -0.8F) {
     angleDiff += 1.0F;
   }
   if(angleDiff > 0.8F) {//��� �������
      angleDiff -= 1.0F;
   }
   encoFlt.TmpFltr = encoFlt.TmpFltr +((angleDiff - encoFlt.TmpFltr) * 0.125F);
     // ������������ ������������� �������� � ������ ��� �������.
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
  * @brief  �������� ������ R-�������
  * @param  encoStatus - ������� ������ ������
  * @param  encoder - ��������� �� ���������  ��������� ��������
  * @param  spdVal  - ������� �������� ������
  * @retval ������ ������
  */

#define SPD_SIGN_STATE_CNT  (1000000/encoBlockPnt->encoProcessingPeriod) //����� ����������� �������� � ����� ����������� - 2 ���
#define SPD_VAL_STATE_CNT  (1000000/encoBlockPnt->encoProcessingPeriod)  //����� ����������� �������� �� �������� �������� - 1 ���
#define DELTA_TIME ((float)encoBlockPnt->encoProcessingPeriod/1000000.0F)       //����� ����� ������� ��������
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
  case PWM_OFF: //�������� ���������
    RSignalErrStatus = 0;
    encoBlockPnt->phasingByRSignal = 0;
    rotateAngle = 0;
    state1Cnt = state2Cnt = spdValStateCnt = 0; //����� ���������
    RSignalControlState = (encoBlockPnt->PWMOn == 1) ? STAB_SPD_SIGN_WAIT : RSignalControlState; //���� ������ ��
    break;
  case STAB_SPD_SIGN_WAIT: //��������� �������� ����������� �������� � ����� �����������
    if(spdVal > 0){                        //���� �������� �������������
      state2Cnt = (state2Cnt != 0) ? --state2Cnt : state2Cnt; //��������� �������� ��������� ������������� ��������
      ++state1Cnt; //������� ��������� ������������� ��������
      RSignalControlState = (state1Cnt == SPD_SIGN_STATE_CNT) ? STAB_SPD_VAL_WAIT : RSignalControlState; //���� ����������� ��������� ������� � �������� �������� ��������
      state2Cnt = (state1Cnt == SPD_SIGN_STATE_CNT) ? 0 : state2Cnt; //����� ��������� ��� ���������� ��������� � ����� ���������
      state1Cnt = (state1Cnt == SPD_SIGN_STATE_CNT) ? 0 : state1Cnt; //����� ��������� ��� ���������� ��������� � ����� ���������
    }else if (spdVal < 0){                 //���� �������� �������������
      state1Cnt = (state1Cnt != 0) ? --state1Cnt : state1Cnt; //��������� �������� ��������� ������������� ��������
      ++state2Cnt; //������� ��������� ������������� ��������
      RSignalControlState = (state2Cnt == SPD_SIGN_STATE_CNT) ? STAB_SPD_VAL_WAIT : RSignalControlState; //���� ����������� ��������� ������� � �������� �������� ��������
      state1Cnt = (state2Cnt == SPD_SIGN_STATE_CNT) ? 0 : state1Cnt; //����� ��������� ��� ���������� ��������� � ����� ���������
      state2Cnt = (state2Cnt == SPD_SIGN_STATE_CNT) ? 0 : state2Cnt; //����� ��������� ��� ���������� ��������� � ����� ���������
    }
    RSignalControlState = (encoBlockPnt->PWMOn == 0) ? PWM_OFF : RSignalControlState; //�������� ���
    break;
  case STAB_SPD_VAL_WAIT: //��������� �������� �������� �� ���������� ��������
    if(fabsf(spdVal) >= (1.0F / FREQ_BASE)){ //���� �������� �������� ��������
      spdValStateCnt++;
      RSignalControlState = (spdValStateCnt == SPD_VAL_STATE_CNT) ? R_SIGNAL_WAIT : RSignalControlState; //������� � ��������� �������� R-������� ���� ���� ���������� ��������
      spdValStateCnt = (spdValStateCnt == SPD_VAL_STATE_CNT) ? 0 : spdValStateCnt; //����� ��������
    }else{
      spdValStateCnt = (spdValStateCnt != 0) ? spdValStateCnt - 1 : spdValStateCnt;
      RSignalControlState = STAB_SPD_SIGN_WAIT;
    }
    RSignalControlState = (encoBlockPnt->PWMOn == 0) ? PWM_OFF : RSignalControlState; //�������� ���
    break;
  case R_SIGNAL_WAIT: //��������� �������� R-�������
    absEncoSpd = fabsf(fltSpd); //������ �������� ��������
    absEncoSpd = (absEncoSpd / polePairsNum) * FREQ_BASE; //������� �������� ������
    rotateAngle = rotateAngle + absEncoSpd * DELTA_TIME; //������� �������� ����, � ���. ��. (�������� vdt)
    
    debugMaxRotateAngle = (rotateAngle > debugMaxRotateAngle) ? rotateAngle : debugMaxRotateAngle;
    
    RSignalControlState = (rotateAngle >= FULL_ROTATION + DELTA_ROTATION) ? R_SIGNAL_MISS : RSignalControlState; //���� ��� ������ ��������, �� R-������ �� �������� - ������
    rotateAngle =  (encoBlockPnt->RsignalFlg == 1) ? 0 : rotateAngle; //����� ������������ ��������� ��� R-�������
    encoBlockPnt->phasingByRSignal = (encoBlockPnt->RsignalFlg == 1) ? 1 : encoBlockPnt->phasingByRSignal; //���� ���������� R-������� ��� �������� �� ������� �������
    RSignalControlState = ((encoBlockPnt->autoPhasingOn == 1) && (encoBlockPnt->RsignalFlg == 1)) ? PWM_OFF_WAIT : RSignalControlState; //� ������ ������������� R-������ �������������� ���� ��� �� ����
    RSignalControlState = (encoBlockPnt->PWMOn == 0) ? PWM_OFF : RSignalControlState; //�������� ���
     
    break;
  case R_SIGNAL_MISS:      //��������� ���������� R-�������
    RSignalErrStatus = (encoBlockPnt->autoPhasingOn == 1) ? R_MISS_IN_TUNE_ERR : R_MISS_IN_RUNNIG_ERR;
    RSignalControlState = (encoBlockPnt->PWMOn == 0) ? PWM_OFF : RSignalControlState;
    //RSignalErrStatus = (encoder->PWMOn == 0) ? ENCO_OK : RSignalErrStatus;
    break;
  case PWM_OFF_WAIT: //�������� ���������� ��� ����� ������������� R-������� � ������ ���������������
    RSignalControlState = (encoBlockPnt->PWMOn == 0) ? PWM_OFF : RSignalControlState; //�������� ���
    break;
  }
  errStatus = (RSignalErrStatus != ENCO_OK) ? RSignalErrStatus : errStatus;
  return(errStatus);
}



/**
  * @brief  �������� ������ �������� ���� enDat
  * @param encoder - ��������� �� ��������� ��������� ���������
  * @param position - ��������� � �������� ���
  * @retval ������ ������
  */
uint16_t enDatEncoErrDetect(encoBlockStatus *encoBlockPnt, unsigned long long position){
  uint16_t errStatus = ENCO_OK;
  
  errStatus = enDatDataExchangeErrDetect(errStatus, encoBlockPnt, position); // �������� ���������� ������ �� ��������� enDat
  if(encoBlockPnt->baseEncoMotorData.fastSpdUse == USE){
    errStatus = enDatEncoderIncrSignalBreakCheck(errStatus, encoBlockPnt); //����� ���������� ��������
    //� ����� ���� �����������. ����� �� ���, ���� ���� �������� ��������������
    /*
    errStatus = encoderIncrSignalBreakCheck(errStatus, encoBlockPnt);      //�������� �������
    errStatus = fastSinCosSignalFaultDetect(errStatus, encoBlockPnt);
    */
    errStatus = encoderIncrSignalLowSpdBreakCheck(errStatus, encoBlockPnt); //�������� ��������������
    
  }
  return(errStatus);
}


/**
  * @brief  �������� ������ �������� ���� sinCos
  * @param encoder - ��������� �� ��������� ��������� ���������
  * @param  spdVal - ������� �������� ��������
  * @retval ������ ������
  */
uint16_t sinCosEncoErrDetect(encoBlockStatus *encoBlockPnt, float spdVal, float phaseAdd){
  uint16_t errStatus = ENCO_OK;
  errStatus = sinCosEncoderCablesBreakCheck(errStatus, encoBlockPnt);     //�������� ������ ������ ��������
  errStatus = sinCosEncoderAbsSignalBreakCheck(errStatus, encoBlockPnt);  //�������� ������ ��������� �������� �������: D � C
  //errStatus = encoderIncrSignalBreakCheck(errStatus, encoBlockPnt);       //�������� ������ ������������ ���������� ��������
  errStatus = sinCosRSignalBreakCheck(errStatus, encoBlockPnt, spdVal);   //�������� ���������� R-�������
  errStatus = encoderIncrSignalLowSpdBreakCheck(errStatus, encoBlockPnt);
  errStatus = (encoBlockPnt->drvMode == VECTOR_MODE) ? checkSinCosPhasingMiss(errStatus, encoBlockPnt, phaseAdd) : errStatus; //�������� �������������� ������������� �����������. �������� ����������� � ��������� ������
  return(errStatus);
}


/**
  * @brief  �������� ������ ������ �������� ���� SIN/COS
  * @param  encoErrStatus - ������� ������ ������ ��������
  * @param  encoder - ��������� �� ��������� ��������� ���������
  * @retval ������ ������
  */

#define MIN_ADC_ZERO -100 //���������� �������� ������ ���� ���
#define MAX_ADC_ZERO 100  //���������� �������� ������ ���� ���
#define MIN_ERR_VAL -8   //����������� ��������� �������� ������ ���� �������������� �������
#define MAX_ERR_VAL  8   //������������ ��������� �������� ������ ���� �������������� ������� 
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
    
    if ((slowSin >= MIN_ERR_VAL) && (slowSin <= MAX_ERR_VAL) && (slowCos >= MIN_ERR_VAL) && (slowCos <= MAX_ERR_VAL)){ //�������� ��������� ��������
      D_C_missCnt = (D_C_missCnt < CABLE_BREAK_CNT) ? (D_C_missCnt + 1) : D_C_missCnt;
      CD_ErrStatus = (D_C_missCnt == CABLE_BREAK_CNT) ? C_OR_D_MISS : CD_ErrStatus; //������ - ��� ����������� ��������: D � �
    }else{
      D_C_missCnt = (D_C_missCnt > 0) ? (D_C_missCnt - 1) : D_C_missCnt;
      CD_ErrStatus = (D_C_missCnt == 0) ? ENCO_OK : CD_ErrStatus;
    }
     
    if ((fastSin >= MIN_ERR_VAL) && (fastSin <= MAX_ERR_VAL) && (fastCos >= MIN_ERR_VAL) && (fastCos <= MAX_ERR_VAL)){ //�������� ��������� ��������
      A_B_missCnt = (A_B_missCnt < CABLE_BREAK_CNT) ? (A_B_missCnt + 1) : A_B_missCnt;
      AB_ErrStatus = (A_B_missCnt == CABLE_BREAK_CNT) ? A_OR_B_MISS : AB_ErrStatus; //������ - ��� ������������ ���������� �������� A � B
    }else{
      A_B_missCnt = (A_B_missCnt > 0) ? (A_B_missCnt - 1) : A_B_missCnt;
      AB_ErrStatus = (A_B_missCnt == 0) ? ENCO_OK : AB_ErrStatus;
    }
    
    sinCosErrStatus = ((CD_ErrStatus == C_OR_D_MISS) && (AB_ErrStatus == A_OR_B_MISS)) ? CABLE_BREAK_ERR : sinCosErrStatus; //����� ������
    sinCosErrStatus = ((CD_ErrStatus == C_OR_D_MISS) && (AB_ErrStatus == ENCO_OK)) ? C_OR_D_MISS : sinCosErrStatus;     //��� D � �
    sinCosErrStatus = ((CD_ErrStatus == ENCO_OK) && (AB_ErrStatus == A_OR_B_MISS)) ? A_OR_B_MISS : sinCosErrStatus;     //��� A � B
    
    //��������������� ���������� ����� ������ � ��� ��� �������� �� ������� �������
    if(encoBlockPnt->autoPhasingOn == 1){ //����� ���������������
      sinCosErrStatus = (sinCosErrStatus == C_OR_D_MISS) ? C_D_MISS_IN_TUNE_ERR : sinCosErrStatus; //������ ���������� ��� ������ �� �������
      sinCosErrStatus = (sinCosErrStatus == A_OR_B_MISS) ? A_B_MISS_IN_TUNE_ERR : sinCosErrStatus; //������ ���������� ��� ������ �� �������
    }else{ //������� �����
      sinCosErrStatus = (sinCosErrStatus == C_OR_D_MISS) ? C_D_MISS_IN_RUNNING_ERR : sinCosErrStatus; //������ ���������� ��� ������ �� �������
      sinCosErrStatus = (sinCosErrStatus == A_OR_B_MISS) ? A_B_MISS_IN_RUNNING_ERR : sinCosErrStatus; //������ ���������� ��� ������ �� �������
    }
  
    return sinCosErrStatus;
  
}


/**
  * @brief  �������� �������������� ������ ������������ ���������� �������� �������� ���� enDat
  * @param  encoErrStatus - ������ ������ ������ ��������
  * @param  encoder - ��������� �� ���������-��������� ��������
  * @retval ������ ������
  */

uint16_t enDatEncoderIncrSignalBreakCheck(uint16_t encoErrStatus, encoBlockStatus *encoBlockPnt){
    uint16_t sinCosErrStatus  = encoErrStatus;
    static uint16_t AB_ErrStatus = ENCO_OK;
    static uint16_t A_B_missCnt = 0;
    s16 fastSin, fastCos;
    
    fastSin = encoBlockPnt->analogSignals.fastSin;
    fastCos = encoBlockPnt->analogSignals.fastCos;
  
    if ((fastSin >= MIN_ERR_VAL) && (fastSin <= MAX_ERR_VAL) && (fastCos >= MIN_ERR_VAL) && (fastCos <= MAX_ERR_VAL)){ //�������� ��������� ��������
      A_B_missCnt++;
      AB_ErrStatus = (A_B_missCnt == CABLE_BREAK_CNT) ? /*A_OR_B_MISS*/A_B_MISS_IN_RUNNING_ERR : AB_ErrStatus; //������ - ��� ������������ ���������� �������� A � B
    }else{
      A_B_missCnt = (A_B_missCnt > 0) ? --A_B_missCnt : A_B_missCnt;
      AB_ErrStatus = (A_B_missCnt == 0) ? ENCO_OK : AB_ErrStatus;
    }
    
    sinCosErrStatus = (AB_ErrStatus != ENCO_OK) ? AB_ErrStatus : sinCosErrStatus;
    return sinCosErrStatus;
}

/**
  * @brief  �������� ���������� ������ �� ��������� edDat
  * @param  encoStatus - ������� ������ ������ �������� 
  * @param  encoder - ��������� �� ���������  ��������� ��������
  * @param  position - ��������� � �������� ���
  * @retval ������ ������
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
        alarm1 = (position >> (bitResolution + NUM_CRC_BITS)) & 0x01;//!��� ������ err1
        alarm2 = 0;     //!��� ������ err2        
         break;
    case ECN1325:
        alarm2 = (position >> (bitResolution + NUM_CRC_BITS)) & 0x01;//!��� ������ err2
        alarm1 = (position >> (bitResolution + NUM_CRC_BITS + 1)) & 0x01;//!��� ������ err
        break;    
    }
    tmp = ((alarm1 == 1) || (alarm2 == 1)) ? 1 : 0; //������ ������ �� ��������� enDat
    //-----���������� ������-------//
    switch(fltState){
    case 0: //��������� "�� ������"
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
    case 1: //��������� ������ "��� ������"
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
  * @brief  �������� ������ ������������ ���������� �������� �������� ���� enDat
  * @param  encoErrStatus - ������� ������ ������ ���������
  * @param  encoder - ��������� �� ��������� ��������� ���������
  * @retval ������ ������
  */

#define ERR_TIME  2        //20 �� - ����� �������� ������������
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
    case ERR_WAIT: //��������� �������� �������
      errStatus = encoErrStatus;
      if(incrSignalErr != ENCO_OK){ //���� ���������� ������ 
        errStatus = incrSignalErr;
        errDetectState = ERR_HOLD; //������� � ��������� ��������� ������
        heldErr = incrSignalErr;
      }
      break;
    case ERR_HOLD: //��������� ��������� ������
      errStatus = heldErr;
      errHoldCnt++;
      errDetectState = (errHoldCnt == ONE_SEC_TACT_CNT) ? ERR_WAIT : errDetectState; //���� ��������� ����� - ������� � ��������� �������� ������
      errHoldCnt = (errHoldCnt == ONE_SEC_TACT_CNT ) ? 0 : errHoldCnt; //����� ��������
      break;
    }
    return errStatus;
}


/**
  * @brief  �������� ������ �������� D � C �������� ���� sinCos
  * @param  encoStatus  - ������� ������ ������ ���������
  * @param  encoder  - ��������� �� ��������� ��������� ���������
  * @retval ������ ������
  */

#define MIN_SQRT_ALLOW_VAL 0.81F //���������� ���������� �������� ����� �� ����� ��������� sin � cos
#define MAX_SQRT_ALLOW_VAL 1.50F //����������� ���������� �������� ����� �� ����� ��������� sin � cos

#define MIN_SQRT_ALLOW_VAL_INCR 0.5F //���������� ���������� �������� ����� �� ����� ��������� sin � cos
#define MAX_SQRT_ALLOW_VAL_INCR 1.50F //����������� ���������� �������� ����� �� ����� ��������� sin � cos

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
    normalizeSin = (float)slowSin / /*encoBlockPnt->ADC_Amplitude*/SIN_COS_ADC_AMPL; //��������������� sin
    normalizeCos = (float)slowCos / /*encoBlockPnt->ADC_Amplitude*/SIN_COS_ADC_AMPL; //��������������� cos
    tmp = powf(normalizeSin, 2) + powf(normalizeCos, 2); //�����  ��������� ���������� �������� sin � cos
    sqrtVal = sqrtf(tmp);

    if(!((sqrtVal >= MIN_SQRT_ALLOW_VAL) && (sqrtVal <= MAX_SQRT_ALLOW_VAL))){ //���������� �������� ����� ��������� - � �������� 0,7...1,2
        errCnt++;
        if(errCnt >= SLOW_SIGNAL_ERR_TIME){
          absPosSignalD_Err = ((slowSin >= MIN_ERR_VAL) && (slowSin <= MAX_ERR_VAL)) ? D_MISS_IN_WORK : ENCO_OK;     //������ D
          absPosSignalC_Err = ((slowCos >= MIN_ERR_VAL) && (slowCos <= MAX_ERR_VAL)) ? C_MISS_IN_WORK : ENCO_OK;     //������ C
          absPosSignalErr = ((absPosSignalD_Err == D_MISS_IN_WORK) && (absPosSignalC_Err == ENCO_OK))? D_MISS_IN_WORK : absPosSignalErr;       //���� D � ������, C ����
          absPosSignalErr = ((absPosSignalC_Err == C_MISS_IN_WORK) && (absPosSignalD_Err == ENCO_OK))? C_MISS_IN_WORK : absPosSignalErr;       //���� � � ������, D ����
          absPosSignalErr = ((absPosSignalC_Err == C_MISS_IN_WORK) && (absPosSignalD_Err == D_MISS_IN_WORK))? C_OR_D_MISS : absPosSignalErr;   //���� D � C � ������
          absPosSignalErr = ((absPosSignalC_Err == ENCO_OK) && (absPosSignalD_Err == ENCO_OK))? C_OR_D_MISS : absPosSignalErr; //���� D � C �� � ������� ������ - �������
          
          //����������� ���������� ��� ������ �� �������
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
  * @brief  ��������� ������ �������� ���������� ��� ��������� ������������ �������� A � B
  * @param  encoBlockPnt  - ��������� �� ��������� ��������� ���������
  * @retval
  */
void enDat_SSI_Encoder_A_B_Init(encoBlockStatus *encoBlockPnt){
  GPIO_InitTypeDef      GPIO_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  uint16_t encoInputSetting = 0;
  uint16_t pulseResolution;
  
  pulseResolution = encoBlockPnt->baseEncoMotorData.pulseResolution;
  TIM_Cmd(TIM1, DISABLE); //������� ������� TIM1
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //!������������ ������� TIM1
  
  //!��������� ���� PA8 � �������� �������� ����� 1 ������� TIM1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //!�������������� �������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //!��������
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  //!�������� � ����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;       //!PA8
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  GPIO_PinAFConfig(GPIOA,  GPIO_PinSource8,  GPIO_AF_6); 
  
 
  //!��������� ���� PA9 � �������� �������� ����� 2 ������� TIM1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //!�������������� �������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //!��������
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  //!�������� � ����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;       //!PA9
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  GPIO_PinAFConfig(GPIOA,  GPIO_PinSource9,  GPIO_AF_6);
  
  /*��������� ������� TIM1 �� ����� ������ ��������*/
  encoInputSetting = getEncoInputSettFromFlash(); //����� ��������� ������ ������������� �������
  encoInputSetting = (encoInputSetting == 0) ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
  TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, encoInputSetting, TIM_ICPolarity_Rising); //��������� ������ ��������
  TIM_SetAutoreload(TIM1, (pulseResolution - 1)); //��������� �������� ���������� �������-�������� ������ ��������

 /*��������� ������ ������� ������� TIM1 ��� ��������� ������ ������ ������������ �����*/
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //����� �1:
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = encoInputSetting;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8; 
  TIM_ICInitStructure.TIM_ICFilter = 5;     
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE); //���������� ���������� ������� TIM1 �� ������� �� ������ 1
  
  //����� �2
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter = 5;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE); //���������� ����������  ������� TIM1 �� ������� �� ������ 2 
  
  TIM_Cmd(TIM1, ENABLE); //������ ������� TIM1 

}

/**
  * @brief  ��������� ������ SPI ��� ��������� ���� SSI
  * @param  encoder  - ���������-��������� ��������
  * @retval 
  */
void SSISpiInit(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure;

  EndatSpiData.EndatSpiState = StopMode; // ������� ��������� 
  
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
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // �������� ��� �����
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
  * @brief  �������� �������� ���� �� ������������ ���������
  * @param  endatAngle - ������� ����, ����������� �� endat
  * @param  incrEncoModulAngle - ������� ���� ������ ������������� ��������
  * @retval ���� ������
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
  * @brief  ������ �������� �� ���������� ���� ������ ������������� ��������
  * @param  discrThetaMechPU - ������� ����, ����������� �� �������� �� ������ ������������� ��������
  * @param  encoder - ��������� �� ��������� ��������
  * @retval �������� �� ������������ ����
  */
float incrAngleSpdCalc(float discrThetaMechPU, encoBlockStatus *encoBlockPnt)
{
   u16 polePairsNum;
   u16 encoProcessingPeriod;
   float angleDiff;
   static float K = 0;
   static float encoFltStrg[64]={0};                              //!����� ��� ����������  ���������� ����
   static ENCOFLT encoFlt = {encoFltStrg, 32, 0, 0, 0};         //!��������� � ������� ��� ������� �������� �� ���������� ����
   
   polePairsNum = encoBlockPnt->baseEncoMotorData.polePairsNum; // ����� ��� �������
   encoProcessingPeriod = encoBlockPnt->encoProcessingPeriod;   //������ ��������� ������ ��������
   encoFlt.storageLen = encoBlockPnt->fltStrgLen;
   angleDiff = discrThetaMechPU - (*(encoFlt.thetaStrg + encoFlt.storagePos));   //!���������� ����������� ����
   *(encoFlt.thetaStrg + encoFlt.storagePos)= discrThetaMechPU;              //!�� ������� ������� ������ ���������� ��������� ����������� ����
   encoFlt.storagePos=(encoFlt.storagePos+1)&(encoFlt.storageLen-1); //!����� �������� ������� � ��������� ���������� ����
   // �������� �������� ����� �������
   if(angleDiff < -0.8F) {
     angleDiff += 1.0F;
   }
   if(angleDiff > 0.8F) {//��� �������
      angleDiff -= 1.0F;
   }
   encoFlt.TmpFltr = encoFlt.TmpFltr +((angleDiff - encoFlt.TmpFltr) * 0.125F);
     // ������������ ������������� �������� � ������ ��� �������.
   if(encoBlockPnt->PWMOn == 0){
     K = (float)polePairsNum * 1000000.0F / (encoProcessingPeriod * FREQ_BASE * encoFlt.storageLen); 
   }
   encoFlt.fltSpeed = K * encoFlt.TmpFltr;
   return(encoFlt.fltSpeed); //�������� �� ���������� ����
}

/**
  * @brief  ��������� ��������� �������� �������-�������� ������ �������� �� �������� ����, ���������� �� ��������� EnDat
            � �������� ���������� ���������
  * @param  encoBlockPnt - ��������� �� ��������� ��������
  * @param  encoErrStatus - ������� ������ ��������
  * @retval ������ ������
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
        baseIncrementCntVal = (s16)(mechThetaPU * pulseResolution + 0.5F); //�������� �������� ������������ ����
        TIM_SetCounter(TIM1, baseIncrementCntVal); //��������� �������� � ��������� ��������
        cntState = TIM_GetCounter(TIM1);           //�������� �� ���������� ���������
        if(cntState == baseIncrementCntVal){       //���� ������� ����������� � ������ ��������..
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
  * @brief  �������� ������ ������������ �������� �� ����� ��������
  * @param encoErrStatus - ������� ������ ������
  * @param encoder - ��������� �� ���������-��������� ��������
  * @retval ������ ������
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
    normalizeSin = (float)fastSin / SIN_COS_ADC_AMPL; //��������������� sin
    normalizeCos = (float)fastCos / SIN_COS_ADC_AMPL; //��������������� cos
    tmp = powf(normalizeSin, 2) + powf(normalizeCos, 2); //�����  ��������� ������������ �������� sin � cos
    sqrtVal = sqrtf(tmp);
    
    
    maxSqrtVal = (sqrtVal > maxSqrtVal) ? sqrtVal : maxSqrtVal;
    minSqrtVal = (sqrtVal < minSqrtVal) ? sqrtVal : minSqrtVal;

    if(!((sqrtVal >= MIN_SQRT_ALLOW_VAL_INCR) && (sqrtVal <= MAX_SQRT_ALLOW_VAL_INCR))){ //���������� �������� ����� ��������� - � �������� 0,5...1,5
        errCnt++;
        if(errCnt >= INCR_SIGNAL_ERR_TIME){
          incrPosSignalA_Err = ((fastSin >= MIN_ERR_VAL) && (fastSin <= MAX_ERR_VAL)) ? A_MISS_IN_WORK : ENCO_OK;  
          incrPosSignalB_Err = ((fastCos >= MIN_ERR_VAL) && (fastCos <= MAX_ERR_VAL)) ? B_MISS_IN_WORK : ENCO_OK;    
          incrPosSignalErr = ((incrPosSignalA_Err == A_MISS_IN_WORK) && (incrPosSignalB_Err == ENCO_OK))? A_MISS_IN_WORK : incrPosSignalErr; 
          incrPosSignalErr = ((incrPosSignalB_Err == B_MISS_IN_WORK) && (incrPosSignalA_Err == ENCO_OK))? B_MISS_IN_WORK : incrPosSignalErr;   
          incrPosSignalErr = ((incrPosSignalB_Err == A_MISS_IN_WORK) && (incrPosSignalA_Err == B_MISS_IN_WORK))? A_OR_B_MISS : incrPosSignalErr; 
          incrPosSignalErr = ((incrPosSignalB_Err == ENCO_OK) && (incrPosSignalA_Err == ENCO_OK))? A_OR_B_MISS : incrPosSignalErr;
          
          //����������� ���������� ��� ������ �� �������
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
  * @brief  ����� ����� ������� ������������ ��������
  * @param autoFastSpdSign - ����� ����� ���������� ������������ ��������
  * @param encoBlockPnt - ��������� �� ���������-��������� ��������
  * @retval ���� ������������ ��������: 1 ��� -1
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
  * @brief  ��������� ��������� ��������
  * @param
  * @param
  * @retval
  */
#define MAX_DIVIDE_VAL 65535U    //����������� ��������� �������� ������������ ������� ������� ����
#define MAX_ARR_VAL    65536U    //����������� ��������� �������� �������� ������������
#define MIN_REF_FREQ    0.05F    //����������� ������� �������
#define MAX_REF_FREQ  80
#define MIN_ARR_VAL   50
#define PWM_FREQ_FACTOR     2    //�����������, ����������� ����������� ������� ������������� ������� �� ������� "����" 

void encoEmulInit(encoBlockStatus *encoBlockPnt){
  static u16 BDTRflg = 0;
  GPIO_InitTypeDef GPIO_InitStructure; //!��������� ��� ��������� ������
  TIM_OCInitTypeDef TIM_OCInitStruct;
  TIM_TypeDef* encoEmulTIM = TIM15;
  TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); //!������������ ����� B
  
  //��������� ���� PB14 ����� � ��������� ��������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //!�������������� �������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //!��������
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;     //!�������� � ����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;          //!PB14
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource14,  GPIO_AF_1); //!PB14 ������������ ��� ������� TIM15
  
  //��������� ���� PB15 ����� B ��������� ��������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //!�������������� �������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //!��������
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;     //!�������� � ����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;          //!PB15
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource15,  GPIO_AF_1); //!PB15 ������������ ��� ������� TIM15
  
  //��������� ������ 1 TIM15
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE); //������������ ������� 15
  
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_Toggle; //��� � ������ ������, ������������ �� ����������
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //���������� ������ (CC1E = 0)
  TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable; //��������������� ����� ��������
  TIM_OCInitStruct.TIM_Pulse = 0; //�������� �������� ���������
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;       //�������� ��������� �� ����������
  TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCPolarity_High;     //�������� ��������� �� ����������
  TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;   //�� ���.
  TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset; //�� ���.
  TIM_OC1Init(encoEmulTIM, &TIM_OCInitStruct); //��������� ������ 1 ������� TIM15
  
  //��������� ������ 2 TIM15
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_Toggle; //��� � ������ ������, ������������ �� ����������
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //���������� ������ (CC2E = 0)
  TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable; //��������������� ����� ��������
  TIM_OCInitStruct.TIM_Pulse = 0; //�������� �������� ���������
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;       //�������� ��������� �� ����������
  TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCPolarity_High;     //�������� ��������� �� ����������
  TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;   //�� ���.
  TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset; //�� ���.
  TIM_OC2Init(encoEmulTIM, &TIM_OCInitStruct); //��������� ������ 2 ������� TIM15
  
  //������� ��������� �������
  TIM_InternalClockConfig(encoEmulTIM);               //������ ����������� �� ����������� ���������
  encoEmulStartSettigsSet(encoEmulTIM, encoBlockPnt); //������� ��������� ������� ���������
  if(BDTRflg == 0){
    BDTRflg = 1;
    TIM_CtrlPWMOutputs(encoEmulTIM, ENABLE);            //���������� ���������� ������ ��������� (MOE = 1)
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
  * @brief  ������ �������� ������������ ������� TIM15 ��������� ��������
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
  TIM_TypeDef* TIMx = TIM15; //������, ������������ ��� ������������ ������� ��������� ��������
  
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

  if((fabsf(electricSpeed) < minEncoEmulRef) || (encoEmulMode == ENCO_EMUL_OFF)){   //�������� ����� 0 ��� �������� ��������
    ARR_val = MAX_ARR_VAL;
    TIM_SetAutoreload(TIMx, ARR_val / 2);
    TIM_SetCompare1(TIMx, ARR_val - 1);
    TIM_SetCompare2(TIMx, ARR_val - 1);
    return;
  }
  firstResetting = 0;
  //������ ���������� ������ ��������� ��� ����������� ��������� ������� ������� ���������
  realTimFreq = encoBlockPnt->realTimClockFreq;                         //������� ������� ������������ �������
  polePairsNum  = encoBlockPnt->baseEncoMotorData.polePairsNum;         //����� ��� �������
  encoEmulResol = 1 << encoBlockPnt->baseEncoMotorData.encoEmulResol;   //���������� ���������
  ARR_val = (u32)(realTimFreq * polePairsNum / PWM_FREQ_FACTOR / electricSpeed / encoEmulResol + 0.5F) - 1;
  ARR_val = (ARR_val > MAX_ARR_VAL) ? MAX_ARR_VAL : ARR_val;   //�������� �� ���������� �������
  TIM_SetAutoreload(TIMx, ARR_val);      //��������� ������� ������� ���������
  compareRegCalc(encoBlockPnt->calculatedData.electricSpd, TIMx, ARR_val); //������ �������� ��������� ���������
  timerState = TIM_GetCR1State(TIMx);
  encoEmulOutputEnable(TIMx); //���������� ������� ���������
  if((timerState & TIM_CR1_CEN) == 0){ //��������� ������ ���� ����������
     TIM_Cmd(TIMx, ENABLE);
  }
}


/**
  * @brief  ������ ��������� ��������� �������, ������������ ������ ��������� ��������
  * @param electricSpeed  �������� �������� ��������
  * @param ARR_val ������� �������� �������� ���������� �������
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
  * @brief  ��������� ������� ��������� �������� � ������ �������
  * @param TIMx - ��������� �� ������, ������������ ��� ������������
  *               ������� ���������
  * @retval
  */
void encoEmulOutputDesable(TIM_TypeDef* TIMx){
  TIMx->CCER &= (uint32_t)~TIM_CCER_CC1E;
  TIMx->CCER &= (uint32_t)~TIM_CCER_CC2E;
}

/**
  * @brief  ���������� ������� ��������� ��������
  * @param TIMx - ��������� �� ������, ������������ ��� ������������
  *               ������� ���������
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
  * @brief  ��������� �������� ��������� � ��������� �������
  * @param TIMx - ��������� �� ������, ������������ ��� ������������ ��������
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
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; //!��������� ��� ������� ��������� �������
  NVIC_InitTypeDef NVIC_InitStructure;
  
  encoEmulResol = 1 << encoBlockPnt->baseEncoMotorData.encoEmulResol; //���������� ��������� ��������
  polePairsNum =  encoBlockPnt->baseEncoMotorData.polePairsNum;  //����� ��� ������� ���������
  
  if(polePairsNum == 0){
    return; //���� ������ �� �������� ������ �� �������, ��������� �� ���������
  }
  
  
  //��������� ������� ������������ �������, ��� ������� �� ����������� ��������� ������� �������
  //80 �� � �������� ������������ ����� ��������� ����������� �������� 50.
  //�������� 50 ������������� ������������ ��������, ���� �������� �����������
  //������������ ������� �������� ������ ��������� 1%
  
  timClockFreq = PWM_FREQ_FACTOR * MIN_ARR_VAL * MAX_REF_FREQ * encoEmulResol / polePairsNum; //������� ������������
  prescalerVal = (u16)((float)APB2Clock / timClockFreq + 0.5F);                                  //������������ ������� �������
  prescalerVal = (prescalerVal == 0) ? 1 : prescalerVal;
  realTimClockFreq = (float)APB2Clock / prescalerVal;                                     //�������� ������� ������������ ������� TIM15
  encoBlockPnt->realTimClockFreq = realTimClockFreq;
  
  //������ �������� �������� ������������, ��� �������, ��� ������������ ������� ������������
  //�������� ����� ������������ ������ �������� �� ����������� ������� 0,05 ��
  emulPeriodReg = (u32)(realTimClockFreq * polePairsNum / PWM_FREQ_FACTOR / MIN_REF_FREQ / encoEmulResol + 0.5F);
  emulPeriodReg = (emulPeriodReg > MAX_ARR_VAL) ? MAX_ARR_VAL : emulPeriodReg;
  minRefFreq = realTimClockFreq * polePairsNum / PWM_FREQ_FACTOR / emulPeriodReg / encoEmulResol; //�������� ����������� �������, ��� �������� �������� ������������ ������� ��������
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
  
  //��������� ����������
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM15_IRQn; //TIM6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE; //���������� ���������
  NVIC_Init(&NVIC_InitStructure);
  
  TIM_ITConfig(TIMx, TIM_IT_CC1, DISABLE); //���������� ���������
  TIM_ITConfig(TIMx, TIM_IT_CC2, DISABLE); //���������� ���������
}

