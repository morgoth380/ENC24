/**
  ******************************************************************************
  * @file    ENC24/ENC24_Toggle/main.c 
  * @author  Lungor
  * @version V1.0.0
  * @date    27.01.2021
  * @brief   Main program body
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "string.h"
#include "uart.h"
#include "enco.h" 
#include "math.h"
#include "Global_include.h"
#include "Global_define.h"

#define WD_CLK 40000U   //������� ������������ ����������� �������, ���
#define WD_TIMEOUT 0.1F //������� ����������� �������, �

void gpioInit (void);
void initUart1(void);
unsigned char DelayMs(void);
void Delay(__IO uint32_t nTime);
void debugTimeMeasTimerInit(void);
void indicationInit(void);
void processingPeriodTimerCalc(u16 processingPeriod);
void watchDogInit(void);

int main(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
  initUart1();    //��������� UART
  watchDogInit(); //��������� ����������� �������
  
  while (1)
  {
    encoBlock.encoBlockPerifInitPnt(&encoBlock); //������������� ��������� ����������
    encoBlock.encoBlockCalcPnt(&encoBlock);      //������ �������� � ����
    IWDG_ReloadCounter();                        //����� ����������� �������
  }       
}      
      

/**
  * @brief  �������� ���� ����� ��������� ���������
  * @retval - ��� ����� ��������� ���������
  */

encoBlockType encoBlockTypeDef(void){
  encoBlockType encoBlock;
  encoBlock = (encoBlockType)(0x0003U & GPIO_ReadInputData(GPIOB)); //��� ����� ���������
  return(encoBlock);
}


/**
  * @brief  �������� ���� ����������
  * @param encoBlockPnt - �������� �� ���������-��������� ����� ���������
  * @retval
  */

void MCU_TypeDef(encoBlockStatus *encoBlockPnt)
{
  u32 MCU_ID_Code;
  MCU_ID_Code = *((u32 *)MCU_IDCODE);
  MCU_ID_Code &= 0x0FFFUL;
  encoBlockPnt->MCU_ID_Code = MCU_ID_Code;
}

/**
  * @brief  ������������� ��������� ����������
  * @param  encoBlockPnt - ��������� �� ������-��������� ����� ���������
  * @retval
  */
void dataChangePerifReinit(encoBlockStatus *encoBlockPnt){
   
  switch(encoBlockPnt->baseEncoMotorData.blockType){
  case INCREMENTAL:
    incrementalModeInit(encoBlockPnt); //������������� ��������� ��� ��������� ���������������� ��������
    break;
  case SERIAL:
    serialModeInit(encoBlockPnt);     //������������� ��������� ��� ��������� �������� � ���������������� �����������
    if(encoBlockPnt->baseEncoMotorData.fastSpdUse == USE){
      encoderAdcInit(encoBlockPnt); //������������� ��������� ��� ��������� ���������� �������� sin/cos
    } 
    break;
  case SIN_COS:
    sinCosDigitModeInit(encoBlockPnt); //������������� ��������� ��� ��������� �������� sin/cos
    encoderAdcInit(encoBlockPnt);
    break;
  }

  encoEmulInit(encoBlockPnt); //��������� ��������� ��������
  indicationInit();           //��������� ������������ ��������� �����
  // ������ ����. ������� ��������
  encoBlockPnt->K1SpdFiltr = (float)encoBlockPnt->encoSpdFltrTime / (encoBlockPnt->encoSpdFltrTime + encoBlockPnt->encoProcessingPeriod);
  encoBlockPnt->K2SpdFiltr = (float)encoBlockPnt->encoProcessingPeriod / (encoBlockPnt->encoSpdFltrTime + encoBlockPnt->encoProcessingPeriod);
  
}

/**
  * @brief ����� ����������������� ��������� ��� ��������� ���������� �������� ��� ���������
  * @param  encoBlockPnt - ��������� �� ������-��������� ����� ���������
  * @retval
  */
void encoBlockPerifInit(encoBlockStatus *encoBlockPnt){
  u8 *pntToStaticData;
  u8 *pntToBaseData;
  u16 baseDataLength;
  static baseEncoDataType initEncoData = BASE_RESET_PARAMS;   //������� �������� � ���������� �������� ������
 
  pntToStaticData = (u8 *)&initEncoData.blockType;
  pntToBaseData = (u8 *)&encoBlockPnt->baseEncoMotorData.blockType;
  baseDataLength = sizeof(baseEncoDataType);
  encoBlockPnt->baseEncoMotorData.blockType = encoBlockTypeDef();
  MCU_TypeDef(encoBlockPnt);  //���������� ID ����������
  
  /*���� ������ �������� ����������, �������� ����������������� ���������*/  
  if(memcmp(pntToStaticData, pntToBaseData, baseDataLength)){   
    dataChangePerifReinit(encoBlockPnt);                    //����� ����������������
    debugTimeMeasTimerInit();  //��������� ����������� ������� ��������� ������������ ���������� ����
    memcpy(pntToStaticData, pntToBaseData, baseDataLength); //��������� ������
  }
}

/**
  * @brief ���������� �������� ��������
  * @param  encoBlockPnt - ��������� �� ������-��������� ����� ���������
  * @retval
  */
void encoSpdFlt(encoBlockStatus *encoBlockPnt){
  float electricSpd;
  float shadowSpeedElectric;
  
  if (isnan(encoBlockPnt->calculatedData.electricSpd)){
    encoBlockPnt->calculatedData.electricSpd = 0;
  }
  
  if (isnan(encoBlockPnt->K1SpdFiltr)){
    encoBlockPnt->K1SpdFiltr = 0;
  }
  
  if (isnan(encoBlockPnt->K2SpdFiltr)){
    encoBlockPnt->K2SpdFiltr = 0;
  }
  
  electricSpd = encoBlockPnt->calculatedData.electricSpd;
  shadowSpeedElectric = encoBlockPnt->calculatedData.shadowSpeedElectric;
    
  // ��� ������� ������� � ������ ������� �������������.
  electricSpd = encoBlockPnt->K1SpdFiltr * electricSpd + encoBlockPnt->K2SpdFiltr * shadowSpeedElectric;     
  encoBlockPnt->calculatedData.electricSpd = electricSpd;
}

/**
  * @brief  ������������� ��������� ���������� ��� ������ �� ����������������� ���������
  * @param  encoBlockPnt - ��������� �� ������-��������� ����� ���������
  * @retval
  */
void serialModeInit(encoBlockStatus *encoBlockPnt){ 
  switch(encoBlockPnt->baseEncoMotorData.serialMode){ //����� ����������������� ���������
  case ENDAT2_0:
  case ECN1313:
  case ECN1325:
    EndatSpiInit();
    break;
  case SSI_BIN:
  case SSI_GRAY:
    SSISpiInit();
    break;
  }
}


/**
  * @brief  ������ �������� � ���� ��������
  * @param  encoBlockPnt - ��������� �� ������-��������� ����� ���������
  * @retval
  */
void encoBlockCalc(encoBlockStatus *encoBlockPnt){
  unsigned long long serialPosition;
  encoBlockType blockType;
  blockType = encoBlockPnt->baseEncoMotorData.blockType;
  
  if (encoBlockPnt->procStatus == DATA_DONE) {
    switch (blockType) { // ��������� ��� �����
      
      case INCREMENTAL:
        incrementDataProcessing(encoBlockPnt); //!������ �������� ���������������� �������� 
        break;
      case SERIAL:
        serialPosition = serialDataSel(encoBlockPnt); //������� � ����������� �� ���� ����������������� ���������
        serialDataProcessing(serialPosition, encoBlockPnt);
        break; 
      case SIN_COS: 
        sinCosDataProcessing(encoBlockPnt);
        break;
      default:                 
        break;        
    } 
    encoSpdFlt(encoBlockPnt);
    encoEmulCalc(encoBlockPnt); //������ ������� ��������� ��������
    encoBlockPnt->procStatus = DATA_WAIT;   // ���� ����� ���� ���������
  }
}

/**
  * @brief  ���������� ������� �������� � ����������� �� ���� ����������������� ���������
  * @param  encoBlockPnt - ��������� �� ������-��������� ����� ���������
  * @retval ������� ��������
  */
unsigned long long serialDataSel(encoBlockStatus *encoBlockPnt){
  serialModesType serialMode;
  unsigned long long serialPos;
  
  serialMode = encoBlockPnt->baseEncoMotorData.serialMode;
  switch(serialMode){
  case ENDAT2_0:
  case ECN1313:
  case ECN1325:
    serialPos = encoBlockPnt->EndatPosition;
    break;
  case SSI_GRAY:
  case SSI_BIN:
    serialPos = encoBlockPnt->SSIPosition;
    break;
  }
  return(serialPos);
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
unsigned char DelayMs()
{
    unsigned char i;
    for(i = 0; i < 1; i++) {
	i++;
        i--;
    }
    
    return 0;
}


/**
  * @brief  ��������� ������� ���������� ������������ ���������� ����
  * @param  
  * @retval 
  */
void debugTimeMeasTimerInit(void)
{
  u16 PrescalerValue;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);                      //!������������ ������� TIM2
  TIM_TimeBaseStructure.TIM_Period = 0xFFFFUL; //!0xFFFF - �������� ����� ���������� �������� ����� ���������� �� ����������
  TIM_TimeBaseStructure.TIM_Prescaler = 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  PrescalerValue = 1;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

    /* Enable the TIM7 global Interrupt */         
  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
  
  // Prescaler configuration //
  TIM_PrescalerConfig(TIM7, PrescalerValue, TIM_PSCReloadMode_Immediate);
  TIM_Cmd(TIM7, ENABLE);
}

/**
  * @brief  ��������� ��������� ��� ������������ ��������� �����
  * @param  
  * @retval 
  */
void indicationInit(void)
{
  u16 prescalerValue;
  u16 periodVal;
  GPIO_InitTypeDef      GPIO_InitStructure;       //!��������� ��� ��������� ������
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  periodVal = 0xFFFFUL;
  prescalerValue = (u16)(DATA_EXCHANGE_WAIT * APB2Clock / (periodVal + 1) + 0.5F);
  
  //��������� ����� ���������� ����������� ��������� ������
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   //!�����
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //!
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; //!�������� ����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;      //PC15
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  //��������� ������� �������� �������� �� �������� ������
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);
  TIM_TimeBaseStructure.TIM_Period = periodVal;  
  TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
  
  //��������� ����������
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM17_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  TIM_ITConfig(TIM17, TIM_IT_Update, ENABLE);
  
  TIM_PrescalerConfig(TIM17, prescalerValue, TIM_PSCReloadMode_Immediate);
  
  TIM_Cmd(TIM17, ENABLE); 
}

/**
  * @brief  ��������� ����������� �������
  * @param  
  * @retval 
  */
void watchDogInit(void)
{
  u16 prescalerVals[] = {4, 8, 16, 32, 64, 128, 256};
  u16 prescaler;
  u16 reloadVal;
  
  prescaler = IWDG_Prescaler_4;
  
  //������ �������� �������� ������������
  reloadVal = (u16)(WD_TIMEOUT * WD_CLK / prescalerVals[prescaler] + 0.5F) - 1;
  
  IWDG_Enable();
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(prescaler);
  IWDG_SetReload(reloadVal); //0.1 c
  while(IWDG_GetFlagStatus(IWDG_FLAG_PVU | IWDG_FLAG_RVU | IWDG_FLAG_WVU) == SET){
    ;
  }
  IWDG_ReloadCounter();
}

