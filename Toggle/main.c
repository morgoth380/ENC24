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

#define WD_CLK 40000U   //Частота тактирования сторожевого таймера, кГц
#define WD_TIMEOUT 0.1F //Таймаут сторожевого таймера, с

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
  initUart1();    //Настройка UART
  watchDogInit(); //Настройка сторожевого таймера
  
  while (1)
  {
    encoBlock.encoBlockPerifInitPnt(&encoBlock); //Инициализация периферии процессора
    encoBlock.encoBlockCalcPnt(&encoBlock);      //Расчет скорости и фазы
    IWDG_ReloadCounter();                        //Сброс сторожевого таймера
  }       
}      
      

/**
  * @brief  Проверка типа блока обработки энкодеров
  * @retval - тип блока обработки энкодеров
  */

encoBlockType encoBlockTypeDef(void){
  encoBlockType encoBlock;
  encoBlock = (encoBlockType)(0x0003U & GPIO_ReadInputData(GPIOB)); //Тип блока энкодеров
  return(encoBlock);
}


/**
  * @brief  Проверка типа процессора
  * @param encoBlockPnt - указтель на структуру-описатель блока энкодеров
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
  * @brief  Инициализация периферии процессора
  * @param  encoBlockPnt - указатель на объект-описатель блока энкодеров
  * @retval
  */
void dataChangePerifReinit(encoBlockStatus *encoBlockPnt){
   
  switch(encoBlockPnt->baseEncoMotorData.blockType){
  case INCREMENTAL:
    incrementalModeInit(encoBlockPnt); //инициализация периферии для обработки инкрементального энкодера
    break;
  case SERIAL:
    serialModeInit(encoBlockPnt);     //инициализация периферии для обработки энкодера с последовательным иноерфейсом
    if(encoBlockPnt->baseEncoMotorData.fastSpdUse == USE){
      encoderAdcInit(encoBlockPnt); //инициализация периферии для обработки аналоговых сигналов sin/cos
    } 
    break;
  case SIN_COS:
    sinCosDigitModeInit(encoBlockPnt); //инициализация периферии для обработки энкодера sin/cos
    encoderAdcInit(encoBlockPnt);
    break;
  }

  encoEmulInit(encoBlockPnt); //Настройка эмулятора энкодера
  indicationInit();           //Настройка светодиодной индикации блока
  // Расчет коэф. фильтра скорости
  encoBlockPnt->K1SpdFiltr = (float)encoBlockPnt->encoSpdFltrTime / (encoBlockPnt->encoSpdFltrTime + encoBlockPnt->encoProcessingPeriod);
  encoBlockPnt->K2SpdFiltr = (float)encoBlockPnt->encoProcessingPeriod / (encoBlockPnt->encoSpdFltrTime + encoBlockPnt->encoProcessingPeriod);
  
}

/**
  * @brief Вызов переинициализации периферии при изменении параметров энкодера или двигателя
  * @param  encoBlockPnt - указатель на объект-описатель блока энкодеров
  * @retval
  */
void encoBlockPerifInit(encoBlockStatus *encoBlockPnt){
  u8 *pntToStaticData;
  u8 *pntToBaseData;
  u16 baseDataLength;
  static baseEncoDataType initEncoData = BASE_RESET_PARAMS;   //Текущие учтенные в настройках энкодера данные
 
  pntToStaticData = (u8 *)&initEncoData.blockType;
  pntToBaseData = (u8 *)&encoBlockPnt->baseEncoMotorData.blockType;
  baseDataLength = sizeof(baseEncoDataType);
  encoBlockPnt->baseEncoMotorData.blockType = encoBlockTypeDef();
  MCU_TypeDef(encoBlockPnt);  //Считывание ID процессора
  
  /*Если данные энкодера изменились, вызываем переинициализацию периферии*/  
  if(memcmp(pntToStaticData, pntToBaseData, baseDataLength)){   
    dataChangePerifReinit(encoBlockPnt);                    //Вызов переиницилизации
    debugTimeMeasTimerInit();  //Настройка отладочного таймера измерения длительности выполнения кода
    memcpy(pntToStaticData, pntToBaseData, baseDataLength); //Сохраняем данные
  }
}

/**
  * @brief Фильтрация скорости энкодера
  * @param  encoBlockPnt - указатель на объект-описатель блока энкодеров
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
    
  // ФНЧ первого порядка с учетом времени дискретизации.
  electricSpd = encoBlockPnt->K1SpdFiltr * electricSpd + encoBlockPnt->K2SpdFiltr * shadowSpeedElectric;     
  encoBlockPnt->calculatedData.electricSpd = electricSpd;
}

/**
  * @brief  Инициализация периферии процессора для обмена по последовательному протоколу
  * @param  encoBlockPnt - указатель на объект-опсиатель блока энкодеров
  * @retval
  */
void serialModeInit(encoBlockStatus *encoBlockPnt){ 
  switch(encoBlockPnt->baseEncoMotorData.serialMode){ //Режим последовательного протокола
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
  * @brief  Расчет скорости и фазы энкодера
  * @param  encoBlockPnt - указатель на объект-описатель блока энкодеров
  * @retval
  */
void encoBlockCalc(encoBlockStatus *encoBlockPnt){
  unsigned long long serialPosition;
  encoBlockType blockType;
  blockType = encoBlockPnt->baseEncoMotorData.blockType;
  
  if (encoBlockPnt->procStatus == DATA_DONE) {
    switch (blockType) { // Проверяем тип блока
      
      case INCREMENTAL:
        incrementDataProcessing(encoBlockPnt); //!Расчет скорости инкрементального энкодера 
        break;
      case SERIAL:
        serialPosition = serialDataSel(encoBlockPnt); //Позиция в зависимости от типа последовательного протокола
        serialDataProcessing(serialPosition, encoBlockPnt);
        break; 
      case SIN_COS: 
        sinCosDataProcessing(encoBlockPnt);
        break;
      default:                 
        break;        
    } 
    encoSpdFlt(encoBlockPnt);
    encoEmulCalc(encoBlockPnt); //расчет частоты эмулятора энкодера
    encoBlockPnt->procStatus = DATA_WAIT;   // Ждем новый такт обработки
  }
}

/**
  * @brief  Считывание позиции энкодера в зависимости от типа последовательного протокола
  * @param  encoBlockPnt - указатель на объект-описатель блока энкодеров
  * @retval Позиция энкодера
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
  * @brief  Настройка таймера вычисления длительности выполнения кода
  * @param  
  * @retval 
  */
void debugTimeMeasTimerInit(void)
{
  u16 PrescalerValue;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);                      //!тактирование таймера TIM2
  TIM_TimeBaseStructure.TIM_Period = 0xFFFFUL; //!0xFFFF - значение после достижения которого будет прерывание по обновлению
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
  * @brief  Настройка периферии для светодиодной индикации блока
  * @param  
  * @retval 
  */
void indicationInit(void)
{
  u16 prescalerValue;
  u16 periodVal;
  GPIO_InitTypeDef      GPIO_InitStructure;       //!Структура для настройки портов
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  periodVal = 0xFFFFUL;
  prescalerValue = (u16)(DATA_EXCHANGE_WAIT * APB2Clock / (periodVal + 1) + 0.5F);
  
  //Настройка порта управления светодиодом индикации аварии
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   //!Выход
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //!
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; //!Подтяжка вниз
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;      //PC15
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  //Настройка таймера ожидания запросов от верхнего уровня
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);
  TIM_TimeBaseStructure.TIM_Period = periodVal;  
  TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
  
  //настройка прерывания
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
  * @brief  Настройка сторожевого таймера
  * @param  
  * @retval 
  */
void watchDogInit(void)
{
  u16 prescalerVals[] = {4, 8, 16, 32, 64, 128, 256};
  u16 prescaler;
  u16 reloadVal;
  
  prescaler = IWDG_Prescaler_4;
  
  //Расчет значения регистра перезагрузки
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

