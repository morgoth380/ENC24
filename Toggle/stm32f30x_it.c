/**
  ******************************************************************************
  * @file    GPIO/GPIO_Toggle/stm32f30x_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-October-2012
  * @brief   Main Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x_it.h"
#include "enco.h"
#include "math.h"

/** @addtogroup STM32F30x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  FIRST_CAPTURE 0  //!Код первого захвата по сигналу энкодера
#define  DC_OFFSET   2048  //!DC-смещение
#define  REV_DIR  0x0010
#define NUM_FIRST_BIT  4 //16

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint16_t *ADC_SIN1, *ADC_COS1, *ADC_SIN2, *ADC_COS2;
extern u16 glowErrState;
extern u16 R_signalFlg;
u16 debugOvfNum;
u16 queryStatus = 0;
  
/* Private function prototypes -----------------------------------------------*/
void enDatIT_Pocessing(void);
void SSI_IT_Pocessing(void);
u16 getRxDataSize(encoBlockStatus *encoBlockPnt);
void getAnalogSignals(encoBlockStatus *encoBlockPnt);
void serialDataQuery(encoBlockStatus *encoBlockPnt);
/* Private functions ---------------------------------------------------------*/

extern ENDAT_SPI_BUFFER EndatSpiData;  
//extern __IO uint32_t ADC1_ValueTab[ADC_SIZE_BUFFER];
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F30x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f30x.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * TIMER 6 Interrupt handler
  */

void TIM6_DAC_IRQHandler(void){
  encoBlockType blockType;
  encoBlockStatus *encoBlockPnt;
  encoBlockPnt = &encoBlock;
   
  blockType = encoBlockPnt->baseEncoMotorData.blockType;
  
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    switch(blockType){
    case SERIAL:
      encoBlockPnt->incrEncoPos = TIM_GetCounter(TIM1); //Защелкиваем позицию энкодера
      serialDataQuery(encoBlockPnt);  //Сформировать данные и активировать запрос
      getAnalogSignals(encoBlockPnt); //Считать значения аналоговых сигналов
      break;
    case SIN_COS:
      encoBlockPnt->incrEncoPos = TIM_GetCounter(TIM1); //Защелкиваем позицию энкодера
      getAnalogSignals(encoBlockPnt);                   //Считать значения аналоговых сигналов
      encoBlockPnt->procStatus = DATA_DONE;             //Флаг разрешения для main.c
      break;
    case INCREMENTAL:
      encoBlockPnt->incrEncoPos = TIM_GetCounter(TIM2); //Защелкиваем позицию энкодера
      encoBlockPnt->procStatus = DATA_DONE;             //Флаг разрешения для main.c
      break;
    }  
  } 
}
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
/**
  * DMA1 Interrupt handler
  */
void DMA1_Channel1_IRQHandler (void)
{ 
  if (DMA_GetITStatus (DMA1_FLAG_TC1) != RESET) {
  }
    
}
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
/**
  * DMA1 Interrupt handler
  */
void DMA1_Channel2_IRQHandler (void)
{
//  u16  static ttt2 = 0;
  
  if (DMA_GetITStatus (DMA1_FLAG_TC2) != RESET) {
//    ttt2 ++;
  }
}
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
/**
  * SPI1 Interrupt handler
  */
// В size передаем значение от 4 до 16 - сколько реально будем передавать.
void Set_SPI1_Datasize (u8 size)  
{  
    u16  tempReg; 
    tempReg = SPI1->CR2; 
    tempReg &= (u16)~SPI_CR2_DS; 
    tempReg |= ((size-1) << 8); 
    SPI1->CR2 = tempReg; 
}

void SPI1_IRQHandler (void)
{
   encoBlockStatus *encoBlockPnt;
   encoBlockPnt = &encoBlock;
   serialModesType serialMode;
   volatile u16 tempReg;
   
   serialMode = encoBlockPnt->baseEncoMotorData.serialMode;
   // Проверка Busy
   if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) {  
     return;
   }
 
   if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET)
   {
     switch (serialMode){
       case ENDAT2_0:
       case ECN1313:
       case ECN1325:
         enDatIT_Pocessing();
         break;
       case SSI_GRAY:
       case SSI_BIN:  
       SSI_IT_Pocessing();
       break;
     }
   }
 
 
  if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
  { 
    tempReg = (u16)SPI_ReceiveData8(SPI1);
  }
  
}
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
/**
  * TIMER 2 Interrupt handler
  */

void TIM2_IRQHandler(void){
    u16  IFState = 0;
  // Собираем все прерывания таймера
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //!сбрасываем флаг обновления таймера
    IFState |= TIM_IT_Update;
  } 
  
  if(TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET)  //!если прерывание вызвано по событию захвата (поступил передний фронт сигнала энкодера)
  {
    /* Clear TIM1 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);    //!сбрасываем флаг захвата
  } 
  
    if(TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET)  //!если прерывание вызвано по событию захвата (поступил передний фронт сигнала энкодера)
  {
    /* Clear TIM1 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);    //!сбрасываем флаг захвата
  }
}


/**
  * @brief  Прерывание по захвату от сигнала референтной метки 
  *         Прерывание по обновлению при таймауте ожидания импульса инкрементального
  *         энкодера
  * @param  None
  * @retval None
  */
u16 R_Event;
void TIM3_IRQHandler(void){
  encoBlockStatus *encoBlockPnt;
  
  encoBlockPnt = &encoBlock;
  if(TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET){ //если поступил референтный сигнал
    R_Event = 1; //флаг события в обработчик энкодера
    encoBlockPnt->RsignalFlg = 1; //флаг для передачи в логгер
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);      //!сбрасываем флаг обновления таймера TIM3
  }
}

//Таймер для отладки. Измерение периода вызова функции обработки данных энкодера
void TIM7_IRQHandler(void){
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET){ 
     TIM_ClearITPendingBit(TIM7, TIM_IT_Update);      
     debugOvfNum++;                                  
  } 
}



/**
  * @brief Контроль обрыва инкрементных аналоговых сигналов энкодера sin/cos и enDat
  * @param
  * @retval
  */
#define NO_CAPTURE    0
#define CHANNEL_1     1
#define CHANNEL_2     2
#define CAPT_ERR_NUM  4

u16 incrSignalErr = ENCO_OK;
void TIM1_CC_IRQHandler (void){
    static u16 errCnt = 0;
    static s16 prevCaptureChannel = NO_CAPTURE;
    u16 tmpErr;
    encoBlockStatus *encoBlockPnt;
  
    encoBlockPnt = &encoBlock;
    tmpErr = ENCO_OK;
    errCnt = (encoBlockPnt->PWMOn == 0) ? 0 : errCnt; //сброс счетчика при выключенном ШИМ
    prevCaptureChannel = (encoBlockPnt->PWMOn == 0) ? NO_CAPTURE : prevCaptureChannel;
    incrSignalErr = (encoBlockPnt->PWMOn == 0) ? ENCO_OK : incrSignalErr;
    
    if(TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET)  //!если прерывание от канала 1
    {
      switch(prevCaptureChannel){ //проверка предыдущего канала захвата
      case CHANNEL_1: //источник захвата не изменился - не ОК
         prevCaptureChannel = CHANNEL_1;
         errCnt++;
         tmpErr = (errCnt == CAPT_ERR_NUM) ? B_MISS_IN_WORK : tmpErr;
         if(tmpErr == B_MISS_IN_WORK){ //если аввария была, проверяем текущий режим работы
           tmpErr = (encoBlockPnt->autoPhasingOn == 1) ? A_B_MISS_IN_TUNE_ERR : A_B_MISS_IN_RUNNING_ERR; //меняем код аварии с внутреннего на внешний
         }
         errCnt = (errCnt == CAPT_ERR_NUM) ? 0 : errCnt;
         break;
      case CHANNEL_2: //источник захвата изменился - ОК
         errCnt = 0;
         prevCaptureChannel = CHANNEL_1;
         break;
      default: //NO_CAPTURE
         errCnt = 0;
         prevCaptureChannel = CHANNEL_1;
         break; 
      }
      TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);    //!сбрасываем флаг захвата 
    }else if(TIM_GetITStatus(TIM1, TIM_IT_CC2) == SET){ //иначе проверка прерывания от канала 2
      switch(prevCaptureChannel){
      case CHANNEL_1: //источник захвата изменился - ОК
         prevCaptureChannel = CHANNEL_2;
         errCnt = 0;
         break;
      case CHANNEL_2: //источник захвата не изменился - не ОК
         prevCaptureChannel = CHANNEL_2;
         errCnt++;
         tmpErr = (errCnt == CAPT_ERR_NUM) ? A_MISS_IN_WORK : tmpErr;
         if(tmpErr == A_MISS_IN_WORK){ //если аввария была, проверяем текущий режим работы
           tmpErr = (encoBlockPnt->autoPhasingOn == 1) ? A_B_MISS_IN_TUNE_ERR : A_B_MISS_IN_RUNNING_ERR; //меняем код аварии с внутреннего на внешний
         }
         errCnt = (errCnt == CAPT_ERR_NUM) ? 0 : errCnt;
        break;
      default: //NO_CAPTURE
         errCnt = 0;
         prevCaptureChannel = CHANNEL_2;
         break; 
      }
      TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);    //!сбрасываем флаг захвата 
    }
    incrSignalErr = tmpErr;
}


/**
  * @brief  Обработка прерывания при обмене по протоколу enDat
  * @retval
  */
void enDatIT_Pocessing(){
  encoBlockStatus *encoBlockPnt;
  u16  tempReg = 0, i = 0;
  u16  static maxPing = 0;

  encoBlockPnt = &encoBlock;
  switch (EndatSpiData.EndatSpiState){
    case  SendRequest: // Передаем запрос
    if (EndatSpiData.TxDataSize) { // Если биты не переданы - здесь не ноль
      SPI_DATA_DIRECT_WRITE();
      Set_SPI1_Datasize(EndatSpiData.TxDataSize); // Задаем размер передачи          
      SPI_I2S_SendData16(SPI1, EndatSpiData.SendCommand);// Передаем данные
      EndatSpiData.TxDataSize = 0;    // Сбросить биты передачи  
      getAnalogSignals(encoBlockPnt); // Считываем данные аналоговых сигналов
    }
    else {                    
      EndatSpiData.EndatSpiState = WaitStartBit;          
      EndatSpiData.RxInd = 0; // Текущее кол-во принятых бит данных                    
      // Меняем режим работы SPI1
      
      Set_SPI1_Datasize(NUM_FIRST_BIT); // Задаем размер данных на ожидании стартового бита - 4 бита пакет 
      
      SPI_Cmd(SPI1, DISABLE);
      SPI1->CR1 &= ~(0x03); // Обнуляем CPOL и CPHA
      SPI1->CR1 |= SPI_CPOL_Low | SPI_CPHA_2Edge;
      SPI_Cmd(SPI1, ENABLE);
      
      SPI_DATA_DIRECT_READ(); // Переходим на чтение
                           
      maxPing = 0;
      tempReg = SPI_I2S_ReceiveData16(SPI1); // Вычитать данные для очистки FIFO - На всякий случай
      tempReg = SPI_I2S_ReceiveData16(SPI1);
      tempReg = SPI_I2S_ReceiveData16(SPI1);
      //SPI_I2S_SendData16(SPI1, 0x0);
      SPI_SendData8(SPI1, 0x0); // Долбилка тактовых импульсов при ожидании стартового бита 
    }                   
    break;  
    
    case WaitStartBit: // Ожидаем стартовый бит                          
       tempReg = SPI_ReceiveData8(SPI1); 
       if ( tempReg != 0) {// Если принят не ноль, значит пришел стартовый бит
           // Определяем сколько полезных бит мы приняли из принятых 4 бит
         for (i=NUM_FIRST_BIT; i>0; i--) {
           if ( tempReg & (1<<(i-1)) ){ // Проверяем биты от старшего к младшему
               EndatSpiData.RxInd = i-1; // Кол-во принятых бит
               EndatSpiData.EndatPosition |= (tempReg & 0x000F); // Запоминаем данные
               EndatSpiData.EndatPosition &= ~(1<<(i-1)); // Стираем стартовый бит
               break;
           }                  
         }
         
         EndatSpiData.EndatSpiState = ReadFirst12Bit; // Переходим в режим чтения первых 12 бит
         Set_SPI1_Datasize(12); // Задаем размер данных на чтение первого сегмента 12 бит
         SPI_I2S_SendData16(SPI1, 0x0); // Долбилка тактовых импульсов               
       }
       else { //если тикнули 4 раза, но стартового бита не было, то тикаем еще 4 раза
          SPI_SendData8(SPI1, 0x0);
       }           
       
       if (++maxPing > NUM_FIRST_BIT) EndatSpiData.EndatSpiState = StopMode;
       break;
           
     case ReadFirst12Bit: // Вычитываем данные 12-битного сегмента по Endat 
         EndatSpiData.EndatPosition <<= 12; // Задвинуть данные на 12 бит влево
         tempReg = SPI_I2S_ReceiveData16(SPI1);
         EndatSpiData.EndatPosition |= (tempReg & 0x0FFF);
         EndatSpiData.RxInd += 12; // ДОбавляем еще 12 бит
         i = EndatSpiData.RxDataSize - EndatSpiData.RxInd; // Определяем сколько осталось
        
         if (i > 12) { // Если размер большой, читаем 12 бит
           Set_SPI1_Datasize(12); // Задаем размер данных на чтение доп. сегмента 12 бит
           SPI_I2S_SendData16(SPI1, 0x0); // Долбилка тактовых импульсов                             
         } 
         else {
           // Если ждем короткий пакет, переводим SPI1 в конечный режим работы
           Set_SPI1_Datasize(i); // Задаем размер конечного сегмента                      
           SPI_SendData8(SPI1, 0x000); // Долбилка тактовых импульсов 
           EndatSpiData.EndatSpiState = ReadLastBit;
         }
         break;
          
     case ReadLastBit: // Читаем последние данные
         i = EndatSpiData.RxDataSize - EndatSpiData.RxInd; // Определяем сколько осталось
         EndatSpiData.EndatPosition <<= i; // Задвигаем данные влево для оставшихся бит
        
         tempReg = SPI_ReceiveData8(SPI1);
         EndatSpiData.EndatPosition |= (tempReg & ((1<<i)-1));
         EndatSpiData.RxInd += i; // Добавляем остаток
        
         EndatSpiData.EndatSpiState = StopMode;
         SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE); 
         SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
         SPI_Cmd(SPI1, DISABLE);
         
         SPI1->CR1 &= ~(0x03); // Обнуляем CPOL и CPHA
         SPI1->CR1 |= (SPI_CPOL_High | SPI_CPHA_2Edge);  // Задаем протокол передачи CPHA и CPOL
         SPI_Cmd(SPI1, ENABLE);
          
         EndatSpiData.PosState = PacketDone; //!теперь можно обрабатывать полученный пакет с учетом ранее сохраненных значениях быстрых сигналов sin/cos
         encoBlockPnt->EndatPosition = EndatSpiData.EndatPosition;   
         encoBlockPnt->procStatus = DATA_DONE;                    // флаг разрешения для main.c   
         queryStatus = 0;
         break;

    case StopMode:                         
       EndatSpiData.EndatSpiState = StopMode;
       SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE); 
       SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
       
       SPI1->CR1 &= ~(0x03); // Обнуляем CPOL и CPHA
       SPI1->CR1 |= SPI_CPOL_High | SPI_CPHA_2Edge;  // Задаем протокол передачи CPHA и CPOL
       break;
            
    default:
        SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);        
        break;
   }
}


/**
  * @brief  Обработка прерывания при обмене по протоколу SSI
  * @retval
  */
void SSI_IT_Pocessing(){
  encoBlockStatus *encoBlockPnt;
  u16  tempReg = 0;
  u16 GrayToBin; 
  serialModesType serialMode; 
  
  encoBlockPnt = &encoBlock;
  serialMode = encoBlockPnt->baseEncoMotorData.serialMode;
  switch (SSI_SpiData.SsiSpiState)
   {
      case  SsiSendRequest:
          getAnalogSignals(encoBlockPnt); // Считываем данные аналоговых сигналов
          SSI_SpiData.SsiSpiState = SsiReadData;
          SSI_SpiData.RxInd = 0; // Текущее кол-во принятых бит данных                    
          // Меняем режим работы SPI1
          Set_SPI1_Datasize(14);
          SPI_Cmd(SPI1, DISABLE);
          SPI1->CR1 &= ~(0x03); // Обнуляем CPOL и CPHA
          SPI1->CR1 |= SPI_CPOL_High | SPI_CPHA_2Edge; //Садим линию на низкий уровень. Загребаем данные по 1-му обратному фронту
          SPI_Cmd(SPI1, ENABLE);
          SPI_DATA_DIRECT_READ(); // Переходим на чтение                
          tempReg = SPI_I2S_ReceiveData16(SPI1); // Вычитать данные для очистки FIFO - На всякий случай
          tempReg = SPI_I2S_ReceiveData16(SPI1);
          tempReg = SPI_I2S_ReceiveData16(SPI1); 
          SPI_I2S_SendData16(SPI1, 0x0); // Долбилка тактовых импульсов
          SSI_SpiData.SsiSpiState = SsiReadData; // Переходим в режим чтения первых 12 бит        
          break;  
                
    case SsiReadData: // Вычитываем данные 12-битного сегмента по Endat 
            tempReg = SPI_I2S_ReceiveData16(SPI1);
            tempReg &= 0x1FFF;
            if(serialMode == SSI_GRAY){
               GrayToBin = 0;
               while(tempReg > 0){
                  GrayToBin = GrayToBin ^ tempReg;
                  tempReg = tempReg >> 1;
               }
            }else{ //SSI_BIN
              GrayToBin = tempReg;
            }
            SSI_SpiData.SSIPosition = GrayToBin;
            encoBlockPnt->SSIPosition = SSI_SpiData.SSIPosition;
            encoBlockPnt->procStatus = DATA_DONE;
            SPI1->CR1 |= SPI_CPOL_High | SPI_CPHA_2Edge;
            SSI_SpiData.PosState = SsiPacketDone; 
            SSI_SpiData.SsiSpiState = SsiStopMode;
            break;
   case SsiStopMode:                         
           SSI_SpiData.SsiSpiState = SsiStopMode;
           SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE); 
           SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
           
           SPI1->CR1 &= ~(0x03); // Обнуляем CPOL и CPHA
           SPI1->CR1 |= SPI_CPOL_High | SPI_CPHA_2Edge;
          break; 
  default:
          SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);        
        break;
    }
  }


void serialDataQuery(encoBlockStatus *encoBlockPnt){
  encoBlockType blockType;
  serialModesType serialMode;
  blockType = encoBlockPnt->baseEncoMotorData.blockType;
  serialMode = encoBlockPnt->baseEncoMotorData.serialMode;
  
  if(queryStatus == 0){ //Есди предыдущий запрос выполнен
    queryStatus = 1;
    if(blockType == SERIAL){
      switch(serialMode){
      case ENDAT2_0:
      case ECN1313:
      case ECN1325:
      case SSI_GRAY:
      case SSI_BIN:
        EndatSpiData.txSpiCnt++;
        EndatSpiData.SendCommand = (serialMode == ECN1325) ? 0xE0 : 0x1C;    //!0b0000011100; // Команда запроса
        EndatSpiData.TxDataSize = 10;             //!Передаем 10 бит запроса
        EndatSpiData.EndatSpiState = SendRequest; //!Посылаем запрос на чтение данных   
        EndatSpiData.EndatPosition = 0;           //!Обнулить перед приемом
        EndatSpiData.RxInd = 0;
        SSI_SpiData.TxDataSize = 10;              //!Передаем 10 бит запроса
        SSI_SpiData.SsiSpiState = SsiSendRequest; //!Посылаем запрос на чтение данных   
        SSI_SpiData.SSIPosition = 0;              //!Обнулить перед приемом
        SSI_SpiData.RxInd = 0;
        EndatSpiData.RxDataSize = getRxDataSize(encoBlockPnt);  //Расчет длины считываемых данных
        SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE); // Запрос на передачу, путем активирования прерывания на передачу
        encoBlockPnt->queryCnt++; //ДЛЯ ОТЛАДКИ
        break;
      }
    }
  }
}

u16 getRxDataSize(encoBlockStatus *encoBlockPnt){
  serialModesType serialMode;
  u16 RxDataSize;
  u16 bitResolution;
  serialMode = encoBlockPnt->baseEncoMotorData.serialMode;
  bitResolution = encoBlockPnt->baseEncoMotorData.bitResolution;
  
  switch(serialMode){
  case ENDAT2_0:
  case ECN1313:
    RxDataSize = (NUM_CRC_BITS + NUM_ERR_BITS21 + bitResolution);  // кол-во бит для Endat2.1
    break;
  case ECN1325:
    RxDataSize = (NUM_CRC_BITS + NUM_ERR_BITS22 + bitResolution);  // кол-во бит для Endat2.2
    break;
  case SSI_GRAY:
  case SSI_BIN:
    RxDataSize = bitResolution;
    break;
  }
  return(RxDataSize);
}

void getAnalogSignals(encoBlockStatus *encoBlockPnt){
  encoBlockType blockType;
  
  blockType = encoBlockPnt->baseEncoMotorData.blockType;
  encoBlockPnt->analogSignals.fastSin = (*ADC_SIN2 - DC_OFFSET) >> 2;
  encoBlockPnt->analogSignals.fastCos = (*ADC_COS2 - DC_OFFSET) >> 2;
  
  if(blockType == SIN_COS){
     encoBlockPnt->analogSignals.slowSin = (*ADC_SIN1 - DC_OFFSET) >> 2; 
     encoBlockPnt->analogSignals.slowCos = (*ADC_COS1 - DC_OFFSET) >> 2;
  }
}

void TIM1_BRK_TIM15_IRQHandler(void)
{
  u16 stateA;
  u16 stateB;
  s16 spdSign;
  float electricSpeed;
  encoBlockStatus *encoBlockPnt;
  
  encoBlockPnt = &encoBlock;
  electricSpeed = encoBlockPnt->calculatedData.electricSpd;
  
  if (TIM_GetITStatus(TIM15, TIM_IT_CC1) != RESET){
    TIM_ClearITPendingBit(TIM15, TIM_IT_CC1);
    stateA = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
    stateB = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);
    spdSign = (electricSpeed < 0.0F) ? NEGATIVE_SPD : POSITIVE_SPD;
    //Проверка соотношения фаз сигналов A и B эмулятоа энкодера.
    //В случае неправльного соотношения фаз, выполняется перенастройка модуля совпадения
    switch(spdSign){
    case POSITIVE_SPD:
      if((stateA == SET) && (stateB == RESET)){ //Если A опережает B то норм
      }else if((stateA == SET) && (stateB == SET)){//Если A отстает от B то делаем инверсию
        TIM15->CCER ^= TIM_CCER_CC1P;              //Инверсия реакции на совпадение              
      }
      break;
    case NEGATIVE_SPD:
      if((stateA == SET) && (stateB == SET)){ //Если A отстает от B то норм
      }else if((stateA == SET) && (stateB == RESET)){ //Если A отстает от B то делаем инверсию
        TIM15->CCER ^= TIM_CCER_CC1P;                 //Инверсия реакции на совпадение
      }
      break;
    }
  }
}


/**
  * @brief  Обработка таймаута таймера управления индикацией аварии
            обмена с верхним уровнем
  * @retval
  */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM17, TIM_IT_Update) != RESET){
    TIM_ClearITPendingBit(TIM17, TIM_IT_Update);
    glowErrState = !glowErrState;         //Сменить состояние
    if(glowErrState == SET){
      GPIO_SetBits(GPIOC, GPIO_Pin_15);   //Включить свечение светодиода
    }else{
      GPIO_ResetBits(GPIOC, GPIO_Pin_15); //Отключить свечение светодиода
    }
  }
}


/**
  * @brief  Обработка поступления Z-сигнала от инкрементального энкодера
  * @retval
  */
void TIM1_UP_TIM16_IRQHandler(void){
  static u16 debug = 0;
  
  if (TIM_GetITStatus(TIM16, TIM_IT_CC1) != RESET){
    TIM_ClearITPendingBit(TIM16, TIM_IT_CC1);
    R_signalFlg = 1;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
