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
#define  FIRST_CAPTURE 0  //!��� ������� ������� �� ������� ��������
#define  DC_OFFSET   2048  //!DC-��������
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
      encoBlockPnt->incrEncoPos = TIM_GetCounter(TIM1); //����������� ������� ��������
      serialDataQuery(encoBlockPnt);  //������������ ������ � ������������ ������
      getAnalogSignals(encoBlockPnt); //������� �������� ���������� ��������
      break;
    case SIN_COS:
      encoBlockPnt->incrEncoPos = TIM_GetCounter(TIM1); //����������� ������� ��������
      getAnalogSignals(encoBlockPnt);                   //������� �������� ���������� ��������
      encoBlockPnt->procStatus = DATA_DONE;             //���� ���������� ��� main.c
      break;
    case INCREMENTAL:
      encoBlockPnt->incrEncoPos = TIM_GetCounter(TIM2); //����������� ������� ��������
      encoBlockPnt->procStatus = DATA_DONE;             //���� ���������� ��� main.c
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
// � size �������� �������� �� 4 �� 16 - ������� ������� ����� ����������.
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
   // �������� Busy
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
  // �������� ��� ���������� �������
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //!���������� ���� ���������� �������
    IFState |= TIM_IT_Update;
  } 
  
  if(TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET)  //!���� ���������� ������� �� ������� ������� (�������� �������� ����� ������� ��������)
  {
    /* Clear TIM1 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);    //!���������� ���� �������
  } 
  
    if(TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET)  //!���� ���������� ������� �� ������� ������� (�������� �������� ����� ������� ��������)
  {
    /* Clear TIM1 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);    //!���������� ���� �������
  }
}


/**
  * @brief  ���������� �� ������� �� ������� ����������� ����� 
  *         ���������� �� ���������� ��� �������� �������� �������� ����������������
  *         ��������
  * @param  None
  * @retval None
  */
u16 R_Event;
void TIM3_IRQHandler(void){
  encoBlockStatus *encoBlockPnt;
  
  encoBlockPnt = &encoBlock;
  if(TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET){ //���� �������� ����������� ������
    R_Event = 1; //���� ������� � ���������� ��������
    encoBlockPnt->RsignalFlg = 1; //���� ��� �������� � ������
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);      //!���������� ���� ���������� ������� TIM3
  }
}

//������ ��� �������. ��������� ������� ������ ������� ��������� ������ ��������
void TIM7_IRQHandler(void){
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET){ 
     TIM_ClearITPendingBit(TIM7, TIM_IT_Update);      
     debugOvfNum++;                                  
  } 
}



/**
  * @brief �������� ������ ������������ ���������� �������� �������� sin/cos � enDat
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
    errCnt = (encoBlockPnt->PWMOn == 0) ? 0 : errCnt; //����� �������� ��� ����������� ���
    prevCaptureChannel = (encoBlockPnt->PWMOn == 0) ? NO_CAPTURE : prevCaptureChannel;
    incrSignalErr = (encoBlockPnt->PWMOn == 0) ? ENCO_OK : incrSignalErr;
    
    if(TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET)  //!���� ���������� �� ������ 1
    {
      switch(prevCaptureChannel){ //�������� ����������� ������ �������
      case CHANNEL_1: //�������� ������� �� ��������� - �� ��
         prevCaptureChannel = CHANNEL_1;
         errCnt++;
         tmpErr = (errCnt == CAPT_ERR_NUM) ? B_MISS_IN_WORK : tmpErr;
         if(tmpErr == B_MISS_IN_WORK){ //���� ������� ����, ��������� ������� ����� ������
           tmpErr = (encoBlockPnt->autoPhasingOn == 1) ? A_B_MISS_IN_TUNE_ERR : A_B_MISS_IN_RUNNING_ERR; //������ ��� ������ � ����������� �� �������
         }
         errCnt = (errCnt == CAPT_ERR_NUM) ? 0 : errCnt;
         break;
      case CHANNEL_2: //�������� ������� ��������� - ��
         errCnt = 0;
         prevCaptureChannel = CHANNEL_1;
         break;
      default: //NO_CAPTURE
         errCnt = 0;
         prevCaptureChannel = CHANNEL_1;
         break; 
      }
      TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);    //!���������� ���� ������� 
    }else if(TIM_GetITStatus(TIM1, TIM_IT_CC2) == SET){ //����� �������� ���������� �� ������ 2
      switch(prevCaptureChannel){
      case CHANNEL_1: //�������� ������� ��������� - ��
         prevCaptureChannel = CHANNEL_2;
         errCnt = 0;
         break;
      case CHANNEL_2: //�������� ������� �� ��������� - �� ��
         prevCaptureChannel = CHANNEL_2;
         errCnt++;
         tmpErr = (errCnt == CAPT_ERR_NUM) ? A_MISS_IN_WORK : tmpErr;
         if(tmpErr == A_MISS_IN_WORK){ //���� ������� ����, ��������� ������� ����� ������
           tmpErr = (encoBlockPnt->autoPhasingOn == 1) ? A_B_MISS_IN_TUNE_ERR : A_B_MISS_IN_RUNNING_ERR; //������ ��� ������ � ����������� �� �������
         }
         errCnt = (errCnt == CAPT_ERR_NUM) ? 0 : errCnt;
        break;
      default: //NO_CAPTURE
         errCnt = 0;
         prevCaptureChannel = CHANNEL_2;
         break; 
      }
      TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);    //!���������� ���� ������� 
    }
    incrSignalErr = tmpErr;
}


/**
  * @brief  ��������� ���������� ��� ������ �� ��������� enDat
  * @retval
  */
void enDatIT_Pocessing(){
  encoBlockStatus *encoBlockPnt;
  u16  tempReg = 0, i = 0;
  u16  static maxPing = 0;

  encoBlockPnt = &encoBlock;
  switch (EndatSpiData.EndatSpiState){
    case  SendRequest: // �������� ������
    if (EndatSpiData.TxDataSize) { // ���� ���� �� �������� - ����� �� ����
      SPI_DATA_DIRECT_WRITE();
      Set_SPI1_Datasize(EndatSpiData.TxDataSize); // ������ ������ ��������          
      SPI_I2S_SendData16(SPI1, EndatSpiData.SendCommand);// �������� ������
      EndatSpiData.TxDataSize = 0;    // �������� ���� ��������  
      getAnalogSignals(encoBlockPnt); // ��������� ������ ���������� ��������
    }
    else {                    
      EndatSpiData.EndatSpiState = WaitStartBit;          
      EndatSpiData.RxInd = 0; // ������� ���-�� �������� ��� ������                    
      // ������ ����� ������ SPI1
      
      Set_SPI1_Datasize(NUM_FIRST_BIT); // ������ ������ ������ �� �������� ���������� ���� - 4 ���� ����� 
      
      SPI_Cmd(SPI1, DISABLE);
      SPI1->CR1 &= ~(0x03); // �������� CPOL � CPHA
      SPI1->CR1 |= SPI_CPOL_Low | SPI_CPHA_2Edge;
      SPI_Cmd(SPI1, ENABLE);
      
      SPI_DATA_DIRECT_READ(); // ��������� �� ������
                           
      maxPing = 0;
      tempReg = SPI_I2S_ReceiveData16(SPI1); // �������� ������ ��� ������� FIFO - �� ������ ������
      tempReg = SPI_I2S_ReceiveData16(SPI1);
      tempReg = SPI_I2S_ReceiveData16(SPI1);
      //SPI_I2S_SendData16(SPI1, 0x0);
      SPI_SendData8(SPI1, 0x0); // �������� �������� ��������� ��� �������� ���������� ���� 
    }                   
    break;  
    
    case WaitStartBit: // ������� ��������� ���                          
       tempReg = SPI_ReceiveData8(SPI1); 
       if ( tempReg != 0) {// ���� ������ �� ����, ������ ������ ��������� ���
           // ���������� ������� �������� ��� �� ������� �� �������� 4 ���
         for (i=NUM_FIRST_BIT; i>0; i--) {
           if ( tempReg & (1<<(i-1)) ){ // ��������� ���� �� �������� � ��������
               EndatSpiData.RxInd = i-1; // ���-�� �������� ���
               EndatSpiData.EndatPosition |= (tempReg & 0x000F); // ���������� ������
               EndatSpiData.EndatPosition &= ~(1<<(i-1)); // ������� ��������� ���
               break;
           }                  
         }
         
         EndatSpiData.EndatSpiState = ReadFirst12Bit; // ��������� � ����� ������ ������ 12 ���
         Set_SPI1_Datasize(12); // ������ ������ ������ �� ������ ������� �������� 12 ���
         SPI_I2S_SendData16(SPI1, 0x0); // �������� �������� ���������               
       }
       else { //���� ������� 4 ����, �� ���������� ���� �� ����, �� ������ ��� 4 ����
          SPI_SendData8(SPI1, 0x0);
       }           
       
       if (++maxPing > NUM_FIRST_BIT) EndatSpiData.EndatSpiState = StopMode;
       break;
           
     case ReadFirst12Bit: // ���������� ������ 12-������� �������� �� Endat 
         EndatSpiData.EndatPosition <<= 12; // ��������� ������ �� 12 ��� �����
         tempReg = SPI_I2S_ReceiveData16(SPI1);
         EndatSpiData.EndatPosition |= (tempReg & 0x0FFF);
         EndatSpiData.RxInd += 12; // ��������� ��� 12 ���
         i = EndatSpiData.RxDataSize - EndatSpiData.RxInd; // ���������� ������� ��������
        
         if (i > 12) { // ���� ������ �������, ������ 12 ���
           Set_SPI1_Datasize(12); // ������ ������ ������ �� ������ ���. �������� 12 ���
           SPI_I2S_SendData16(SPI1, 0x0); // �������� �������� ���������                             
         } 
         else {
           // ���� ���� �������� �����, ��������� SPI1 � �������� ����� ������
           Set_SPI1_Datasize(i); // ������ ������ ��������� ��������                      
           SPI_SendData8(SPI1, 0x000); // �������� �������� ��������� 
           EndatSpiData.EndatSpiState = ReadLastBit;
         }
         break;
          
     case ReadLastBit: // ������ ��������� ������
         i = EndatSpiData.RxDataSize - EndatSpiData.RxInd; // ���������� ������� ��������
         EndatSpiData.EndatPosition <<= i; // ��������� ������ ����� ��� ���������� ���
        
         tempReg = SPI_ReceiveData8(SPI1);
         EndatSpiData.EndatPosition |= (tempReg & ((1<<i)-1));
         EndatSpiData.RxInd += i; // ��������� �������
        
         EndatSpiData.EndatSpiState = StopMode;
         SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE); 
         SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
         SPI_Cmd(SPI1, DISABLE);
         
         SPI1->CR1 &= ~(0x03); // �������� CPOL � CPHA
         SPI1->CR1 |= (SPI_CPOL_High | SPI_CPHA_2Edge);  // ������ �������� �������� CPHA � CPOL
         SPI_Cmd(SPI1, ENABLE);
          
         EndatSpiData.PosState = PacketDone; //!������ ����� ������������ ���������� ����� � ������ ����� ����������� ��������� ������� �������� sin/cos
         encoBlockPnt->EndatPosition = EndatSpiData.EndatPosition;   
         encoBlockPnt->procStatus = DATA_DONE;                    // ���� ���������� ��� main.c   
         queryStatus = 0;
         break;

    case StopMode:                         
       EndatSpiData.EndatSpiState = StopMode;
       SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE); 
       SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
       
       SPI1->CR1 &= ~(0x03); // �������� CPOL � CPHA
       SPI1->CR1 |= SPI_CPOL_High | SPI_CPHA_2Edge;  // ������ �������� �������� CPHA � CPOL
       break;
            
    default:
        SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);        
        break;
   }
}


/**
  * @brief  ��������� ���������� ��� ������ �� ��������� SSI
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
          getAnalogSignals(encoBlockPnt); // ��������� ������ ���������� ��������
          SSI_SpiData.SsiSpiState = SsiReadData;
          SSI_SpiData.RxInd = 0; // ������� ���-�� �������� ��� ������                    
          // ������ ����� ������ SPI1
          Set_SPI1_Datasize(14);
          SPI_Cmd(SPI1, DISABLE);
          SPI1->CR1 &= ~(0x03); // �������� CPOL � CPHA
          SPI1->CR1 |= SPI_CPOL_High | SPI_CPHA_2Edge; //����� ����� �� ������ �������. ��������� ������ �� 1-�� ��������� ������
          SPI_Cmd(SPI1, ENABLE);
          SPI_DATA_DIRECT_READ(); // ��������� �� ������                
          tempReg = SPI_I2S_ReceiveData16(SPI1); // �������� ������ ��� ������� FIFO - �� ������ ������
          tempReg = SPI_I2S_ReceiveData16(SPI1);
          tempReg = SPI_I2S_ReceiveData16(SPI1); 
          SPI_I2S_SendData16(SPI1, 0x0); // �������� �������� ���������
          SSI_SpiData.SsiSpiState = SsiReadData; // ��������� � ����� ������ ������ 12 ���        
          break;  
                
    case SsiReadData: // ���������� ������ 12-������� �������� �� Endat 
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
           
           SPI1->CR1 &= ~(0x03); // �������� CPOL � CPHA
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
  
  if(queryStatus == 0){ //���� ���������� ������ ��������
    queryStatus = 1;
    if(blockType == SERIAL){
      switch(serialMode){
      case ENDAT2_0:
      case ECN1313:
      case ECN1325:
      case SSI_GRAY:
      case SSI_BIN:
        EndatSpiData.txSpiCnt++;
        EndatSpiData.SendCommand = (serialMode == ECN1325) ? 0xE0 : 0x1C;    //!0b0000011100; // ������� �������
        EndatSpiData.TxDataSize = 10;             //!�������� 10 ��� �������
        EndatSpiData.EndatSpiState = SendRequest; //!�������� ������ �� ������ ������   
        EndatSpiData.EndatPosition = 0;           //!�������� ����� �������
        EndatSpiData.RxInd = 0;
        SSI_SpiData.TxDataSize = 10;              //!�������� 10 ��� �������
        SSI_SpiData.SsiSpiState = SsiSendRequest; //!�������� ������ �� ������ ������   
        SSI_SpiData.SSIPosition = 0;              //!�������� ����� �������
        SSI_SpiData.RxInd = 0;
        EndatSpiData.RxDataSize = getRxDataSize(encoBlockPnt);  //������ ����� ����������� ������
        SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE); // ������ �� ��������, ����� ������������� ���������� �� ��������
        encoBlockPnt->queryCnt++; //��� �������
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
    RxDataSize = (NUM_CRC_BITS + NUM_ERR_BITS21 + bitResolution);  // ���-�� ��� ��� Endat2.1
    break;
  case ECN1325:
    RxDataSize = (NUM_CRC_BITS + NUM_ERR_BITS22 + bitResolution);  // ���-�� ��� ��� Endat2.2
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
    //�������� ����������� ��� �������� A � B �������� ��������.
    //� ������ ������������ ����������� ���, ����������� ������������� ������ ����������
    switch(spdSign){
    case POSITIVE_SPD:
      if((stateA == SET) && (stateB == RESET)){ //���� A ��������� B �� ����
      }else if((stateA == SET) && (stateB == SET)){//���� A ������� �� B �� ������ ��������
        TIM15->CCER ^= TIM_CCER_CC1P;              //�������� ������� �� ����������              
      }
      break;
    case NEGATIVE_SPD:
      if((stateA == SET) && (stateB == SET)){ //���� A ������� �� B �� ����
      }else if((stateA == SET) && (stateB == RESET)){ //���� A ������� �� B �� ������ ��������
        TIM15->CCER ^= TIM_CCER_CC1P;                 //�������� ������� �� ����������
      }
      break;
    }
  }
}


/**
  * @brief  ��������� �������� ������� ���������� ���������� ������
            ������ � ������� �������
  * @retval
  */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM17, TIM_IT_Update) != RESET){
    TIM_ClearITPendingBit(TIM17, TIM_IT_Update);
    glowErrState = !glowErrState;         //������� ���������
    if(glowErrState == SET){
      GPIO_SetBits(GPIOC, GPIO_Pin_15);   //�������� �������� ����������
    }else{
      GPIO_ResetBits(GPIOC, GPIO_Pin_15); //��������� �������� ����������
    }
  }
}


/**
  * @brief  ��������� ����������� Z-������� �� ���������������� ��������
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
