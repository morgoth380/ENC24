/**
  ******************************************************************************
  * @file    GPIO/GPIO_Toggle/stm32f30x_it.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-October-2012
  * @brief   This file contains the headers of the interrupt handlers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F30X_IT_H
#define __STM32F30X_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void TIM6_DAC_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM1_UP_TIM16_IRQHandler(void);
void SPI1_IRQHandler (void);
void DMA1_Channel1_IRQHandler (void);
void DMA1_Channel2_IRQHandler (void);
void TIM1_CC_IRQHandler(void);
void TIM1_BRK_TIM15_IRQHandler(void);
void TIM1_TRG_COM_TIM17_IRQHandler(void);


void Set_SPI1_Datasize (u8 size);
unsigned char DelayMs1(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F30X_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/