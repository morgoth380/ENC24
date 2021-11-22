
#include "flashDrv.h"
#include "stm32f30x.h"

/**
  * @brief  ������������� FLASH-������ ����� �������
  * @param  
  * @retval 
  */
#define FLASH_KEY1 ((uint32_t)0x45670123)
#define FLASH_KEY2 ((uint32_t)0xCDEF89AB)
void flash_unlock(void) {
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
}

/**
  * @brief  ���������� FLASH-������
  * @param  
  * @retval 
  */
void flash_lock() {
  FLASH->CR |= FLASH_CR_LOCK;
}


/**
  * @brief  �������� ���������� flash ������ ��� ������ ��� ��������
  * @param  
  * @retval true ���� ����� ������� ��� ���������� �� flash
  */
uint8_t flash_ready(void) {
  return !(FLASH->SR & FLASH_SR_BSY);
}


/**
  * @brief  �������� �������� ������
  * @param  
  * @retval
  */
void flash_erase_page(uint32_t address) {
  FLASH->CR|= FLASH_CR_PER; //������������� ��� �������� ����� ��������
  FLASH->AR = address; // ������ � �����
  FLASH->CR|= FLASH_CR_STRT; // ��������� �������� 
  while(!flash_ready())
    ;  //���� ���� �������� ��������. 
  FLASH->CR &= ~FLASH_CR_PER; //���������� ��� �������
}

/**
  * @brief  ������ ������ �� FLASH-������
  * @param  address ���� ����� �������� ������
  * @param  data ������������ ��������
  * @retval
  */
/*
void flash_write(uint32_t address, _iq data) {
  FLASH->CR |= FLASH_CR_PG; //��������� ���������������� �����
  while(!flash_ready()){ //������� ���������� ����� � ������
    ;
  }
  *(__IO uint16_t*)address = (uint16_t)data; //����� ������� 2 ����
  while(!flash_ready()){
    ;
  }
  address+=2;
  data>>=16;
  *(__IO uint16_t*)address = (uint16_t)data; //����� ������� 2 �����
  while(!flash_ready()){
    ;
  }
  FLASH->CR &= ~(FLASH_CR_PG); //��������� ���������������� �����
   
}
*/


void flash_write(uint32_t address, encoFlashMemDataType *encoFlashMemData) {
  uint16_t i;
  uint16_t *pnt = (uint16_t *)encoFlashMemData;
   __IO uint16_t* flashAddr = (__IO uint16_t*)address;
  FLASH->CR |= FLASH_CR_PG; //��������� ���������������� flash-������
  uint16_t tmp = sizeof(encoFlashMemDataType); //������� ���� ������ ���� ��������
  tmp /= 2; //������� 2� ������� ���� ������ ���� ��������
  
  for(i = 0; i < tmp; i++){
    while(!flash_ready()){ //������� ���������� flash-������ � ������
      ;
    }
     *flashAddr = (uint16_t)(pnt[i]); //����� ������� 2 �����
     ++flashAddr;
  }
  while(!flash_ready()){
    ;
  }
  FLASH->CR &= ~(FLASH_CR_PG); //��������� ���������������� �����    
}


/**
  * @brief  ������ ��������������� �������� ������ �� FLASH-������.
  *         ������ ��� ������ ������ �������� ��������� ������ ������
  *         �������� � ��������� ����� ������� ��������. ����� ����������
  *         �������������� ������� �����.
  * @param  address ������ ����� ������� ������
  * @retval ��������� ��������
  */
float corrPhaseRead(uint32_t address) {
  float *corrAnglePnt;
  float corrAngle;
  __IO s32 *flashPnt;
  
  flashPnt = (__IO s32*) (address + 2 * sizeof(s32)); //����� ��������� � �������������� �����
  
  if(*flashPnt == -1L){
    corrAngle = 0.0F;
  }else{
    corrAnglePnt = (float *)flashPnt;
    corrAngle = *corrAnglePnt;
  }
  return (corrAngle);
}

/**
  * @brief  ������ ������� �������� ����������� ����� ������ ��������
  * @param  
  * @retval ������� �������� ��������� �����
  */
uint32_t flashInputSet_read(uint32_t address){
  return (*(__IO uint32_t*) (address));
}


s32 flash_FastSpdSignRead(uint32_t address) {
  return ( *(__IO s32*) (address + sizeof(s32)) );
}