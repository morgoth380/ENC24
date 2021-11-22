
#include "flashDrv.h"
#include "stm32f30x.h"

/**
  * @brief  Разблокировка FLASH-памяти перед записью
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
  * @brief  Блокировка FLASH-памяти
  * @param  
  * @retval 
  */
void flash_lock() {
  FLASH->CR |= FLASH_CR_LOCK;
}


/**
  * @brief  Проверка готовности flash памяти для записи или стирания
  * @param  
  * @retval true если можно стирать или записывать во flash
  */
uint8_t flash_ready(void) {
  return !(FLASH->SR & FLASH_SR_BSY);
}


/**
  * @brief  Стирание страницы памяти
  * @param  
  * @retval
  */
void flash_erase_page(uint32_t address) {
  FLASH->CR|= FLASH_CR_PER; //Устанавливаем бит стирания одной страницы
  FLASH->AR = address; // Задаем её адрес
  FLASH->CR|= FLASH_CR_STRT; // Запускаем стирание 
  while(!flash_ready())
    ;  //Ждем пока страница сотрется. 
  FLASH->CR &= ~FLASH_CR_PER; //Сбрасываем бит обратно
}

/**
  * @brief  Запись данных во FLASH-память
  * @param  address куда будут записаны данные
  * @param  data записываемое значение
  * @retval
  */
/*
void flash_write(uint32_t address, _iq data) {
  FLASH->CR |= FLASH_CR_PG; //Разрешаем программирование флеша
  while(!flash_ready()){ //Ожидаем готовности флеша к записи
    ;
  }
  *(__IO uint16_t*)address = (uint16_t)data; //Пишем младшие 2 бата
  while(!flash_ready()){
    ;
  }
  address+=2;
  data>>=16;
  *(__IO uint16_t*)address = (uint16_t)data; //Пишем старшие 2 байта
  while(!flash_ready()){
    ;
  }
  FLASH->CR &= ~(FLASH_CR_PG); //Запрещаем программирование флеша
   
}
*/


void flash_write(uint32_t address, encoFlashMemDataType *encoFlashMemData) {
  uint16_t i;
  uint16_t *pnt = (uint16_t *)encoFlashMemData;
   __IO uint16_t* flashAddr = (__IO uint16_t*)address;
  FLASH->CR |= FLASH_CR_PG; //Разрешаем программирование flash-памяти
  uint16_t tmp = sizeof(encoFlashMemDataType); //сколько байт должно быть записано
  tmp /= 2; //сколько 2х байтных слов должно быть записано
  
  for(i = 0; i < tmp; i++){
    while(!flash_ready()){ //Ожидаем готовности flash-памяти к записи
      ;
    }
     *flashAddr = (uint16_t)(pnt[i]); //Пишем младшие 2 байта
     ++flashAddr;
  }
  while(!flash_ready()){
    ;
  }
  FLASH->CR &= ~(FLASH_CR_PG); //Запрещаем программирование флеша    
}


/**
  * @brief  Чтение корректирующего фазового сдвига из FLASH-памяти.
  *         Первые две ячейки памяти занимают настройка входов модуля
  *         энкодера и настройка знака быстрой скорости. Затем расположен
  *         корректирующий фазовый сдвиг.
  * @param  address откуда будут считаны данные
  * @retval считанное значение
  */
float corrPhaseRead(uint32_t address) {
  float *corrAnglePnt;
  float corrAngle;
  __IO s32 *flashPnt;
  
  flashPnt = (__IO s32*) (address + 2 * sizeof(s32)); //Адрес параметра с корректирующим углом
  
  if(*flashPnt == -1L){
    corrAngle = 0.0F;
  }else{
    corrAnglePnt = (float *)flashPnt;
    corrAngle = *corrAnglePnt;
  }
  return (corrAngle);
}

/**
  * @brief  Чтение текущей настрйки направления счета модуля энкодера
  * @param  
  * @retval текущее значение настройки входа
  */
uint32_t flashInputSet_read(uint32_t address){
  return (*(__IO uint32_t*) (address));
}


s32 flash_FastSpdSignRead(uint32_t address) {
  return ( *(__IO s32*) (address + sizeof(s32)) );
}