#ifndef _FLASH_DRV_H_
#define _FLASH_DRV_H_

#include "flashDrv.h"
#include "stm32f30x.h"
#include "IQmathLib.h"
#include "enco.h"
#define BASE_PAGE_ADDR  0x0800F800  //адрес последней страницы FLASH-памяти для хранения фазовой поправки
#define NO_INIT_VAL -1L

void flash_unlock(void);
void flash_lock(void);
uint8_t flash_ready(void);
void flash_erase_page(uint32_t address);
void flash_write(uint32_t address, encoFlashMemDataType *encoFlashMemData);
float corrPhaseRead(uint32_t address);
uint32_t flashInputSet_read(uint32_t address);
s32 flash_FastSpdSignRead(uint32_t address);

#endif /*_FLASH_DRV_H_*/