#ifndef FLASH_MEMORY_H
#define FLASH_MEMORY_H





#include "stm32f4xx_hal.h"

void WriteValuesToFlash(uint32_t StartAddress, uint32_t tempData, uint32_t humiData);
void Flash_Erase(uint32_t StartPageAddress);
void Flash_Write(uint32_t Address, uint32_t Data);
void ReadAllValuesFromFlash(uint32_t StartAddress, uint32_t* tempData, uint32_t* humiData );

#endif // FLASH_MEMORY_H
