#include "flash_memory.h"


void WriteValuesToFlash(uint32_t StartAddress, uint32_t tempData, uint32_t humiData)
{
  // Erase the flash memory page before writing
  //Flash_Erase(StartAddress);


  // Write temperature and humidity data
  Flash_Write(StartAddress, tempData);       // Write temperature data
  Flash_Write(StartAddress + 4, humiData);   // Write humidity data

	 // Move to the next address (4 bytes for temp + 4 bytes for humidity)

}





void Flash_Erase(uint32_t StartPageAddress)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    // Fill EraseInit structure
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = FLASH_SECTOR_3;
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        // Error occurred while page erase.
        // Handle the error appropriately
        Error_Handler();
    }

    HAL_FLASH_Lock();
}

void Flash_Write(uint32_t Address, uint32_t Data)
{
    HAL_FLASH_Unlock();

    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, Address, Data) != HAL_OK)
    {
        // Error occurred while writing data in Flash memory.
        // Handle the error appropriately
        Error_Handler();
    }

    HAL_FLASH_Lock();
}

void ReadAllValuesFromFlash(uint32_t StartAddress, uint32_t* tempData, uint32_t* humiData)
{
    uint32_t i = 0;
    tempData[i]=0;
    humiData[i]=0;
    while (1) {
        // Read temperature data
        uint32_t temp = *(__IO uint32_t*)(StartAddress );

        // Check for invalid or unwritten data (assuming 0xFFFFFFFF is unwritten flash data)
        if (temp == 0xFFFFFFFF)
        {
            break;
        }

        tempData[i] = temp;

        // Read humidity data
        uint32_t humi = *(__IO uint32_t*)(StartAddress +  4);
        if (humi == 0xFFFFFFFF)
        {
            break;
        }

        humiData[i] = humi;

        i++;
    	StartAddress += 8;
    }


   }

