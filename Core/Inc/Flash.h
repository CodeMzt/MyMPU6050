#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>

uint32_t MyFLASH_ReadWord(uint32_t Address);
uint16_t MyFLASH_ReadHalfWord(uint32_t Address);
uint8_t MyFLASH_ReadByte(uint32_t Address);
void MyFLASH_EraseAllPages(void);
void MyFLASH_ErasePage(uint32_t PageAddress,uint8_t Num);
void MyFLASH_ProgramWord(uint32_t Address,uint32_t Data);
void MyFLASH_ProgramHalfWord(uint32_t Address,uint16_t Data);

#endif
