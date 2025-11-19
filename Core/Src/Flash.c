/********************************************************************************

* @File Flash.c

* @Author: Ma Ziteng

* @Version: 1.0

* @Date: 2025-11

* @Description: Flash闪存读写

********************************************************************************/

#include "stm32f1xx_hal.h"

#include "Flash.h"

/**
  * @brief		读取一个字
  * @param		Address 读取的起始地址
  * @return  	读出的一字（4个字节）
  * @note			__IO 用于保护 等效于 volatile
  */
uint32_t MyFLASH_ReadWord(uint32_t Address){
	return *((__IO uint32_t* )Address);
}

/**
  * @brief		读取半字
  * @param		Address 读取的起始地址
  * @return  	读出的半字（2个字节）
  * @note			__IO 用于保护 等效于 volatile
  */
uint16_t MyFLASH_ReadHalfWord(uint32_t Address){
	return *((__IO uint16_t* )Address);
}

/**
  * @brief		读取一个字节
  * @param		Address 读取的起始地址
  * @return  	读出的一个字节
  * @note			__IO 用于保护 等效于 volatile
  */
uint8_t MyFLASH_ReadByte(uint32_t Address){
	return *((__IO uint8_t* )Address);
}

/**
  * @brief		全擦除
  * @param		无
  * @return  	无
  */
void MyFLASH_EraseAllPages(void){
	HAL_FLASH_Unlock();
	// 定义擦除结构体
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError = 0;

	// 配置擦除参数
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_MASSERASE;
	EraseInitStruct.Banks = FLASH_BANK_1;  // 根据具体型号选择
	EraseInitStruct.PageAddress = 0x00000000;  
	EraseInitStruct.NbPages = 1;  
	// 执行擦除
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
    // 擦除失败处理
    Error_Handler();
	}
	
	HAL_FLASH_Lock();
}
/**
  * @brief		页擦除
  * @param		PageAddress		擦除页的起始地址
  * @param		Num						擦除页的数量
  * @return  	无
  */
void MyFLASH_ErasePage(uint32_t PageAddress , uint8_t Num){
	HAL_FLASH_Unlock();
	// 定义擦除结构体
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError = 0;

	// 配置擦除参数
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks = FLASH_BANK_1;  // 根据具体型号选择
	EraseInitStruct.PageAddress = PageAddress;  
	EraseInitStruct.NbPages = Num;  
	// 执行擦除
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
    // 擦除失败处理
    Error_Handler();
	}
	
	HAL_FLASH_Lock();
}

/**
  * @brief		写入一字
  * @param		Address			起始地址
  * @param		Data				32位数据
  * @return  	无
  */
void MyFLASH_ProgramWord(uint32_t Address,uint32_t Data){
	HAL_FLASH_Unlock();
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Data) != HAL_OK) {
    // 写入失败处理
    Error_Handler();
	}
	HAL_FLASH_Lock();
}

/**
  * @brief		写入半字
  * @param		Address			起始地址
  * @param		Data				16位数据
  * @return  	无
  */
void MyFLASH_ProgramHalfWord(uint32_t Address,uint16_t Data){
	HAL_FLASH_Unlock();
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, Data) != HAL_OK) {
    // 写入失败处理
    Error_Handler();
	}
	HAL_FLASH_Lock();
}
