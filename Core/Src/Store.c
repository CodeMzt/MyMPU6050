/********************************************************************************

* @File Store.c

* @Author: Ma Ziteng

* @Version: 1.0

* @Date: 2025-10

* @Description: 闪存数据处理，上层操作

********************************************************************************/

#include "stm32f10x.h"                  // Device header
#include "Store.h"
#include "Flash.h"

//存储校准数据，需要3kb即3页闪存
#define		STORE_ADDRESS1	0x0801F400
#define		STORE_ADDRESS2	0x0801F800
#define		STORE_ADDRESS3	0x0801FC00

extern uint8_t pitch_bias[1024],roll_bias[1024],yaw_bias[1024];

/**
	* @brief			上电初始化
  * @param		无
  * @retval  	1		首次初始化，需要启动校准
  * @retval  	0		非首次初始化，不需要启动校准
  */
uint8_t Store_Init(void){
	uint16_t i,temp;
//	MyFLASH_ErasePage(STORE_ADDRESS1,3);
	//如果第一项直接是FFFF 则直接跳过
	if(MyFLASH_ReadHalfWord(STORE_ADDRESS1+500)==0xFFFF)
		return 1;
	for(i=0;i<512;i++){
		temp = MyFLASH_ReadHalfWord(STORE_ADDRESS1 + i*2);
		pitch_bias[2*i] = 	(temp & 0x00FF);
		pitch_bias[2*i+1] = ((temp>>8) & 0x00FF);
	}
	for(i=0;i<512;i++){
		temp = MyFLASH_ReadHalfWord(STORE_ADDRESS2 + i*2);
		roll_bias[2*i] = 		(temp & 0x00FF);
		roll_bias[2*i+1] = ((temp>>8) & 0x00FF);
	}
	for(i=0;i<512;i++){
		temp = MyFLASH_ReadHalfWord(STORE_ADDRESS3 + i*2);
		yaw_bias[2*i] = 	(temp & 0x00FF);
		yaw_bias[2*i+1] = ((temp>>8) & 0x00FF);
	}
	return 0;
}

/**
	* @brief		存储校准数据
  * @param		无
  * @return  	无
  */
void Store_Save(void){
	uint16_t i,temp;
	//若多数据分开存，需要修改此擦除的代码
	MyFLASH_ErasePage(STORE_ADDRESS1,3);
	for(i=0;i<512;i++){
		temp = pitch_bias[2*i] | (pitch_bias[2*i+1]<<8) ;
		MyFLASH_ProgramHalfWord(STORE_ADDRESS1 + 2*i,temp);
	}
	for(i=0;i<512;i++){
		temp = roll_bias[2*i] | (roll_bias[2*i+1]<<8) ;
		MyFLASH_ProgramHalfWord(STORE_ADDRESS2 + 2*i,temp);
	}
	for(i=0;i<512;i++){
		temp = yaw_bias[2*i] | (yaw_bias[2*i+1]<<8) ;
		MyFLASH_ProgramHalfWord(STORE_ADDRESS3 + 2*i,temp);
	}
}

