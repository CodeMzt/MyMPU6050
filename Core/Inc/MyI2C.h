#ifndef __MYI2C_H__
#define __MYI2C_H__

#include "main.h"

void MyI2C_Init(void);;
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_ReceiveByte(uint8_t AckBit);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);

#endif
