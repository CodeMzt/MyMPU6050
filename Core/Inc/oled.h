//
// Created by Mozart on 2025/11/8.
//

#ifndef OLED_H
#define OLED_H
#include <stdint.h>

void OLED_Init(void) ;
void OLED_ShowChar(uint8_t x,uint8_t y,char Char ,uint8_t FontSize) ;
void OLED_Clear(void);
void OLED_ShowImage(uint8_t x,uint8_t y,uint8_t Width,uint8_t Height,const unsigned char *Img) ;
void OLED_ShowString(uint8_t x,uint8_t y,char* Str,uint8_t FontSize) ;
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t Num,uint8_t Len,uint8_t FontSize);
void OLED_ShowSignNum(uint8_t x,uint8_t y,int32_t Num,uint8_t Len,uint8_t FontSize);
void OLED_ShowHexNum(uint8_t x,uint8_t y,uint32_t Num,uint8_t Len,uint8_t FontSize);
void OLED_UpdateScreen(uint8_t Begin,uint8_t End);
void OLED_EraseArea(uint8_t x,uint8_t y,uint8_t Width,uint8_t Height) ;

#endif //OLED_H