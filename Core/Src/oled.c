/********************************************************************************

* @File oled.c

* @Author: Ma Ziteng

* @Version: 2.0

* @Date: 2025-11

* @Description: 0.96 OLED显示驱动，参见产品手册编写

********************************************************************************/
#include "oled.h"

#include "i2c.h"
#include "oledfont.h"
#include "delay.h"

/**
 *  @note 提供以下接口
 *  I2C_Transmit
 *  OLED_ADDRESS
 *  TIMEOUT
 */
#define     I2C_Transmit(ADDRESS,Control,Data,Len,Timeout)    HAL_I2C_Mem_Write(&hi2c1,ADDRESS,Control,I2C_MEMADD_SIZE_8BIT,Data,Len,Timeout)
#define     OLED_ADDRESS    0x78
#define     TIMEOUT         1000

//命令树
uint8_t command[23]={
    0xAE,
    0xD5,0x80,
    0xA8,0x3F,
    0xD3,0x00,
    0x40,
    0xA1,
    0xC8,
    0xDA,0x12,
    0x81,0xCF,
    0xD9,0xF1,
    0xDB,0x30,
    0xA4,0xA6,
    0x8D,0x14,
    0xAF
};

//缓存数组,占用1kb空间。8行，每行一个字节宽；128列，每列一个位宽
uint8_t OLED_DisplayBuf[8][128];



/**
* @brief    写入命令
* @param    Command       命令字节
* @retval   1   ERROR
* @retval   2   BUSY
* @retval   3   TIMEOUT
* @retval   0   OK
*/
uint8_t OLED_WriteCommand(uint8_t Command) {
    //Control字节 Co->0 非连续，即一个控制字节+n个内容字节 D/C->0 表示后面的全为命令字节 故为0000 0000
    return I2C_Transmit(OLED_ADDRESS,0x00,&Command,1,TIMEOUT);
}

/**
* @brief    写入数据
* @param    Data       数据字节
* @retval   1   ERROR
* @retval   2   BUSY
* @retval   3   TIMEOUT
* @retval   0   OK
*/
uint8_t OLED_WriteData(uint8_t Data) {
    return I2C_Transmit(OLED_ADDRESS,0x40,&Data,1,TIMEOUT);
}
/**
* @brief    写入长数据
* @param    Data       数据字节
* @param    Len        数据长度
* @retval   1   ERROR
* @retval   2   BUSY
* @retval   3   TIMEOUT
* @retval   0   OK
*/
uint8_t OLED_WriteLenData(uint8_t* Data,uint8_t Len) {
    return I2C_Transmit(OLED_ADDRESS,0x40,Data,Len,TIMEOUT);
}

/**
* @brief    写入长命令
* @param    Command       命令字节
* @param    Len           命令长度
* @retval   1   ERROR
* @retval   2   BUSY
* @retval   3   TIMEOUT
* @retval   0   OK
*/
uint8_t OLED_WriteLenCommand(uint8_t* Command,uint8_t Len) {
    return I2C_Transmit(OLED_ADDRESS,0x00,Command,Len,TIMEOUT);
}

/**
* @brief    设置光标位置
* @param    Page    页位置
* @param    x       x坐标位置
* @return   无
*/
void OLED_SetCursor(uint8_t Page,uint8_t x) {
    uint8_t Command[]={0x00 | (x & 0x0F),0x10 | ((x & 0xF0)>>4),0xB0 | Page};
    OLED_WriteLenCommand(Command,3);
    // OLED_WriteCommand(0x00 | (x & 0x0F));//0x00-> 0000 0000 高四位首位为0表示写入x坐标低4位
    // OLED_WriteCommand(0x10 | ((x & 0xF0)>>4));//0x10-> 0001 0000 高四位首位为1表示写入x坐标高4位
    // OLED_WriteCommand(0xB0 | Page);      //0xB0-> 1011 0000 高四位表示写入页地址
}

/**
* @brief    将缓存刷新到显存中
* @param    无
* @return   无
*/
void OLED_UpdateScreen(uint8_t Begin,uint8_t End) {
    uint8_t i;
    for (i=Begin;i<End;i++) {
        OLED_SetCursor(i,0);
        OLED_WriteLenData(OLED_DisplayBuf[i],128);
    }
}

/**
* @brief    擦除框选区域
* @param    x   起始x位置（0，127）
* @param    y   起始y位置（0，127）
* @param    Width   框选宽度（像素）
* @param    Height   框选高度度（像素）
* @return   无
*/
void OLED_EraseArea(uint8_t x,uint8_t y,uint8_t Width,uint8_t Height) {
    uint8_t i,ImagePage,bias = y%8;
    for (i=0;i<Width;i++) {
        for (ImagePage=0;ImagePage<Height/8;ImagePage++) {
            OLED_DisplayBuf[y/8 + ImagePage][x+i] &= ~(0xFF<<bias);
            OLED_DisplayBuf[y/8 + ImagePage + 1][x+i] &= ~(0xFF>>(8-bias));
        }
    }
}

/**
* @brief    OLED清屏
* @param    无
* @return   无
*/
void OLED_Clear(void) {
    uint8_t i,j;
    for (i=0;i<8;i++)
        for (j=0;j<128;j++)
            OLED_DisplayBuf[i][j]=0;//写完数据后，光标自动移动到下个字节（右移）
    OLED_UpdateScreen(0,8);
}

/**
* @brief    OLED显示字符
* @param    x   起始x位置（0，127）
* @param    y   起始y位置（0，127）
* @param    Width   图像宽度（像素）
* @param    Height   图像高度度（像素）
* @param    Img     图像字模数组
* @return   无
*/
void OLED_ShowImage(uint8_t x,uint8_t y,uint8_t Width,uint8_t Height,const unsigned char *Img) {
    OLED_EraseArea(x,y,Width,Height);
    uint8_t i,ImagePage,bias = y%8;
    for (i=0;i<Width;i++) {
        for (ImagePage=0;ImagePage<Height/8;ImagePage++) {
            OLED_DisplayBuf[y/8 + ImagePage][x+i] |= Img[ImagePage*Width+i]<<bias;
            OLED_DisplayBuf[y/8 + ImagePage + 1][x+i] |= Img[ImagePage*Width+i]>>(8-bias);
        }
    }
}

/**
* @brief    OLED显示字符
* @param    x   屏幕上横坐标（0~15）
* @param    y   屏幕上纵坐标（0~3）
* @param    Char   要显示的字符
* @return   无
*/
void OLED_ShowChar(uint8_t x,uint8_t y,char Char ,uint8_t FontSize) {
    if (FontSize == 6)
        OLED_ShowImage(x,y,6,8,F6x8[Char - ' ']);
    else if (FontSize == 8)
        OLED_ShowImage(x,y,8,16,F8X16[Char - ' ']);

}

/**
* @brief    OLED显示字符串
* @param    x   屏幕上横坐标
* @param    y   屏幕上纵坐标
* @param    Str   要显示的字符串
* @return   无
*/
void OLED_ShowString(uint8_t x,uint8_t y,char* Str,uint8_t FontSize) {
    uint8_t i;
    for (i=0;Str[i];i++)
        OLED_ShowChar(x+i*FontSize,y,Str[i],FontSize);
}

/**
* @brief    OLED显示无符号数字
* @param    x   屏幕上横坐标（0~15）
* @param    y   屏幕上纵坐标（0~3）
* @param    Num   要显示的数字
* @param    Len   要显示的数字的长度
* @return   无
*/
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t Num,uint8_t Len,uint8_t FontSize) {
    uint8_t i,Char;
    x+=(Len*FontSize-1);
    for (i=0;i<Len;i++) {
        Char = Num % 10 + '0';
        Num /= 10;
        OLED_ShowChar(x-i*FontSize,y,Char,FontSize);
    }

}

/**
* @brief    OLED显示有符号数字
* @param    x   屏幕上横坐标（0~15）
* @param    y   屏幕上纵坐标（0~3）
* @param    Num   要显示的数字
* @param    Len   要显示的数字的长度
* @return   无
*/
void OLED_ShowSignNum(uint8_t x,uint8_t y,int32_t Num,uint8_t Len,uint8_t FontSize) {
    uint8_t i,Char;
    if (Num<0) {
        OLED_ShowChar(x,y,'-',FontSize);
        x+=FontSize,Len-=FontSize,Num=-Num;
    }
    x+=(Len*FontSize-1);
    for (i=0;i<Len;i++) {
        Char = Num % 10 + '0';
        Num /= 10;
        OLED_ShowChar(x-i*FontSize,y,Char,FontSize);
    }
}

/**
* @brief    OLED显示16进制数字
* @param    x   屏幕上横坐标（0~15）
* @param    y   屏幕上纵坐标（0~3）
* @param    Num   要显示的数字
* @param    Len   要显示的数字的长度
* @return   无
*/
void OLED_ShowHexNum(uint8_t x,uint8_t y,uint32_t Num,uint8_t Len,uint8_t FontSize) {
    uint8_t i,Char;
    x+=(Len*FontSize-1);
    for (i=0;i<Len;i++) {
        Char = Num % 16 ;
        Num /= 16;
        if (Char<=9)
            OLED_ShowChar(x-i*FontSize,y,Char+'0',FontSize);
        else
            OLED_ShowChar(x-i*FontSize,y,Char-10+'A',FontSize);
    }

}

/**
* @brief    OLED初始化，发送命令
* @param    无
* @return   无
*/
void OLED_Init(void) {
    // Delay_ms(100);//建议加上100ms延时,如果前面有长时操作，此行可以注释掉
    OLED_WriteLenCommand(command,23);
    // Delay_ms(100);//建议加上100ms延时,如果后面有长时操作，此行可以注释掉
    OLED_SetCursor(0,0);
    OLED_Clear();
}