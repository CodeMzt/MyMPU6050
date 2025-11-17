/********************************************************************************

* @File MyI2C.c

* @Author: Ma Ziteng

* @Version: 1.0

* @Date: 2025-10

* @Description: I2C软件模拟

********************************************************************************/
#include "MyI2C.h"
#include "delay.h"



/*引脚配置层*/
 


/**
  * @brief		I2C写SCL引脚电平
  * @param		PinState 协议层传入的当前需要写入SCL的电平
  * @return  	无
  * @note：		此函数需要用户实现内容，当PinState为0时，需要置SCL为低电平，当PinState为1时，需要置SCL为高电平
  */
void MyI2C_W_SCL(uint8_t PinState){
	HAL_GPIO_WritePin(SCL_GPIO_Port,SCL_Pin,(GPIO_PinState)PinState);
	Delay_us(3);	//延时10us，防止时序频率超过要求
}

/**
  * @brief		I2C写SDA引脚电平
  * @param		BitValue 协议层传入的当前需要写入SDA的电平，范围0~1
  * @return		无
  * @note			此函数需要用户实现内容，当BitValue为0时，需要置SDA为低电平，当BitValue为1时，需要置SDA为高电平
  */
void MyI2C_W_SDA(uint8_t PinState){
	HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin,(GPIO_PinState)PinState);
	Delay_us(3);
}

/**
  * @brief		I2C读SDA引脚电平
  * @param		无
  * @return		协议层需要得到的当前SDA的电平，范围0~1
  * @note			此函数需要用户实现内容，当前SDA为低电平时，返回0，当前SDA为高电平时，返回1
  */
uint8_t MyI2C_R_SDA(void)
{
	GPIO_PinState PinState;
	PinState = HAL_GPIO_ReadPin(SDA_GPIO_Port,SDA_Pin);		//读取SDA电平
	Delay_us(3);												//延时10us，防止时序频率超过要求
	return (uint8_t)PinState;											//返回SDA电平
}

/**
	*	GPIO引脚初始化
	*	
	*
	*/

/*协议层*/

/**
  * @brief		I2C起始
  * @param		无
  * @return		无
  */
void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);							//释放SDA，确保SDA为高电平
	MyI2C_W_SCL(1);							//释放SCL，确保SCL为高电平
	MyI2C_W_SDA(0);							//在SCL高电平期间，拉低SDA，产生起始信号
	MyI2C_W_SCL(0);							//起始后把SCL也拉低，即为了占用总线，也为了方便总线时序的拼接
}

/**
  * @brief		I2C终止
  * @param		无
  * @return		无
  */
void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);							//拉低SDA，确保SDA为低电平
	MyI2C_W_SCL(1);							//释放SCL，使SCL呈现高电平
	MyI2C_W_SDA(1);							//在SCL高电平期间，释放SDA，产生终止信号
}

/**
  * @brief		I2C发送一个字节
  * @param		Byte 要发送的一个字节数据，范围：0x00~0xFF
  * @return		无
  */
void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)				//循环8次，主机依次发送数据的每一位
	{
		/*两个!可以对数据进行两次逻辑取反，作用是把非0值统一转换为1，即：!!(0) = 0，!!(非0) = 1*/
		MyI2C_W_SDA(!!(Byte & (0x80 >> i)));//使用掩码的方式取出Byte的指定一位数据并写入到SDA线
		MyI2C_W_SCL(1);						//释放SCL，从机在SCL高电平期间读取SDA
		MyI2C_W_SCL(0);						//拉低SCL，主机开始发送下一位数据
	}
	MyI2C_W_SDA(1);
}

/**
  * @brief		I2C接收一个字节
  * @param		无
  * @return		接收到的一个字节数据，范围：0x00~0xFF
  */
uint8_t MyI2C_ReceiveByte(uint8_t AckBit)
{
	uint8_t i, Byte = 0x00;					//定义接收的数据，并赋初值0x00，此处必须赋初值0x00，后面会用到
	MyI2C_W_SDA(1);							//接收前，主机先确保释放SDA，避免干扰从机的数据发送
	for (i = 0; i < 8; i ++)				//循环8次，主机依次接收数据的每一位
	{
		MyI2C_W_SCL(1);						//释放SCL，主机机在SCL高电平期间读取SDA
		if (MyI2C_R_SDA()){Byte |= (0x80 >> i);}	//读取SDA数据，并存储到Byte变量
													//当SDA为1时，置变量指定位为1，当SDA为0时，不做处理，指定位为默认的初值0
		MyI2C_W_SCL(0);						//拉低SCL，从机在SCL低电平期间写入SDA
	}
	MyI2C_SendAck(AckBit);
	return Byte;							//返回接收到的一个字节数据
}

/**
  * @brief		I2C发送应答位
  * @param		AckBit 要发送的应答位，范围：0~1，0表示应答，1表示非应答
  * @return		无
  */
void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SCL(0);							
	MyI2C_W_SDA((GPIO_PinState)AckBit);					//主机把应答位数据放到SDA线
	MyI2C_W_SCL(1);							//释放SCL，从机在SCL高电平期间，读取应答位
	MyI2C_W_SCL(0);							//拉低SCL，开始下一个时序模块
	MyI2C_W_SDA(1);					
}
	
/**
  * @brief		I2C接收应答位
  * @param		无
  * @return		接收到的应答位，范围：0~1，0表示应答，1表示非应答
  */
uint8_t MyI2C_ReceiveAck(void){
	uint8_t AckBit;							//定义应答位变量
	MyI2C_W_SDA(1);							//接收前，主机先确保释放SDA，避免干扰从机的数据发送
	MyI2C_W_SCL(1);							//释放SCL，主机机在SCL高电平期间读取SDA
	AckBit = MyI2C_R_SDA();					//将应答位存储到变量里
	MyI2C_W_SCL(0);							//拉低SCL，开始下一个时序模块
	return AckBit;							//返回定义应答位变量
}



/**
  * @brief		初始化
  * @param		无
  * @return  	无
  * @note：		兼容
  */
void MyI2C_Init(void){
		MyI2C_Stop();
}
