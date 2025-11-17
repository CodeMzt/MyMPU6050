/********************************************************************************

* @File mpu6050.c

* @Author: Ma Ziteng

* @Version: 2.0

* @Date: 2025-11

* @Description: mpu6050姿态解算

********************************************************************************/



#include "mpu6050.h"
#include "main.h"
#include "MyI2C.h"
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "delay.h"

#define I2C_Init							MyI2C_Init
#define I2C_Start 						MyI2C_Start
#define I2C_Stop 							MyI2C_Stop
#define I2C_SendByte					MyI2C_SendByte
#define I2C_ReceiveAck				MyI2C_ReceiveAck
#define I2C_ReceiveByte(Ack)  MyI2C_ReceiveByte(Ack)
#define delay_ms							Delay_ms

/**
  * @brief		获取毫秒级时间戳
  * @param		储存时间戳的指针
  * @return  	无
  */
void myget_ms(unsigned long *time)
{
	uint32_t RTC_Counts;
	RTC_Counts=RTC->DIVH;
	RTC_Counts=RTC_Counts<<16;
	RTC_Counts+=RTC->DIVL;
	*time = RTC_Counts;
}

/**
  * @brief		mpu6050指定地址写一个
  * @param		slave_addr	从机地址
	*	@param		reg_addr		寄存器地址
	*	@param		data				数据
  * @return  	成功
  */
unsigned short mpu_write(unsigned char slave_addr, unsigned char reg_addr,
	unsigned char const data){
		int i;
		I2C_Start();
		I2C_SendByte(slave_addr<<1|0);
		I2C_ReceiveAck();
		I2C_SendByte(reg_addr);
		I2C_ReceiveAck();
		I2C_SendByte(data);
		I2C_ReceiveAck();
		I2C_Stop();
		return 0;
}

/**
  * @brief		mpu6050指定地址写
  * @param		slave_addr	从机地址
	*	@param		reg_addr		寄存器地址
	*	@param		length			写入数据长度
	*	@param		*data				数据指针
  * @return  	成功
  */
unsigned short mpu_lenwrite(unsigned char slave_addr, unsigned char reg_addr,
	unsigned char length, unsigned char const *data){
		int i;
		I2C_Start();
		I2C_SendByte(slave_addr<<1|0);
		I2C_ReceiveAck();
		I2C_SendByte(reg_addr);
		I2C_ReceiveAck();
		for(i=0;i<length;i++){
			I2C_SendByte(data[i]);
			I2C_ReceiveAck();
		}
		I2C_Stop();
		return 0;
}
	
/**
  * @brief		mpu6050指定地址读一个
  * @param		slave_addr	从机地址
	*	@param		reg_addr		寄存器地址
	*	@param		*data				数据指针
  * @return  	成功
  */
unsigned short mpu_read(unsigned char slave_addr, unsigned char reg_addr,
	unsigned char *data){
		int i;
		I2C_Start();
		I2C_SendByte(slave_addr<<1 | 0);
		I2C_ReceiveAck();
		I2C_SendByte(reg_addr);
		I2C_ReceiveAck();
		I2C_Start();
		I2C_SendByte(slave_addr<<1 | 1);
		I2C_ReceiveAck();
		*data=I2C_ReceiveByte(1);
		I2C_Stop();
		return 0;
}

/**
  * @brief		mpu6050指定地址读
  * @param		slave_addr	从机地址
	*	@param		reg_addr		寄存器地址
	*	@param		length			读取数据长度
	*	@param		*data				数据指针
  * @return  	成功
  */
unsigned short mpu_lenread(unsigned char slave_addr, unsigned char reg_addr,
	unsigned char length, unsigned char *data){
		int i;
		I2C_Start();
		I2C_SendByte(slave_addr<<1 | 0);
		I2C_ReceiveAck();
		I2C_SendByte(reg_addr);
		I2C_ReceiveAck();
		I2C_Start();
		I2C_SendByte(slave_addr<<1 | 1);
		I2C_ReceiveAck();
		for(i=0;i<length;i++)
			data[i]=I2C_ReceiveByte((i==length-1)?1:0);
		I2C_Stop();
		return 0;
}
	
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    return mpu_write(MPU_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=mpu_write(MPU_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
    return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}
		
/**
  * @brief		mpu6050初始化
  * @param		无
  * @return  	0 成功；否则失败
  */
uint8_t mpu6050_Init(void){
	uint8_t res;
	I2C_Init();
	mpu_write(MPU_ADDR,MPU_PWR_MGMT1_REG,0X80);//复位MPU6050
	mpu_write(MPU_ADDR,MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050
	mpu_write(MPU_ADDR,MPU_GYRO_CFG_REG,0x18);//设置陀螺仪满量程范围
	mpu_write(MPU_ADDR,MPU_ACCEL_CFG_REG,0x18);//设置加速度传感器满量程范围
	MPU_Set_Rate(DEFAULT_MPU_HZ);						//设置采样率50Hz
  mpu_write(MPU_ADDR,MPU_INT_EN_REG,0X00);	//关闭所有中断
  mpu_write(MPU_ADDR,MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
  mpu_write(MPU_ADDR,MPU_FIFO_EN_REG,0X00);	//关闭FIFO
  mpu_write(MPU_ADDR,MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
  mpu_read(MPU_ADDR,MPU_DEVICE_ID_REG,&res);
	if(res == MPU_ADDR + 8)//器件ID正确	不知道为什么，返回70
    {
        mpu_write(MPU_ADDR,MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
        mpu_write(MPU_ADDR,MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
        MPU_Set_Rate(200);						//设置采样率
    } else return 1;
  return 0;
}




//q30格式,long转float时的除数.
#define q30  1073741824.0f

/**
	*	陀螺仪方向设置
	*	表示陀螺仪物理安装方向与DMP内部坐标系的映射关系（旋转矩阵）
	*	根据安装情况修改此矩阵
	*/
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1
                                         };

																				 
																				 
/**
  * @brief		方向向量转为标量
  * @param		row	三维行向量 (x,y,z)
  * @return  	对应标量值
  */
unsigned short inv_row_2_scale(const signed char *row){
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // Vector Zero
    return b;
}

/**
	* @brief		陀螺仪方向矩阵转换为标量，按三位输出
  * @param		mtx	三维矩阵 
  * @return  	对应标量值
  */
unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx){
    unsigned short scalar;

    scalar = inv_row_2_scale(mtx);
			//mtx + 3指针后移；得到的数据左移3位（*8）最终 000 000 000
			//注:short 是16位的
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

/**
	* @brief		原点自检
  * @param		无
  * @return  	0：正常
  */	
uint8_t run_self_test(void)
{
    int result;
    //char test_packet[4] = {0};
    long gyro[3], accel[3];
    result = mpu_run_self_test(gyro, accel);
    if (result == 0x3){
        /* Test passed. We can trust the gyro data here, so let's push it down
        * to the DMP.
        */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
        return 0;
    } else return 1;
}

/**
  * @brief		mpu_dmp初始化
  * @param		无
  * @return  	0 成功；否则失败
  */
uint8_t mpu_dmp_init(void){
	uint8_t result;	
	
	//0.初始化I2C总线，一般这个过程由main()来实现
	
	//1.6050初始化
	result = mpu_init();
	//2.初始化mpu6050失败
	if(result){
		Error_Handler();
		return 10;
	}
	//3.配置陀螺仪和加速度传感器
	if(mpu_set_sensors(INV_XYZ_ACCEL | INV_XYZ_GYRO))	return 1;
	//4.配置FIFO(先入先出)，保障数据正常转运
	if(mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO))	return 2;
	//5.配置采样率
	if(mpu_set_sample_rate(DEFAULT_MPU_HZ))	return 3;
	//6.加载并验证dmp映像
	if(dmp_load_motion_driver_firmware())	return 4;
	//7.设置陀螺仪方向矩阵
	if(dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))	return 5;
	//8.设置dmp功能
	/**
		DMP可用特性
		使用内置DMP获取GYRO和ACC时, DMP可用特性(dmp_enable_feature)有:
		DMP_FEATURE_LP_QUAT:
			Generate a gyro-only quaternion on the DMP at 200Hz. 
			Integrating the gyro data at higher rates reduces numerical errors 
			(compared to integration on the MCU at a lower sampling rate).
		DMP_FEATURE_6X_LP_QUAT:
			Generate 6-axis quaternions from the DMP.
		DMP_FEATURE_TAP:
			Detect taps along the X, Y, and Z axes.
		DMP_FEATURE_ANDROID_ORIENT:
			Google's screen rotation algorithm. Triggers an event at the four orientations where the screen should rotate.
		DMP_FEATURE_PEDOMETER
			Pedometer counts steps and the amount of time those steps were taken in.
		DMP_FEATURE_SEND_RAW_ACCEL :
			Add raw accelerometer data to the FIFO.
		DMP_FEATURE_SEND_RAW_GYRO:
			Add raw gyro data to the FIFO.
		DMP_FEATURE_SEND_CAL_GYRO:
			Add calibrated gyro data to the FIFO. It cannot be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
		DMP_FEATURE_GYRO_CAL:
			Calibrates the gyro data after eight seconds of no motion.
	*/
	if(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |DMP_FEATURE_ANDROID_ORIENT |
												DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL))	return 6;
	//9.设置dmp输出（FIFO）速率
	if(dmp_set_fifo_rate(DEFAULT_MPU_HZ))	return 7;
	//11.开启dmp
	if(mpu_set_dmp_state(1))	return 9;
	//10.原点校准
	if(run_self_test())
		return 8
			;
	return result;
}

/**
  * @brief		获取角速度数据
  * @param		gx		
  * @param		gy	
  * @param		gz			
  * @return  	0 成功；否则失败
  */
uint8_t mpu_get_gyroscope(short *gx,short *gy,short *gz){
    uint8_t buf[6],res;
    res=mpu_lenwrite(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
    if(res==0){
        *gx=((uint16_t)buf[0]<<8)|buf[1];
        *gy=((uint16_t)buf[2]<<8)|buf[3];
        *gz=((uint16_t)buf[4]<<8)|buf[5];
    }
    return res;
}

/**
  * @brief		获取加速度数据
  * @param		ax		
  * @param		ay	
  * @param		az			
  * @return  	0 成功；否则失败
  */
uint8_t mpu_get_accelerometer(short *ax,short *ay,short *az){
    uint8_t buf[6],res;
    res=mpu_lenwrite(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
    if(res==0){
        *ax=((uint16_t)buf[0]<<8)|buf[1];
        *ay=((uint16_t)buf[2]<<8)|buf[3];
        *az=((uint16_t)buf[4]<<8)|buf[5];
    }
    return res;
}

/**
  * @brief		获取四元数，解算姿态
  * @param		pitch		俯仰角 	精度:0.1°   范围:-90.0° <---> +90.0°
  * @param		roll		横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
  * @param		yaw			航向角  精度:0.1°   范围:-180.0°<---> +180.0°
  * @return  	0 成功；否则失败
  */
uint8_t mpu_dmp_get_data(float *pitch,float *roll,float *yaw){
    float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))return 1;
    /* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
     * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
    **/
    /*if (sensors & INV_XYZ_GYRO )
    send_packet(PACKET_TYPE_GYRO, gyro);
    if (sensors & INV_XYZ_ACCEL)
    send_packet(PACKET_TYPE_ACCEL, accel); */
    /* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
     * The orientation is set by the scalar passed to dmp_set_orientation during initialization.
    **/
    if(sensors&INV_WXYZ_QUAT){
        q0 = quat[0] / q30;	//q30格式转换为浮点数
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;
        //计算得到俯仰角/横滚角/航向角
        *pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
        *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
        *yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
    } else return 2;
    return 0;
}
