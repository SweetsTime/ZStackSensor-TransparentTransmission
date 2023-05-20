/*******************************************************************************
 * 文件名称：LCD.h
 * 功    能：数码管驱动
 *           
 * 硬件连接：数码管与CC2530的硬件连接关系如下：
 *             
 *
 *               SPILED-data 串行数据             P1.6
 *               SPILED-CLK 移位寄存器时钟输入   P1.5
 *               GPH      存储寄存器时钟输入   P1.3
 * 作    者：w
 * 公    司：无锡泛太科技有限公司
 ******************************************************************************/


#ifndef LCD_H
#define LCD_H 
/* 相关引脚定义 */
/*===================================================*/

#if(SENSOR_TYPE ==0X6E)
#define      SPILEDdata        P1_7
#define      SPILEDCLK         P1_5=P1_6
#define      GPH               P1_3=P2_0
#else 
#define      SPILEDdata        P1_6
#define      SPILEDCLK         P1_5
#define      GPH               P1_3
#define      DS1               P1_2
#define      DS2               P1_1
#endif 
/*===================================================*/
/* 相关引脚输出电平定义 */
/*===================================================*/
#define      H_SPILEDdata()       SPILEDdata = 1
#define      L_SPILEDdata()       SPILEDdata = 0
#define      H_SPILEDCLK()       SPILEDCLK = 1 
#define      L_SPILEDCLK()       SPILEDCLK = 0
#define      H_GPH()             GPH = 1
#define      L_GPH()             GPH = 0

/*===================================================*/

#define      NOP()              asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
extern void LCD_SPILEDSendData(unsigned char  Data);
extern void LCD_SPILEDSendDatal(unsigned char Data);
extern void LCD_SPILEDOUT(void);
extern void SendData(unsigned char Data);
extern void SendDataS(unsigned char * Data,unsigned char len);
#endif
 