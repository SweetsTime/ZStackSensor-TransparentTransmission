
#ifndef _IIC_H
#define _IIC_H

/*******************************************************************************
 * INCLUDES
 */
#include <ioCC2530.h>
#include "GenericApp.h"

/*******************************************************************************
 * CONSTANTS AND MACROS
 */
//端口连接定义
#define SCL P1_3         //I2C总线的时钟
#define SDA P1_4         //I2C总线的数据
#define IIC_DATAOUT; P1DIR|=0x18       //数据输出
#define IIC_DATAIN P1SEL&=~0x10;P1DIR&=~0X10
#if(SENSOR_TYPE =='C')
//地址定义
#define AddWr 0xa0  /*器件地址选择及写标志*/
#define AddRd 0xa1  /*器件地址选择及读标志*/
#endif
#if(SENSOR_TYPE =='F')
//地址定义
#define AddWr 0xEE  /*器件地址选择及写标志*/
#define AddRd 0xEF  /*器件地址选择及读标志*/
#endif
//地址定义
#if !defined( AddWr )
#define AddWr 0xa0  /*器件地址选择及写标志*/
#endif
#if !defined( AddRd )
#define AddRd 0xa1  /*器件地址选择及读标志*/
#endif

//命令字定义
#define NOPS  asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
extern void mDelay(unsigned j);
extern void Single_Write_(unsigned char,unsigned char );
extern unsigned char Single_Read_(unsigned char);
extern void WrToROM(unsigned char *,unsigned char ,unsigned char);
extern void RdFromROM(unsigned char *,unsigned char ,unsigned char);


/*****************************************************************************/
#endif

