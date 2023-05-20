#ifndef __18B20_H__
#define __18B20_H__
#include <ioCC2530.h>
#include "math.h"    //Keil library  
#include "stdio.h"


/************************************************************
* 宏定义													
************************************************************/
#define CelsiurPerLSB   0.0625 //DS18B20的16位二进制温度,每LSB代表的摄氏度
#define DQ_RD    P1DIR&=(~0X04)  
#define DQ_WR    P1DIR|=0x04 
#define SET_DQ0  P1_2=0
#define SET_DQ1  P1_2=1
#define GET_DQ   P1_2
#define delay_1us()  {asm("nop"); asm("nop");asm("nop"); asm("nop");asm("nop"); asm("nop");asm("nop"); asm("nop");asm("nop"); asm("nop");asm("nop"); asm("nop");asm("nop"); asm("nop");asm("nop"); asm("nop");asm("nop"); asm("nop");}

unsigned int ReadTemperature(void);
void Delay(unsigned int i);
void Init_DS18B20(void);
void WriteOneChar(unsigned char dat);
unsigned char ReadOneChar(void);
#endif
