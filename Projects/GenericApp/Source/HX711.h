#ifndef __HX711_H__
#define __HX711_H__
#include <ioCC2530.h>
#include "math.h"    //Keil library  
#include "stdio.h"
#define uchar unsigned char
#define uint unsigned int

#define DATA P1_2         //定义通讯数据端口
#define CLR_ADSK() P1_1=0;
#define SET_ADSK() P1_1=1;
unsigned long ReadCount(char);
void AD_filter(void);
void Tozero(void);
extern long AD_Compar;
#endif