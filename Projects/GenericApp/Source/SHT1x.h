#ifndef __SHT1X_H__
#define __SHT1X_H__
#include <ioCC2530.h>
#include "math.h"    //Keil library  
#include "stdio.h"

#define uchar unsigned char
#define uint unsigned int
#define DATA P1_2         //定义通讯数据端口
#define SCK P1_1
#define  SHT_DATAOUT  P1DIR|=0x04 ;   //数据输出
#define SHT_DATAIN  P1DIR&=~0X04;  
typedef union  
{ unsigned int i;      //定义了两个共用体
  float f; 
} valuen; 

enum {TEMP,HUMI};      //TEMP=0,HUMI=1

//extern uint tempvalue,tempvalue2;
//extern uint flag;
//extern float wenduf,shiduf;
//extern unsigned int wendu,shidu;   
 
#define noACK 0             //用于判断是否结束通讯
#define ACK   1             //结束数据传输
                            //adr  command  r/w 
#define STATUS_REG_W 0x06   //000   0011    0 
#define STATUS_REG_R 0x07   //000   0011    1 
#define MEASURE_TEMP 0x03   //000   0001    1 
#define MEASURE_HUMI 0x05   //000   0010    1 
#define RESET        0x1e   //000   1111    0 
#define _nop_() asm("NOP"); asm("NOP");  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");  asm("NOP");
/****************定义函数****************/
void s_transstart(void);               //启动传输函数
void s_connectionreset(void);          //连接复位函数
char s_write_byte(unsigned char value);//DHT90写函数
char s_read_byte(unsigned char ack);   //DHT90读函数
char s_measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode);//测量温湿
void calc_dht90(float *p_humidity ,float *p_temperature);//温湿度补偿
void delay_n10us(uint n);                     //延时函数

#endif
