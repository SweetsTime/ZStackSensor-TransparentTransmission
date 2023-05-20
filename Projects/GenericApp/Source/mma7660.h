
#include <ioCC2530.h>
//#include  "sysint.h"
#define DATAOUT P1DIR|=0x18       //Êý¾ÝÊä³ö
#define DATAIN P1SEL&=~0x10;P1DIR&=~0X10
#define I2C_SDA    P1_4
#define I2C_SCL   P1_3

#define Delay20us 	34              //Loop times to delay 20uS (Need to change according to system clock)
#define Delay10us 	14              //Loop times to delay 45uS (Need to change according to system clock)
#define Delay5us	5              //Loop times to delay 21uS (Need to change according to system clock)
#define Delay2p5us	2              //Loop times to delay 2.5uS (Need to change according to system clock)

#define FSL_MMA_AddW 0x98          //MMA7660 Address for Writing
#define FSL_MMA_AddR 0x99          //MMA7660 Address for Reading
#define FSL_MMA_StartAddress 0x00  //MMA7660 g-value registers' start address

#define MMA7660_XOUT    0x00
#define MMA7660_YOUT    0x01
#define MMA7660_ZOUT    0x02
#define MMA7660_TILT    0x03
#define MMA7660_SRST    0x04
#define MMA7660_SPCNT   0x05
#define MMA7660_INTSU   0x06
#define MMA7660_MODE    0x07
#define MMA7660_SR      0x08
#define MMA7660_PDET    0x09
#define MMA7660_PD      0x0A
#define _NOP() asm("NOP");asm("NOP");asm("NOP");

#define RawDataLength   6

 void IIC_Start(void);
 void IIC_Stop (void);
 void IIC_SendByte(int sData);
 int  IIC_ChkAck(void); 

 void FSL_MMA_IICWrite(int RegAdd, int Data);
 void MMA7660_Init(void);
 void MMA7660_Startup(void);
 int FSL_MMA_IICRead(int RegAdd);
 void IIC_Read_MMA7660_XYZ6(int *pX, int *pY, int *pZ);
 void IIC_RepeatedStart(void);
 int mma7660_IICRead_Alert(int RegAdd);
 void mma_delay(int time) ;
 void MMA7660_XYZ_Read_and_Filter(void);
