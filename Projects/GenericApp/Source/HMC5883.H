#include "ioCC2530.h"
#include  <math.h>    //Keil library  
#include  <stdio.h>   //Keil library	
#define   uchar unsigned char
#define   uint unsigned int	
//使用的端口，请按照以下接线
//P1_2;
#define	HMC5883_SCL P1_3      //IIC时钟引脚定义
#define	HMC5883_SDA P1_4      //IIC数据引脚定义
#define HMC5883_DATAOUT P1DIR|=0x10   //数据输出
#define HMC5883_DATAIN  P1DIR&=(~0x10)   //数据输入 上拉

#define	SlaveAddress   0x3C	  //定义器件在IIC总线中的从地址
typedef unsigned char BYTE;
BYTE BUF[8];                         //接收数据缓存区      	
void Init_HMC5883(void);            //初始化5883
void  Single_Write_HMC5883(uchar REG_Address,uchar REG_data);   //单个写入数据
uchar Single_Read_HMC5883(uchar REG_Address);                   //单个读取内部寄存器数据
void  Multiple_Read_HMC5883(void);                                  //连续的读取内部寄存器数据
//以下是模拟iic使用函数-------------
void HMC5883_Start(void);
void HMC5883_Stop(void);
void HMC5883_SendACK(uchar ack);
uchar HMC5883_RecvACK(void);
void HMC5883_SendByte(BYTE dat);
BYTE HMC5883_RecvByte(void);
void HMC5883_ReadPage(void);
void HMC5883_WritePage(void);
void HMC5883_DelayUs(uint16 usec);
void HMC5883_DelayMs(uint16 msec);
//-----------------------------------
void HMC5883_DelayUs(uint16 usec)
{ usec>>= 1;
    while(usec--)
    {
        asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
        asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    }
}
void HMC5883_DelayMs(uint16 msec)
{ while(msec--)
        HMC5883_DelayUs(1000);
  
}
/**************************************
起始信号
**************************************/
void HMC5883_Start()
{  
    HMC5883_SDA = 1;                    //拉高数据线
    HMC5883_SCL = 1;                    //拉高时钟线
    HMC5883_DelayUs(5);                 //延时
    HMC5883_SDA = 0;                    //产生下降沿
     HMC5883_DelayUs(5);                 //延时
    HMC5883_SCL = 0;                    //拉低时钟线
}

/**************************************
停止信号
**************************************/
void HMC5883_Stop()
{   
    HMC5883_SDA = 0;                    //拉低数据线
    HMC5883_SCL = 1;                    //拉高时钟线
    HMC5883_DelayUs(5);                  //延时
    HMC5883_SDA = 1;                    //产生上升沿
     HMC5883_DelayUs(5);                  //延时
}

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
void HMC5883_SendACK(uchar ack)
{   
    HMC5883_SDA = ack;                  //写应答信号
    HMC5883_SCL = 1;                    //拉高时钟线
     HMC5883_DelayUs(5);                  //延时
    HMC5883_SCL = 0;                    //拉低时钟线
    HMC5883_DelayUs(5);                  //延时
}

/**************************************
接收应答信号
**************************************/
uchar HMC5883_RecvACK(void)
{  uint8 HMC5883_CY = 0x00;
  HMC5883_DATAIN;
    HMC5883_SCL = 1;                    //拉高时钟线
     HMC5883_DelayUs(5);                  //延时
    HMC5883_CY = HMC5883_SDA;                   //读应答信号
    HMC5883_SCL = 0;                    //拉低时钟线
     HMC5883_DelayUs(5);                 //延时
     HMC5883_DATAOUT;
    return HMC5883_CY;
}

/**************************************
向IIC总线发送一个字节数据
**************************************/
void HMC5883_SendByte(BYTE dat)
{
    BYTE i;

    for (i=0; i<8; i++)         //8位计数器
    { 
      if (dat&0x80)HMC5883_SDA=1;
       else HMC5883_SDA=0 ;                   //送数据口
      
                   //移出数据的最高位
        HMC5883_SCL = 1;                //拉高时钟线
        HMC5883_DelayUs(5);              //延时
        HMC5883_SCL = 0;                //拉低时钟线
         HMC5883_DelayUs(5);              //延时
         dat <<= 1;   
    }
    HMC5883_RecvACK();

}

/**************************************
从IIC总线接收一个字节数据
**************************************/
BYTE HMC5883_RecvByte(void)
{
    BYTE i;
    BYTE dat = 0;

    HMC5883_SDA = 1;                    //使能内部上拉,准备读取数据,
     HMC5883_DelayUs(5);            //延时
     HMC5883_DATAIN;
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        HMC5883_SCL = 1;                //拉高时钟线
         HMC5883_DelayUs(5);            //延时
        dat |= HMC5883_SDA;             //读数据               
        HMC5883_SCL = 0;                //拉低时钟线
         HMC5883_DelayUs(5);             //延时
    }
    HMC5883_DATAOUT;
    return dat;
}

//***************************************************

void Single_Write_HMC5883(uchar REG_Address,uchar REG_data)
{
    HMC5883_Start();                  //起始信号
    HMC5883_SendByte(SlaveAddress);   //发送设备地址+写信号
    HMC5883_SendByte(REG_Address);    //内部寄存器地址，请参考中文pdf 
    HMC5883_SendByte(REG_data);       //内部寄存器数据，请参考中文pdf
    HMC5883_Stop();                   //发送停止信号
}

//********单字节读取内部寄存器*************************
uchar Single_Read_HMC5883(uchar REG_Address)
{  uchar REG_data;
    HMC5883_Start();                          //起始信号
    HMC5883_SendByte(SlaveAddress);           //发送设备地址+写信号
    HMC5883_SendByte(REG_Address);                   //发送存储单元地址，从0开始	
    HMC5883_Start();                          //起始信号
    HMC5883_SendByte(SlaveAddress+1);         //发送设备地址+读信号
    REG_data=HMC5883_RecvByte();              //读出寄存器数据
    HMC5883_SendACK(1);   
    HMC5883_Stop();                           //停止信号
    return REG_data; 
}
//******************************************************
//
//连续读出HMC5883内部角度数据，地址范围0x3~0x5
//
//******************************************************
void Multiple_Read_HMC5883(void)
{   uchar i;
    HMC5883_Start();                          //起始信号
    HMC5883_SendByte(SlaveAddress);           //发送设备地址+写信号
    HMC5883_SendByte(0x03);                   //发送存储单元地址，从0x32开始	
    HMC5883_Start();                          //起始信号
    HMC5883_SendByte(SlaveAddress+1);         //发送设备地址+读信号
    for (i=0; i<6; i++)                      //连续读取6个地址数据，存储中BUF
    {
        BUF[i] = HMC5883_RecvByte();          //BUF[0]存储0x32地址中的数据
        if (i == 5)
        {
           HMC5883_SendACK(1);                //最后一个数据需要回NOACK
        }
        else
        {
          HMC5883_SendACK(0);                //回应ACK
       }
   }
    HMC5883_Stop();                          //停止信号
    HMC5883_DelayMs(5); 
}

//初始化HMC5883，根据需要请参考pdf进行修改****
void Init_HMC5883(void)
{ HMC5883_DATAOUT;
     Single_Write_HMC5883(0x02,0x00);  //
}