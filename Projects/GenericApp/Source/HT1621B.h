#include <ioCC2530.h>
#include <string.h>
#define uint unsigned int
#define uchar unsigned char
#define _Nop()  DelayUS(5);
#define BIAS 0x52 //0b1000 0101 0010 1/3duty 4com
#define SYSDIS 0X00 //0b1000 0000 0000 关振系统荡器和LCD偏压发生器
#define SYSEN 0X02 //0b1000 0000 0010 打开系统振荡器
#define LCDOFF 0X04 //0b100 0000 0100 0关LCD偏压
#define LCDON 0X06 //0b1000 0000 0110 打开LCD偏压
#define XTAL 0x28 //0b1000 0010 1000 外部接时钟
#define RC256 0X30 //0b1000 0011 0000 内部时钟
#define TONEON 0X12 //0b1000 0001 0010 打开声音输出
#define TONEOFF 0X10 //0b1000 0001 0000 关闭声音输出
#define WDTDIS 0X0A //0b1000 0000 1010 禁止看门狗

//HT1621控制位（液晶模块接口定义，根据自已的需要更改）
#define HT1621_DAT P1_5 //HT1621数据引脚

//#define HT1621_CS P1_7 //HT1621使能引脚 旧版
//#define HT1621_WR P1_6 //HT1621时钟引脚 旧版
#define HT1621_CS P1_3 //HT1621使能引脚
#define HT1621_WR P1_4 //HT1621时钟引脚
//uchar  Ht1621Tab[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//uchar  table1[]={0x0e,0x06,0x0c,0x0e,0x06,0x0a,0x0a,0x0e,0x0e,0x0e};
//uchar  table2[]={0x0b,0x00,0x07,0x05,0x0c,0x0d,0x0f,0x00,0x0f,0x0d};

//函数声明
void DelayMS(uint iMs);
void Ht1621_Init(void);
void Ht1621Wr_Data(uchar Data,uchar cnt);
void Ht1621WrCmd(uchar Cmd);
void Ht1621WrOneData(uchar Addr,uchar Data);
void Ht1621WrAllData(uchar Addr,uchar *p,uchar cnt);


void DelayUS(uchar);
//delay us
void DelayUS(uchar us) //5,7,9
{
 while(--us);
}

//delay ms
void DelayMS(uint iMs)
{
 uint i,j;
 for(i=0;i<iMs;i++)
 for(j=0;j<65;j++) DelayUS(1);
}

/******************************************************
写数据函数,cnt为传送数据位数,数据传送为低位在前
*******************************************************/
void Ht1621Wr_Data(uchar Data,uchar cnt)
{
 uchar i;
 for (i=0;i<cnt;i++)
      {
       HT1621_WR=0;
       _Nop();
       HT1621_DAT=((Data&0x80)>>7);
       _Nop();
       HT1621_WR=1;
       _Nop();
      Data<<=1;
     }

}

/********************************************************
函数名称：void Ht1621WrCmd(uchar Cmd)
功能描述: HT1621命令写入函数
全局变量：无
参数说明：Cmd为写入命令数据
返回说明：无


说 明：写入命令标识位100
********************************************************/
void Ht1621WrCmd(uchar Cmd)
{
 HT1621_CS=0;
 _Nop();
 Ht1621Wr_Data(0x80,4); //写入命令标志100
 Ht1621Wr_Data(Cmd,8); //写入命令数据
 HT1621_CS=1;
 _Nop();
}
/********************************************************
函数名称：void Ht1621WrOneData(uchar Addr,uchar Data)
功能描述: HT1621在指定地址写入数据函数
全局变量：无
参数说明：Addr为写入初始地址，Data为写入数据
返回说明：无
说 明：因为HT1621的数据位4位，所以实际写入数据为参数的后4位
********************************************************/
void Ht1621WrOneData(uchar Addr,uchar Data)
{
 HT1621_CS=0;
 Ht1621Wr_Data(0xa0,3); //写入数据标志101
 Ht1621Wr_Data(Addr<<2,6); //写入地址数据
 Ht1621Wr_Data(Data<<4,4); //写入数据
 HT1621_CS=1;
 _Nop();
}
/********************************************************
HT1621测试程序，2008-2-13, 22:41:43
函数名称：void Ht1621WrAllData(uchar Addr,uchar *p,uchar cnt)
功能描述: HT1621连续写入方式函数
全局变量：无
参数说明：Addr为写入初始地址，*p为连续写入数据指针，
                  cnt为写入数据总数
返回说明：无
说 明：HT1621的数据位4位，此处每次数据为8位，写入数据
           总数按8位计算
********************************************************/
void Ht1621WrAllData(uchar Addr,uchar *p,uchar cnt)
{
 uchar i;
 HT1621_CS=0;
 Ht1621Wr_Data(0xa0,3); //写入数据标志101
 Ht1621Wr_Data(Addr<<2,6); //写入地址数据
 for (i=0;i<cnt;i++)
      {
        Ht1621Wr_Data(*p,8); //写入数据
        p++;
       }
 HT1621_CS=1;
 _Nop();
}
/********************************************************
函数名称：void Ht1621_Init(void)
功能描述: HT1621初始化
全局变量：无
参数说明：无
返回说明：无
版 本：1.0
说 明：初始化后，液晶屏所有字段均显示
********************************************************/
void Ht1621_Init(void)
{
 HT1621_CS=1;
 HT1621_WR=1;
 HT1621_DAT=1;
 DelayMS(20); //延时使LCD工作电压稳定
 Ht1621WrCmd(BIAS);
 Ht1621WrCmd(RC256); //使用内部振荡器
 Ht1621WrCmd(SYSDIS);
 Ht1621WrCmd(WDTDIS);
 Ht1621WrCmd(SYSEN);
 Ht1621WrCmd(LCDON);
}
/*
void main(void)
{	
uchar i,j,t;
   P1DIR |= 0XF0;   
   P2DIR |= 0X01; 
 Ht1621_Init(); //上电初始化LCD
 DelayMS(500); //延时一段时间
 while(1){
Ht1621WrAllData(0,Ht1621Tab,18);//清除1621寄存器数据，暨清屏 //SEG0～SEG17 COM0-3=0X00 0000 全灭
			Ht1621WrOneData(1,table1[9]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
                        Ht1621WrOneData(2,table2[9]);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
                         DelayMS(5000);
                        Ht1621WrOneData(3,table1[8]);//SEG3 COM0-3=table1[8]=0X0E 1110 数字8
			Ht1621WrOneData(4,table2[8]);//SEG4 COM0-3=table2[8]=0X0F 1111 数字8
                         DelayMS(5000);
                        Ht1621WrOneData(5,table1[7]);
			Ht1621WrOneData(6,table2[7]);
 DelayMS(5000);
			Ht1621WrOneData(7,table1[6]);
			Ht1621WrOneData(8,table2[6]);
 DelayMS(5000);
			Ht1621WrOneData(9,table1[5]);
			Ht1621WrOneData(10,table2[5]);
 DelayMS(5000);
			Ht1621WrOneData(11,table1[4]);
			Ht1621WrOneData(12,table2[4]);
 DelayMS(5000);
			Ht1621WrOneData(13,table1[3]);
			Ht1621WrOneData(14,table2[3]);
 DelayMS(5000);
			Ht1621WrOneData(15,table1[2]);
			Ht1621WrOneData(16,table2[2]);
            DelayMS(5000);
			 Ht1621WrAllData(0,Ht1621Tab,18);//清除1621寄存器数据，暨清屏 //SEG0～SEG17 COM0-3=0X00 0000 全灭
               for (i=0;i<18;i++) //18个字段 SEG0～SEG17 
                    {
                     t=0x01;
                     for (j=0;j<4;j++)//4个一组 COM0-3
                          {
                           Ht1621WrOneData(i,t); //SEG0 COM0-3. SEG1 COM0-3. SEG2 COM0-3. SEG3 COM0-3. .......
                           t<<=1;
                           t++;
                           DelayMS(5000);
                          }
                     }
               }
}
*/