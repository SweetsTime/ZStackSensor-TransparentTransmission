#include <18B20.h>
/************************************************************
* 程序功能: DS18B20测温
* 硬件说明:	           AT89S51	DS18B20
*			   P1_4		DG
* 作    者:	
************************************************************/
/************************************************************
* 函数功能:小延时函数,约(2*i μ)s				    				
* 入口参数:无										
* 出口参数:无	
* 作    者:												
**********************************************************/
void Delay(unsigned int i)
{while (i--);
}
/************************************************************
* 函数功能:DS18B20初始化复位脉冲函数									
* 入口参数:无										
* 出口参数:DQ	
* 作    者:												
**********************************************************/
void Init_DS18B20(void)
{ 
   DQ_WR;
   SET_DQ1; //写1 
   SET_DQ0; //写0       //单片机将DQ拉低,发送复位脉冲
   Delay(413);          //精确延时 480us~960us
   SET_DQ1; //写1       //拉高总线
   delay_1us();
          //精确延时 15us~60us
   DQ_RD;
   Delay(32);
   for(int i=0;i<30000;i++)
    if(!GET_DQ)i=30000; 
   for(int i=0;i<30000;i++)
    if(!GET_DQ)i=30000; 
  // while(GET_DQ)       //如果DQ=0则存在脉冲有效，初始化成功，如果DQ=1
   Delay(20);           //稍做延时后
    DQ_WR;
   SET_DQ1; //写1 
   Delay(405);
   
}
/************************************************************
* 函数功能:DS18B20写1字节命令函数									
* 入口参数:无										
* 出口参数:DQ	
* 作    者:												
**********************************************************/
void WriteOneChar(unsigned char Data)
{    unsigned char i;
    
    for(i=0;i<8;i++)
    {   DQ_WR; //写
       SET_DQ0;
       delay_1us();  //4//4us
        delay_1us(); 
        delay_1us(); 
        delay_1us(); 
       if(Data&0x01) SET_DQ1;
       Data>>=1;
       Delay(48);         //等待时间隙 60-120
       SET_DQ1;	
       delay_1us();
    }

}
/************************************************************
* 函数功能:DS18B20读1字节数据函数									
* 入口参数:无										
* 出口参数:1字节数据
* 作    者:												
**********************************************************/
unsigned char ReadOneChar(void)
{   unsigned char i;
    unsigned char Data=0;
    for(i=0;i<8;i++)
    {
      DQ_WR; //写
      SET_DQ0;  // 给脉冲信号
      delay_1us();
      delay_1us();
      SET_DQ1;  // 给脉冲信号
      delay_1us();
      DQ_RD; //读
      Delay(4); 
      if(GET_DQ)Data|=(0x01<<i);
      Delay(54) ;  //等待时间隙60-120
    } 
     DQ_WR; //写
     SET_DQ1;  
     delay_1us();
    
    return  Data;
}
/************************************************************
* 函数功能:DS18B20读二进制16位温度									
* 入口参数:无										
* 出口参数:二进制16位温度
* 作    者:												
**********************************************************/
unsigned int ReadTemperature(void)
{   unsigned char a=0,b=0;//,c[8];
    unsigned int THex = 0;//,i=0;
    Init_DS18B20();
    WriteOneChar(0xCC); // 跳过读序号列号的操作
    WriteOneChar(0x44); // 启动温度转换
    //Delay(30000); 
    Init_DS18B20();
    WriteOneChar(0xCC); //跳过读序号列号的操作
    WriteOneChar(0xBE); //读取温度寄存器等（共可读9个寄存器）前两个就是温度
    a = ReadOneChar();	//温度的低8位
    b = ReadOneChar();  //温度的高8位
	THex = b;
    THex=THex*256;
    THex = (THex | a);
    return(THex);
}
