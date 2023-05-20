/************************************************************
* 宏定义			

************************************************************/
#include "IIC.h"
/************************************************************

* 变量初始化												
************************************************************/
void mDelay(unsigned j);
void Start(void);
void Stop(void);
void Ack(void);
void NoAck(void);
void Send(unsigned char Data);
unsigned char Read(void);
/************************************************************
* 函数功能:延时k ms函数										
* 入口参数:k(1-128)											
* 出口参数:无	
* 作    者:												
**********************************************************/
//延时程序
void mDelay(unsigned j)
{
	unsigned int i;
	for(;j>0;j--)
	{	for(i=0;i<500;i++)
		{;}
    }
}
//发送起始条件
void Start(void)
{IIC_DATAOUT;
	SDA=1;
	SCL=1;
	NOPS;
	SDA=0;
	NOPS;
}
//发送停止条件
void Stop(void)
{IIC_DATAOUT;
	SDA=0;
	SCL=1;
	NOPS;
	SDA=1;
	NOPS;	
}
//应答位
void Ack(void)
{IIC_DATAOUT;
	SDA=0;
	NOPS;
	SCL=1;
	NOPS;
	SCL=0;
}
//非应答
void NoAck(void)
{IIC_DATAOUT;
	SDA=1;
	NOPS;
	SCL=1;
	NOPS;
	SCL=0;	
}
//发送数据子程序，Data为要发送的数据a
void Send(unsigned char Data)
{	 unsigned char BitCounter=8;//位数控制
	 unsigned char temp;//中间变量控制
        IIC_DATAOUT;
	do{ 
	 temp=Data;
	 SCL=0;
	 NOPS;
	 if((temp&0x0080)==0x0080)//如果最高位是1
	    SDA=1;
	 else
		SDA=0;
	 SCL=1;
	 temp=Data<<1;//左移
	 Data=temp;
	 BitCounter--;
	 }while(BitCounter);
	 SCL=0;
         NOPS;
         SDA=1;
          NOPS;
}
//读一个字节的数据，并返回该字节的值
unsigned char Read(void)
{   unsigned char temp=0;
    unsigned char temp1=0;
    unsigned BitCounter=8;
   IIC_DATAOUT;
	SDA=1;
        IIC_DATAIN ;
	do{	SCL=0;
		NOPS;
		SCL=1;
		NOPS;
		if(SDA)
		temp=temp|0x01;
		else
		temp=temp&0xfe;
		if(BitCounter-1)
		{	temp1=temp<<1;
			temp=temp1;
		}
		BitCounter--;
	  }while(BitCounter);
         IIC_DATAOUT;
	return(temp);	
}
//***************************************************

void Single_Write_(unsigned char REG_Address,unsigned char REG_data)
{
    Start();                  //起始信号
#if(SENSOR_TYPE =='F')
    Send(AddWr);  //发送设备地址+写信号
    Ack();
#endif
    Send(REG_Address);    //内部寄存器地址，请参考中文pdf 
    Ack();
   Send(REG_data);       //内部寄存器数据，请参考中文pdf
   Ack();
   Stop();                   //发送停止信号
   NOPS;
}

//********单字节读取内部寄存器*************************
unsigned char Single_Read_(unsigned char REG_Address)
{  unsigned char REG_data;
    Start();                          //起始信号
#if(SENSOR_TYPE =='F')
    Send(AddWr);          //发送设备地址+写信号
    Ack();
#endif
    Send(REG_Address);                   //发送存储单元地址，从0开始	
    Ack();
#if(SENSOR_TYPE =='F')
   Start();                          //起始信号
   Send(AddRd);         //发送设备地址+读信号
   Ack();
#endif
    REG_data=Read();              //读出寄存器数据
    SCL=0;
	 NoAck();  
	Stop();                           //停止信号
         NOPS;
           NOPS;
    return REG_data; 
}

//向IIC中写数据
void WrToROM(unsigned char Data[],unsigned char Address,unsigned char Num)
{   unsigned char i;
    unsigned char *Pdata;
    Pdata=Data;
	for(i=0;i<Num;i++)
	{	Start();
		Send(AddWr);
		Ack();
		Send(Address+i);
		Ack();
		Send(*(Pdata+i));
		Ack();
		Stop();
		mDelay(20);
	}
}
//	读数据
void RdFromROM(unsigned char Data[],unsigned char Address,unsigned char Num)
{
	unsigned char i;
	unsigned char *Pdata;
	Pdata=Data;
	for(i=0;i<Num;i++)
	{
		Start();
		Send(AddWr);
		Ack();
		Send(Address+i);
		Ack();
		Start();
		Send(AddRd);
		Ack();
		*(Pdata+i)=Read();
		SCL=0;
        NoAck();
		Stop();
	}
}
