#include <ioCC2530.h>
#include <string.h>                                 //头文件中引用自身
#include "stdio.h"
#define uint unsigned int
#define uchar unsigned char
#define  _nop_() asm("NOP")
//MLX90615
//宏定义I/O端口和SMBus信号的方向
//----------------------------------------------------------------------------------------------------------------------------------------//
#define _SDA_OUTPUT   P1DIR |= 0X10;            //设置SDA为开漏输出
#define _SDA_INPUT  P1DIR&= (~0X10); 	  //设置SDA为高阻输入
#define _SCL_IO  P1DIR |= 0X08;  	//设置SDA为开漏输出
#define SDA  P1_4                                                                //指定SDA线给P14
#define SCL  P1_3                                                                //指定SCL线给P13

//----------------------------------------------------------------------------------------------------------------------------------------//

void Delay_mlx90615(unsigned int N);
void start_bit(void);
void stop_bit(void);
void send_bit(unsigned char bit_out);
unsigned char receive_bit(void);
unsigned char slave_ack(void);
void TX_byte(unsigned char TX_buffer);
unsigned char RX_byte(unsigned char ack_nack);
unsigned char PEC_cal(unsigned char pec[],int n);
void EEPROM_WRITE(unsigned char slave_addW,unsigned char cmdW,unsigned char DataL,unsigned char DataH);
unsigned long int MEM_READ(unsigned char slave_addR, unsigned char cmdR);
void CALTEMP(unsigned long int TEMP,unsigned char *Datan);
void dec2hex(float e,unsigned int *c);

void Delay_mlx90615(unsigned int N)
{ 
    unsigned int i;
    for(i=0;i<N;i++)
	{ 
    _nop_();}  
}
//----------------------------------------------------------------------------------------------------------------------------------------//

//函数名: start_bit
//功能: 在SMBus总线上产生起始状态
//注解: 参考"系统管理总线说明书-版本2.0"
//----------------------------------------------------------------------------------------------------------------------------------------//
void start_bit(void)
{
   _SDA_OUTPUT;		                    //设置SDA为输出
   SDA=1;				       //设置SDA线为高电平
   _nop_();_nop_();				  
   SCL=1;				       //设置SCL线为高电平
   Delay_mlx90615(5);				       //在终止和起始状态之间产生总线空闲时间(Tbuf=4.7us最小值)
   SDA=0;				       //设置SDA线为低电平
   Delay_mlx90615(5);				      
   //（重复）开始状态后的保持时间，在该时间后，产生第一个时钟信号
  					       //Thd:sta=4us最小值
   SCL=0;				       //设置SCL线为低电平
   _nop_();_nop_();

}
//----------------------------------------------------------------------------------------------------------------------------------------//
//函数名: stop_bit
//功能: 在SMBus总线上产生终止状态
//注解: 参考"系统管理总线说明书-版本2.0"
//----------------------------------------------------------------------------------------------------------------------------------------//
void stop_bit(void)
{
  _SDA_OUTPUT;				 //设置SDA为输出
  SCL=0;			     		 //设置SCL线为低电平
  Delay_mlx90615(5);
  SDA=0;					 //设置SDA线为低电平
  Delay_mlx90615(5);
  SCL=1;				             //设置SCL线为高电平
  Delay_mlx90615(5);				             //终止状态建立时间(Tsu:sto=4.0us最小值)
  SDA=1;				             //设置SDA线为高电平 
}
//----------------------------------------------------------------------------------------------------------------------------------------//
//函数名: send_bit
//功能:在SMBus总线上发送一位数据
//----------------------------------------------------------------------------------------------------------------------------------------//
void send_bit(unsigned char bit_out)
{
   
  _SDA_OUTPUT;			   //设置SDA为开漏输出以在总线上传送数据

  if(bit_out==0)				   //核对字节的位
            					   //如果bit_out=1，设置SDA线为高电平
             SDA=0;
  else							  
             SDA=1;				   //如果bit_out=0，设置SDA线为低电平

  _nop_();				   //
  _nop_();				   //Tsu:dat=250ns 最小值
  _nop_();				   //
  SCL=1;				   //设置SCL线为高电平
  Delay_mlx90615(4);				  //时钟脉冲高电平脉宽(10.6us)
  SCL=0;				  //设置SCL线为低电平
  Delay_mlx90615(4);                                                  //时钟脉冲低电平脉宽  
}
//----------------------------------------------------------------------------------------------------------------------------------------//
//函数名: receive_bit
//功能：在SMBus总线上接收一位数据
//----------------------------------------------------------------------------------------------------------------------------------------//
unsigned char receive_bit(void)
{
  unsigned char bit_in;
  _SDA_INPUT;				                //设置SDA为高阻输入
  SCL=1;					   //设置SCL线为高电平
  Delay_mlx90615(2);
  if(SDA==1)					   //从总线上读取一位，赋给bit_in
       bit_in=1;
  else
       bit_in=0;
  Delay_mlx90615(2);
  SCL=0;					   //设置SCL线为低电平
  Delay_mlx90615(4);
  return bit_in;                                                           //返回bit_in值
}
//----------------------------------------------------------------------------------------------------------------------------------------//
//函数名: slave_ack
//功能: 由受控器件MLX90615中读取确认位
//返回值:  unsigned char ack
// 1 - ACK
// 0 - NACK
//----------------------------------------------------------------------------------------------------------------------------------------//
unsigned char slave_ack(void)
{
   unsigned char ack;
   ack=0;
   _SDA_INPUT;				    //设置SDA为高阻输入
   SCL=1;					    //设置SCL线为高电平
   Delay_mlx90615(2);    
   if(SDA==1)					    //从总线上读取一位，赋给ack
         ack=0;
   else
         ack=1; 
   Delay_mlx90615(2);   
   SCL=0;					    //设置SCL线为低电平
   Delay_mlx90615(4);   
   return ack;
}
//----------------------------------------------------------------------------------------------------------------------------------------//
//发送一个字节
//函数名: TX_byte
//功能: 在SMBus总线上发送一个字节
//参数: unsigned char TX_buffer (将要在总线上发送的字节)
//注解: 先发送字节的高位

//----------------------------------------------------------------------------------------------------------------------------------------//
void TX_byte(unsigned char TX_buffer)
{
   unsigned char Bit_counter;
   unsigned char bit_out;
     
   for(Bit_counter=8;Bit_counter;Bit_counter--)
   {
       if(TX_buffer&0x80)
		     bit_out=1;	               //如果TX_buffer的当前位是1,设置bit_out为1
		else
		     bit_out=0;	         	  //否则，设置bit_out为0
     send_bit(bit_out);			  //发送SMBus总线上的当前位   
     TX_buffer<<=1;		               //核对下一位		  
	}			            	                      
}
//----------------------------------------------------------------------------------------------------------------------------------------//
//接收一个字节
//函数名: RX_byte
//功能: 在SMBus总线上接收一个字节
//参数: unsigned char ack_nack (确认位)
//0 - 主控器件发送 ACK
//1 - 主控器件发送 NACK
//返回值:  unsigned char RX_buffer (总线接收的字节)
//注解: 先接收字节的高位
//----------------------------------------------------------------------------------------------------------------------------------------//
unsigned char RX_byte(unsigned char ack_nack)
{
    unsigned char RX_buffer;
    unsigned char Bit_counter;
    for(Bit_counter=8;Bit_counter;Bit_counter--)
    {
		if(receive_bit()==1)	                //由SDA线读取一位
		   {
			RX_buffer<<=1;		   //如果位为"1"，赋"1"给RX_buffer 
			RX_buffer|=0x01;
		   }
		else				   //如果位为"0"，赋"0"给RX_buffer
		   {
			RX_buffer<<=1;
			RX_buffer&=0xfe;
		   }		
      } 
	 send_bit(ack_nack);			   //发送确认位
	 return RX_buffer;
}

//计算PEC包裹校验码
//函数名: PEC_cal
//功能: 根据接收的字节计算PEC码
//参数: unsigned char pec[], int n
//返回值: pec[0] - 该字节包含计算所得crc数值
//----------------------------------------------------------------------------------------------------------------------------------------//
unsigned char PEC_cal(unsigned char pec[],int n)
{
     unsigned char crc[6];
     unsigned char Bitposition=47;
     unsigned char shift;
     unsigned char i;
     unsigned char j;
     unsigned char temp;
  do{
          crc[5]=0;           			        //载入 CRC数值 0x000000000107
          crc[4]=0;
          crc[3]=0;
          crc[2]=0;
          crc[1]=0x01;
          crc[0]=0x07;
          Bitposition=47;     		                     //设置Bitposition的最大值为47
          shift=0;
          //在传送的字节中找出第一个"1"

          i=5;                			        //设置最高标志位 (包裹字节标志)
          j=0;                			        //字节位标志，从最低位开始
          while((pec[i]&(0x80>>j))==0 && (i>0))	  
	  {
             Bitposition--;
             if(j<7)
	   {
                    j++;
                 }
             else
	      {
                   j=0x00;
                   i--;
                   }
           }//while语句结束，并找出Bitposition中为"1"的最高位位置
          shift=Bitposition-8;                                   //得到CRC数值将要左移/右移的数值"shift"
	                                                              //对CRC数据左移"shift"位
          while(shift)
	     {
              for(i=5;i<0xFF;i--)
		 {  
                    if((crc[i-1]&0x80) && (i>0))          //核对字节的最高位的下一位是否为"1"
		     {   			       //是 - 当前字节 + 1
                          temp=1;		       //否 - 当前字节 + 0
                     }				       //实现字节之间移动"1"
                    else
	             {
                          temp=0;
                     }
                     crc[i]<<=1;
                     crc[i]+=temp;
                  } 

                  shift--;
              } 
           //pec和crc之间进行异或计算
           for(i=0;i<=5;i++)
		   {
                   pec[i]^=crc[i];
		   }  
      }while(Bitposition>8); 
	return pec[0];                                    //返回计算所得的crc数值
} 
//----------------------------------------------------------------------------------------------------------------------------------------//
//由MLX90615 RAM/EEPROM 读取的数据
//函数名: MEM_READ
//功能: 给定受控地址和命令时由MLX90615读取数据
//参数: unsigned char slave_addR (受控地址)
//         unsigned char cmdR (命令)
//返回值: unsigned long int Data
//----------------------------------------------------------------------------------------------------------------------------------------//
unsigned long int MEM_READ(unsigned char slave_addR, unsigned char cmdR)
{	
	 unsigned char DataL;		                           //
	 unsigned char DataH;				 //由MLX90615读取的数据包
	 unsigned char PEC;				 //
	 unsigned long int Data;			              //由MLX90615返回的寄存器数值
	 unsigned char Pecreg;				 //存储计算所得PEC字节
              unsigned char arr[6];				 //存储已发送字节的缓冲器
	 unsigned char ack_nack;
	 unsigned char SLA;										
	 SLA=(slave_addR<<1);
   begin:		             
	 start_bit();                 			               //发送起始位
	 TX_byte(SLA);					  //发送受控器件地址，写命令
	 if(slave_ack()==0)
	 {
	     stop_bit();
	     goto begin;
              }						  //发送命令
              TX_byte(cmdR);
	 if(slave_ack()==0)
	 {
	     stop_bit();
	     goto begin;
              }					
	 start_bit(); 	                                                      //发送重复起始位				
	 TX_byte(SLA+1);                                                   //发送受控器件地址，读命令
	 if(slave_ack()==0)
	 {
	     stop_bit();
	     goto begin;
	 }
	 DataL=RX_byte(0);				  //
							  //读取两个字节数据
	 DataH=RX_byte(0);				  //
	 PEC=RX_byte(ack_nack);		               //读取MLX90615的PEC码
	 if(ack_nack==1)				  //主控器件发送ack 或是 nack
              //取决于pec计算，如果PEC是不正确的，发送nack并返回到goto begin
	 {
	     stop_bit();
	     goto begin;
	 }						
	 stop_bit();                                                               //发送终止位
	 arr[5]=(SLA);
              arr[4]=cmdR;
              arr[3]=(SLA+1);               
              arr[2]=DataL;
              arr[1]=DataH;
              arr[0]=0;                  										 
	 Pecreg=PEC_cal(arr,6);  			  //调用计算 CRC 的函数
	 if(PEC==Pecreg)
	    {
		  ack_nack=0;
	    }
	  else
	    {
		 ack_nack=1;
	    }
	  Data=(( unsigned int)DataH<<8)+DataL;
	  return Data;

}
//----------------------------------------------------------------------------------------------------------------------------------------//	
//MLX90615 EEPROM中写入数据
//函数名: EEPROM_WRITE
//功能: 根据命令写入相关数据到给定受控器件地址的MLX90615
//参数: unsigned char slave_addW (受控器件地址)
//unsigned char cmdW (命令)
//unsigned char DataL 
//unsigned char DataH
//----------------------------------------------------------------------------------------------------------------------------------------//	
void EEPROM_WRITE(unsigned char slave_addW,unsigned char cmdW,unsigned char DataL,unsigned char DataH)
{
     unsigned char Pecreg;			             //存储计算所得PEC字节
     unsigned char SLA;
     unsigned char arr[6];					//存储将要发送字节的缓冲器
     SLA=(slave_addW<<1);
     arr[5]=0;
     arr[4]=SLA;
     arr[3]=cmdW;
     arr[2]=DataL;
     arr[1]=DataH;
     arr[0]=0;
     Pecreg=PEC_cal(arr,6);

   begin:
     start_bit();						 //发送起始位
     TX_byte(SLA);					 //发送受控地址，写命令
     if(slave_ack()==0)								  
     {
         stop_bit();
         goto begin;
     }
     TX_byte(cmdW);					  //发送命令
     if(slave_ack()==0)
     {
         stop_bit();
         goto begin;
     }
     TX_byte(DataL);					 //发送数据低位字节
     if(slave_ack()==0)
     {
         stop_bit();
         goto begin;
     }
     TX_byte(DataH);					 //发送数据高位字节
     if(slave_ack()==0)
     {
         stop_bit();
         goto begin;
     }
     TX_byte(Pecreg);					 //发送PEC码
     if(slave_ack()==0)
     {
         stop_bit();
         goto begin;
     }
     stop_bit();	            				              //发送终止位
     Delay_mlx90615(200);						 //等候5ms
    
}
//----------------------------------------------------------------------------------------------------------------------------------------//
//函数名: CALTEMP
//功能: 计算温度
//参数: unsigned long int TEMP (由MLX90615中读到的数据)
//返回值:     unsigned int mah 
//                 mah是数组mah[5]的首地址
//注解: 将十六进制代码转换为温度数据的公式为T=(Data)*0.02-273.15
//----------------------------------------------------------------------------------------------------------------------------------------//
void CALTEMP(unsigned long int TEMP,unsigned char *mah)
{
      unsigned long int T;
      unsigned int a,b;
      T=TEMP*2;
      if(T>=27315)
            {
               T=T-27315;
               a=T/100;  //整数部分
               b=T-a*100;//小数部分
            }
          else
               {
                  T=27315-T;
                  a=T/100;  //整数部分
                  b=T-a*100;//小数部分
                }
	    mah[0]=a;//整数部分
            mah[1]=b;//小数部分
}

/*
void main(void)
{    unsigned char     slaveaddress;	 			  
     unsigned long int    DATA;  
     unsigned char tempbat[5];
     
     _SCL_IO;                                            //引用宏定义-设置SCL为开漏式I/O口
     _SDA_OUTPUT;                                //引用宏定义-设置SDA为开漏式输出
     SCL=0;				//
     Delay_mlx90615(30000);		 //SMBus请求时间，将PWM模式转换为SMBus模式(21ms - 39ms)
     SCL=1;				//	
     initUARTtest();
     while(1)
     {
          slaveaddress=MEM_READ(0x00,0x10);
         //读取存于MLX90615 EEPROM "00h"地址中的SMBus地址
         DATA=MEM_READ(slaveaddress,0x27);
         //基于上述地址由MLX90615的内存07h中读取物体温度
          CALTEMP(DATA,tempbat);	
         //基于所得的十六进制温度格式计算实际温度
          UartTX_Send_String(tempbat,5);	        
          for(int p=0;p<100;p++)
          Delay_mlx90615(30000);		 
     }
 }
*/