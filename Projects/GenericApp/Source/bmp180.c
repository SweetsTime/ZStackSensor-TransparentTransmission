#include <ioCC2530.h>
#include "bmp180.h"
#include "delay.h"
#define dev_addr_r  0xef//读寄存器地址
#define BMP180_SLAVE_ADDR  0xee//写寄存器地址
#define OSS 0

int16_t ac1;
int16_t ac2;
int16_t ac3;
uint16_t ac4;
uint16_t ac5;
uint16_t ac6;
int16_t b1;
int16_t b2;
int16_t mb;
int16_t mc;
int16_t md;

  short num = 8;
int16_t num1 = 4000;
int16_t num2 = 2;
uint16_t num3 = 32768;
uint16_t num4 = 50000;
uint16_t num5 = 3038;
int16_t num6 = -7357;
int16_t num7 = 3791;

int16_t num8 = 10;

    
long pressure;
long temperature;
    
void I2C2_GPIO_Config(void);
void I2C2_Start(void);
void I2C2_Stop(void);
void Sendack(uint8_t h);
uint8_t I2C2_Check_ack(void);
uint8_t I2C2_Write_Byte(uint8_t b);
uint8_t I2C2_Read_Byte(void);
void I2C2_Write(uint8_t Slave_Addr, uint8_t REG_Address,uint8_t REG_data);

void I2C2_GPIO_Config(void)
{
  P1DIR |= 0x18; //配置1.3 1.4端口为输出模式
}

void I2C2_Start(void)
{   
  SDA_H;    
  SCL_H;   
  halMcuWaitUs(20);
  SDA_L;   
  halMcuWaitUs(20);//大于4个微秒
  SCL_L;
}


void I2C2_Stop(void)
{
    
  SCL_L;   
  SDA_L;   
  SCL_H;
  halMcuWaitUs(20);//大于4个微秒  
  SDA_H;
  halMcuWaitUs(20);//大于4个微秒
}

void Sendack(uint8_t h)
{
    
  SCL_L;
  if(h==0)
    SDA_L;//有应答信号
  else
    SDA_H;    
  halMcuWaitUs(20);
  SCL_H;   
  halMcuWaitUs(20);   
  SCL_L;

}



uint8_t I2C2_Check_ack(void)

{
  uint8_t count = 0;
  SCL_L;   
  SDA_H;//要读低电平需先拉高再读,否则读到的是错误数据,很重要！
  halMcuWaitUs(20); 
  SCL_H;
  halMcuWaitUs(20);  
  
  while(SDA_read)
  {
    count++;
    if(count > 250)
    {
      SCL_L;   
      halMcuWaitUs(20); 
      return 1;
    }
  }
  SCL_L;   
  halMcuWaitUs(20);
  return 0;

}



uint8_t I2C2_Write_Byte(uint8_t b)

{
    
  uint8_t e=8;
    
  while(e--)
  {
        
    SCL_L;
    halMcuWaitUs(20);    
    if(b&0x80)
      SDA_H;      
    else 
      SDA_L;    
    halMcuWaitUs(20);
    b<<=1;      
    SCL_H;
    halMcuWaitUs(20);
  }
    
  SCL_L;
  halMcuWaitUs(20);   
  return(I2C2_Check_ack());

}



uint8_t I2C2_Read_Byte(void)

{
    
  uint8_t i=8;   
  uint8_t c=0;   
  SCL_H;
  halMcuWaitUs(20);    
  while(i--)
  {       
    c<<=1;        
          
    if(SDA_read)
      c|=0x01;       
    else c&=0xfe;   
    halMcuWaitUs(20);
    SCL_L;       
    halMcuWaitUs(20);     
    SCL_H; 
    halMcuWaitUs(20);
  }   
  SCL_L;   
  return c;

}



void I2C2_Write(uint8_t Slave_Addr, uint8_t REG_Address,uint8_t REG_data)
{
    
  I2C2_Start();
    
  I2C2_Write_Byte(Slave_Addr);
    
  I2C2_Write_Byte(REG_Address);
    
  I2C2_Write_Byte(REG_data);
    
  I2C2_Stop();

}
int16_t BMP180_Read_2B(uint8_t addr);

int16_t BMP180_Read_2B(uint8_t addr)
{
    uint8_t msb = 0, lsb = 0;
    I2C2_Start();//起始信号
    if(I2C2_Write_Byte(BMP180_SLAVE_ADDR) == 0)//发送设备地址+写信号
    {
      I2C2_Write_Byte(addr);//发送存储单元地址
      I2C2_Start();//起始信号
      I2C2_Write_Byte(dev_addr_r);//发送设备地址+读信号
      msb=I2C2_Read_Byte();
      Sendack(0);
      lsb=I2C2_Read_Byte();
      Sendack(1);
      I2C2_Stop();
    }
    return (short)((msb << 8) | lsb);
}
long BMP180_Read_TEMP(void);             
long BMP180_Read_TEMP(void)
{
    //int16_t temp;
    I2C2_Write(BMP180_SLAVE_ADDR, 0xF4, 0x2E);
    halMcuWaitMs(200); 
    return (long)BMP180_Read_2B(0xF6);
}
long BMP180_Read_Pressure(void);
long BMP180_Read_Pressure(void)
{
    //long pressure;
    I2C2_Write(BMP180_SLAVE_ADDR, 0xF4, (0x34 + (OSS << 6)));
    halMcuWaitMs(200);
    return ((long)(BMP180_Read_2B(0xF6)&0x0000ffff));
}

void Init_BMP180(void)
{
    I2C2_GPIO_Config();
    ac1 = BMP180_Read_2B(0xAA);
    ac2 = BMP180_Read_2B(0xAC);
    ac3 = BMP180_Read_2B(0xAE);
    ac4 = BMP180_Read_2B(0xB0);
    ac5 = BMP180_Read_2B(0xB2);
    ac6 = BMP180_Read_2B(0xB4);
    b1 =  BMP180_Read_2B(0xB6);
    b2 =  BMP180_Read_2B(0xB8);
    mb =  BMP180_Read_2B(0xBA);
    mc =  BMP180_Read_2B(0xBC);
    md =  BMP180_Read_2B(0xBE);
}

void Multiple_Read_BMP180()
{ 
    long ut;
    long up;
    long x1, x2, b5, b6, x3, b3, p,b7;
    long xx1,xx2,bb4,bb3,bbb3;
    unsigned long b4;
    unsigned int tempe;
    
    ut = BMP180_Read_TEMP();
    up = BMP180_Read_Pressure();

    xx1 = (int32_t)ut - ac6;
    x1 = (xx1 * ac5) >> 15;
    xx2 = (int32_t)mc;
    x2 = (xx2 << 11)/(x1 + md);
    b5 = x1 + x2;
    tempe = b5 + num;
    temperature = tempe >> 4;
    //temperature = ((b5 + 8) >> 4);

    //dat->press = BMP085_Read_Pressure();
    b6 = b5 - num1;
    x1 = (b2 * (b6 * b6) >> 12) >> 11;
    x2 = (ac2 * b6) >> 11;
    x3 = x1 + x2;
    bb3 = (int32_t)ac1;
    bbb3 = ((bb3 * num2 * num2 + x3) << OSS) + num2;
    b3 = bbb3 / num2 / num2;
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + num2) >> 2;
    bb4 = ((uint32_t)(x3 + num3));
    b4 = (bb4 * ac4) >> 15;
    b7 = ((uint32_t)up - b3) * (num4 >> OSS);
    if( b7 < 0x80000000)
        p = (b7 * num2) / b4 ;
    else
        p = (b7 / b4) * num2;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * num5) >> 16;
    x2 = (num6 * p) >> 16;
    pressure = p + ((x1 + x2 + num7) >> 4);

   // *press = pressure;
   // *temp = t_integer;
}
