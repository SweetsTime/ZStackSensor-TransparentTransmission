/*******************************************************************************
 * 文件名称：hal_led.c
 * 功    能：OLED驱动
 *           使用硬件SPI总线驱动128*64点阵OLED液晶
 * 硬件连接：液晶模块与CC2530的硬件连接关系如下：
 *                液晶模块                       CC2530
 *                  CS                            P1.2
 *                  SDA                           P1.6
 *                  SCK                           P1.5
 *                  RESET                         P0.0
 *                  D/C#                          P2.2
 *                字库芯片
 *                  CS#                           P2.1
 *                  SCLK                          P1.5
 *                  SI                            P1.6
 *                  SO                            P1.7
 *
 * 作    者：w
 * 公    司：无锡泛太科技有限公司
 ******************************************************************************/

/* 包含头文件 */
/********************************************************************/
#include "ioCC2530.h"
#include "hal_defs.h"
#include "LCD_SPI.h"
/********************************************************************/


/* 本地变量 */
/********************************************************************/
uint8 X_Witch = 6;
uint8 Y_Witch = 1;
uint8 X_Witch_cn = 16;
uint8 Y_Witch_cn = 16;
uint8 Dis_Zero = 0;
/********************************************************************/

#define FUNCTION_SET(options,OLED_DC)       halOLED_control(options,OLED_DC)

/*********************************************************************
 * 函数名称：LCD_TimeDelay
 * 功    能：延时函数
 * 入口参数：Timers  延时时间参数
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void LCD_TimeDelay(uint16 Timers);
void LCD_TimeDelay(uint16 Timers)
{
  uint16 i;

  while(Timers)
  {
    Timers--;
    for(i = 0; i < 100; i++)NOP();
  }
}


/*********************************************************************
 * 函数名称：LCD_SPISSSet
 * 功    能：置SS线状态
 * 入口参数：Status SS线状态
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void LCD_SPISSSet(uint8 Status);
void LCD_SPISSSet(uint8 Status)
{
  NOP();NOP();NOP();
  if(Status)				
    H_LCD_CSn();
  else
    L_LCD_CSn();

  NOP();NOP();NOP();NOP();
}

/*********************************************************************
 * 函数名称：LCD_SPI_DC
 * 功    能：置DATA/COMMAND线状态
 * 入口参数：Status SS线状态
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void LCD_SPI_DC(uint8 Status);
void LCD_SPI_DC(uint8 Status)
{
  NOP();NOP();NOP();
  if(Status)				
    LCD_DATA();
  else
    LCD_COMMAND();

  NOP();NOP();NOP();NOP();
}

/*********************************************************************
 * 函数名称：GT20L_SPI
 * 功    能：片选GT20
 * 入口参数：Status SS线状态
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void GT20L_SPI(uint8 Status);
void GT20L_SPI(uint8 Status)
{
  NOP();NOP();NOP();
  if(Status)				
    L_GT20_CSn();
  else
    H_GT20_CSn();

  NOP();NOP();NOP();NOP();
}
/*********************************************************************
 * 函数名称：LCD_SPISendData
 * 功    能：通过串行SPI口输送一个byte的数据置模组
 * 入口参数：Data  要传送的数据
 * 出口参数：temp
 * 返 回 值：SPI接到数据
 ********************************************************************/
uint8 LCD_SPISendDatas(uint8 Data);
uint8 LCD_SPISendDatas(uint8 Data)
{
  uint8 i=0;
  uint8 temp=0;

  for(i = 0; i < 8; i++)
  {
    NOP();NOP();NOP();NOP();NOP();NOP(); // 适当插入一些空操作以保证SPI时钟速度小于2MHz
    L_LCD_SCK();	
		
    if(Data&0x80)
      H_LCD_SDA();
    else
      L_LCD_SDA();

    NOP();NOP();NOP();NOP();NOP(); //适当插入一些空操作以保证SPI时钟速度小于2MHz
    NOP();NOP();NOP();NOP();NOP();
    H_LCD_SCK();
    if(GT20_MISO==1)
    {
      temp=temp | BV(7-i);
    }
    NOP();NOP();NOP();NOP();NOP();
		
    Data = Data << 1;		   //数据左移一位
  }
  NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
  return temp;
}
/*********************************************************************
 * 函数名称：halOLED_control
 * 功    能：通过串行SPI口输送一个byte的数据置模组
 * 入口参数：数据cmd, 命令控制 OLED_DC
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void halOLED_control(uint8 cmd, uint8 OLED_DC);
void halOLED_control(uint8 cmd, uint8 OLED_DC)
{
  LCD_SPISSSet(0);	   //SS置低电平
  if(OLED_DC==COMMAND)
  {
    LCD_SPI_DC(0);
  }
  else
  {
    LCD_SPI_DC(1);
  }
  LCD_SPISendDatas(cmd);   //送指令0x80
  LCD_SPI_DC(1);
  LCD_SPISSSet(1);	   //完成操作置SS高电平
}

/*********************************************************************
 * 函数名称：HalLed_SET_XY
 * 功    能：设置 x y 地址
 * 入口参数：x   x地址
 *           y   y地址
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLed_SET_XY(unsigned char x,unsigned char y);
void HalLed_SET_XY(unsigned char x,unsigned char y)
{
  FUNCTION_SET(0x22,COMMAND);	//传送指令0x22
  FUNCTION_SET(y,COMMAND);	//要显示字符的左上角的Y轴位置
  FUNCTION_SET(Dis_Y_MAX ,COMMAND);	//要显示字符的左上角的Y轴位置
  FUNCTION_SET(0x21,COMMAND);	//传送指令0x21
  FUNCTION_SET(x,COMMAND);	//要显示字符的左上角的X轴位置
  FUNCTION_SET(Dis_X_MAX ,COMMAND);	//要显示字符的左上角的Y轴位置
}

/*********************************************************************
 * 函数名称：HalGT20L_TX_CMD
 * 功    能：GT20L硬件写入控制
 * 入口参数：cmd   写入数据
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
int8 HalGT20L_TX_CMD(uint8 cmd );
int8 HalGT20L_TX_CMD(uint8 cmd )
{
  return LCD_SPISendDatas(cmd);
}

/*********************************************************************
 * 函数名称：halGT20L_HRD_Font
 * 功    能：GT23L读取字符码值函数
 * 入口参数： Dst 字库地址, no_bytes 一个字符需要的码值 字节数 ,  *buffer 码值
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void halGT20L_HRD_Font(uint16 * Dst, uint8 no_bytes,uint8 *buffer);
void halGT20L_HRD_Font(uint16 * Dst, uint8 no_bytes,uint8 *buffer)
{
    unsigned char i = 0;
    GT20L_SPI(1);                                // enable device
    HalGT20L_TX_CMD(0x0B);                       // read command
    HalGT20L_TX_CMD(Dst[1]);  // send 3 address bytes
    HalGT20L_TX_CMD(((Dst[0]) >> 8));
    HalGT20L_TX_CMD(Dst[0] & 0xFF);
    HalGT20L_TX_CMD(0xFF);                       //dummy byte
    for (i = 0; i < no_bytes; i++)              // read until no_bytes is reached
    {
      buffer[i] =HalGT20L_TX_CMD(0xFF);    // receive byte and store at address 80H - FFH
    }
    GT20L_SPI(0);                                // disable device
}
/*********************************************************************
 * 函数名称：halASCII_Searh_ADDR
 * 功    能：ASCII码字符内码
 * 入口参数：uint8 ASCIICode,uint16 * CODE_ADDR
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void halASCII_Searh_ADDR(uint8 ASCIICode,uint16 * CODE_ADDR);
void halASCII_Searh_ADDR(uint8 ASCIICode,uint16 * CODE_ADDR)
{
  uint16 ASCII_Code_Addr = 0;
 // ASCII_Code_Addr = ((((uint16)ASCIICode) - (uint16)0x20) * 8) +0x66C0;//切换字体
  ASCII_Code_Addr = ((((uint16)ASCIICode) - (uint16)0x20) * 8) +0xbfC0;
  CODE_ADDR[0]=ASCII_Code_Addr ;
  CODE_ADDR[1]=0x03;
}

/*********************************************************************
 * 函数名称：LCD_WriteChar
 * 功    能：显示ASCII码字符
 * 入口参数：x  要显示的字符x地址
 *           y  要显示的字符y地址
 *           a  要显示的ASCII码字符值
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void LCD_WriteChar(uint8 x, uint8 y, const char a);
void LCD_WriteChar(uint8 x, uint8 y, const char a)
{
  uint8 i=0;
  uint16 ASCII_CODE_ADDR[2];
  uint8 font_buffer[8];

  halASCII_Searh_ADDR(a ,ASCII_CODE_ADDR);
  halGT20L_HRD_Font(ASCII_CODE_ADDR,8,font_buffer);
  HalLed_SET_XY(x,y);
  for(i=0;i<8;i++)
  {
  FUNCTION_SET(font_buffer[i],DATAs);
  }

}


/********************************************************************
 * 函数名称：LCD_WriteString
 * 功    能：在x、y为起始坐标处写入一串字符
 * 入口参数：x  X轴坐标，取值范围：0 - 127
 *           y  Y轴坐标，取值范围0-63
 *           p  要显示的字符串
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void LCD_WriteString(uint8 x, uint8 y, const char *p);
void LCD_WriteString(uint8 x, uint8 y, const char *p)
{
  while(*p != 0)
  {
    LCD_WriteChar(x, y, *p);
    x += 7;
    if(x > Dis_X_MAX)
    {
      x = Dis_Zero;
      if((Dis_Y_MAX - y) < Y_Witch) break;
      else y += 1;
    }
    p+=1;
  }
}

/********************************************************************
 * 函数名称：LCD_WriteStringL
 * 功    能：在x、y为起始坐标处写入一串字符
 * 入口参数：x  X轴坐标，取值范围：0 - 127
 *           y  Y轴坐标，取值范围0-63
 *           p  要显示的字符串
 *           l  要显示的字符串长度
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void LCD_WriteStringL(uint8 x, uint8 y, const char *p ,uint16 l);
void LCD_WriteStringL(uint8 x, uint8 y, const char *p ,uint16 l)
{
  while(l != 0)
  {
    LCD_WriteChar(x, y, *p);
    x += 7;
    if(x > Dis_X_MAX)
    {
      x = Dis_Zero;
      if((Dis_Y_MAX - y) < Y_Witch) break;
      else y += 1;
    }
    p+=1;
    l-=1;
  }
}



/********************************************************************
 * 函数名称：LCD_SetBackLight
 * 功    能：设置背光亮度等级
 * 入口参数：Deg  亮度等级0~127
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void LCD_SetBackLight(uint8 Deg);
void LCD_SetBackLight(uint8 Deg)
{
  GT20L_SPI(1);                                // enable device
  HalGT20L_TX_CMD(0x81);
  LCD_SPISendDatas(Deg);	   //背光设置亮度值
  GT20L_SPI(0);
}


/*********************************************************************
 * 函数名称：LCD_Clear
 * 功    能：LCD清屏
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无

 ********************************************************************/
void LCD_Clear(void);
void LCD_Clear(void)
{
  uint8 i,j;
  //清屏操作
  FUNCTION_SET(0x22,COMMAND);
  FUNCTION_SET(0,COMMAND);
  FUNCTION_SET(7,COMMAND);
  FUNCTION_SET(0x21,COMMAND);
  FUNCTION_SET(0,COMMAND);
  FUNCTION_SET(127,COMMAND);

  for(i=0;i<8;i++)
  {
    for(j=0;j<128;j++)
      FUNCTION_SET(0x00,DATAs);
  }
}



/*********************************************************************
 * 函数名称：LCD_Init
 * 功    能：LCD初始化
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void LCD_Init(void);
void LCD_Init(void)
{
  P0DIR |= (0x01<<0);  // P0.0为输出
  P1DIR |= (0x01<<2);  // P1.2为输出
  P1DIR |= (0x01<<5);  // P1.5为输出
  P1DIR |= (0x01<<6);  // P1.6为输出

  P1DIR &= (0X7F);  // P1.7为输入
  P2DIR |= (0x01<<1);  // P2.1为输出
  P2DIR |= (0x01<<2);  // P2.2为输出

  /* 复位LCD */
  L_LCD_RESETn();
  LCD_TimeDelay(3000);
  H_LCD_RESETn();
  LCD_TimeDelay(3000);

  //Charge Pump Setting
  FUNCTION_SET(0x8d,COMMAND);
  FUNCTION_SET(0x14,COMMAND);
  FUNCTION_SET(0xaf,COMMAND);
  //Set Display Clock
  FUNCTION_SET(0xD5,COMMAND);
  FUNCTION_SET(0xF0,COMMAND);
  //Set Pre-charge Period
  FUNCTION_SET(0xD9,COMMAND);
  FUNCTION_SET(0x11,COMMAND);
  //Set VCOMH Deselect Level
  FUNCTION_SET(0xDb,COMMAND);
  FUNCTION_SET(0x0,COMMAND);
  //Set Norma Display
  FUNCTION_SET(0xa6,COMMAND);
  //Entire Display ON
  //FUNCTION_SET(0xa5,COMMAND);
  //Set Contrast Control
  FUNCTION_SET(0x81,COMMAND);
  FUNCTION_SET(0xff,COMMAND);  //1-256
  //Set Segment Re-map
  FUNCTION_SET(0xa1,COMMAND);
  //Set COM Output Scan Direction
  FUNCTION_SET(0xc8,COMMAND);
  //Set Memory Addressing Mode
  FUNCTION_SET(0x20,COMMAND);
  FUNCTION_SET(0x00,COMMAND);
  LCD_SetBackLight(100); // 调节背光亮度
}





