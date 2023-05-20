/*******************************************************************************
 * 文件名称：LCD.h
 * 功    能：LCD驱动
 *           驱动12232点阵图形液晶
 * 硬件连接：液晶模块与CC2530的硬件连接关系如下：
 *                液晶模块                         CC2530
 *
 *               SPI-CLK(PIN2) 串行数据             P1.2
 *               SPI-MOSI(PIN3)移位寄存器时钟输入   P1.5
 *               GPH(PIN5)   存储寄存器时钟输入     P1.6
 * 作    者：w
 * 公    司：无锡泛太科技有限公司
 ******************************************************************************/


/* 包含头文件 */
/********************************************************************/
#include "ioCC2530.h"
#include "LCD12232.H"
/********************************************************************/
static const unsigned char __code ASCII[] =   
{      
	0xF8,0xFC,0x04,0xC4,0x24,0xFC,0xF8,0x00,  // -0-
	0x07,0x0F,0x09,0x08,0x08,0x0F,0x07,0x00,

	0x00,0x10,0x18,0xFC,0xFC,0x00,0x00,0x00,  // -1-
	0x00,0x08,0x08,0x0F,0x0F,0x08,0x08,0x00,

	0x08,0x0C,0x84,0xC4,0x64,0x3C,0x18,0x00,  // -2-
	0x0E,0x0F,0x09,0x08,0x08,0x0C,0x0C,0x00,

	0x08,0x0C,0x44,0x44,0x44,0xFC,0xB8,0x00,  // -3-
	0x04,0x0C,0x08,0x08,0x08,0x0F,0x07,0x00,

	0xC0,0xE0,0xB0,0x98,0xFC,0xFC,0x80,0x00,  // -4-
	0x00,0x00,0x00,0x08,0x0F,0x0F,0x08,0x00,

	0x7C,0x7C,0x44,0x44,0xC4,0xC4,0x84,0x00,  // -5-
	0x04,0x0C,0x08,0x08,0x08,0x0F,0x07,0x00,

	0xF0,0xF8,0x4C,0x44,0x44,0xC0,0x80,0x00,  // -6-
	0x07,0x0F,0x08,0x08,0x08,0x0F,0x07,0x00,

	0x0C,0x0C,0x04,0x84,0xC4,0x7C,0x3C,0x00,  // -7-
	0x00,0x00,0x0F,0x0F,0x00,0x00,0x00,0x00,

	0xB8,0xFC,0x44,0x44,0x44,0xFC,0xB8,0x00,  // -8-
	0x07,0x0F,0x08,0x08,0x08,0x0F,0x07,0x00,

	0x38,0x7C,0x44,0x44,0x44,0xFC,0xF8,0x00,  // -9-
	0x00,0x08,0x08,0x08,0x0C,0x07,0x03,0x00,

	

	0xE0,0xF0,0x98,0x8C,0x98,0xF0,0xE0,0x00,  // -A-
	0x0F,0x0F,0x00,0x00,0x00,0x0F,0x0F,0x00,

	0x04,0xFC,0xFC,0x44,0x44,0xFC,0xB8,0x00,  // -B-
	0x08,0x0F,0x0F,0x08,0x08,0x0F,0x07,0x00,

	0xF0,0xF8,0x0C,0x04,0x04,0x0C,0x18,0x00,  // -C-
	0x03,0x07,0x0C,0x08,0x08,0x0C,0x06,0x00,

	0x04,0xFC,0xFC,0x04,0x0C,0xF8,0xF0,0x00,  // -D-
	0x08,0x0F,0x0F,0x08,0x0C,0x07,0x03,0x00,

	0x04,0xFC,0xFC,0x44,0xE4,0x0C,0x1C,0x00,  // -E-
	0x08,0x0F,0x0F,0x08,0x08,0x0C,0x0E,0x00,

	0x04,0xFC,0xFC,0x44,0xE4,0x0C,0x1C,0x00,  // -F-
	0x08,0x0F,0x0F,0x08,0x00,0x00,0x00,0x00,
        
        0x20,0x60,0xC0,0x80,0xC0,0x60,0x20,0x00,  // -x-
	0x08,0x0C,0x07,0x03,0x07,0x0C,0x08,0x00,
        
        0x0C,0x3C,0xF0,0xC0,0xF0,0x3C,0x0C,0x00,  // -X-
	0x0C,0x0F,0x03,0x00,0x03,0x0F,0x0C,0x00,
        
};

//汉字库: 宋粗体16.dot 纵向取模下高位,数据排列:从左到右从上到下
//左半屏显示字模
static const unsigned char __code wuxifan[]={ 
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 ,
      
      0x00,0x40,0x42,0x42,0x42,0x42,0xFE,0x42,
      0xC2,0x42,0x43,0x42,0x60,0x40,0x00,0x00,
      0x00,0x80,0x40,0x20,0x18,0x06,0x01,0x00,
      0x3F,0x40,0x40,0x40,0x40,0x40,0x70,0x00,

      0x80,0x40,0x70,0xCF,0x48,0x48,0x00,0xFE,
      0x4A,0x4A,0x4A,0x4A,0x7F,0x82,0x00,0x00,
      0x00,0x02,0x02,0x7F,0x22,0x12,0x24,0x13,
      0x49,0x27,0x59,0x87,0x41,0x3F,0x01,0x00,

      0x10,0x62,0x04,0x8C,0x00,0x44,0x44,0x44,
      0x4C,0x74,0x42,0x43,0xC2,0x40,0x00,0x00,
      0x04,0x04,0xFE,0x01,0x20,0x10,0x10,0x28,
      0x44,0x42,0x41,0x41,0x40,0x60,0x20,0x00
              
};
 
//右半屏显示字模
 static const unsigned char __code taikeji[]={ 

      0x20,0x20,0x20,0x20,0x20,0x20,0x20,0xFF,
      0x20,0x20,0x20,0x20,0x20,0x30,0x20,0x00,
      0x40,0x40,0x20,0x20,0x10,0x0C,0x0B,0x30,
      0x03,0x0C,0x10,0x10,0x20,0x60,0x20,0x00,

      0x24,0x24,0x24,0xA4,0xFE,0xA3,0x22,0x00,
      0x24,0x48,0x00,0xFF,0x00,0x80,0x00,0x00,
      0x10,0x08,0x06,0x01,0xFF,0x00,0x01,0x02,
      0x02,0x02,0x02,0xFF,0x01,0x01,0x01,0x00,
      
      0x10,0x10,0x10,0xFF,0x10,0x10,0x88,0x88,
      0x88,0xFF,0x88,0x88,0x8C,0x08,0x00,0x00,
      0x04,0x44,0x82,0x7F,0x01,0x80,0x81,0x46,
      0x28,0x10,0x28,0x26,0x41,0xC0,0x40,0x00,
  
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
        

};
              
static const unsigned char __code bmp01[]={
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      
      
      0x10,0x92,0x54,0x30,0xFF,0x50,0x94,0x32,
      0xD8,0x17,0x10,0x10,0xF0,0x18,0x10,0x00,
      0x02,0x82,0x4E,0x33,0x22,0x52,0x8E,0x40,
      0x23,0x14,0x08,0x16,0x61,0xC0,0x40,0x00,

      0x10,0x10,0x10,0xFF,0x90,0x50,0xFE,0x92,
      0x92,0x92,0xF2,0x92,0x92,0xDF,0x82,0x00,
      0x02,0x42,0x81,0x7F,0x40,0x38,0x07,0xFC,
      0x44,0x44,0x47,0x44,0x44,0xFE,0x04,0x00,
        
      0x00,0x00,0x00,0x00,0x60,0x60,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x03,0x03,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

/*********************************************************************
 * 函数名称：LCD_SPISendData
 * 功    能：通过串行SPI口输送一个byte的数据置模组
 * 入口参数：Data  要传送的数据
 * 出口参数：temp
 * 返 回 值：SPI接到数据
 ********************************************************************/
void LCD_SPISendData(unsigned char Dat)
{
  unsigned char i;

  for(i = 0; i < 8; i++)
  {
    L_SPImosi();
    
    SPIdata=((Dat&0x80)>>7);
    NOP();
    NOP();
    Dat<<=1;		   //数据左移一位
    H_SPImosi();
  }
}
/*********************************************************************
 * 函数名称：LCD_SPIOUT
 * 功    能：595数据输出
 * 入口参数：
 * 出口参数：
 * 返 回 值：
 ********************************************************************/
void LCD_SPIOUT()
{
   L_GPH();
    NOP();
    NOP();
   H_GPH();
}
/*********************************************************************
 * 函数名称：
 * 功    能：595数据输出
 * 入口参数：
 * 出口参数：
 * 返 回 值：
 ********************************************************************/
void SendDataMI16(unsigned char MI,unsigned char NI)
{
  LCD_SPISendData(MI);
  LCD_SPISendData(NI);
  LCD_SPIOUT();
    NOP();
    NOP();
  LCD_SPISendData(0x10);
  LCD_SPISendData(NI);
  LCD_SPIOUT();

}

void SendDataMD16(unsigned char MD,unsigned char ND)
{
  LCD_SPISendData(MD);
  LCD_SPISendData(ND);
  LCD_SPIOUT();
    NOP();
    NOP();
  LCD_SPISendData(0x18);
  LCD_SPISendData(ND);
  LCD_SPIOUT();

}
/*********************************************************************
 * 函数名称：lcd初始化
 * 功    能：
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void LCD_lni(void)
{
  SendDataMI16(witch_MI,0XE2);
  SendDataMI16(witch_SI,0XE2);//复位
  
  SendDataMI16(witch_MI,0XAE);
  SendDataMI16(witch_SI,0XAE);//POWER SAVE
  
  SendDataMI16(witch_MI,0XA4);
  SendDataMI16(witch_SI,0XA4);//动态驱动
  
  SendDataMI16(witch_MI,0Xa9);
  SendDataMI16(witch_SI,0Xa9);//1/32占空比
  
  SendDataMI16(witch_MI,0XA0); 
  SendDataMI16(witch_SI,0XA0); 
  
  SendDataMI16(witch_MI,0XEE); 
  SendDataMI16(witch_SI,0XEE);//写模式
  
  SendDataMI16(witch_MI,0X00);
  SendDataMI16(witch_MI,0Xc0);
  
  SendDataMI16(witch_SI,0X00);
  SendDataMI16(witch_SI,0XC0);
  
  SendDataMI16(witch_MI,0XAF);
  SendDataMI16(witch_SI,0XAF);
}
/*********************************************************************
 * 函数名称：设置数据页
 * 功    能：
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void Setpage(unsigned char page0,unsigned char page1)
{
    SendDataMI16(witch_MI,0xB8|page1);
    SendDataMI16(witch_SI,0XB8|page0);
}
/*********************************************************************
 * 函数名称：设置数据地址
 * 功    能：
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void Setadderss(unsigned char adderss0,unsigned char adderss1)
{
    SendDataMI16(witch_MI,adderss1&0x7f);
    SendDataMI16(witch_SI,adderss0&0x7f);
}
/*********************************************************************
 * 函数名称：写数据在主页
 * 功    能：
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void Putchar0(unsigned char ch)
{
  SendDataMD16(witch_MD,ch);
}
/*********************************************************************
 * 函数名称：写数据在副页
 * 功    能：
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/                                                             
void Putchar1(unsigned char ch)
{
  SendDataMD16(witch_SD,ch);
}
/*********************************************************************
 * 函数名称：lcd清屏
 * 功    能：
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void LCD_clrscr(void)
{
  unsigned char i,page;
  for(page=0;page<4;page++)
  {
    Setpage(page,page);
    Setadderss(0,0);
      for(i=0;i<61;i++)
      {
        Putchar0(0);
        Putchar1(0);
      }
        
  }
}
/*********************************************************************
 * 函数名称：lcd左半屏上面显示4个汉字
 * 功    能：
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void show_left_up()
{
	int page=0;
	int col=0;
	for(page=0;page<2;page++)
        {
	Setadderss(0,0);
	if(page==0||page==2)
	{
		for(col=0;col<16;col++)
		{
			Setpage(0,page);
			Putchar0(wuxifan[col%16]);
		}
                for(col=16;col<32;col++)
		{
			Setpage(0,page);
			Putchar0(wuxifan[col%16+32]);
		}
                   for(col=32;col<48;col++)
		{
			Setpage(0,page);
			Putchar0(wuxifan[col%16+64]);
		}
                   for(col=48;col<64;col++)
		{
			Setpage(0,page);
			Putchar0(wuxifan[col%16+96]);
		}
                
	}
	else
	{
		for(col=0;col<16;col++)
		{
			Setpage(0,page);
			Putchar0(wuxifan[col%16+16]);
			
		}
                for(col=16;col<32;col++)
		{
			Setpage(0,page);
			Putchar0(wuxifan[col%16+48]);
			
		}
                for(col=32;col<48;col++)
		{
			Setpage(0,page);
			Putchar0(wuxifan[col%16+80]);
			
		}
                for(col=48;col<64;col++)
		{
			Setpage(0,page);
			Putchar0(wuxifan[col%16+112]);
			
		}
	}
        }
}
/*********************************************************************
 * 函数名称：lcd右半屏上面显示4个汉字
 * 功    能：
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void show_right_up()
{
	int page=0;
	int col=0;
	for(page=0;page<2;page++)
        {
	Setadderss(0,0);
	if(page==0||page==2)
	{
		for(col=0;col<16;col++)
		{
                    Setpage(page,0);
                   Putchar1(taikeji[col%16]);
		}

                for(col=16;col<32;col++)
		{
                    Setpage(page,0);
                   Putchar1(taikeji[col%16+32]);
		}
                for(col=32;col<48;col++)
		{
                    Setpage(page,0);
                   Putchar1(taikeji[col%16+64]);
		}
                for(col=48;col<64;col++)
		{
                    Setpage(page,0);
                   Putchar1(taikeji[col%16+96]);
		}
	}
	else
	{
		for(col=0;col<16;col++)
		{
                    Setpage(page,0);
                    Putchar1(taikeji[col%16+16]);
			
		}
                for(col=16;col<32;col++)
		{
			Setpage(page,0);
			Putchar1(taikeji[col%16+48]);
			
		}
                for(col=32;col<48;col++)
		{
			Setpage(page,0);
			Putchar1(taikeji[col%16+80]);
			
		}
                for(col=48;col<64;col++)
		{
			Setpage(page,0);
			Putchar1(taikeji[col%16+112]);
	        }
        }
        }
}

/*********************************************************************
 * 函数名称：lcd左半屏下面显示4个汉字
 * 功    能：
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void show_left_down()
{
	int page=0;
	int col=0;
	for(page=2;page<4;page++)
        {
	Setadderss(0,0);
	if(page==0||page==2)
	{
		for(col=0;col<16;col++)
		{
			Setpage(0,page);
			Putchar0(bmp01[col%16]);
		}
                for(col=16;col<32;col++)
		{
			Setpage(0,page);
			Putchar0(bmp01[col%16+32]);
		}
                   for(col=32;col<48;col++)
		{
			Setpage(0,page);
			Putchar0(bmp01[col%16+64]);
		}
                   for(col=48;col<64;col++)
		{
			Setpage(0,page);
			Putchar0(bmp01[col%16+96]);
		}
                
	}
	else
	{
		for(col=0;col<16;col++)
		{
			Setpage(0,page);
			Putchar0(bmp01[col%16+16]);
			
		}
                for(col=16;col<32;col++)
		{
			Setpage(0,page);
			Putchar0(bmp01[col%16+48]);
			
		}
                for(col=32;col<48;col++)
		{
			Setpage(0,page);
			Putchar0(bmp01[col%16+80]);
			
		}
                for(col=48;col<64;col++)
		{
			Setpage(0,page);
			Putchar0(bmp01[col%16+112]);
			
		}
	}
        }
}
/*********************************************************************
 * 函数名称：lcd右半屏下面显示4个汉字
 * 功    能：
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void show_right_down(unsigned char number)
{ 
  int page=0;
	int col=0;
	for(page=2;page<4;page++)
        {
	
	if(page==0||page==2)
	{        Setadderss(0,0);
		for(col=0;col<8;col++)
		{
                    Setpage(page,0);
                   Putchar1(ASCII[0*16+col%8]);
		} 
                Setadderss(8,0);
		for(col=0;col<8;col++)
		{
                    Setpage(page,0);
                   Putchar1(ASCII[0X10*16+col%8]);
		}
                Setadderss(16,0);
		for(col=0;col<8;col++)
		{
                    Setpage(page,0);
                  Putchar1(ASCII[(number>>4)*16+col%8]);
		}
                Setadderss(24,0);
                for(col=8;col<16;col++)
		{
                    Setpage(page,0);
                   Putchar1(ASCII[(number&0x0f)*16+col%8]);
		}
	}
        else
	{         Setadderss(0,0);
		for(col=0;col<8;col++)
		{
			Setpage(page,0);
			Putchar1(ASCII[0*16+col%8+8]);
			
		}
                Setadderss(8,0);
                for(col=8;col<16;col++)
		{
			Setpage(page,0);
			Putchar1(ASCII[0X10*16+col%8+8]);
			
		}
                 Setadderss(16,0);
		for(col=0;col<8;col++)
		{
			Setpage(page,0);
			Putchar1(ASCII[(number>>4)*16+col%8+8]);
			
		}
                Setadderss(24,0);
                for(col=8;col<16;col++)
		{
			Setpage(page,0);
			Putchar1(ASCII[(number&0x0f)*16+col%8+8]);
			
		}
        
        }
	
               
        }
    

}

