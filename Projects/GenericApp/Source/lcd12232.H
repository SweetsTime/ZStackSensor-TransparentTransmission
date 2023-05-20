/*******************************************************************************
 * 文件名称：LCD.h
 * 功    能：LCD驱动
 *           驱动12232点阵图形液晶
 * 硬件连接：液晶模块与CC2530的硬件连接关系如下：
 *                液晶模块                         CC2530
 *
 *               SPI-CLK(PIN2) 串行数据             P1.7
 *               SPI-MOSI(PIN3)移位寄存器时钟输入   P1.6
 *               GPH(PIN5)   存储寄存器时钟输入     P1.5
 * 作    者：w
 * 公    司：无锡泛太科技有限公司
 ******************************************************************************/

#ifndef LCD12232_H_
#define LCD12232_H_

/* 包含头文件 */
/********************************************************************/
#include "ioCC2530.h"

#define      SPIdata           P1_6
#define      SPImosi           P1_5
#define      GPH               P1_3

#define      H_SPImosi()       SPImosi = 1 
#define      L_SPImosi()       SPImosi = 0

#define      H_GPH()             GPH = 1
#define      L_GPH()             GPH = 0



#define NOP()  asm("NOP")

#define witch_MI 0x11   //左边写指令
#define witch_MD 0x19  //左边写数据
#define witch_SI 0x12  //右边写指令
#define witch_SD 0x1A  //右边写数据

void LCD_SPISendData(unsigned char Dat);

void LCD_SPIOUT(void);

void SendDataMI16(unsigned char MI,unsigned char NI);

void SendDataMD16(unsigned char MD,unsigned char ND);

void LCD_lni(void);

void Setpage(unsigned char page0,unsigned char page1);

void Setadderss(unsigned char adderss0,unsigned char adderss1);

void Putchar0(unsigned char ch);
                                                            
void Putchar1(unsigned char ch);

void LCD_clrscr(void);

void show_left_up(void);

void show_right_up(void);

void show_left_down(void);

extern  void show_right_down(unsigned char number);

#endif
 