/**************************************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $
  NLME_SetPollRate( 0 );
ZDApp_StopJoiningCycle();

  Description:    Generic Application (no Profile).
  Copyright 2004-2009 Texas Instruments Incorporated. All rights reserved.
  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for 
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/
 
/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 15 seconds.  The application will also
  receive "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "OSAL_Nv.h"
#include "LQ12864.h"
#include "delay.h"

#include "GenericApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */

#include "hal_adc.h"
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"  
#include "hal_uart.h"
#include "MT_UART.h"
#include "appr51.h"
#if((SENSOR_TYPE =='U')||(SENSOR_TYPE ==0X06))
#include "18B20.h"
#endif 
#if(SENSOR_TYPE =='E')
#include "SHT1x.h"  
#endif 
#if(SENSOR_TYPE ==0X85)
#include "MLX90615.h" 
  char data[9]={0x5A,0x5a,0x45,0x04,0x00,0x00,0xaa,0xaa,0x00};
#endif 
#if(SENSOR_TYPE ==0X08) 
#define TRF7970A   //如果 是TRF7970A打开此定义 。如果是TRF7960屏蔽此定义
#endif
#if(SENSOR_TYPE ==0X82)
#include "HMC5883.H"  
#define PI  3.14159265
#define M_PI  3.14159265
unsigned char T_MUMPT_y=0;    // 记录数值进行比较
unsigned char T_MUMPT_z=0;    // 记录数值进行比较
float HMC5883_x,HMC5883_y,HMC5883_z;
#endif 
#if(SENSOR_TYPE ==0X07)
unsigned char AB;
#endif
#if(SENSOR_TYPE ==0X14) 
#if defined(PH_E201C)
#include "18B20.h"
#endif 
#endif 

#if(ZDO_COORDINATOR==2)  //ZIGBEE  AT命令模式 透传模式
uint8 FT_AT=0;//0 命令模式//1 透传模式
#endif

#if defined (NB_IOT_S10)
  unsigned char NBliucheng=0;//流程标号
  unsigned int NBjishu=0;//计数周期
  unsigned char NB_IMEI[16]={0};//IMEI号
  unsigned char NB_S=0;//内容长度
#endif

#if(SENSOR_TYPE ==0X76)
 unsigned int lens=0;
 unsigned char p=0;
#endif
#if(SENSOR_TYPE ==0X65)
#include "lcd12232.h"
#endif 
#if(SENSOR_TYPE ==0X6B)
//#define LEDA8  //定义 A8广告机 屏蔽 led显示屏
//#define LED16X64  //定义led显示屏
#endif
#if(SENSOR_TYPE ==0X6C)
#define CS P1_7//片选信号
 #define SDA P1_6//数据信号
 #define SCLK P1_5//时钟信号 
#define code __code
#define com  0
#define dat  1
const unsigned char code tab0[]={
"  无锡泛太科技    公交管理系统  "}; 
const unsigned char code tab1[]={
"    路公交未驶出公交首末站, 请乘客耐心等待"};   
const unsigned char code tab2[]={
"    路公交已到达本站, 请乘客按次序上下车"};  
const unsigned char code tab3[]={
"    路公交离本站还有    站, 预计    分钟后到达"};

unsigned char str0[];
//unsigned char str1[]={"0123456789"};
const unsigned char code str1[]={
"0 1 2 3 4 5 6 7 8 9 " 
};
const unsigned char code str2[]="无锡公交首末站  ";
const unsigned char code str3[]="  无锡火车站    ";
const unsigned char code str4[]="    江南大学    ";
const unsigned char code str5[]="    蠡湖公园    ";

#endif 
#if(SENSOR_TYPE ==0X6F)
#define CS P1_7//片选信号
 #define SDA P1_6//数据信号
 #define SCLK P1_5//时钟信号 
#define code __code
#define com  0
#define dat  1
#endif 
#if(SENSOR_TYPE ==0X28)
#include "LCD_SPI.h"
#endif 
#if(SENSOR_TYPE ==0X33)
#include "B_LUX_V30.h"
#endif 
#if(SENSOR_TYPE ==0X83)
unsigned char PDI=0;
#endif  

#if(SENSOR_TYPE ==0X6A) 
#if defined(HX711P) 
#include <HX711.h>   
#endif 
#endif 
#if(SENSOR_TYPE ==0X79)
#include "SHT1x.h"  
unsigned char PDI=0;
unsigned char ERR_TEMP=0;
unsigned char T_TEMP=0;
unsigned char T_ADC=0;
#endif  
#if(SENSOR_TYPE ==0X66)
#include "LCD.h"
unsigned char Disp_Tab[10] ={0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90};
unsigned int LedNumVal=88;
unsigned char LEDtime=0;//LED显示时间记录
unsigned char LED_HL=0;//LED显示数码短
#endif 

#if(SENSOR_TYPE ==0X78)
unsigned char data_buf[9]={0};
unsigned char data_RPY=0;
//int ROLL=0,PITCH=0,YAW=0;
//int rpy[3]={0},Acc[3]={0},Gyr[3]={0},Mag[3]={0},Q[4]={0};

#endif  
#if(SENSOR_TYPE ==0X81)
 unsigned int max_byte ,bytee,t; 
#define SO P1_5 
#define CS P1_6 
#define SCK P1_7
#endif
#if(SENSOR_TYPE ==0X6E)
#include "LCD.h"
#endif 
#if(SENSOR_TYPE =='Q')
#include <HX711.h>
#include <HT1621B.h>
uchar  Ht1621Tab[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uchar  table1[]={0x0B,0x00,0x07,0x05,0x0C,0x0D,0x0F,0x00,0x0F,0x0D};//0123456789
uchar  table2[]={0x0E,0x06,0x0C,0x0E,0x06,0x0A,0x0A,0x0E,0x0E,0x0E};//0123456789
unsigned char  RS=0, RSn=0;
unsigned char  HXS=0,HX_Q=0;
unsigned int  HXtime=0;
unsigned char AD_SHINING=83;  //调节承重误差参数  50-150之间
#endif 
//定时3
/* 清除TIMER3中断标志位 */
/********************************************************************/
#define CLR_TIMER34_IF( bitMask ) TIMIF=(TIMIF&0x40)|(0x3F&(~bitMask))
/* 初始化定时器3宏定义 */
#define TIMER34_INIT(timer)   \
   do {                       \
      T##timer##CTL   = 0xEF; \
   } while (0)
// 定时器3使能
#define TIMER3_RUN(value)      (T3CTL = (value) ? T3CTL | 0x10 : T3CTL & ~0x10)
/********************************************************************/
#define NOP()  asm("NOP")
#if(SENSOR_TYPE =='B')
#include  "mma7660.h"
#endif 
#if(SENSOR_TYPE =='C')
#include "IIC.h"
#endif 
#if(SENSOR_TYPE =='F')
#if defined(BMP085) 
#include "IIC.h"
#endif 
#if defined(BMP180) 
#include "bmp180.h"
      
#endif 
#endif 



/*********************************************************************
 * MACROS
 */
#if(SENSOR_TYPE =='B')
#define uchar unsigned char
#define uint unsigned int
int Xavg8,Yavg8,Zavg8;//存放平均值
int Xraw[RawDataLength],Yraw[RawDataLength],Zraw[RawDataLength]; //8个空间的数组，采集8个数取一次平均
int Xnew8,Ynew8,Znew8;//新采集到的数据
int RawDataPointer = 0;
#endif  
#if(SENSOR_TYPE =='C')
#define _BH1710FVC_DEF_
#define uchar unsigned char
#define uint unsigned int
//1710指令定义
#define power_down 0x00
#define power_on 0x01
#define reset 0x07
#define h_resolution_mod 0x10
#define m_resolution_mod 0x13
#define l_resolution_mod 0x16
#define onetime_h 0x20
#define onetime_m 0x23
#define onetime_l 0x26
#define write_1710 0x46
#define read_1710 0x47
#endif
#if((SENSOR_TYPE ==0X24)||(SENSOR_TYPE ==0X80))
uint8 CSB_Len_H;
uint8 CSB_Len_L;
#endif
#if(SENSOR_TYPE ==0X40)
uint8 LED_START=0;
#endif
#if(SENSOR_TYPE =='G')
#define A1 P1_0	//定义A线圈正端口
#define A2 P1_1	//定义A线圈负端口
#define B1 P1_2	//定义B线圈正端口
#define B2 P1_3	//定义B线圈负端口
#define Coil_A1   {A1=0;A2=1;B1=1;B2=1;}  //A线圈通正向电压
#define Coil_B1   {A1=1;A2=1;B1=0;B2=1;}	//B线圈通正向电压
#define Coil_A2   {A1=1;A2=0;B1=1;B2=1;}  //A线圈通反向电压
#define Coil_B2   {A1=1;A2=1;B1=1;B2=0;}	//B线圈通反向电压
#define Coil_OFF  {A1=1;A2=1;B1=1;B2=1;}  //全部断电 
uint8 Coil_AB=0;
#endif
#if(SENSOR_TYPE ==0X67)
#define  NSYNC P1_7 //dac7512 使能端
#define  SCLK P1_6 //dac7512 时钟端
#define  DIN P1_5  //dac7512 数据端
#endif
#if(SENSOR_TYPE ==0X30)
#define  NSYNC P1_7 //dac7512 使能端
#define  SCLK P1_6 //dac7512 时钟端
#define  DIN P1_5  //dac7512 数据端

#define B01 0x0D//(1 =14.8度)电机角度设定 0X07 42BYGH408AA(1=7.2)设定0X0D  默认 0x0D
#define A1 P1_0	//定义A线圈正端口
#define A2 P1_1	//定义A线圈负端口
#define B1 P1_2	//定义B线圈正端口
#define B2 P1_3	//定义B线圈负端口
#define Coil_A1   {A1=0;A2=1;B1=1;B2=1;}  //A线圈通正向电压
#define Coil_B1   {A1=1;A2=1;B1=0;B2=1;}	//B线圈通正向电压
#define Coil_A2   {A1=1;A2=0;B1=1;B2=1;}  //A线圈通反向电压
#define Coil_B2   {A1=1;A2=1;B1=1;B2=0;}	//B线圈通反向电压
#define Coil_OFF  {A1=1;A2=1;B1=1;B2=1;}  //全部断电 
uint8 a68=0,b68=0,c68=0; //a68方向 b68角度 c68 时间
#endif
#if(SENSOR_TYPE ==0X73)
#define DIR P1_7
#define CP P1_6
uint8 a68=0;
int b68=0;
uint8 C68=0,A8n=0;
#endif
#if(SENSOR_TYPE =='D')
#define S0 P1_6
#define S1 P1_7
#define OE	P1_5	
#define L2      P1_1
#define L3	P1_2
#define OUT P1_0
#define uchar unsigned char
#define uint unsigned int
volatile uint times = 0;	//脉冲计数
float OUTR,OUTG,OUTB;
#endif
#if(SENSOR_TYPE =='F')
#if defined(BMP085) 
#define XCLR P1_5 
#define XCLROUT P1DIR|=0x20       //数据输出 
#define uchar unsigned char
#define uint unsigned int
//变量声明
//BMP085中E2PROM存储的标定参数
int AC1=0;   				//(0xAA,0xAB)
int AC2=0;  				//(0xAC,0xAD)
int AC3=0;  				//(0xAE,0xAF)
unsigned int AC4=0;				//(0xB0,0xB1)
unsigned int AC5=0;				//(0xB2,0xB3)
unsigned int AC6=0;				//(0xB4,0xB5)
int B1=0;				//(0xB6,0xB7)
int B2=0;				//(0xB8,0xB9)
int MB=0;				//(0xBA,0xBB)
int MC=0;				//(0xBC,0xBD)
int MD=0;				//(0xBE,0xBF)
unsigned long T=0;				//温度
unsigned long p=0;				//压力
#endif

#endif
   //标志 长度 目地地址  原地地址  类型 数值 数值 
uint8 Send_data[0X80] = {0XFA,0X0A,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00};
uint8 Send_datalend=0; //内容+校验和长度
uint8 AF_OK=0;//无线数据发送成功与否标志  1成功 0失败
uint8 AF_RF=0;//无线数据发送失败，重发标志
uint8 AF_RFn=0;//重发次数
uint8 AF_RFnd=0;//重发组数
uint8 AF_RFnds=0;//重发顺序号 按顺序发送
#if defined( SENSOR_TYPE_Coord )
uint8  chnnd[5][120];//数据缓存
#endif

#if((SENSOR_TYPE ==0X31)||(SENSOR_TYPE ==0XA3))
uint8  chnndr[120];//数据缓存
#endif

unsigned int GenericApp_applicationdata=0;
unsigned char GenericApp_time=0;
unsigned int  GenericApp_applicationbuf=0;
unsigned int GenericApp_uart_data=0;
unsigned char  GenericApp_CR;
uint8  openoff=0;
uint8  PWRMGR_ALWAYS_ON_OFF=0;
unsigned char OSAL_SET_CPU_INTO_SLEEP=0;
unsigned int GenericApp_sbum=0;
unsigned int GenericApp_KSH_H=0;
unsigned int GenericApp_KSH=0;    //86盒 按键计数
unsigned int GenericApp_KSH86=0;  //86盒 按键速度计数
//**********************************************************88
unsigned long T3_t1Val=100;//定时3 时间计数设置 ms
unsigned long t1Val=0;  //定时3 计数变量
unsigned char T_MG=0;    // 控制设备 状态信息变量
uint8 T_MGSbit=0;    // 状态信息 数据信息  连数据信息记录
//关闭定时  主动   控制类  综合型  
unsigned char  GenericApp_appIO=0;
unsigned char GenericApp_BX=0;
unsigned int GenericApp_ON_OFF=0;
unsigned int GenericApp_switch=0;//开关量定时反馈
unsigned int GenericApp_switchk2=0;//开关量定时反馈
#define LMOUT P2_0
/*********************************************************************
 * CONSTANTS
 */
unsigned char T_MUMPT1=0;    // 记录数值进行比较
unsigned int T_MUMPTS=0;    //   近似真实数据转换
#if(SENSOR_TYPE ==0X22)
unsigned char  T_MUMPTZ1=0;//记录最大数值 
#endif 
#if((SENSOR_TYPE ==0X18)||(SENSOR_TYPE ==0X5A)||(SENSOR_TYPE ==0X5B)||(SENSOR_TYPE ==0X70))
unsigned char  GenericApp_uart_485=0;
#endif 
#if(SENSOR_TYPE ==0X23)
unsigned char  SFG_R30X=0;
unsigned char  SFG_R30XA[3]={0};
#endif 
#if(SENSOR_TYPE ==0X31)
uint8 Uart_Rev_Buff_1[6]={0};
uint8 Uart_Rev_Buff_2[6]={0};
uint8 car_stata;
uint8 car_op;//读卡计数
uint8 car_sum;
uint8 holzer=0;
#endif
/*********************************************************************
 * TYPEDEFS
 */
/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID
};
const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.
devStates_t GenericApp_NwkState;
byte GenericApp_TransID;  // This is the unique message ID (counter)
afAddrType_t GenericApp_DstAddr;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
//void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void GenericApp_HandleKeys( byte shift, byte keys );
void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void GenericApp_SendTheMessage( void );
void GenericApp_SendFT( void );
void GenericAppEndDeviceEB(void);

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
#if(SENSOR_TYPE =='B')
void MMA7660_XYZ_Read_and_Filter(void);
void MMA7660_XYZ_Read_and_Filter(void)
{
  int i;
  int temp16;
  IIC_Read_MMA7660_XYZ6(&Xnew8, &Ynew8, &Znew8);
  if((++RawDataPointer)>=RawDataLength) RawDataPointer = 0;
  Xraw[RawDataPointer] = Xnew8;
  Yraw[RawDataPointer] = Ynew8;
  Zraw[RawDataPointer] = Znew8;
  for(i=0, temp16=0;i<RawDataLength;i++)
  {
    temp16 += (int)Xraw[i];
  }
  Xavg8 = (int)(temp16/RawDataLength);
  for(i=0, temp16=0;i<RawDataLength;i++)
  {
    temp16 += (int)Yraw[i];
  }
  Yavg8 = (int)(temp16/RawDataLength);
  for(i=0, temp16=0;i<RawDataLength;i++)
  {
    temp16 += (int)Zraw[i];
  }
  Zavg8 = (int)(temp16/RawDataLength);
}
#endif  
#if((SENSOR_TYPE ==0X6C)||(SENSOR_TYPE ==0X6F))
void wr_lcd(unsigned char dat_com,unsigned char content);
void delay_us(unsigned int us_value);
void delay_ms(unsigned int ms_value);
void init_lcd (void);
void txt_disp(const unsigned char code *cn,unsigned char adr,unsigned char len);
void lcd_disp(unsigned char *cn,unsigned char adr,unsigned char len);
void hzkdis(unsigned char adr,const unsigned char code *s);
void hzkdis_b(unsigned char adr,unsigned char *s,unsigned char len);
void lcd_txt_clr(void);
void lcd_gra_clr(void);
void sendbyte(unsigned char zdata);
/******************************************************************************************
*函数名称： init_lcd
*功能描述： 初始化
******************************************************************************************/
void init_lcd (void)
{
 wr_lcd(com,0x30);  /*30---功能设定,8-bit模式*/   
 wr_lcd(com,0x30);  /*30---功能设定,基本指令*/  
 wr_lcd(com,0x01);  /*清屏，地址指针指向00H*/
 wr_lcd(com,0x06);  /*光标的移动方向*/
 wr_lcd(com,0x0c);  /*开显示，关游标*/
}

/********************************************************************
* 名称 : hzkdis()
* 功能 : 显示字符串
* 输入 : *s
* 输出 : 无
***********************************************************************/
void hzkdis(unsigned char adr,const unsigned char code *s)
{
 wr_lcd(com,0x30);
//delay_ms(50); 
//wr_lcd(com,0x80);
 int i=0;char x[4]={0};
switch(adr)
{
case 0:x[0]=0x80;x[1]=0x90;x[2]=0x88;x[3]=0x98;break; 
case 1:x[0]=0x90;x[1]=0x88;x[2]=0x98;break; 
case 2:x[0]=0x88;x[1]=0x98;break; 
case 3:x[0]=0x98;break; 
default:break;
}
while(*s > 0)
{ switch(i)
{case 0:wr_lcd(com,x[0]);break; 
case 16:wr_lcd(com,x[1]);break;
case 32:wr_lcd(com,x[2]);break;
case 48:wr_lcd(com,x[3]);break;
default:break;
}
wr_lcd(dat,*s);
s++;
i++;
delay_ms(50);
}
}
/********************************************************************
* 名称 : hzkdis-b()
* 功能 : 显示字符串
* 输入 : *s
* 输出 : 无
***********************************************************************/
void hzkdis_b(unsigned char adr,unsigned char *s,unsigned char len)
{
 wr_lcd(com,0x30);
//delay_ms(50); 
//wr_lcd(com,0x80);
 int i=0;char x[4]={0};
switch(adr)
{
case 0:x[0]=0x80;x[1]=0x90;x[2]=0x88;x[3]=0x98;break; 
case 1:x[0]=0x90;x[1]=0x88;x[2]=0x98;break; 
case 2:x[0]=0x88;x[1]=0x98;break; 
case 3:x[0]=0x98;break; 
default:break;
}
while(i < len)
{ switch(i)
{case 0:wr_lcd(com,x[0]);break; 
case 16:wr_lcd(com,x[1]);break;
case 32:wr_lcd(com,x[2]);break;
case 48:wr_lcd(com,x[3]);break;
default:break;
}
wr_lcd(dat,*s);
s++;
i++;
delay_ms(50);
}
}
/******************************************************************************************
*函数名称： lcd_disp
*功能描述： 显示一行汉字或字符
******************************************************************************************/
void lcd_disp(unsigned char *cn,unsigned char adr,unsigned char len)
{
 unsigned char i;
 wr_lcd(com,0x30);
 wr_lcd(com,adr);   //第一到四行地址分别是0x80,0x90,0x88,0x98
 for(i=0;i<len;i++)
  wr_lcd(dat,cn[i]);
}
/******************************************************************************************
*函数名称： txt_disp
*功能描述： 显示一行汉字或字符
******************************************************************************************/
void txt_disp(const unsigned char code *cn,unsigned char adr,unsigned char len)
{
 unsigned char i;
 wr_lcd(com,0x30);
 wr_lcd(com,adr);   //第一到四行地址分别是0x80,0x90,0x88,0x98
 for(i=0;i<len;i++)
  wr_lcd(dat,cn[i]);
}
/********************************************************************
* 名称 : sendbyte()
* 功能 : 按照液晶的串口通信协议，发送数据
* 输入 : zdata
* 输出 : 无
***********************************************************************/
void sendbyte(unsigned char zdata)
{
unsigned int i;
for(i=0; i<8; i++)
{
if((zdata << i) & 0x80)
{
SDA = 1;
}
else 
{
SDA = 0;
}
SCLK = 0;
SCLK = 1;
}
}
/******************************************************************************************
*函数名称：wr_lcd
*功能描述：写入指令/数据子程序 
******************************************************************************************/
void wr_lcd (unsigned char dat_comm,unsigned char content)
{CS = 1;
if(dat_comm)
sendbyte(0xfa);
else sendbyte(0xf8);
sendbyte(content & 0xf0);
sendbyte((content << 4) & 0xf0);
delay_us(2);
}

/******************************************************************************************
*函数名称：delay_us
*功能描述：延时us 
******************************************************************************************/
void delay_us(unsigned int t)   //delay time
{
 unsigned int i,j;
for(i=0; i<t; i++)
for(j=0; j<10; j++);
}
/******************************************************************************************
*函数名称：delay_ms
*功能描述：延时ms
******************************************************************************************/
void delay_ms(unsigned int ms_value)
{
 unsigned int i,j;
 for(i=0;i<ms_value;i++)
  for(j=0;j<20;j++)
   delay_us(1);
}

/******************************************************************************************
*函数名称： lcd_txt_clr
*功能描述： 文本区清RAM函数
******************************************************************************************/
void lcd_txt_clr(void)
{
 wr_lcd(com,0x30);
 wr_lcd(com,0x01);
 delay_ms(20);
}
/******************************************************************************************
*函数名称： lcd_gra_clr
*功能描述： 图形区清RAM函数
******************************************************************************************/
void lcd_gra_clr(void)
{
 unsigned char i,j;
 wr_lcd(com,0x34);      //8Bit扩充指令集,即使是36H也要写两次
 for(j=0;j<32;j++)
  {
   for(i=0;i<16;i++)
    {
     wr_lcd(com,0x34);
     wr_lcd(com,0x80+j);
     wr_lcd(com,0x80+i);
     wr_lcd(com,0x30);
     wr_lcd(dat,0);
     wr_lcd(dat,0);
    }
  }
}
#endif 
#if(SENSOR_TYPE ==0X78)
void send_Instruction(void);
void send_Instruction(void)
{halMcuWaitMs(1000);
	unsigned  char send_data[3]={0};
	send_data[0]=0xa5;
	send_data[1]=0x15;//加计功能字节
	send_data[2]=(unsigned char)(send_data[0]+send_data[1]);//指令校验和
         HalUARTWrite(HAL_UART_PORT_0, send_data,3);//发送加速度输出指令
	halMcuWaitMs(10);
	send_data[0]=0xa5;
	send_data[1]=0x25;
	send_data[2]=(unsigned char)(send_data[0]+send_data[1]);
	 HalUARTWrite(HAL_UART_PORT_0,send_data,3);//发送陀螺仪数据输出指令
	halMcuWaitMs(10);
	send_data[0]=0xa5;
	send_data[1]=0x35;
	send_data[2]=(unsigned char)(send_data[0]+send_data[1]);
	 HalUARTWrite(HAL_UART_PORT_0,send_data,3);//发送磁场数据输出指令
	halMcuWaitMs(10);
	send_data[0]=0xa5;
	send_data[1]=0x45;
	send_data[2]=(unsigned char)(send_data[0]+send_data[1]);
	 HalUARTWrite(HAL_UART_PORT_0,send_data,3);//发送欧拉角数据输出指令
	halMcuWaitMs(10);
	send_data[0]=0xa5;
	send_data[1]=0x65;
	send_data[2]=(unsigned char)(send_data[0]+send_data[1]);
	 HalUARTWrite(HAL_UART_PORT_0,send_data,3);//发送四元数数据输出指令
}

#endif 
#if(SENSOR_TYPE ==0X81)
unsigned int read_max6675(void);
unsigned int read_max6675(void)    
  { max_byte=0;   
       unsigned char i=0;    
      CS=0;   
      SCK=0;        
        for(i=0;i<16;i++)    
         {    
          halMcuWaitMs(1);	  
          SCK=1;     
          halMcuWaitMs(1);	   
          if(SO)     
           max_byte|=(1<<(15-i));   
          SCK=0;    
         }   
        CS=1;   
        SCK=0;    
        bytee=max_byte;   
        if(bytee&0x8000)max_byte=0x8000;   //错误  D15 模拟位   0正常 1 模拟位
        else if(bytee&0x4)max_byte=0x8001;   //错误 D2 电偶开路位检测 0 连接 1 开路
        else   
         {   
           max_byte=max_byte&0x7ff8;   
           max_byte=max_byte>>3;   
          }   
       return max_byte;    



  }    
#endif  
#if(SENSOR_TYPE =='F')
#if defined(BMP085) 
void BMP085_init(void);
void BMP085_sample(uchar oss);
//====================================================
//压力传感器驱动函数 （BMP085）
//====================================================
void BMP085_init(void)//读取BMP085中E2PROM的参数，并赋给全局变量
{		
	char memo[22];
	uchar i;
	//void *pf;
	//从EEPROM中读取预设参数
	for(i=0;i<22;i++){memo[i]=Single_Read_(0xAA+i);}
        AC1=(memo[0]<<8)+memo[1];
	AC2=(memo[2]<<8)+memo[3];	
        AC3=(memo[4]<<8)+memo[5];	
        AC4=(memo[6]<<8)+memo[7];	
        AC5=(memo[8]<<8)+memo[9];	
        AC6=(memo[10]<<8)+memo[11];	
        B1=(memo[12]<<8)+memo[13];	
        B2=(memo[14]<<8)+memo[15];	
        MB=(memo[16]<<8)+memo[17];	
        MC=(memo[18]<<8)+memo[19];	
        MD=(memo[20]<<8)+memo[21];	
        //AC1=408;
       /*  AC2=-72;
         AC3=-14383;
        AC4=32741;
        AC5=32757;
        AC6=23153;
        B1=6190;
        B2=4;
        MB=-32768;
        MC=-8711;
        MD=2868; 
        */
}
void BMP085_sample(uchar oss)//温度、压力采样
//oss为工作方式：
//0-ultra low power;
//1-standard;
//2-high resolution;
//3-ultra high resolution
{
	 long UT=0;			//温度原始值	
	 long UP=0;			//压力原始值
	 long X1,X2,X3;
	long B3,B5,B6;
	unsigned long B4,B7;
	char i,BMPmemo[3];
	//void *pf;
	Single_Write_(0xF4,0x2E);	//发命令采温度
	mDelay(100); //延时						//等待AD，延迟4.5ms以上
	for(i=0;i<2;i++){BMPmemo[i]=Single_Read_(0xF6+i);}//读取结果	
        UT=BMPmemo[0]*256+BMPmemo[1];
       // UT=UT-5000;  //误差补偿（根据实际测试）
	Single_Write_(0xF4,0x34+(oss<<6));//发命令采压力
	mDelay(200); //延时								//延迟时间视工作方式而定，具体查手册
	for(i=0;i<3;i++){BMPmemo[i]=Single_Read_(0xF6+i);}//读取结果	
        
        UP=BMPmemo[0]*65536+BMPmemo[1]*256+BMPmemo[2];
	UP=(UP>>(8-oss));
        
        X1=(UT-AC6)*AC5/32768; 		//计算温度
	X2=MC;
	X2=X2*2048/(X1+MD);
	B5=X1+X2;
	T=(B5+8)/16;
	B6=B5-4000;			//计算压力
	X1=B2;
	X1=(X1*(B6*B6/4096))/2048;
	X2=AC2;
	X2=X2*B6/2048;
	X3=X1+X2;
	B3=AC1;
	B3=(((B3*4+X3)<<oss)+2)/4;	
	X1=AC3;
	X1=X1*B6/8192;
	X2=B1;
	X2=(X2*(B6*B6/4096))/65536;
	X3=((X1+X2)+2)/4;
	B4=AC4;
	B4=B4*(unsigned long)(X3+32768)/32768;
	B7=((unsigned long)UP-B3)*(50000>>oss);	
	if(B7<0x80000000){p=(B7*2)/B4;}
	else {p=(B7/B4)*2;}
	X1=(p/256)*(p/256); 
	X1=(X1*3038)/65536;        
	X2=(-7357*p)/65536; 
	p=p+(X1+X2+3791)/16;  
}
#endif 
#endif  

#if(SENSOR_TYPE =='D')
float TCS3200(uchar t);
//===================TCS3200测颜色=========================
float TCS3200(uchar t)
{       float OUTS;
        uint8 n=1;
        switch(t)
       { case 1:L2=0;L3=0;break;//R
         case 2:L2=1;L3=1;break;//G
         case 3:L2=0;L3=1;break;//B
         default:L2=1;L3=0;break;//无
        }
	for(unsigned int i=0;i<100;i++);
	times = 0;
	OE=0;//检测一次
	for(unsigned int i=0;i<50000;i++)
          {if(n)
            {if(OUT){times++;n=0;}}
             if(!n){if(!OUT)n=1;}
            }
	OE=1;
	OUTS= times;
	times = 0;
	OE=0;			//两次
	for(unsigned int i=0;i<50000;i++)
          {if(n)
            {if(OUT){times++;n=0;}}
             if(!n){if(!OUT)n=1;}
            }
	OE=1;	
	OUTS += times;
	times = 0;
	OE=0;			//三次
	for(unsigned int i=0;i<50000;i++)
          {if(n)
            {if(OUT){times++;n=0;}}
             if(!n){if(!OUT)n=1;}
            }
	OE=1;	
	OUTS += times;
	times = 0;
	OE=0;				//四次
	for(unsigned int i=0;i<50000;i++)
          {if(n)
            {if(OUT){times++;n=0;}}
             if(!n){if(!OUT)n=1;}
            }
	OE=1;	
	OUTS += times;
	OUTS/=4.0;					//取四次的平均值
	return OUTS;
}
#endif    
#if((SENSOR_TYPE ==0X27)||(SENSOR_TYPE ==0X6F))
void SYN_FrameInfo(uint8 Music,uint8 GBK,uint8 *HZdata,uint8 lens);
/***********************************************************
* 名    称：  YS-SYN6288 文本合成函数
* 功    能：  发送合成文本到SYN6288芯片进行合成播放
* 入口参数：Music(背景音乐选择):0无背景音乐。1-15：相关背景音乐
            GBK : 汉字编码格式  0: GB2312  1: GBK  2:BIG5  3 :UNICODE
            *HZdata:文本指针变量 
* 出口参数：
* 说    明： 本函数只用于文本合成，具备背景音乐选择。默认波特率9600bps。					 
**********************************************************/
void SYN_FrameInfo(uint8 Music,uint8 GBK,uint8 *HZdata,uint8 lens)
{
/****************需要发送的文本**********************************/ 
		 unsigned  char  Frame_Info[85];
         unsigned  char  HZ_Length;  
		 unsigned  char  ecc  = 0;  			//定义校验字节
	     unsigned  int i=0; 
		 HZ_Length =strlen(HZdata); 			//需要发送文本的长度
                 if(HZ_Length>lens)
                   HZ_Length =lens; 
 
/*****************帧固定配置信息**************************************/           
		 Frame_Info[0] = 0xFD ; 			//构造帧头FD
		 Frame_Info[1] = 0x00 ; 			//构造数据区长度的高字节
		 Frame_Info[2] = HZ_Length + 3; 		//构造数据区长度的低字节
		 Frame_Info[3] = 0x01 ; 			//构造命令字：合成播放命令		 		 
		 Frame_Info[4] =( 0x00 | Music<<4 )|GBK;  //构造命令参数：背景音乐设定

/*******************校验码计算***************************************/		 
		 for(i = 0; i<5; i++)   				//依次发送构造好的5个帧头字节
	     {  
	         ecc=ecc^(Frame_Info[i]);		//对发送的字节进行异或校验	
	     }

	   	 for(i= 0; i<HZ_Length; i++)   		//依次发送待合成的文本数据
	     {  
	         ecc=ecc^(HZdata[i]); 				//对发送的字节进行异或校验		
	     }		 
/*******************发送帧信息***************************************/		  
		  memcpy(&Frame_Info[5], HZdata, HZ_Length);
		  Frame_Info[5+HZ_Length]=ecc;
		//  PrintCom(Frame_Info,5+HZ_Length+1);
                  HalUARTWrite(HAL_UART_PORT_0,Frame_Info,5+HZ_Length+1);//智能语言播报
}
#endif    
#if(SENSOR_TYPE ==0X66)
void display(void);
void display()			   //显示子程序
{     
	unsigned int LedOut[2];
        LedOut[0]=Disp_Tab[LedNumVal%100/10];
	LedOut[1]=Disp_Tab[LedNumVal%100%10];
          SendData(LedOut[LED_HL]);
	  switch(LED_HL)					  
	     {	case 0:DS1=1;DS2=0;break; 
		case 1:DS2=1;DS1=0;break;
	    }
}
#endif 
#if((SENSOR_TYPE ==0X30)||(SENSOR_TYPE ==0X67))
void DAchange(unsigned int dat);
//da驱动程序，入口参数dat,必须小于4096！4096对应5v电压！
void DAchange(unsigned int dat)
{
unsigned char DA,i,j,DAL,DAH;
 NSYNC=1;
if(dat>4095)dat=0;
else
{DAL=dat%256;
  DAH=dat/256;
    NSYNC=0;
    i=0;
        while(i<2)
        {if(i<1)DA=DAH;else DA=DAL;
        for(j=8;j>0;j--)
        {
            if(DA&0x80)DIN=1;else DIN=0;
            DA=(DA<<1);SCLK=1;SCLK=0;
         }
        i=i+1;
        }
    NSYNC=1;
    }
}
#endif
#if(SENSOR_TYPE ==0X13) 
#if defined(HUABANG) 
float ieee754(void);
float ieee754(void)
{
  typedef union                                        
{
   float Float_buf;
   unsigned char  unchar_Buf[4];
}un_DtformConver;

un_DtformConver   DtformConver;

unsigned char i;
long long_buf;
long long_bufS;
float Float_buf;
	
long_buf=0;
long_bufS=Send_data[8];
long_buf=long_bufS<<24;
long_bufS=Send_data[9];
long_buf+=long_bufS<<16;
long_bufS=Send_data[10];
long_buf+=long_bufS<<8;
long_buf+=Send_data[11];
 //long_bufS=long_buf(((long)Send_data[8]<<24)+((long)Send_data[9]<<16)+((long)Send_data[10]<<8)+Send_data[11]);
 
 memset((unsigned char *)&DtformConver.unchar_Buf[0],0,4);

  for(i=0;i<4;i++)
 {
    DtformConver.unchar_Buf[i] = (unsigned char )(long_buf>>(i*8));
 }
 Float_buf = DtformConver.Float_buf;
 
  return Float_buf;
}
#endif
#endif
#if(SENSOR_TYPE =='S')
void DimmingAppControl(unsigned char cmd,unsigned char ch);
void DimmingAppControl(unsigned char cmd,unsigned char ch)
{
	unsigned char i,dl,dh;			
	unsigned int datas=0;
	if(cmd<0X76)
	{if(cmd==0)
          dh=cmd;
         else
          dh = 0X76-cmd;
	if(ch == 1)
	dh |= 0x80;	
	dl = ~dh;
	datas  = dl;
       //datas=0xaa;
	datas |= dh<<8;					//最终要发送的数据为16位
							//高8位命令 低8位取反效验
	P1_2 = 0;					//拉低片选
	halMcuWaitUs(50);
	for(i=0;i<16;i++)
	{
		P1_1 = 0;
		halMcuWaitUs(10);					//这里大约是100us
		if(datas & 0x8000)	P1_0 = 1;
		else 			P1_0 = 0;
		halMcuWaitUs(90);
		P1_1 = 1;			
		halMcuWaitUs(100);					//这里大约是100us
		datas <<= 1;
	}
	P1_0 = 1;
	P1_2 = 1;
	P1_1 = 1;
        }
}
#endif 


//定时3启动或关闭
#define BOOL int
void halSetTimer3Period(uint16 period);
__interrupt void T3_IRQ(void);

//设置定时器1的定时周期
/*********************************************************************
 * 函数名称：halSetTimer3Period
 * 功    能：设置定时器3定时周期
 * 入口参数：period   定时周期
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void halSetTimer3Period(uint16 period)
{
  /* 给T3CC0写入最终计数值period */
  T3CC0 = period & 0xFF;             // 把period值写入T3CC0
}
/*********************************************************************
 * 函数名称：T3_IRQ
 * 功    能：定时器3中断服务函数
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
#pragma vector=T3_VECTOR
__interrupt void T3_IRQ(void)
{

    EA=FALSE;
   if(TIMIF & 0x01)
    { t1Val++;
    //if(t1Val>=12500)  //100ms
    if(t1Val>=(125*T3_t1Val))  //T3_t1Val ms    
    { t1Val=0;
      GenericApp_applicationbuf=60001;
      GenericApp_time=0;
    }
    }
     EA=TRUE;
 TIMIF &= ~0x01;
 }
/*********************************************************************
 * @fn      GenericApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */

void GenericApp_Init( byte task_id )
{
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;
  openoff=0xaa;
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().
  GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;;
  GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;;
  GenericApp_DstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR_DEVALL;
#if defined( POWER_SAVING )
  GenericApp_DstAddr.addr.shortAddr =NWK_BROADCAST_SHORTADDR_AABB;
#endif
  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;
  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );
  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );
#if defined( ZigBee_C_R_E_Transparent )
#if !defined( SENSOR_TYPE_Coord )
#if defined( SENSOR_TYPE_R_E )
         Send_data[4]= SENSOR_TYPE_R_E;
         Send_data[5]= SENSOR_TYPE;
#endif
#endif
#endif
#if defined( SENSOR_TYPE_Coord )
#if (ZDO_COORDINATOR)
#if(ZDO_COORDINATOR==2)  //ZIGBEE  AT命令模式 透传模式
      uint16 nv_data,nv_data1;
    osal_nv_read(ZCD_NV_APP_PANID_H,0,sizeof(nv_data),&nv_data);
    osal_nv_read(ZCD_NV_APP_PANID_L,0,sizeof(nv_data1),&nv_data1);
    if((nv_data!=0)&&(zgConfigPANID==0XFFFF)&&(nv_data1!=0)&&(nv_data1==nv_data))
    {zgConfigPANID=nv_data;
   //  HalUARTWrite(HAL_UART_PORT_0,"AAAAA",5);
    }
#else
     ZigBee_C_R_E_Engineering_G();
#endif 
#endif 
#endif  
Send_data[0]=0XFA;
#if defined( SENSOR_TYPE_Coord )
Send_data[0]=0XFD;
#endif
#if defined( SENSOR_TYPE_POWER )
Send_data[0]=0XFD;
#if defined( CC2530_V30 )
 Send_data[0]=0XFE;
#endif
#endif
T_MGSbit=0x0A;//00001010可控类 综合型 

#if defined (NB_IOT_S10)
  NBliucheng=0;
#endif
  
#if((SENSOR_TYPE =='E')||(SENSOR_TYPE =='B')||(SENSOR_TYPE =='C')||(SENSOR_TYPE =='D')\
  ||(SENSOR_TYPE =='F')||(SENSOR_TYPE =='U')||(SENSOR_TYPE ==0X10)||(SENSOR_TYPE ==0X21)\
    ||(SENSOR_TYPE ==0X11)||(SENSOR_TYPE ==0X1A)||(SENSOR_TYPE ==0X0F)||(SENSOR_TYPE ==0X13)||(SENSOR_TYPE ==0X14)\
      ||(SENSOR_TYPE ==0X24)||(SENSOR_TYPE ==0X6A)||(SENSOR_TYPE =='A')||(SENSOR_TYPE =='J')||\
        (SENSOR_TYPE =='L')||(SENSOR_TYPE =='M')||(SENSOR_TYPE ==0X33)\
  ||(SENSOR_TYPE =='N')||(SENSOR_TYPE ==0X04)||(SENSOR_TYPE ==0X22)||(SENSOR_TYPE ==0X0C)\
    ||(SENSOR_TYPE ==0X16)||(SENSOR_TYPE ==0X19)||(SENSOR_TYPE ==0X61)||(SENSOR_TYPE ==0X62)||(SENSOR_TYPE ==0X63)\
      ||(SENSOR_TYPE ==0X5A)||(SENSOR_TYPE ==0X5B)||(SENSOR_TYPE ==0X72) ||(SENSOR_TYPE ==0X34)||(SENSOR_TYPE ==0X35)||(SENSOR_TYPE ==0X36)\
        ||(SENSOR_TYPE ==0X37)||(SENSOR_TYPE ==0X38)||(SENSOR_TYPE ==0X39)||(SENSOR_TYPE ==0X3A)||(SENSOR_TYPE ==0X80)\
          ||(SENSOR_TYPE ==0X3D)||(SENSOR_TYPE ==0X81)||(SENSOR_TYPE ==0X82)||(SENSOR_TYPE ==0X3B)||(SENSOR_TYPE ==0X3C)\
            ||(SENSOR_TYPE ==0X3E)||(SENSOR_TYPE ==0X3F)||(SENSOR_TYPE ==0X74)||(SENSOR_TYPE ==0X84)||(SENSOR_TYPE ==0X85)\
             ||(SENSOR_TYPE ==0X75)||(SENSOR_TYPE ==0X78) )
      GenericApp_applicationbuf=60001;
      GenericApp_time=1;
#if defined( POWER_SAVING )
      GenericApp_time=25;
#endif
#endif

  
#if((SENSOR_TYPE =='A')||(SENSOR_TYPE =='J')||(SENSOR_TYPE =='L')||(SENSOR_TYPE =='M')||\
  (SENSOR_TYPE =='N')||(SENSOR_TYPE ==0X04)||(SENSOR_TYPE ==0X0C)||(SENSOR_TYPE ==0X19)\
    ||(SENSOR_TYPE ==0X22)||(SENSOR_TYPE ==0X61)||(SENSOR_TYPE ==0X62)||(SENSOR_TYPE ==0X63)||(SENSOR_TYPE ==0X72)\
      ||(SENSOR_TYPE ==0X34)||(SENSOR_TYPE ==0X35)||(SENSOR_TYPE ==0X36)||(SENSOR_TYPE ==0X37)||(SENSOR_TYPE ==0X38)\
        ||(SENSOR_TYPE ==0X39)||(SENSOR_TYPE ==0X3A)||(SENSOR_TYPE ==0X3E)||(SENSOR_TYPE ==0X3F)||(SENSOR_TYPE ==0X75)) 
T_MGSbit=0x12;    // 00010010状态信息 数据信息  连数据信息记录
//采集类  BOOL型
#endif  
#if((SENSOR_TYPE =='B')||(SENSOR_TYPE =='C')||(SENSOR_TYPE =='D')||(SENSOR_TYPE =='G')||\
  (SENSOR_TYPE =='I')||(SENSOR_TYPE ==0X77)||(SENSOR_TYPE =='Q')||(SENSOR_TYPE =='T')||(SENSOR_TYPE ==0X03)||(SENSOR_TYPE ==0X07)\
    ||(SENSOR_TYPE ==0X0F)||(SENSOR_TYPE ==0X10)||(SENSOR_TYPE ==0X0E)\
      ||(SENSOR_TYPE ==0X16)||(SENSOR_TYPE ==0X17)||(SENSOR_TYPE ==0X20)||(SENSOR_TYPE ==0X21)\
        ||(SENSOR_TYPE ==0X24)||(SENSOR_TYPE ==0X5A)||(SENSOR_TYPE ==0X5B)||(SENSOR_TYPE ==0X64)||(SENSOR_TYPE ==0X33)||(SENSOR_TYPE ==0X80)) 
T_MGSbit=0x14;    // 00010100状态信息 数据信息  连数据信息记录
//采集类  整型
#endif 
#if((SENSOR_TYPE =='E')||(SENSOR_TYPE =='F')||(SENSOR_TYPE =='U')||(SENSOR_TYPE ==0X81)||(SENSOR_TYPE ==0X82)||(SENSOR_TYPE ==0X85)) 
T_MGSbit=0x16; // 00010110状态信息 数据信息  连数据信息记录
//采集类  浮点型
#endif
#if((SENSOR_TYPE ==0X08)||(SENSOR_TYPE ==0X0A)||(SENSOR_TYPE ==0X0B)||(SENSOR_TYPE ==0X0D)) 
T_MGSbit=0x18;    // 00011000状态信息 数据信息  连数据信息记录
//采集类  字符串  数据第三位开始
#endif
 ////////////////////////////////////////////////////////////////////////////////////////////
//OLED屏显示内容
/*
OLED_Init(); //OLED初始化 
OLED_P8x16Str(0,0,"R       ID:");//第一行 -- 8x16的显示单元显示ASCII码
#if defined( SENSOR_TYPE_Coord )
#if (ZDO_COORDINATOR)
   OLED_P8x16Str(0,0,"C    PANID:");//第一行 -- 8x16的显示单元显示ASCII码
#endif 
#endif 
#if defined( POWER_SAVING )
 OLED_P8x16Str(0,0,"E       ID:");//第一行 -- 8x16的显示单元显示ASCII码
#endif
char *s=0;
uint16 nv_data; 
osal_nv_read(ZCD_NV_APP_CHANLIST,0,sizeof(nv_data),&nv_data);
sprintf(s, "%02d", (nv_data&0x00ff)); 
OLED_P8x16Str(16,0,s);//第一行 -- 8x16的显示单元显示ASCII码
#if !defined( SENSOR_TYPE_Coord )
sprintf(s, "%02X", SENSOR_TYPE); 
OLED_P8x16Str(40,0,s);//第一行 -- 8x16的显示单元显示ASCII码
#endif 
osal_nv_read(ZCD_NV_APP_PANID_H,0,sizeof(nv_data),&nv_data);
sprintf(s, "%04X", nv_data); 
OLED_P8x16Str(88,0,s);//第一行 -- 8x16的显示单元显示ASCII码
OLED_P8x16Str(0,2,"V2.45");
#if defined( CC2530_V30 )
OLED_P8x16Str(0,2,"V3.00");
#endif
sprintf(s, "%04X", NLME_GetCoordShortAddr()); 
OLED_P8x16Str(48,2,s);
sprintf(s, "%04X", NLME_GetShortAddr());
OLED_P8x16Str(88,2,s);
 byte * Send_d;
  Send_d=NLME_GetExtAddr();
sprintf(s, "%02X%02X%02X%02X%02X%02X%02X%02X", *Send_d,*(Send_d+1),*(Send_d+2),*(Send_d+3),*(Send_d+4),*(Send_d+5),*(Send_d+6),*(Send_d+7));
OLED_P8x16Str(0,4,s);
*/
 ////////////////////////////////////////////////////////////////////////////////////////////
#if(SENSOR_TYPE =='Q') 
  P1DIR|=0x02 ;  //数据输出 P1_1
  P1DIR&=~0X04;  //数据输入 P1_2
  P1DIR |= 0XF9;  
  Send_data[6]='Q';//传感器类型
 Send_datalend=0x03;    
 RSn=50;
 HX_Q=0;
 HXtime=0;
 HXS=0;
 T_MUMPT1=0;
  uint16 nv_data=0X0000;
 osal_nv_read(ZCD_NV_APP_AD_SHINING,0,sizeof(nv_data),&nv_data);  
 if((nv_data>49)&&(nv_data<151))AD_SHINING=nv_data;
 else AD_SHINING=90;
 
 Ht1621_Init(); //上电初始化LCD
 char t;
  P1_0=0;
  for (char i=0;i<15;i++) //30个字段 18个字段 SEG0～SEG17 
         {t=0x01;
          for (char j=0;j<4;j++)//4个一组 COM0-3
                 {
                  Ht1621WrOneData(i,t); //SEG0 COM0-3. SEG1 COM0-3. SEG2 COM0-3. SEG3 COM0-3. .......
                 t<<=1;
                  t++;
                  }
           }
 halMcuWaitMs(5000);//延时5秒 去皮操作 //延时一段时间
  Ht1621WrAllData(0,Ht1621Tab,9);//清除1621寄存器数据，暨清屏 //SEG0～SEG17 COM0-3=0X00 0000 全灭
 Tozero();//自动归零
#endif   
#if(SENSOR_TYPE ==0X08) 
   Send_data[7]=0xA1;//读卡  15693
   GenericApp_appIO=0;
  GenericApp_BX=0;
#endif  
#if(SENSOR_TYPE ==0X0D) 
#if defined(TGRD15693) 
   Send_data[7]=0xA1;//读卡  15693
#endif 
#if defined(PN532) 
   Send_data[7]=0xA2;//M1卡
#endif 
#endif 
#if(SENSOR_TYPE ==0X0A) 
   Send_datalend=0X14;
    Send_data[7]=0xA1;//读卡  15693
#endif  
#if(SENSOR_TYPE ==0X0B) 
   Send_datalend=0X14;
    Send_data[7]=0x01;//读卡  125K
#endif  
#if((SENSOR_TYPE ==0X10)||(SENSOR_TYPE ==0X0F)||(SENSOR_TYPE ==0X17)||(SENSOR_TYPE ==0X21)) 
 Send_datalend=0x03;   
HalAdcInit ();
#endif  
#if((SENSOR_TYPE ==0X11)||(SENSOR_TYPE ==0X1A)||(SENSOR_TYPE ==0X3D)||(SENSOR_TYPE ==0X3B)||(SENSOR_TYPE ==0X3C))
 Send_datalend=0x06;   
 GenericApp_sbum=0;
 T_MUMPT1=0;
HalAdcInit ();
#endif  
#if(SENSOR_TYPE ==0X84) 
 Send_datalend=0x05;   
 GenericApp_sbum=0;
  P1DIR|=0x1E;   //数据输出
 T_MUMPT1=0;
HalAdcInit ();
#endif 
#if(SENSOR_TYPE ==0X23) 
   Send_datalend=0X05;
#endif 
#if(SENSOR_TYPE ==0X24)
   P1DIR|=0x04;   //数据输出 P1_2
   P1DIR&=~0x02;  //数据输入 P1_1
   P1_1=0;   P1_2=0;     //初始化引脚
   Send_datalend=0x05;
#endif 
#if(SENSOR_TYPE ==0X80)
   P2DIR&=~0x01;  //数据输入 P2_0  起始检测点
   P2INP&=~0x01;//上拉
   Send_datalend=0x07;
#endif 
#if(SENSOR_TYPE ==0X27)
  // uint8 syn[11]={0XFD,0X00,0X08,0X01,0X01,0X5B,0X76,0X31,0X36,0X5D,0X82};//音量最大
  // uint8 syn[11]={0XFD,0X00,0X08,0X01,0X01,0X5B,0X76,0X31,0X31,0X5D,0X82};//音量最中大
   
   //FD 00 07 01 01 5B 76 39 5D B3 //音量小中
   //FD 00 07 01 01 5B 76 32 5D 82// 音量小
     Send_datalend=3;
      SYN_FrameInfo(0,1,"msgh",250) ;//ringo
      // HalUARTWrite(HAL_UART_PORT_0,syn,11);//智能语言播报
#endif
#if(SENSOR_TYPE ==0X6F)
  
 P1DIR|=0XE0;
 P1DIR|=0x1E;       //P1.1-P1.4数据输出
 P1|=0X1E;           //
 delay_ms(1);
 init_lcd();
 lcd_gra_clr();
 lcd_txt_clr(); 
 //hzkdis_b(0,"msgh",4);
      hzkdis_b(0,"  无锡泛太科技",14);
      Send_datalend=0x05;
      SYN_FrameInfo(0,1,"msgh",250) ;//ringo
      // HalUARTWrite(HAL_UART_PORT_0,syn,11);//智能语言播报
#endif
      
#if(SENSOR_TYPE ==0X28)
{ 
  LCD_Init();   // LCD初始化
  LCD_Clear();  // LCD清屏
  /* 显示信息 */
  LCD_WriteString(0,6,"WWW.fantaitech.com");
 // LCD_WriteString(0,3,"FANTAI ELECTRONICS");
 // LCD_WriteString(0,5,"WWW.fantaitech.com");
 // LCD_WriteString(6,7,"453:0510-88236212");
  Send_datalend=5;
} 
#endif 
#if(SENSOR_TYPE ==0X31)
car_stata=0;
car_op=0; //清零，可处理读卡
car_sum=0;
  P1SEL&=~0X01;
  P1DIR&=~0X01;  //P1_0输入模式
#endif
#if(SENSOR_TYPE ==0X64)
{ Send_datalend=0x03;
  P1SEL&=~0Xff;
  P1DIR&=~0X99;
  P1DIR|=0X55;
}  
#endif
#if(SENSOR_TYPE ==0X65)
 P0SEL=0X00;
    P1SEL&=0X00;
    P1DIR|=0Xff;
    P1INP|=0Xf0;
    Send_datalend=0x05;
    LCD_lni();    //lcd初始化
    LCD_clrscr(); //lcd清屏
    show_left_up();  //lcd左半屏上面4个字显示函数   无锡泛
    show_right_up();//lcd右半屏上面4个字显示函数   太科技
    show_left_down();
#endif
#if(SENSOR_TYPE ==0X6C)
 P1DIR|=0XE0;
 P1DIR|=0x1E;       //P1.1-P1.4数据输出
 P1|=0X1E;           //
Send_datalend=0x05;
 delay_ms(1);
 init_lcd();
 lcd_gra_clr();
 lcd_txt_clr(); 
  hzkdis(0,tab0);
#endif
#if(SENSOR_TYPE ==0X33)
  P1SEL &= ~(0x18);		//选择IO口
  B_LUX30_Init();
#endif
#if((SENSOR_TYPE ==0X66)||(SENSOR_TYPE ==0X6E))
 P0SEL=0X00;
    P1SEL&=0X00;
    P1DIR|=0Xff;
    P1INP|=0Xf0;
     P2SEL&=0X00;
    P2DIR|=0X01;
    Send_datalend=0x05;
    uint8 spt[2]={0,0};
     SendDataS(spt,2);
#endif
#if(SENSOR_TYPE ==0X76)
     P2DIR&=~0X01;//输入
    P2INP&=~0x01;//上拉
    Send_datalend=0x05;
#endif
#if(SENSOR_TYPE ==0X30)
    P1DIR|=0XE0;//输出
    P1DIR|=0X0f;
    P2DIR&=~0X01;//输入
    P2INP&=~0x01;//上拉
    Coil_OFF;
    Send_datalend=0x05;
    DAchange(4095);
#endif
#if(SENSOR_TYPE ==0X73)
    P1DIR|=0Xf0;   //P1.6 P1.7输出
    P2DIR&=~0X01;//输入
    P2INP&=~0x01;//上拉
    A8n=1;
    Send_datalend=0x05;
#endif
    
#if(SENSOR_TYPE ==0X7C)
      uint8  hr[5]={0XF7,0x03,0xB1,0X52,0X00};
     HalUARTWrite(HAL_UART_PORT_0,hr,5);//开关绑定模式 
#endif
#if(SENSOR_TYPE =='D') 
{ Send_datalend=0x05;
       P1DIR|=0xff;       //数据输出
       P1SEL&=~0x01;P1DIR&=~0X01;
       P2SEL|=0x01;P2DIR|=0X01;
       P1_3=0;
       for(unsigned int i=0;i<50000;i++);
        S0=1;S1=1;
//----------------------白平衡调整-----------------------------
	 OUTR = 255.0/TCS3200(1);
	 OUTG = 255.0/TCS3200(2);
	 OUTB = 255.0/TCS3200(3);
} 
#endif                           
#if(SENSOR_TYPE =='S') 
    P1SEL &=0xF7;
    P1DIR |= 0x07;//P1.1,P1.2,P1.3数据输出
    P1DIR &= ~(0xE0);//P1.5,P1.6,P1.7数据输入
    P1INP |= 0x07;
  Send_datalend=0x05;                         
#endif   
#if(SENSOR_TYPE ==0X67) 
    P0SEL &=0x30;
    P0DIR |= 0x30;//P0.4,P0.5,数据输出
    P1DIR|=0XE0;//输出
    P1DIR|=0X0F;//输出
    P1_0=0;
    P1_1=0;
    P1_2=0;
    P1_3=0;
    P0_4=0;
    P0_5=0;
  Send_datalend=0x05;   
int mled=0;
//DAchange(4095);
    mled=1000;
    for(int i=0;i<3000;i++)
    {halMcuWaitMs(1);
    DAchange(mled);
    mled+=1;
    }
DAchange(0);
#endif  
#if(ENSOR_TYPE ==0X68) 
    P0DIR |= 0x0F;//P0.0-3,数据输出
    P0_0=0;P0_1=0;P0_2=0;P0_3=0;
  Send_datalend=0x05;                         
#endif  
#if(SENSOR_TYPE ==0X6A) 
#if defined(ADCP) 
    HalAdcInit ();
#endif 
    Send_data[7]=0;
#if defined(HX711P) 
     P1DIR|=0x02 ;  //数据输出 P1_1
    P1DIR&=~0X04;  //数据输入 P1_2
    Tozero();//自动归零
#endif 
  Send_datalend=0x05;                         
#endif 
#if(SENSOR_TYPE ==0X03)  //P1.0-P1.3  D0-D3  P0.7 VT
   
    P1DIR &= ~(0x1F);//P1.0,P1.1,P1.2,P1.3,P1.4数据输入
     P1INP |= 0x1F;
   Send_datalend=0x03;                         
#endif   
#if(SENSOR_TYPE ==0X05) 
 //P1DIR&=(~0x06);       //P1.1 P1.2数据输入
P1DIR|=0xC0;       //P1.6,P1.7数据输出
P1_6=0;
P1_7=0;
Send_datalend=0x05;                                             
#endif   
#if(SENSOR_TYPE =='U') 
  Send_datalend=0x04;
 P1SEL &= ~0X04;P1INP|=0X04;
#endif    
#if(SENSOR_TYPE =='H') 
 P1DIR|=0xC1;       //P1.0,P1.6,P1.7数据输出
P1_0=1;
P1_6=1;
P1_7=1;
Send_datalend=0x05;                         
#endif      
#if((SENSOR_TYPE =='P')||(SENSOR_TYPE ==0X02))
Send_datalend=0x05;
#endif  
#if(SENSOR_TYPE ==0X77)
Send_datalend=0x03;
  P1SEL&=~0Xff;
  P1DIR&=~0Xff;
 // P1INP&=~0Xff;P2INP&=~0X40;
#endif  
#if(SENSOR_TYPE =='I')
{ Send_datalend=0x05;
  P1SEL&=~0Xff;
  P1DIR&=~0Xff;
 // P1INP|= 0Xff;
  
  P2SEL&=~0X07;
  P2DIR&=~0X07;
 // P2INP|= 0X07;
  P0SEL&=~0X10;
  P0DIR&=~0X10;
  //P0INP|= 0X10;
}  
#endif  

#if(SENSOR_TYPE =='G')
{ Send_datalend=0x08;
 P1DIR|=0X0f;
 Coil_OFF;
Coil_AB=0;
} 
#endif                            
#if(SENSOR_TYPE =='F')
#if defined(BMP085) 
{ Send_datalend=0x06;
 XCLROUT;
  BMP085_init();	//读取BMP085的E2PROM中的11个标定参数
}
#endif 
#if defined(BMP180) 

#define XCLR P1_5 
#define XCLROUT P1DIR|=0x20       //数据输出
Send_datalend=0x06;
 XCLROUT;
 XCLR=1; 
 
 Init_BMP180();
 
#endif
#endif                         
#if(SENSOR_TYPE =='C')
{ Send_datalend=0x03;}
#endif            

#if(SENSOR_TYPE =='B')
{ Send_datalend=0x05;
  MMA7660_Startup();
} 
#endif   
#if((SENSOR_TYPE ==0X13)||(SENSOR_TYPE ==0X5A)||(SENSOR_TYPE ==0X5B)||(SENSOR_TYPE ==0X70))
      P1DIR|=0x01;       //P1.0数据输出  485控制端
  P1_0=0;
  Send_data[6]=SENSOR_TYPE;
  Send_data[7]=0XAA;
   Send_datalend=0X04;
  //U0UCR|=0X38;
#endif 
#if(SENSOR_TYPE ==0X14) 
  P1DIR|=0x01;       //P1.0数据输出  485控制端
  P1_0=0;
   Send_datalend=0X07;
#if defined(PH_E201C)
 P1SEL &= ~0X04;P1INP|=0X04;
   HalAdcInit ();
#endif 
#endif 
#if(SENSOR_TYPE ==0X74)
  P1DIR|=0x01;       //P1.0数据输出  485控制端
  P1_0=0;
  Send_datalend=0X05;
#endif 
#if(SENSOR_TYPE ==0X15)
{ 
P1DIR|=0x1E;       //P1.0-P1.4数据输出
P1=0X1E;           //
Send_datalend=0x05;
} 
#endif 
#if(SENSOR_TYPE =='K')
{ 
P1DIR&=(~0x06);       //P1.1 P1.2数据输入
P1SEL &= ~0X06;P1INP|=0X06;  //上啦
P1DIR|=0x01;       //P1.0数据输出
P1_0=1;  T_MG=0X00;       //继电器关
Send_datalend=0x05;
} 
#endif  
#if(SENSOR_TYPE ==0X79)
PDI=0;
ERR_TEMP=0;
P1DIR|=0x21;       //P1_5  P1_0数据输出
P1_5=P1_0=0X01;         //继电器关
Send_datalend=0x05;
  P1SEL &= ~0X06;P1INP|=0X04; P1DIR|=0x06;
  s_connectionreset(); //温湿度初始化
 HalAdcInit (); //ad采集初始化
#endif  
#if(SENSOR_TYPE ==0X83)
{
P0DIR&=(~0xf0);       //P07 P06 P05数据输入
P0SEL &= ~0Xf0;P0INP|=0Xf0;  //上啦
P1DIR|=0xFF;       //P1数据输出
P1=0XFF; T_MG=0X00;       //继电器关
Send_datalend=0x05;
} 
#endif  
#if(SENSOR_TYPE ==0X40)
{ 
P1DIR|=0x0E;       //P1.1 P1.2 P1.3数据输出
P1DIR|=0x91;       //P1.7 P1.4 P1.0数据输出
P1_1=1;           //
P1_2=1;           //
P1_3=1;           //
P1_0=0;           //蜂鸣器
P1_4=0;           //红灯
P1_7=0;           //蓝灯
Send_datalend=0x05;
} 
#endif 

#if(SENSOR_TYPE ==0X07)
AB=0x0A;
#endif 
#if(SENSOR_TYPE ==0X12)
{ 
P1DIR&=(~0x06);       //P1.1 P1.2数据输入
P1SEL &= ~0X06;P1INP|=0X06;  //上啦
P1DIR|=0xC1;       //P1.0,P1.6,P1.7数据输出
P1_0=1;//继电器关
P1_6=1;
P1_7=1;     
Send_datalend=0x05;
} 
#endif 
#if(SENSOR_TYPE ==0X18)
{ 
P1DIR&=(~0x06);       //P1.1 P1.2数据输入
P1SEL &= ~0X06;P1INP|=0X06;  //上啦
P1DIR|=0xC1;       //P1.0,P1.6,P1.7数据输出
P1_0=1;   
U0UCR|=0X38;
Send_datalend=0x05;
} 
#endif 
#if(SENSOR_TYPE ==0X06)
Send_datalend=0x05;
 P1SEL &= ~0X04;P1INP|=0X04;
 ReadTemperature(); 
#endif
#if(SENSOR_TYPE ==0X09)
Send_datalend=0x05;
#endif

#if defined (NB_IOT_S10)
P0SEL &= ~0X10;
P0DIR|=0x10;       //P0.4,数据输出
P0_4=0;   
#endif

#if(SENSOR_TYPE =='E')
{ Send_datalend=0x06;
  P1SEL &= ~0X06;P1INP|=0X04; P1DIR|=0x06;
  s_connectionreset();
} 
#endif 
#if(SENSOR_TYPE ==0X85)
      _SCL_IO;                                            //引用宏定义-设置SCL为开漏式I/O口
     _SDA_OUTPUT;                                //引用宏定义-设置SDA为开漏式输出
     SCL=0;				//
     Delay_mlx90615(30000);		 //SMBus请求时间，将PWM模式转换为SMBus模式(21ms - 39ms)
     SCL=1;				//	
#endif 
#if(SENSOR_TYPE ==0X82)
{ Send_datalend=0x09;
  P1SEL &= ~0X18;P1INP|=0X18; P1DIR|=0x18;
  P1DIR&=(~0x04)  ; //P1_2数据输入 上拉
  Init_HMC5883();
} 
#endif 
#if(SENSOR_TYPE ==0X81)
{ 
  Send_datalend=0x06;
  P1DIR&=(~0x20);       // P1.5数据输入
P1SEL &= ~0X20;P1INP|=0X20;  //上啦
P1DIR|=0xC0;       //PP1.6,P1.7数据输出
} 
#endif 
#if(SENSOR_TYPE ==0X78)
Send_datalend=34;
send_Instruction();//向模块发送5个指令

#endif 

#if((SENSOR_TYPE =='A')||(SENSOR_TYPE =='J')||(SENSOR_TYPE =='L')||(SENSOR_TYPE =='M')||(SENSOR_TYPE =='N')\
  ||(SENSOR_TYPE ==0X04)||(SENSOR_TYPE ==0X22)||(SENSOR_TYPE ==0X0C)||(SENSOR_TYPE ==0X16)||(SENSOR_TYPE ==0X19)\
    ||(SENSOR_TYPE ==0X61)||(SENSOR_TYPE ==0X62)||(SENSOR_TYPE ==0X63)||(SENSOR_TYPE ==0X72) ||(SENSOR_TYPE ==0X34)\
      ||(SENSOR_TYPE ==0X35)||(SENSOR_TYPE ==0X36)||(SENSOR_TYPE ==0X37)||(SENSOR_TYPE ==0X38)||(SENSOR_TYPE ==0X39)\
        ||(SENSOR_TYPE ==0X3A)||(SENSOR_TYPE ==0X3E)||(SENSOR_TYPE ==0X3F)||(SENSOR_TYPE ==0X75))
{
  Send_datalend=0x04;
#if(SENSOR_TYPE ==0X72)
    Send_datalend=0x06;
#if defined( SDS011 )
Send_datalend=0x08;
#endif
#endif   
#if(SENSOR_TYPE ==0X22)
Send_datalend=0x05;
#endif
HalAdcInit ();
P2SEL &=( ~0X01);
P2DIR&=(~0x01);       // P2.0数据输入
P2INP=0X00;  //上啦  
P2INP &=(~0X01); //上啦
T_MUMPT1=0;
}  
#endif     

  // Update the display
#ifdef SERIAL_DEBUG_SUPPORTED
  //printf("GenericApp\n"); 
#endif
  
  //ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  //ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );
}
/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;
  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    { 
      switch ( MSGpkt->hdr.event )
      {
        /*case ZDO_CB_MSG:
          GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;*/
          
        case KEY_CHANGE:
          GenericApp_HandleKeys(((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys);
		  break; 

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          GenericApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (GenericApp_NwkState == DEV_ZB_COORD)
              /*|| (GenericApp_NwkState == DEV_ROUTER)
              || (GenericApp_NwkState == DEV_END_DEVICE)*/ )
          {
            // Start sending "the" message in a regular interval.
            osal_start_timerEx( GenericApp_TaskID, GENERICAPP_SEND_MSG_EVT,
                                GENERICAPP_SEND_MSG_TIMEOUT );
          }
////////////////////////////////////////////////////////////////////////////////////////////
//OLED屏显示内容        
char *s=0;
uint16 nv_data; 
osal_nv_read(ZCD_NV_APP_PANID_H,0,sizeof(nv_data),&nv_data);
sprintf(s, "%04X", nv_data); 
OLED_P8x16Str(88,0,s);//第一行 -- 8x16的显示单元显示ASCII码
sprintf(s, "%04X", NLME_GetCoordShortAddr());  
OLED_P8x16Str(48,2,s);
sprintf(s, "%04X", NLME_GetShortAddr());  
OLED_P8x16Str(88,2,s); 
 ////////////////////////////////////////////////////////////////////////////////////////////
          break;
        default:
          break;
}
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  // Send a message out - This event is generated by a timer
  // (setup in GenericApp_Init()).
  if ( events & GENERICAPP_SEND_MSG_EVT )
  {
    // Send "the" message
  //  GenericApp_SendTheMessage();
    // Setup to send message again
   osal_start_timerEx( GenericApp_TaskID, GENERICAPP_SEND_MSG_EVT,
                       GENERICAPP_SEND_MSG_TIMEOUT );
    // return unprocessed events
    return (events ^ GENERICAPP_SEND_MSG_EVT);
  }
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 *//*
void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
      }
#if defined(BLINK_LEDS)
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_2, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            GenericApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}*/

/*********************************************************************
 * @fn      GenericApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void GenericApp_HandleKeys( byte shift, byte keys )
{
  zAddrType_t dstAddr;
  // Shift is used to make each button/switch dual purpose.
  if ( true )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
#if(SENSOR_TYPE =='O')
      char hr[5]={0xCE,0x03,0xFC,0x00,0xDE};
      hr[3]=0XCC;
      HalUARTWrite(HAL_UART_PORT_0, (uint8*)hr,5);
#endif    
#if(SENSOR_TYPE =='G')
if(Coil_AB==0)Coil_AB=1;
else if(Coil_AB<5)Coil_AB=10;
else Coil_AB=0;
#endif 
#if(SENSOR_TYPE =='P')
      uint8  hr[8]={0X01,0x03,0x06,0X80,0X00,0X89,0X0D,0X0A};
    GenericApp_applicationbuf=0;
    GenericApp_SendTheMessage();
     HalUARTWrite(HAL_UART_PORT_0,hr,8);//学习
#endif
#if(SENSOR_TYPE ==0X25)
      uint8  hr[10]={0XF7,0x08,0xAA,0X02,0X01,0X01,0X00,0X0B,0X0B,0X0B};
     Send_datalend=2;
    GenericApp_SendTheMessage();
     HalUARTWrite(HAL_UART_PORT_0,hr,10);//学习红外
#endif
#if(SENSOR_TYPE ==0X7C)
      uint8  hr[5]={0XF7,0x03,0xB1,0X52,0X00};
     Send_datalend=2;
    GenericApp_SendTheMessage();
     HalUARTWrite(HAL_UART_PORT_0,hr,5);//开关绑定模式 
#endif
#if(SENSOR_TYPE ==0X7B)
     Send_datalend=6;
    GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE ==0X71)
      uint8  hr[6]={0XF7, 0X03,0XBB,0X65,0X40, 0X01,};
     Send_datalend=2;
    GenericApp_SendTheMessage();
    HalUARTWrite(HAL_UART_PORT_0,&hr[5],1);halMcuWaitMs(100);
    HalUARTWrite(HAL_UART_PORT_0,hr,5);
#endif
#if(SENSOR_TYPE ==0X70)
      uint8  hr[11]={0XF7,0x09,0x01,0X01,0XC0,0X10,0X10,0X40,0X06,0X06,0X20};
     Send_datalend=2;
   GenericApp_SendTheMessage();
     P1_0=1; 
     GenericApp_uart_485=0;
     HalUARTWrite(HAL_UART_PORT_0,hr,11);
#endif
#if(SENSOR_TYPE ==0X86)
     // uint8  hr[9]={0XF7,0x07,0xBA,0X01,0X01,0XA0,0X00,0X00,0X00};
     Send_datalend=2;
     GenericApp_SendTheMessage();
   //  HalUARTWrite(HAL_UART_PORT_0,hr,9);//智能语言播报
#endif
#if(SENSOR_TYPE ==0X87)
     // uint8  hr[9]={0XF7,0x07,0xBA,0X01,0X01,0XA0,0X00,0X00,0X00};
     Send_datalend=2;
     GenericApp_SendTheMessage();
   //  HalUARTWrite(HAL_UART_PORT_0,hr,9);//智能语言播报
#endif
#if(SENSOR_TYPE ==0X26)
      uint8  hr[9]={0XF7,0x07,0xBA,0X01,0X01,0XA0,0X00,0X00,0X00};
     Send_datalend=2;
     GenericApp_SendTheMessage();
     HalUARTWrite(HAL_UART_PORT_0,hr,9);//智能语言播报
#endif
 #if(SENSOR_TYPE ==0X27)
      SYN_FrameInfo(0,1,"soundy",250) ;
#endif
      
#if(SENSOR_TYPE ==0X28)
       LCD_Init();   // LCD初始化
  LCD_Clear();  // LCD清屏
  /* 显示信息 */
  LCD_WriteString(0,0,"WWW.fantaitech.com");
      GenericApp_switch=0;
       GenericApp_SendTheMessage();
#endif
  #if(SENSOR_TYPE ==0X6F)
      SYN_FrameInfo(0,1,"soundy",250) ;
         lcd_txt_clr(); 
      hzkdis_b(0,"  无锡泛太科技",14);
      GenericApp_switch=0;
       GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE ==0X6B)
#if defined(LED16X64)
      uint8  hr[4]={0X02,0x33,0x01,0X00};
     HalUARTWrite(HAL_UART_PORT_0,hr,4);
#endif
    GenericApp_applicationbuf=0;
    GenericApp_SendTheMessage();
#endif
#if((SENSOR_TYPE ==0X05)||(SENSOR_TYPE ==0X0A)||(SENSOR_TYPE ==0X0B)||\
     (SENSOR_TYPE =='R')||(SENSOR_TYPE =='S')||(SENSOR_TYPE =='H')||(SENSOR_TYPE ==0X65)\
       ||(SENSOR_TYPE ==0X66)||(SENSOR_TYPE ==0X67)||(SENSOR_TYPE ==0X76))
       GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE ==0X6E)
      uint8 spt[2]={0,0};
     SendDataS(spt,2);
    GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE ==0X6C)
         lcd_txt_clr(); 
  hzkdis(2,tab0);
       GenericApp_SendTheMessage();
#endif

#if(SENSOR_TYPE ==0X6D)
    GenericApp_applicationbuf=0;
    GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE ==0XF0)
    GenericApp_applicationbuf=0;
    GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE ==0X30)
       if(a68!=0Xba){a68=0xba;b68=B01;c68=150;}
       else {a68=0xab;b68=B01;c68=0;}
       GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE ==0X73)
    if(a68!=0Xba){a68=0xba;b68=5000;}
       else {a68=0xab;b68=5000;}
    Send_data[10]=0X00;
    uint8 data8[6]={0X05,0XA8,0X01,0XA0,0X10,0X00};
     if(A8n<8)A8n++;
      else A8n=1;
    data8[2]=A8n;
        HalUARTWrite(HAL_UART_PORT_0, data8,0X06);
       GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE ==0X23)
   
    if(SFG_R30X)
    {SFG_R30XA[0]=0X00;SFG_R30X=0;}
    else
    { SFG_R30XA[0]=0XA1;SFG_R30X=1;}
    
     GenericApp_applicationbuf=0;
     GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE ==0X20)
       GenericApp_ON_OFF=10;
#endif
#if(SENSOR_TYPE ==0X77)
      GenericApp_appIO=0x00;
       GenericApp_SendTheMessage(); //应用函数
#endif
#if((SENSOR_TYPE =='I')||(SENSOR_TYPE ==0X64))
      GenericApp_appIO=0x01;
#endif
#if((SENSOR_TYPE =='E')||(SENSOR_TYPE =='B')||(SENSOR_TYPE =='C')||(SENSOR_TYPE =='D')\
  ||(SENSOR_TYPE =='F')||(SENSOR_TYPE =='U')||(SENSOR_TYPE ==0X10)||(SENSOR_TYPE ==0X21)\
    ||(SENSOR_TYPE ==0X11)||(SENSOR_TYPE ==0X1A)||(SENSOR_TYPE ==0X0F)||(SENSOR_TYPE ==0X13)||(SENSOR_TYPE ==0X14)\
      ||(SENSOR_TYPE ==0X17)||(SENSOR_TYPE ==0X24)||(SENSOR_TYPE ==0X6A)||(SENSOR_TYPE =='A')\
        ||(SENSOR_TYPE =='J')||(SENSOR_TYPE =='L')||(SENSOR_TYPE =='M')\
  ||(SENSOR_TYPE =='N')||(SENSOR_TYPE ==0X04)||(SENSOR_TYPE ==0X22)||(SENSOR_TYPE ==0X0C)\
    ||(SENSOR_TYPE ==0X16)||(SENSOR_TYPE ==0X17)||(SENSOR_TYPE ==0X61)||(SENSOR_TYPE ==0X62)\
      ||(SENSOR_TYPE ==0X19)||(SENSOR_TYPE ==0X5A)||(SENSOR_TYPE ==0X5B)||(SENSOR_TYPE ==0X63)||(SENSOR_TYPE ==0X33)||(SENSOR_TYPE ==0X72)\
         ||(SENSOR_TYPE ==0X34)||(SENSOR_TYPE ==0X35)||(SENSOR_TYPE ==0X36)||(SENSOR_TYPE ==0X37)||(SENSOR_TYPE ==0X38)\
           ||(SENSOR_TYPE ==0X39)||(SENSOR_TYPE ==0X3A)||(SENSOR_TYPE ==0X80)||(SENSOR_TYPE ==0X3D)||(SENSOR_TYPE ==0X81)\
             ||(SENSOR_TYPE ==0X82)||(SENSOR_TYPE ==0X3B)||(SENSOR_TYPE ==0X3C)||(SENSOR_TYPE ==0X3E)||(SENSOR_TYPE ==0X3F)\
               ||(SENSOR_TYPE ==0X74)||(SENSOR_TYPE ==0X84)||(SENSOR_TYPE ==0X85)||(SENSOR_TYPE ==0X75)||(SENSOR_TYPE ==0X78))
      GenericApp_applicationbuf=60001; //单次发送 不改变定时状态
     //  GenericApp_time=0;
      // TIMER3_RUN(FALSE);  //关闭定时
if(!(T_MGSbit&(0XC0)))  //1100 0000 判断是否为 关闭定时状态
{
      GenericApp_time=0;
       TIMER3_RUN(FALSE);  //关闭定时
}
#endif
#if(SENSOR_TYPE ==0X79)
          P1_5=~P1_5;
          Send_data[7]=0X02;
          Send_data[8]=0XDD;
          Send_data[9]=P1_5;
          Send_data[9]=~((((Send_data[9]<<1)+P1_0)&0X03)|0XFC);
          Send_datalend=0x04;
          GenericApp_SendTheMessage(); //应用函数
#endif  
#if(SENSOR_TYPE ==0X83)
       P1_1=~P1_1;
      GenericApp_applicationbuf=0;
      GenericApp_SendTheMessage();
#endif  
#if((SENSOR_TYPE =='K')||(SENSOR_TYPE ==0X02)||(SENSOR_TYPE ==0X06)||(SENSOR_TYPE ==0X40)\
  ||(SENSOR_TYPE ==0X09)||(SENSOR_TYPE ==0X12)||(SENSOR_TYPE ==0X18)||(SENSOR_TYPE ==0X15)\
    ||(SENSOR_TYPE ==0X29)||(SENSOR_TYPE ==0X31)||(SENSOR_TYPE ==0XA3))
    //  P1_1=~P1_1;
      GenericApp_applicationbuf=0;
      GenericApp_SendTheMessage();
#endif
 ////////////////////////////////////////////////////////////////////////////////////////////
//OLED屏显示内容
OLED_Init(); //OLED初始化 
OLED_P8x16Str(0,0,"R       ID:");//第一行 -- 8x16的显示单元显示ASCII码
#if defined( SENSOR_TYPE_Coord )
#if (ZDO_COORDINATOR)
   OLED_P8x16Str(0,0,"C    PANID:");//第一行 -- 8x16的显示单元显示ASCII码
#endif 
#endif 
#if defined( POWER_SAVING )
 OLED_P8x16Str(0,0,"E       ID:");//第一行 -- 8x16的显示单元显示ASCII码
#endif
char *s=0;
uint16 nv_data; 
osal_nv_read(ZCD_NV_APP_CHANLIST,0,sizeof(nv_data),&nv_data);
sprintf(s, "%02d", (nv_data&0x00ff)); 
OLED_P8x16Str(16,0,s);//第一行 -- 8x16的显示单元显示ASCII码
#if !defined( SENSOR_TYPE_Coord )
sprintf(s, "%02X", SENSOR_TYPE); 
OLED_P8x16Str(40,0,s);//第一行 -- 8x16的显示单元显示ASCII码
#endif 
osal_nv_read(ZCD_NV_APP_PANID_H,0,sizeof(nv_data),&nv_data);
sprintf(s, "%04X", nv_data); 
OLED_P8x16Str(88,0,s);//第一行 -- 8x16的显示单元显示ASCII码
OLED_P8x16Str(0,2,"V2.45");
#if defined( CC2530_V30 )
OLED_P8x16Str(0,2,"V3.00");
#endif
sprintf(s, "%04X", NLME_GetCoordShortAddr()); 
OLED_P8x16Str(48,2,s);
sprintf(s, "%04X", NLME_GetShortAddr());
OLED_P8x16Str(88,2,s);
 byte * Send_d;
  Send_d=NLME_GetExtAddr();
sprintf(s, "%02X%02X%02X%02X%02X%02X%02X%02X", *Send_d,*(Send_d+1),*(Send_d+2),*(Send_d+3),*(Send_d+4),*(Send_d+5),*(Send_d+6),*(Send_d+7));
OLED_P8x16Str(0,4,s);
 /////////////////////////////////////////////////////////////////////////////////   

#if !defined (SENSOR_TYPE_Coord)
#if((SENSOR_TYPE =='K')||(SENSOR_TYPE ==0X28))
ZigBee_C_R_E_Engineering_E();
#endif
#endif
  }
    if ( keys & HAL_KEY_SW_2 )
    {
#if(SENSOR_TYPE =='O')
      char hr[5]={0xCE,0x03,0xFC,0x00,0xDE};
      hr[3]=0XAA;
      HalUARTWrite(HAL_UART_PORT_0, (uint8*)hr,5);
      GenericApp_SendTheMessage();
#endif 
#if(SENSOR_TYPE =='G')
if(Coil_AB==0)Coil_AB=1;
else if(Coil_AB<5)Coil_AB=10;
else Coil_AB=0;
#endif 
#if((SENSOR_TYPE ==0X03)||(SENSOR_TYPE ==0X17))
     GenericApp_SendTheMessage();
#endif 
#if(SENSOR_TYPE ==0X07)
 //    if(AB==0X0A)AB=0X0B;
 //    else AB=0X0A;
     Send_datalend=0X04;
GenericApp_SendTheMessage();
#endif 
#if(SENSOR_TYPE ==0X08)
     if(GenericApp_BX)
    { GenericApp_BX=0; 
    Send_data[7]=0xA1;//读卡  15693
     //printf("010C00030410002101020000\n"); //设置 15693卡
    GenericApp_appIO=0;
    }
    else
    {GenericApp_BX=1;
     Send_data[7]=0xA2; //读卡 14443A
   // printf("010C00030410002101020000\n"); //设置 15693卡
     GenericApp_appIO=0;
    }
#endif 
#if(SENSOR_TYPE ==0X0D)
#if defined(TGRD15693) 
     if(GenericApp_BX)
    { GenericApp_BX=0; 
    Send_data[7]=0xA1;//读卡  15693
     //printf("010C00030410002101020000\n"); //设置 15693卡
    GenericApp_appIO=0;
    }
    else
    {GenericApp_BX=1;
     Send_data[7]=0xA2; //读卡 14443A
   // printf("010C00030410002101020000\n"); //设置 15693卡
     GenericApp_appIO=0;
    }
#endif 
#if defined(PN532) 
   
#endif 
#endif 
#if((SENSOR_TYPE =='R')||(SENSOR_TYPE =='S')||(SENSOR_TYPE =='H')||(SENSOR_TYPE ==0X65)||(SENSOR_TYPE ==0X66)||\
  (SENSOR_TYPE ==0X67)||(SENSOR_TYPE ==0X05)||(SENSOR_TYPE ==0X0A)||(SENSOR_TYPE ==0X0B)\
   ||(SENSOR_TYPE ==0X76) )
      GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE ==0X6E)
      uint8 spt[2]={0XFF,0XFF};
     SendDataS(spt,2);
    GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE ==0X6C)
        lcd_txt_clr(); 
  hzkdis(1,tab0);
       GenericApp_SendTheMessage();
#endif

#if(SENSOR_TYPE ==0X6D)
    GenericApp_applicationbuf=0;
    GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE ==0XF0)
    GenericApp_applicationbuf=0;
    GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE =='P')
      uint8  hrx[8]={0X01,0x03,0x06,0X81,0X00,0X8a,0X0D,0X0A};
     GenericApp_applicationbuf=0;
       GenericApp_SendTheMessage();
      HalUARTWrite(HAL_UART_PORT_0,hrx,8);//发送
      // GenericApp_SendTheMessage();
#endif 
#if(SENSOR_TYPE ==0X25)
    Send_datalend=2;
    GenericApp_SendTheMessage();
#endif 
#if(SENSOR_TYPE ==0X7C)
    Send_datalend=2;
    GenericApp_SendTheMessage();
#endif 
#if(SENSOR_TYPE ==0X7B)
    Send_datalend=6;
    GenericApp_SendTheMessage();
#endif 
#if(SENSOR_TYPE ==0X71)
    uint8  hr[6]={0XF7, 0X03,0XBB,0X65,0X60, 0X01,};
    Send_datalend=2;
      HalUARTWrite(HAL_UART_PORT_0,hr,5);
    GenericApp_SendTheMessage();
#endif 
#if(SENSOR_TYPE ==0X70)
      uint8  hr[11]={0XF7,0x09,0x01,0X01,0XA0,0X10,0X10,0X40,0X06,0X06,0X20};
     Send_datalend=2;
    GenericApp_SendTheMessage();
     P1_0=1; 
     GenericApp_uart_485=0;
     HalUARTWrite(HAL_UART_PORT_0,hr,11);
#endif 
#if(SENSOR_TYPE ==0X73)
     if(a68!=0Xba){a68=0xba;b68=5000;}
       else {a68=0xab;b68=5000;}
     Send_data[10]=0X00;
      uint8 data8[6]={0X05,0XA8,0X01,0XA0,0X10,0X00};
     if(A8n>1)A8n--;
      else A8n=8;
    data8[2]=A8n;
        HalUARTWrite(HAL_UART_PORT_0, data8,0X06);
       GenericApp_SendTheMessage();
#endif
 #if(SENSOR_TYPE ==0X86)
    Send_datalend=2;
    GenericApp_SendTheMessage();
#endif 
#if(SENSOR_TYPE ==0X87)
    Send_datalend=2;
    GenericApp_SendTheMessage();
#endif 
#if(SENSOR_TYPE ==0X26)
    Send_datalend=2;
    GenericApp_SendTheMessage();
#endif 
#if(SENSOR_TYPE ==0X27)
     SYN_FrameInfo(0,1,"soundy",250) ;
    
#endif 
     
#if(SENSOR_TYPE ==0X28)
      LCD_Init();   // LCD初始化
  LCD_Clear();  // LCD清屏
  /* 显示信息 */
  LCD_WriteString(0,2,"   WUXIFANTAIKEJI");
        GenericApp_switch=0;
       GenericApp_SendTheMessage();
#endif

#if(SENSOR_TYPE ==0X6F)
      SYN_FrameInfo(0,1,"soundy",250) ;
       // lcd_txt_clr(); 
      hzkdis_b(1,"  公交管理系统",14);
        GenericApp_switch=0;
       GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE ==0X6B)
#if defined(LED16X64)
      uint8  hr[4]={0X02,0x33,0x02,0X00};
     HalUARTWrite(HAL_UART_PORT_0,hr,4);
#endif
     GenericApp_applicationbuf=0;
       GenericApp_SendTheMessage();
#endif 
#if(SENSOR_TYPE ==0X77)
      GenericApp_appIO=0x00;
       GenericApp_SendTheMessage(); //应用函数
#endif
#if((SENSOR_TYPE =='I')||(SENSOR_TYPE ==0X64))
      GenericApp_appIO=0x02;
#endif
#if((SENSOR_TYPE =='E')||(SENSOR_TYPE =='B')||(SENSOR_TYPE =='C')||(SENSOR_TYPE =='D')\
  ||(SENSOR_TYPE =='F')||(SENSOR_TYPE =='U')||(SENSOR_TYPE ==0X10)||(SENSOR_TYPE ==0X21)\
    ||(SENSOR_TYPE ==0X11)||(SENSOR_TYPE ==0X1A)||(SENSOR_TYPE ==0X0F)||(SENSOR_TYPE ==0X13)||(SENSOR_TYPE ==0X14)\
      ||(SENSOR_TYPE ==0X24)||(SENSOR_TYPE ==0X6A)||(SENSOR_TYPE =='A')||(SENSOR_TYPE =='J')||\
        (SENSOR_TYPE =='L')||(SENSOR_TYPE =='M')||(SENSOR_TYPE ==0X33)\
  ||(SENSOR_TYPE =='N')||(SENSOR_TYPE ==0X04)||(SENSOR_TYPE ==0X22)||(SENSOR_TYPE ==0X0C)\
    ||(SENSOR_TYPE ==0X16)||(SENSOR_TYPE ==0X19)||(SENSOR_TYPE ==0X61)||(SENSOR_TYPE ==0X62)||(SENSOR_TYPE ==0X63)\
      ||(SENSOR_TYPE ==0X5A)||(SENSOR_TYPE ==0X5B)||(SENSOR_TYPE ==0X72) ||(SENSOR_TYPE ==0X34)||(SENSOR_TYPE ==0X35)||(SENSOR_TYPE ==0X36)\
        ||(SENSOR_TYPE ==0X37)||(SENSOR_TYPE ==0X38)||(SENSOR_TYPE ==0X39)||(SENSOR_TYPE ==0X3A)||(SENSOR_TYPE ==0X80)\
          ||(SENSOR_TYPE ==0X3D)||(SENSOR_TYPE ==0X81)||(SENSOR_TYPE ==0X82)||(SENSOR_TYPE ==0X3B)||(SENSOR_TYPE ==0X3C)\
            ||(SENSOR_TYPE ==0X3E)||(SENSOR_TYPE ==0X3F)||(SENSOR_TYPE ==0X74)||(SENSOR_TYPE ==0X84)||(SENSOR_TYPE ==0X85)\
             ||(SENSOR_TYPE ==0X75)||(SENSOR_TYPE ==0X78) )
      GenericApp_applicationbuf=60001;
if(!(T_MGSbit&(0XC0)))  //1100 0000 判断是否为 关闭定时状态
{
      GenericApp_time=1;
#if defined( POWER_SAVING )
      GenericApp_time=25;
#endif
}
#endif
#if(SENSOR_TYPE ==0X23)
     if(SFG_R30X)
    {SFG_R30XA[0]=0X00;SFG_R30X=0;}
    else
    { SFG_R30XA[0]=0XA1;SFG_R30X=1;}
    
    GenericApp_applicationbuf=0;
   GenericApp_SendTheMessage();
#endif
#if(SENSOR_TYPE ==0X20)
       GenericApp_ON_OFF=10;
#endif
#if(SENSOR_TYPE ==0X79)
       P1_0=~P1_0; //P1_5=~P1_5;
      Send_data[7]=0X02;
          Send_data[8]=0XDD;
          Send_data[9]=P1_5;
           Send_data[9]=~((((Send_data[9]<<1)+P1_0)&0X03)|0XFC);
          Send_datalend=0x04;
          GenericApp_SendTheMessage(); //应用函数
#endif  
#if(SENSOR_TYPE ==0X83)
      P1_2=~P1_2; //P1_0=~P1_0;
      GenericApp_applicationbuf=0;
      GenericApp_SendTheMessage();
#endif  
#if((SENSOR_TYPE =='K')||(SENSOR_TYPE ==0X02)||(SENSOR_TYPE ==0X06)||(SENSOR_TYPE ==0X40)||\
  (SENSOR_TYPE ==0X09)||(SENSOR_TYPE ==0X12)||(SENSOR_TYPE ==0X18)||(SENSOR_TYPE ==0X15)\
     ||(SENSOR_TYPE ==0X29)||(SENSOR_TYPE ==0X31)||(SENSOR_TYPE ==0XA3))
     // P1_2=~P1_2; //P1_0=~P1_0;
      GenericApp_applicationbuf=0;
      GenericApp_SendTheMessage();
#endif
////////////////////////////////////////////////////////////////////////////////////////////
//OLED屏显示内容
OLED_Init(); //OLED初始化 
OLED_P8x16Str(0,0,"R       ID:");//第一行 -- 8x16的显示单元显示ASCII码
#if defined( SENSOR_TYPE_Coord )
#if (ZDO_COORDINATOR)
   OLED_P8x16Str(0,0,"C    PANID:");//第一行 -- 8x16的显示单元显示ASCII码
#endif 
#endif 
#if defined( POWER_SAVING )
 OLED_P8x16Str(0,0,"E       ID:");//第一行 -- 8x16的显示单元显示ASCII码
#endif
char *s=0;
uint16 nv_data; 
osal_nv_read(ZCD_NV_APP_CHANLIST,0,sizeof(nv_data),&nv_data);
sprintf(s, "%02d", (nv_data&0x00ff)); 
OLED_P8x16Str(16,0,s);//第一行 -- 8x16的显示单元显示ASCII码
#if !defined( SENSOR_TYPE_Coord )
sprintf(s, "%02X", SENSOR_TYPE); 
OLED_P8x16Str(40,0,s);//第一行 -- 8x16的显示单元显示ASCII码
#endif 
osal_nv_read(ZCD_NV_APP_PANID_H,0,sizeof(nv_data),&nv_data);
sprintf(s, "%04X", nv_data); 
OLED_P8x16Str(88,0,s);//第一行 -- 8x16的显示单元显示ASCII码
OLED_P8x16Str(0,2,"V2.45");
#if defined( CC2530_V30 )
OLED_P8x16Str(0,2,"V3.00");
#endif
sprintf(s, "%04X", NLME_GetCoordShortAddr()); 
OLED_P8x16Str(48,2,s);
sprintf(s, "%04X", NLME_GetShortAddr());
OLED_P8x16Str(88,2,s);
 byte * Send_d;
  Send_d=NLME_GetExtAddr();
sprintf(s, "%02X%02X%02X%02X%02X%02X%02X%02X", *Send_d,*(Send_d+1),*(Send_d+2),*(Send_d+3),*(Send_d+4),*(Send_d+5),*(Send_d+6),*(Send_d+7));
OLED_P8x16Str(0,4,s);
 //////////////////////////////////////////////////////////////////////////////////////////// 
      
#if !defined (SENSOR_TYPE_Coord)
    ZigBee_C_R_E_Engineering_F();
#endif
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      //HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );

      // Initiate an End Device Bind Request for the mandatory endpoint
      dstAddr.addrMode = Addr16Bit;
      dstAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(), 
                            GenericApp_epDesc.endPoint,
                            GENERICAPP_PROFID,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            FALSE );
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      // Initiate a Match Description Request (Service Discovery)
      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        GENERICAPP_PROFID,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        FALSE );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none

 */
void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{ 
#if defined( SENSOR_TYPE_Coord )
#if(ZDO_COORDINATOR==2)  //ZIGBEE  AT命令模式 透传模式
  
   if(FT_AT==1) // 透传模式
   {HalUARTWrite(HAL_UART_PORT_0, (uint8*)pkt->cmd.Data,pkt->cmd.DataLength);
   }
 #else
#if defined( CC2530_V30 )
  HalUARTWrite(HAL_UART_PORT_0, (uint8*)pkt->cmd.Data,pkt->cmd.Data[1]+11);
#else
  HalUARTWrite(HAL_UART_PORT_0, (uint8*)pkt->cmd.Data,pkt->cmd.Data[1]+6);
#endif
  
#endif
   HalLedSet ( HAL_LED_1, HAL_LED_MODE_TOGGLE );
#else

   
if((pkt->cmd.Data[6]==0x11)&&(pkt->cmd.Data[7]==0x22)&&(pkt->cmd.Data[8]==0x33)&&(pkt->cmd.Data[9]==0x44))
{if(pkt->cmd.Data[10]==0xCC){if(pkt->cmd.Data[12]==SENSOR_TYPE)openoff=pkt->cmd.Data[11];}
    else
    {
      char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)openoff=pkt->cmd.Data[11]; 
    }
}
#if((SENSOR_TYPE =='E')||(SENSOR_TYPE =='B')||(SENSOR_TYPE =='C')||(SENSOR_TYPE =='D')||(SENSOR_TYPE =='F')\
  ||(SENSOR_TYPE =='U')||(SENSOR_TYPE ==0X10)||(SENSOR_TYPE ==0X21)||(SENSOR_TYPE ==0X11)\
    ||(SENSOR_TYPE ==0X0F)||(SENSOR_TYPE ==0X13)||(SENSOR_TYPE ==0X14)||(SENSOR_TYPE ==0X17)\
      ||(SENSOR_TYPE ==0X24)||(SENSOR_TYPE ==0X5A)||(SENSOR_TYPE ==0X5B)||(SENSOR_TYPE ==0X6A)||(SENSOR_TYPE =='A')\
        ||(SENSOR_TYPE =='J')||(SENSOR_TYPE =='L')||(SENSOR_TYPE =='M')||(SENSOR_TYPE =='N')\
  ||(SENSOR_TYPE ==0X04)||(SENSOR_TYPE ==0X22)||(SENSOR_TYPE ==0X0C)||(SENSOR_TYPE ==0X16)||(SENSOR_TYPE ==0X19)\
    ||(SENSOR_TYPE ==0X61)||(SENSOR_TYPE ==0X62)||(SENSOR_TYPE ==0X63)||(SENSOR_TYPE ==0X1A)||(SENSOR_TYPE ==0X33)\
      ||(SENSOR_TYPE ==0X72) ||(SENSOR_TYPE ==0X34)||(SENSOR_TYPE ==0X35)||(SENSOR_TYPE ==0X36)||(SENSOR_TYPE ==0X37)\
        ||(SENSOR_TYPE ==0X38)||(SENSOR_TYPE ==0X39)||(SENSOR_TYPE ==0X3A)||(SENSOR_TYPE ==0X80)||(SENSOR_TYPE ==0X3D)\
          ||(SENSOR_TYPE ==0X81)||(SENSOR_TYPE ==0X82)||(SENSOR_TYPE ==0X3B)||(SENSOR_TYPE ==0X3C)||(SENSOR_TYPE ==0X3E)\
            ||(SENSOR_TYPE ==0X3F)||(SENSOR_TYPE ==0X84)||(SENSOR_TYPE ==0X85)||(SENSOR_TYPE ==0X75)||(SENSOR_TYPE ==0X78))
//定时3
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==0X00))
      { 
//定时发送命令
        if(pkt->cmd.Data[9]==0xAA)//定时发送命令
        {
           if((pkt->cmd.Data[7]==0xFF)&&(pkt->cmd.Data[8]==0XFF))
           { if(pkt->cmd.Data[11]==0xAA)//
            {T_MGSbit&=~0X80; //0XXXX XXXX
              T_MGSbit|=0X60; //x11x xxx0 默认定时 应答 正常 状态位
              GenericApp_time=1;
#if defined( POWER_SAVING )
              GenericApp_time=25;
#endif
              
              
            }
            if(pkt->cmd.Data[11]==0xBB)//
            { T_MGSbit&=~0XC0;//00XX XXXX 关闭定时 应答 正常 状态位
              TIMER3_RUN(FALSE);  //关闭定时
             GenericApp_time=0;;  
            }
           }
           else
           {
        #if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
        byte * Send_d;
        Send_d=NLME_GetExtAddr();
        for(int i=0;i<8;i++)
        {
        if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
          GetExtAddr++;
        }
           if(GetExtAddr==8)
        #else
      if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
        #endif    
         { if(pkt->cmd.Data[11]==0xAA)//
            {T_MGSbit&=~0X40; //x0xx xxxx
              T_MGSbit|=0XA0; //1x1x xxx0 设定定时  应答 正常 状态位
              /* 使能定时器3的溢出中断 */
             TIMER34_INIT(3);
             if(pkt->cmd.Data[10]<2)
               pkt->cmd.Data[10]=1;
             halSetTimer3Period(100);  //100MS
             T3_t1Val=(10*pkt->cmd.Data[10]); //pkt->cmd.Data[10]秒
             IEN1 |= (0x01 << 3);             // 使能Timer3的中断T3IE
             TIMER3_RUN(TRUE);  
             GenericApp_applicationbuf=60001;
             GenericApp_time=0;  
            }
            if(pkt->cmd.Data[11]==0xBB)//
            {T_MGSbit&=~0XC0;//00XX XXXX 关闭定时 应答 正常 状态位
              T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
              TIMER3_RUN(FALSE);  //关闭定时
             GenericApp_applicationbuf=60001;
             GenericApp_time=0;  
            }
          }
        } 
      } 
//按MAC地址单次取数据命令 (采集类型)
     if(pkt->cmd.Data[9]==0xA1)//按MAC地址单次取数据命令 (采集类型)
        {
        #if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
        byte * Send_d;
        Send_d=NLME_GetExtAddr();
        for(int i=0;i<8;i++)
        {
        if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
          GetExtAddr++;
        }
           if(GetExtAddr==8)
        #else
      if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
        #endif    
         { T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
             GenericApp_applicationbuf=60001;
         }
      }
  }
#endif 
//按类别触发命令
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
  {  
    if((pkt->cmd.Data[2]==0XFF)&&(pkt->cmd.Data[3]==0XFF))
      {T_MGSbit|=0X21; //xx1x xxx1 应答 触发 状态位
      if(Send_datalend==0)Send_datalend=3;
       if(Send_datalend>0X20)Send_datalend=3;
       GenericApp_applicationbuf=0;
       GenericApp_SendTheMessage();
      }
  }


#if(SENSOR_TYPE =='S')
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xAB)//调光
         { if(pkt->cmd.Data[10]==0xA1)  
            {GenericApp_ON_OFF=pkt->cmd.Data[11];
              DimmingAppControl(GenericApp_ON_OFF,0x00);
            GenericApp_applicationbuf=1;
            }
           if(pkt->cmd.Data[10]==0xB2)  
            { GenericApp_ON_OFF=pkt->cmd.Data[11];
              DimmingAppControl(GenericApp_ON_OFF,0x01);
             GenericApp_applicationbuf=1;
            }
         }
       if(pkt->cmd.Data[9]==0xCC)//检测设备
         {
          GenericApp_applicationbuf=2;
         }
         GenericApp_SendTheMessage();
      }
      }
#endif
#if(SENSOR_TYPE =='Q')
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xAB)//默认
         { if(pkt->cmd.Data[10]==0xA1) //设置电子秤系数   50-150之间
            {if((pkt->cmd.Data[11]>49)&&(pkt->cmd.Data[11]<151))
              AD_SHINING=pkt->cmd.Data[11];
               uint16 nv_data=0X0000;
               nv_data=AD_SHINING;
            osal_nv_item_init( ZCD_NV_APP_AD_SHINING,sizeof(nv_data), &nv_data );
            osal_nv_write( ZCD_NV_APP_AD_SHINING, 0,sizeof(nv_data),&nv_data);
            Send_datalend=0x04;  
            Send_data[7]=0;//数值为0
            Send_data[8]=AD_SHINING;//返回设置系数
            Send_data[9]=(Send_data[6]+Send_data[7]+Send_data[8])%256;
            }
         }
         GenericApp_SendTheMessage();
      }
      }
#endif

#if(SENSOR_TYPE ==0X29)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
      #if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
        byte * Send_d;
        Send_d=NLME_GetExtAddr();
        for(int i=0;i<8;i++)
        {
        if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
          GetExtAddr++;
        }
           if(GetExtAddr==8)
      #else
           if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
      #endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xAF)//点阵价格修改
        {
          if(pkt->cmd.Data[10]<5)
             HalUARTWrite(HAL_UART_PORT_0,&pkt->cmd.Data[10],pkt->cmd.Data[10]+1); //03 05 06 bb // 02 05 BB
          else 
            HalUARTWrite(HAL_UART_PORT_0,&pkt->cmd.Data[10],3); //aa 05 bb
           GenericApp_applicationbuf=1;
        } 
        if(pkt->cmd.Data[9]==0xAD)//检测设备
        {
          GenericApp_applicationbuf=2;
        }
        GenericApp_SendTheMessage();
      }
      }  
#endif
#if(SENSOR_TYPE ==0X31)
      if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
      #if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
        byte * Send_d;
        Send_d=NLME_GetExtAddr();
        for(int i=0;i<8;i++)
        {
        if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
          GetExtAddr++;
        }
           if(GetExtAddr==8)
      #else
           if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
      #endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xB4)//向小车发命令
        {
            HalUARTWrite(HAL_UART_PORT_0,&pkt->cmd.Data[10],3);
            //HalUARTWrite(HAL_UART_PORT_0,&pkt->cmd.Data[10],3);
            //GenericApp_applicationbuf=1;
        }
        if(pkt->cmd.Data[9]==0xED)//关闭读卡自动停车
        {
           car_stata=0X01;//关闭读卡自动停车
            GenericApp_applicationbuf=1;
            GenericApp_SendTheMessage();
        }
        
        if((pkt->cmd.Data[9]==0xEA)&&(pkt->cmd.Data[11]==0xEA))
        { HalUARTWrite(HAL_UART_PORT_0,&pkt->cmd.Data[9],3);
        }
     //   GenericApp_SendTheMessage();
      }
      }  
#endif

#if(SENSOR_TYPE ==0X67)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
      unsigned int dat16;
      if(pkt->cmd.Data[9]==0xAA)//正转
         {  P0_4=1;P0_5=0;
            P1_0=1;P1_1=0;
            dat16=(pkt->cmd.Data[10])&0X0F;
            dat16=((dat16<<8)+pkt->cmd.Data[11]);
            DAchange(dat16);
            GenericApp_applicationbuf=1;
            }
           if(pkt->cmd.Data[9]==0xBB)  
            {P0_4=0;P0_5=1;
             P1_0=0;P1_1=1;
             dat16=(pkt->cmd.Data[10])&0X0F;
            dat16=((dat16<<8)+pkt->cmd.Data[11]);
            DAchange(dat16);
             GenericApp_applicationbuf=1;
            }
         if(pkt->cmd.Data[9]==0xCC)  
            {P0_4=0;P0_5=0;
             P1_0=0;P1_1=0;
            DAchange(0);
             GenericApp_applicationbuf=1;
            }
         GenericApp_SendTheMessage();
         }
      }
#endif

#if(SENSOR_TYPE ==0X6D)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
{ 
    // if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))  
     if((pkt->cmd.Data[7]==0X01)&&(pkt->cmd.Data[8]==0X02))  
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        GenericApp_applicationbuf=1;
          HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[9]),pkt->cmd.Data[1]-3);
            GenericApp_SendTheMessage();
      } 
      }  
#endif
#if(SENSOR_TYPE ==0X30)
 if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
      if(pkt->cmd.Data[9]==0xAA)//2向4线步进电机
         { if(pkt->cmd.Data[10]==0xAB) //正转
              {a68=0xAB;b68=pkt->cmd.Data[11];
              }
           if(pkt->cmd.Data[10]==0xBA) //反转
              {a68=0xBA;b68=pkt->cmd.Data[11];
              }
            GenericApp_applicationbuf=1;
          }
        if(pkt->cmd.Data[9]==0xAF)//2向4线步进电机窗帘开启关闭
         { 
           if(pkt->cmd.Data[10]==0xAB) //正转//窗帘开启
              {a68=0xAB;b68=250;
              }
           if(pkt->cmd.Data[10]==0xBA) //反转//窗帘关闭
              {a68=0xBA;b68=250;
              }
            GenericApp_applicationbuf=1;
          }
        if(pkt->cmd.Data[9]==0xB0)//2向4线步进电机
         { if((pkt->cmd.Data[10]==0xAA)&&(pkt->cmd.Data[12]==0xBB)) //正转
              {
                if(pkt->cmd.Data[11]==0x01)//关闸
                {c68=0;if(a68!=0xab) {a68=0xab;b68=B01;}}
                 if(pkt->cmd.Data[11]==0x02) //开闸
                 {c68=150; if(a68!=0xba){a68=0xba;b68=B01;}}
              GenericApp_applicationbuf=1;
              } 
          }
         GenericApp_SendTheMessage();
         }

      }
#endif
#if(SENSOR_TYPE ==0X73)
 if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
           if(pkt->cmd.Data[10]==0xA8)//8路堆垛机
         {pkt->cmd.Data[9]=0x05;
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[9]),0X06);
          }
           if(pkt->cmd.Data[10]==0xAC)//磁线控制器
         {pkt->cmd.Data[9]=0x05;
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[9]),0X06);
          }
        if(pkt->cmd.Data[10]==0xA1)//堆垛机
         { 
           if(pkt->cmd.Data[11]==0xAB) //正转//
              {
                a68=0xAB;b68=5000;
              b68=(((int)(pkt->cmd.Data[12])<<8)+(pkt->cmd.Data[13]));
              }
           if(pkt->cmd.Data[11]==0xBA) //反转//
              {a68=0xba;b68=5000;
             b68=(((int)(pkt->cmd.Data[12])<<8)+(pkt->cmd.Data[13]));
              }
            GenericApp_applicationbuf=1;
           Send_data[10]=0X00;
         GenericApp_SendTheMessage();
          }
         }

      }
#endif
#if(SENSOR_TYPE ==0X6A)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        GenericApp_switch=1;
        Send_data[7]=pkt->cmd.Data[9];
         GenericApp_SendTheMessage();
         }
      }
#endif
#if(SENSOR_TYPE ==0X6B)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xDD) //led屏
         {  uint8  hrx[4]={0X02,0x33,0x01,0X00};
       hrx[2]=pkt->cmd.Data[10];
       HalUARTWrite(HAL_UART_PORT_0,hrx,4);
            GenericApp_applicationbuf=1;
            }
        if(pkt->cmd.Data[9]==0xDE) //DE:A8广告机
         {  
       HalUARTWrite(HAL_UART_PORT_0,&pkt->cmd.Data[9],3);
            GenericApp_applicationbuf=1;
            }
          if(pkt->cmd.Data[9]==0xDC) //DC:字库LED
         {  
       HalUARTWrite(HAL_UART_PORT_0,&pkt->cmd.Data[12],pkt->cmd.Data[13]+2);
            GenericApp_applicationbuf=1;
            }
        Send_data[7]=pkt->cmd.Data[9];
         GenericApp_SendTheMessage();
         }
      }
#endif
#if(SENSOR_TYPE ==0X65)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        show_right_down(pkt->cmd.Data[10]);
        GenericApp_switch=1;
         GenericApp_SendTheMessage();
         }
      }
#endif
#if(SENSOR_TYPE ==0X6C)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
      //  show_right_down(pkt->cmd.Data[10]);
        if(pkt->cmd.Data[10]==0x02)//
     {switch(pkt->cmd.Data[11])
         {case 0x0B:P1_1=0; break;//11路
         case 0x16:P1_2=0; break;//22路
         case 0x37:P1_3=0; break;//55路
         case 0x58:P1_4=0; break;//88路
        default:break;
         }
      }
      lcd_txt_clr(); 
      switch(pkt->cmd.Data[9])
         {case 0x01:txt_disp(str2,0x80,16); break;
         case 0x02:txt_disp(str3,0x80,16); break;
         case 0x03:txt_disp(str4,0x80,16); break;
         case 0x04:txt_disp(str5,0x80,16); break;
        default:break;
         }
          switch(pkt->cmd.Data[10])
         {case 0x01:hzkdis(1,tab1); break;
         case 0x02:hzkdis(1,tab2); break;
         case 0x03:hzkdis(1,tab3); break;
        default:break;
         }
       txt_disp(&str1[(pkt->cmd.Data[11]/10)*2],0x90,2);//80 90 88 98
       txt_disp(&str1[(pkt->cmd.Data[11]%10)*2],0x91,2);//80 90 88 98
        if(pkt->cmd.Data[10]==0x03)
        { txt_disp(&str1[(pkt->cmd.Data[12]/10)*2],0x8a,2);//80 90 88 98
       txt_disp(&str1[(pkt->cmd.Data[12]%10)*2],0x8b,2);//80 90 88 98
       txt_disp(&str1[((pkt->cmd.Data[12]*5)/10)*2],0x98,2);//80 90 88 98
       txt_disp(&str1[((pkt->cmd.Data[12]*5)%10)*2],0x99,2);//80 90 88 98
        }
         
    P1|=0X1E;
   
        GenericApp_switch=1;
         GenericApp_SendTheMessage();
      }
      }
#endif
#if(SENSOR_TYPE ==0X66)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        LedNumVal=pkt->cmd.Data[9];
        GenericApp_switch=1;
         GenericApp_SendTheMessage();
         }
      }
#endif
#if(SENSOR_TYPE ==0X6E)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
      if(pkt->cmd.Data[9]==0X01)//16路继电器控制
      {
        SendDataS(&(pkt->cmd.Data[11]),pkt->cmd.Data[10]);
        GenericApp_switch=1;
         GenericApp_SendTheMessage();
      }
         }
      }
#endif
#if(SENSOR_TYPE ==0XA3)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
      #if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
        byte * Send_d;
        Send_d=NLME_GetExtAddr();
        for(int i=0;i<8;i++)
        {
        if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
          GetExtAddr++;
        }
           if(GetExtAddr==8)
      #else
           if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
      #endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xB5)//向红绿灯发送指令
        {
            HalUARTWrite(HAL_UART_PORT_0,&pkt->cmd.Data[10],5);
            GenericApp_applicationbuf=0;
        } 
        if(pkt->cmd.Data[9]==0xAD)//检测设备
        {
          GenericApp_applicationbuf=2;
        }
        GenericApp_SendTheMessage();
      }
      }  
#endif
#if(SENSOR_TYPE ==0X05)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xDD)//控制220V继电器
         {uint8 uartkey=0;
           if(pkt->cmd.Data[10]==0xA3)  
            {  GenericApp_applicationbuf=1;
              if(0xAA==(pkt->cmd.Data[11]))
              { P1_7=0;//停止
              uartkey=0x02;
             // HalUARTWrite(HAL_UART_PORT_0,&uartkey,1);//发射停命令 02
               for(int i=0;i<30000;i++)
                 for(int i=0;i<100;i++)
                   asm("NOP");
                P1_6=1;//开
                uartkey=0x01;
                 HalUARTWrite(HAL_UART_PORT_0,&uartkey,1);//发射开命令 01
          GenericApp_BX=200;
          GenericApp_ON_OFF=0XAA;
              }
              if(0xA1==(pkt->cmd.Data[11]))
              { if(GenericApp_ON_OFF==0XA2)
                {P1_6=0;//停止
               for(int i=0;i<30000;i++)
                 for(int i=0;i<100;i++)
                   asm("NOP");
                P1_7=1;//开
               GenericApp_BX=50;
               GenericApp_ON_OFF=0XA1;
              }
              if(GenericApp_ON_OFF==0XAA)
                {P1_6=0;//停止
               for(int i=0;i<30000;i++)
                 for(int i=0;i<100;i++)
                   asm("NOP");
                P1_7=1;//开
               GenericApp_BX=100;
               GenericApp_ON_OFF=0XA1;
              }
              if(GenericApp_ON_OFF==0XBB)
                {P1_7=0;//停止
               for(int i=0;i<30000;i++)
                 for(int i=0;i<100;i++)
                   asm("NOP");
                P1_6=1;//开
               GenericApp_BX=50;
               GenericApp_ON_OFF=0XA1;
              }
              }
               if(0xA2==(pkt->cmd.Data[11]))
              { if(GenericApp_ON_OFF==0XA1)
                {P1_7=0;//停止
               for(int i=0;i<30000;i++)
                 for(int i=0;i<100;i++)
                   asm("NOP");
                P1_6=1;//开
               GenericApp_BX=50;
               GenericApp_ON_OFF=0XA2;
              }
              if(GenericApp_ON_OFF==0XAA)
                {P1_6=0;//停止
               for(int i=0;i<30000;i++)
                 for(int i=0;i<100;i++)
                   asm("NOP");
                P1_7=1;//开
               GenericApp_BX=50;
               GenericApp_ON_OFF=0XA2;
              }
               if(GenericApp_ON_OFF==0XBB)
                {P1_7=0;//停止
               for(int i=0;i<30000;i++)
                 for(int i=0;i<100;i++)
                   asm("NOP");
                P1_6=1;//开
               GenericApp_BX=100;
               GenericApp_ON_OFF=0XA2;
              }
              }
              if((pkt->cmd.Data[11])==0xBB)
              { uartkey=0x02;
                // HalUARTWrite(HAL_UART_PORT_0,&uartkey,1);//发射停命令 02
                P1_6=0; //停止
                  for(int i=0;i<30000;i++)
                    for(int i=0;i<100;i++)
                   asm("NOP");
                P1_7=1;   //关
                uartkey=0x03;
                 HalUARTWrite(HAL_UART_PORT_0,&uartkey,1);//发射管命令 03
                GenericApp_BX=200;
             GenericApp_ON_OFF=0XBB;
              }
               if((pkt->cmd.Data[11])==0xCC)
               {uartkey=0x02;
                 HalUARTWrite(HAL_UART_PORT_0,&uartkey,1);//发射停命令 02
                 P1_7=0;   //停止
                P1_6=0;  //停止
               }
            }
         }
        GenericApp_SendTheMessage();
      } 
      }  
#endif
#if(SENSOR_TYPE ==0X06)
    uint8 DD[3]={0x00,0X00,0x00};
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {OSAL_SET_CPU_INTO_SLEEP=1;
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xDD)//智能小车
         {
           DD[0]=pkt->cmd.Data[10];
           if(!DD[0])DD[0]=0XBB;  //以前是发送 0x00 小车停止 现在强制改为 0xBB小车停止
           HalUARTWrite(HAL_UART_PORT_0, (uint8*)DD,1);
            GenericApp_applicationbuf=1;
             GenericApp_SendTheMessage();
         }
        if((pkt->cmd.Data[9]==0xEA)&&(pkt->cmd.Data[11]==0xEA))//采集温度命令
        {uint8 hr[15]={0};
         for(int i=0;i<(pkt->cmd.Data[10]+3);i++)
           hr[i]=pkt->cmd.Data[9+i];
           unsigned int  data;						    
           float Temperature;
           Temperature = ReadTemperature();             //返回16位二进制数
           Temperature *= CelsiurPerLSB;		//转换成浮点型温度值(摄氏度)
           data=(unsigned int)(100*Temperature);   
           hr[2]=(uint8)(data/ 100);
            if(hr[2]>0x30)
            { Temperature = ReadTemperature();             //返回16位二进制数
           Temperature *= CelsiurPerLSB;		//转换成浮点型温度值(摄氏度)
           data=(unsigned int)(100*Temperature);   
           hr[2]=(uint8)(data/ 100);
            }
            hr[1]=pkt->cmd.Data[10];
           if(hr[2]<0x30)
           {
           if((pkt->cmd.Data[10]))
           HalUARTWrite(HAL_UART_PORT_0, (uint8*)hr,(pkt->cmd.Data[10]+3));
           GenericApp_applicationbuf=2; 
           Send_data[8]=hr[2];
           GenericApp_SendTheMessage();
           }
         }
      } 
         OSAL_SET_CPU_INTO_SLEEP=0;
      }
#endif
#if(SENSOR_TYPE ==0X07)

if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {OSAL_SET_CPU_INTO_SLEEP=1;
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xA1)
         { AB=0X0B;
           HalUARTWrite(HAL_UART_PORT_0, &pkt->cmd.Data[11],pkt->cmd.Data[10]);
         }
      } 
      }
#endif
#if(SENSOR_TYPE ==0X09)
    char DD[3]={0x00,0X00,0x00};
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {OSAL_SET_CPU_INTO_SLEEP=1;
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {  T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        //开关量                  //数码管                  //步进电机
        if((pkt->cmd.Data[9]==0xDD)||(pkt->cmd.Data[9]==0xEA)||(pkt->cmd.Data[9]==0xEB))
         {
           DD[0]=pkt->cmd.Data[9];
           DD[1]=pkt->cmd.Data[10];
           DD[2]=pkt->cmd.Data[11];
           HalUARTWrite(HAL_UART_PORT_0, (uint8*)DD,3);
            GenericApp_applicationbuf=1;
             GenericApp_SendTheMessage();
         }
      } 
         OSAL_SET_CPU_INTO_SLEEP=0;
      }
#endif
#if(SENSOR_TYPE ==0X13)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if((pkt->cmd.Data[9]==0X43)&&(pkt->cmd.Data[10]==0XC3)&&(pkt->cmd.Data[11]==0X6F))//读有功电总量
         {
           GenericApp_applicationbuf=60001;
            GenericApp_time=0;
         }
      }
      }  
#endif  

#if(SENSOR_TYPE ==0XF0)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        HalUARTWrite(HAL_UART_PORT_0,&pkt->cmd.Data[9],1);
        GenericApp_applicationbuf=1;
        GenericApp_SendTheMessage();
      }
      }  
#endif  
#if(SENSOR_TYPE ==0X70)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
         {
        T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        GenericApp_applicationdata=0;
        P1_0=1; 
        GenericApp_uart_485=0;
         if((pkt->cmd.Data[10]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[11]==0X0F))
      {
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
        // Send_datalend=14;for(char i=0;i<11;i++)Send_data[8+i]=pkt->cmd.Data[10+i];
       //   Send_data[8]=0XF1; Send_data[12]=((Send_data[12]&0XF0)|0X0A);
         // GenericApp_SendTheMessage();
      }
      if((pkt->cmd.Data[10]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[11]==0X0E))
      {
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
         // Send_datalend=14;for(char i=0;i<11;i++)Send_data[8+i]=pkt->cmd.Data[10+i];
         // Send_data[8]=0XF1; Send_data[12]=((Send_data[12]&0XF0)|0X0A);
        //  GenericApp_SendTheMessage();
      }
      } 
      }  
#endif  
#if(SENSOR_TYPE ==0X74)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
   if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if((pkt->cmd.Data[9]==0X01)&&(pkt->cmd.Data[11]<=0X10))
         {GenericApp_applicationbuf=60001;//应答
          HalUARTWrite(HAL_UART_PORT_0, &pkt->cmd.Data[12],pkt->cmd.Data[11]);
         }
      }
      }  
#endif  
#if(SENSOR_TYPE ==0X14)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
   if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if((pkt->cmd.Data[9]==0X01)&&(pkt->cmd.Data[10]==0X03)&&(pkt->cmd.Data[11]==0X0C))//读有功电总量
         {     GenericApp_applicationbuf=60001;
                GenericApp_time=0;

         //uint8  hr[8]={0x01,0x03,0x00,0x00,0x00,0x0C,0xA5,0xE2};
              //  HalUARTWrite(HAL_UART_PORT_0, hr,8);
         }
      }
      }  
#endif  

#if(SENSOR_TYPE ==0X15)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xDD)//
         {GenericApp_applicationbuf=1;
           if(pkt->cmd.Data[10]==0x01)  
            {
              P1_1=0;  
            }
           if(pkt->cmd.Data[10]==0x02)  
            {
              P1_2=0;  
            }
           if(pkt->cmd.Data[10]==0x03)  
            {
              P1_3=0;  
            }
            if(pkt->cmd.Data[10]==0x04)  
            {
              P1_4=0;  
            }
           GenericApp_SendTheMessage();
         }
        
      }
      }  
#endif  
   
#if(SENSOR_TYPE =='H')
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xDD)//控制220V继电器
         {GenericApp_applicationbuf=1;
           if(pkt->cmd.Data[10]==0x01)  
            {
              if(0xAA==pkt->cmd.Data[11])
              { P1_0=0;   //开继电器
              }if(pkt->cmd.Data[11]==0xBB)
              P1_0=1;   //关继电器
            }
           if(pkt->cmd.Data[10]==0xC0)  
            {  
              if(0xAA==pkt->cmd.Data[11])
              { P1_7=1;   P1_6=0;}   //窗帘开
              if(pkt->cmd.Data[11]==0xBB)
              {  P1_6=1; P1_7=0;}   //窗帘关
            }
         }
       if(pkt->cmd.Data[9]==0xCC)//检测设备
         {
          GenericApp_applicationbuf=2;
         }
         GenericApp_SendTheMessage();
      } 
      }  
#endif
#if(SENSOR_TYPE ==0X7C)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { //uint8  hr[8]={0X01,0x03,0x06,0X80,0X00,0X89,0X0D,0X0A};
        T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        GenericApp_applicationdata=0;
         if((pkt->cmd.Data[11]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[12]==0X0F))
      {
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[11]),pkt->cmd.Data[12]+2);
      }
      if((pkt->cmd.Data[11]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[12]==0X0E))
      {
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[11]),pkt->cmd.Data[12]+2);
      }
      } 
}  
#endif
#if(SENSOR_TYPE ==0X25)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { //uint8  hr[8]={0X01,0x03,0x06,0X80,0X00,0X89,0X0D,0X0A};
        T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        GenericApp_applicationdata=0;
         if((pkt->cmd.Data[10]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[11]==0X0F))
      {
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
      }
      if((pkt->cmd.Data[10]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[11]==0X0E))
      {
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
      }
      } 
}  
#endif

#if(SENSOR_TYPE ==0X7B)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { //uint8  hr[8]={0X01,0x03,0x06,0X80,0X00,0X89,0X0D,0X0A};
        T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        GenericApp_applicationdata=0;
         if((pkt->cmd.Data[10]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[11]==0X0F))
      {
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
      }
      if((pkt->cmd.Data[10]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[11]==0X0E))
      {
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
      }
      } 
}  
#endif
#if(SENSOR_TYPE ==0X71)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { //uint8  hr[8]={0X01,0x03,0x06,0X80,0X00,0X89,0X0D,0X0A};
        T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        GenericApp_applicationdata=0;
         if((pkt->cmd.Data[10]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[11]==0X0F))
      { if((pkt->cmd.Data[12]==0xAA)||(pkt->cmd.Data[12]==0xBB))
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
        else
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[14]),1);
        //HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
      }
      if((pkt->cmd.Data[10]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[11]==0X0E))
      {if((pkt->cmd.Data[12]==0xAA)||(pkt->cmd.Data[12]==0xBB))
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
        else
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[14]),1);
        //HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
      }
      } 
}  
#endif

#if(SENSOR_TYPE ==0X0A)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { //uint8  hr[8]={0X01,0x03,0x06,0X80,0X00,0X89,0X0D,0X0A};
       // T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        GenericApp_applicationdata=0;
       if((pkt->cmd.Data[10]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[11]==0X0F))
      {   HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
      }
      if((pkt->cmd.Data[10]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[11]==0X0E))
      {   HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
      }
      } 
}  
#endif
#if(SENSOR_TYPE ==0X86)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { //uint8  hr[8]={0X01,0x03,0x06,0X80,0X00,0X89,0X0D,0X0A};
        T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        GenericApp_applicationdata=0;
       if((pkt->cmd.Data[11]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[10]==0X0D))
      {
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[10]+1);
      }
      if((pkt->cmd.Data[11]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[10]==0X0E))
      {
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[10]+1);
      }
      } 
}  
#endif
#if(SENSOR_TYPE ==0X87)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { //uint8  hr[8]={0X01,0x03,0x06,0X80,0X00,0X89,0X0D,0X0A};
        T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        GenericApp_applicationdata=0;
        if((pkt->cmd.Data[12]=='{')&&((pkt->cmd.Data[1]-pkt->cmd.Data[11])==0X0E))//07H {M1:00}
      {
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[12]),pkt->cmd.Data[11]);
      }
      if((pkt->cmd.Data[12]=='{')&&((pkt->cmd.Data[1]-pkt->cmd.Data[11])==0X0F))
      {
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[12]),pkt->cmd.Data[11]);
      }
      } 
}  
#endif
#if(SENSOR_TYPE ==0X26)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { //uint8  hr[8]={0X01,0x03,0x06,0X80,0X00,0X89,0X0D,0X0A};
        T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        GenericApp_applicationdata=0;
         if((pkt->cmd.Data[10]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[11]==0X0F))
      {
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
      }
      if((pkt->cmd.Data[10]==0xF7)&&(pkt->cmd.Data[1]-pkt->cmd.Data[11]==0X0E))
      {
        HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
      }
      } 
}  
#endif
#if(SENSOR_TYPE ==0X27)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { //uint8  hr[8]={0X01,0x03,0x06,0X80,0X00,0X89,0X0D,0X0A};
        T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        GenericApp_applicationdata=0;
        SYN_FrameInfo(0,pkt->cmd.Data[11],&pkt->cmd.Data[12],pkt->cmd.Data[1]-0X0E) ;
      //  HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
        
      } 
}  
#endif

#if(SENSOR_TYPE ==0X28)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { //uint8  hr[8]={0X01,0x03,0x06,0X80,0X00,0X89,0X0D,0X0A};
        T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位

         if((pkt->cmd.Data[9])==0XA1)
         {LCD_Init();   // LCD初始化
          LCD_Clear();  // LCD清屏
          /* 显示信息 */
          LCD_WriteStringL(0,pkt->cmd.Data[11],&pkt->cmd.Data[12],pkt->cmd.Data[1]-14);
         }
       
        GenericApp_switch=1;
        GenericApp_SendTheMessage();
      } 
}  
#endif
#if(SENSOR_TYPE ==0X6F)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      { //uint8  hr[8]={0X01,0x03,0x06,0X80,0X00,0X89,0X0D,0X0A};
        T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
         if(((pkt->cmd.Data[10])&0x0F)==0X00)
         { lcd_txt_clr(); 
         hzkdis_b(0,&pkt->cmd.Data[12],(pkt->cmd.Data[1]-0X0E)<64? (pkt->cmd.Data[1]-0X0E):64);
         }
         if(((pkt->cmd.Data[10])&0x0F)==0X01)
         { hzkdis_b(1,"                                                2",48);
         hzkdis_b(1,&pkt->cmd.Data[12],(pkt->cmd.Data[1]-0X0E)<48? (pkt->cmd.Data[1]-0X0E):48);
         }
        if(((pkt->cmd.Data[10])&0x0F)==0X02)
        {hzkdis_b(1,"                                 2",32);
          hzkdis_b(2,&pkt->cmd.Data[12],(pkt->cmd.Data[1]-0X0E)<32? (pkt->cmd.Data[1]-0X0E):32);
        }
        if(((pkt->cmd.Data[10])&0x0F)==0X03)
        {hzkdis_b(1,"                  2",16);
          hzkdis_b(3,&pkt->cmd.Data[12],(pkt->cmd.Data[1]-0X0E)<16? (pkt->cmd.Data[1]-0X0E):16);
        }
        if(((pkt->cmd.Data[10])&0xF0)==0XA0)
        SYN_FrameInfo(0,pkt->cmd.Data[11],&pkt->cmd.Data[12],pkt->cmd.Data[1]-0X0E) ;
      //  HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[10]),pkt->cmd.Data[11]+2);
        GenericApp_switch=1;
        GenericApp_SendTheMessage();
      } 
}  
#endif
#if(SENSOR_TYPE =='P')
    uint8  hr[8]={0X01,0x03,0x06,0X80,0X00,0X89,0X0D,0X0A};
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        GenericApp_applicationbuf=1;
         hr[3]=pkt->cmd.Data[9];
          hr[4]=pkt->cmd.Data[10];
          hr[5]=0x09+hr[3]+hr[4];
           HalUARTWrite(HAL_UART_PORT_0, hr,8);
           GenericApp_SendTheMessage();
      } 
      }  
#endif
#if(SENSOR_TYPE ==0X02)
    uint8  hr[2]={0XFF,0x10};
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0XDC)
        { GenericApp_applicationbuf=1;
         //hr[0]=pkt->cmd.Data[9];
          hr[0]=pkt->cmd.Data[10];
          hr[1]=pkt->cmd.Data[11];
           HalUARTWrite(HAL_UART_PORT_0, hr,1);
         }
        GenericApp_SendTheMessage();
      }  
      }  
#endif
#if(SENSOR_TYPE =='K')
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xDD)//控制220V继电器
         {GenericApp_applicationbuf=1;
           if(pkt->cmd.Data[10]==0x01)  
            {if(pkt->cmd.Data[11]==0xAA)
            {P1_0=0;T_MG=0X01; }   //开继电器
             if(pkt->cmd.Data[11]==0xBB)
             {P1_0=1; T_MG=0X00; }  //关继电器
              if(pkt->cmd.Data[11]==0xCC)
             {P1_0=0; T_MG=0X75; }  //关继电器开 0X75  10秒关闭
            }
         }
        if(pkt->cmd.Data[9]==0xEF)//控制220V继电器
         {GenericApp_applicationbuf=1;
           if(pkt->cmd.Data[11]<=0x01)  
              T_MG=0x0A; //关继电器开  1秒关闭
           else T_MG=pkt->cmd.Data[11]; //关继电器开  T_MG*80ms关闭
         if(pkt->cmd.Data[10]==0x01)  
           P1_0=0; 
         }
       if(pkt->cmd.Data[9]==0xCC)//检测设备
         {
          GenericApp_applicationbuf=2;
         }
        GenericApp_SendTheMessage();
      }
      }  
#endif  
#if(SENSOR_TYPE ==0X79)

if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[10]==0xDD)//控制
         {
           if(pkt->cmd.Data[11]>0)  
            {
              switch (pkt->cmd.Data[11]) 
    { case 0X01:if((pkt->cmd.Data[12])&0X01)P1_0=0;else P1_0=1;
                break;
      case 0X02:if((pkt->cmd.Data[12])&0X02)P1_5=0;else P1_5=1;
                break;
      case 0X03:if((pkt->cmd.Data[12])&0X01)P1_0=0;else P1_0=1;if((pkt->cmd.Data[12])&0X02)P1_5=0;else P1_5=1;
                break;
       default: break;
    }
            }
          Send_data[7]=0X02;
          Send_data[8]=0XDD;
          Send_data[9]=P1_5;
           Send_data[9]=~((((Send_data[9]<<1)+P1_0)&0X03)|0XFC);
          Send_datalend=0x04;
          GenericApp_SendTheMessage(); //应用函数
         }
         if(pkt->cmd.Data[10]==0xD1)//温湿度
         {PDI=0XD1;
         }
         if(pkt->cmd.Data[10]==0xD2)//AD采集
         {PDI=0XD2;
         }
       // GenericApp_SendTheMessage();
      }
      }  
#endif  
#if(SENSOR_TYPE ==0X83)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[10]==0xDD)//控制220V继电器
         {GenericApp_applicationbuf=1;
           if(pkt->cmd.Data[11]>0)  
            {
              PDI=((P1&(~(pkt->cmd.Data[11])))|((~((pkt->cmd.Data[11])&(pkt->cmd.Data[12]))))&(pkt->cmd.Data[11]));
              P1=PDI;
            }
         }
        GenericApp_SendTheMessage();
      }
      }  
#endif  
#if(SENSOR_TYPE ==0X40)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  { 
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xDD)//
         {GenericApp_applicationbuf=1;
         if(pkt->cmd.Data[10]!=0XB1)T_MG=0X00;
      switch (pkt->cmd.Data[10]) 
    { case 0XA1:if(pkt->cmd.Data[11]==0xAA){ P1_3=0; P1_4=1;  }  //LED1 红色 点亮
                if(pkt->cmd.Data[11]==0xBB){P1_3=1;P1_4=0;  }  //LED1 红色 熄灭
                break;
      case 0XA2:if(pkt->cmd.Data[11]==0xAA){P1_2=0; P1_7=1; }   //LED2 蓝色 点亮
                if(pkt->cmd.Data[11]==0xBB){P1_2=1;P1_7=0;}   //LED2 蓝色 熄灭
                break;
      case 0XA3: if(pkt->cmd.Data[11]==0xAA){T_MG=0X01;P1_2=1;P1_7=0;}   //LED2熄灭 LED1闪烁功能
                if(pkt->cmd.Data[11]==0xBB){T_MG=0X00;P1_3=1;P1_4=0;}   //LED1停止闪烁熄灭
                break;
      case 0XA4:if(pkt->cmd.Data[11]==0xAA){P1_2=1; P1_7=0; P1_3=0; P1_4=1; } //LED1打开，LED2关闭功能
                if(pkt->cmd.Data[11]==0xBB){P1_2=1; P1_7=0; P1_3=1; P1_4=0;}//LED1，LED2关闭   
                break;
      case 0XA5:if(pkt->cmd.Data[11]==0xAA){P1_2=0; P1_7=1; P1_3=1; P1_4=0; } //LED2打开，LED1关闭功能
                if(pkt->cmd.Data[11]==0xBB){P1_2=1; P1_7=0; P1_3=1; P1_4=0;}//LED1，LED2关闭   
                break;
      case 0XB1:if(pkt->cmd.Data[11]==0xAA){ P1_1=0; P1_0=1;  }   //开蜂鸣器
                  if(pkt->cmd.Data[11]==0xBB){ P1_1=1;P1_0=0;}   //关蜂鸣器
                break;
       default: break;}GenericApp_SendTheMessage();
         } 
      }
}  
#endif  
#if(SENSOR_TYPE ==0X23)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      {
#if defined( ZigBee_C_R_E_IEEE )
        char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {  T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xAA)//录入
         { SFG_R30XA[0]=0X01;
           SFG_R30XA[1]=pkt->cmd.Data[10];  SFG_R30XA[2]=pkt->cmd.Data[11]; 
           SFG_R30X=1;
         }
         if(pkt->cmd.Data[9]==0xBB)//搜索
         {  SFG_R30XA[0]=0XA1;SFG_R30X=1;
          }
          if(pkt->cmd.Data[9]==0xCC)//删除
         {  SFG_R30XA[0]=0X21;
          SFG_R30XA[1]=pkt->cmd.Data[10];  SFG_R30XA[2]=pkt->cmd.Data[11]; 
         SFG_R30X=1;
         }
      }
      }
#endif
#if(SENSOR_TYPE ==0X12)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xDD)//
         {GenericApp_applicationbuf=1;
           if(pkt->cmd.Data[10]==0x01)  
            {if(pkt->cmd.Data[11]==0xAA)
               P1_0=0;   //开继电器
             if(pkt->cmd.Data[11]==0xDD)
                GenericApp_applicationbuf=6001;
               
              P1_7=1;
              P1_6=0;   //窗帘开
               GenericApp_BX=0;
               GenericApp_ON_OFF=0;
            }
         }
        GenericApp_SendTheMessage();
      }
      }  
#endif  
#if(SENSOR_TYPE ==0X18)
if((pkt->cmd.Data[0]==0xFD)&&(pkt->cmd.Data[6]==SENSOR_TYPE))
      { 
#if defined( ZigBee_C_R_E_IEEE )
  char GetExtAddr=0;
  byte * Send_d;
  Send_d=NLME_GetExtAddr();
  for(int i=0;i<8;i++)
  {
  if(pkt->cmd.Data[i+pkt->cmd.Data[1]-2]==(*Send_d++))
    GetExtAddr++;
  }
     if(GetExtAddr==8)
#else
     if((pkt->cmd.Data[7]==Send_data[4])&&(pkt->cmd.Data[8]==Send_data[5]))
#endif    
      {T_MGSbit|=0X20; //xx1x xxx0 应答 正常 状态位
        if(pkt->cmd.Data[9]==0xDD)//
         {GenericApp_applicationbuf=1;
           if(pkt->cmd.Data[10]==0x01)  
            {if(pkt->cmd.Data[11]==0xAA)
                GenericApp_applicationbuf=6001;
            }
         }
       // GenericApp_SendTheMessage();
      }
 
      }  
#endif 
#endif  
}
/*********************************************************************
 * @fn      GenericApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
#if(ZDO_COORDINATOR==2)  //ZIGBEE  AT命令模式 透传模式
void GenericApp_SendFT( void ) 
{HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
GenericApp_DstAddr.addr.shortAddr=0XFFFF;

  if ( AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       Send_datalend,
                       Send_data, 
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
  }
  
}

#endif

void GenericApp_SendTheMessage( void ) 
{
  HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
#if defined( SENSOR_TYPE_Coord )
#if defined( ZigBee_C_R_E_Engineering )
Send_data[4]=NLME_GetShortAddr()/256;
Send_data[5]=NLME_GetShortAddr()%256;
#endif 
#if defined( ZigBee_C_R_E_Transparent )
GenericApp_DstAddr.addr.shortAddr=0XFFFF;
#else 
GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
GenericApp_DstAddr.addr.shortAddr=(Send_data[2]<<8)+Send_data[3];
#endif
Send_data[1]=Send_datalend;

  if ( AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       Send_data[1]+6,
                       Send_data, 
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
     { 
     AF_OK=1;
   }
   else  //失败
   { AF_OK=0;
   }

  //HalUARTWrite(HAL_UART_PORT_0, Send_data,Send_data[1]+6);
#else
int datase = NLME_GetShortAddr();
 // char theMessageData[] = NLME_GetShortAddr();

#if defined( ZigBee_C_R_E_Transparent )
GenericApp_DstAddr.addr.shortAddr=0XFFFC; //R C
#else 
 GenericApp_DstAddr.addr.shortAddr =NWK_BROADCAST_SHORTADDR_AABB;
#endif
 Send_data[2]=NLME_GetCoordShortAddr()/256; //父节点地址
 Send_data[3]=NLME_GetCoordShortAddr()%256; //父节点地址
//Send_data[4]= NLME_GetShortAddr()/256;
//Send_data[5]= NLME_GetShortAddr()%256;
#if(SENSOR_TYPE =='R')
Send_datalend=0x03;
Send_data[6]='R';
Send_data[7]=0X00;
Send_data[8]=(Send_data[6]+Send_data[7])%256;
GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
GenericApp_DstAddr.addr.shortAddr=0X0000;
#endif  
#if((SENSOR_TYPE =='H')||(SENSOR_TYPE ==0X12)||(SENSOR_TYPE ==0X18)||(SENSOR_TYPE ==0X15))
 Send_data[6]=SENSOR_TYPE;//传感器类型 
 if(GenericApp_applicationbuf==0)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XAA;}
  if(GenericApp_applicationbuf==1)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XBB;}
 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
  GenericApp_applicationbuf=0;
     halMcuWaitMs(280);
#if(SENSOR_TYPE =='H') 
  P1_6=1;P1_7=1;
#endif
#if(SENSOR_TYPE ==0X12) 
  P1_6=1;P1_7=1;
#endif
  #if(SENSOR_TYPE ==0X15) 
  P1=0X1E;
#endif
#endif  
#if(SENSOR_TYPE ==0X23)
 Send_data[6]=SENSOR_TYPE;//传感器类型 
 if(GenericApp_applicationbuf==0)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XAA;}
 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
  GenericApp_applicationbuf=0;
#endif 
  #if(SENSOR_TYPE ==0XA3)
  Send_data[6]=SENSOR_TYPE;//传感器类型 
  if(GenericApp_applicationbuf==0)
  {Send_datalend=0x05;Send_data[7]=Send_data[8]=Send_data[9]=0XAA;
  Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256; }
  if(GenericApp_applicationbuf==1)
  {Send_datalend=0x05;Send_data[7]=Send_data[8]=Send_data[9]=0XBB;
  Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256; }
  if(GenericApp_applicationbuf==2)
  {Send_datalend=0x05;Send_data[7]=0XCC;Send_data[8]=0X03;Send_data[9]=GenericApp_ON_OFF;
  Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256; }
  if(GenericApp_applicationbuf==3)
  {Send_datalend=0x06;
  Send_data[11]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10])%256; }
  GenericApp_applicationbuf=0;
#endif    
#if(SENSOR_TYPE ==0X79)
  Send_data[6]=SENSOR_TYPE;//传感器类型 
#endif  
#if((SENSOR_TYPE =='K')||(SENSOR_TYPE ==0X83))
 Send_datalend=0x06;  
 Send_data[6]=SENSOR_TYPE;//传感器类型 
 if(GenericApp_applicationbuf==0)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XAA;}
  if(GenericApp_applicationbuf==1)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XBB;}
    if(GenericApp_applicationbuf==2)
  {Send_data[7]=0XCC;Send_data[8]=0X03;Send_data[9]=GenericApp_ON_OFF;}
#if(SENSOR_TYPE =='K')
 Send_data[10]=T_MG;
#endif 
#if(SENSOR_TYPE ==0X83)
 Send_data[10]=~P1;
#endif 
 Send_data[11]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10])%256;
  GenericApp_applicationbuf=0;
#endif  
#if(SENSOR_TYPE ==0X73)
 Send_data[6]=SENSOR_TYPE;//传感器类型 
  if(GenericApp_applicationbuf!=3)
  {
 Send_datalend=0x06;  
 if(GenericApp_applicationbuf==0)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XAA;}
  if(GenericApp_applicationbuf==1)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XBB;}
 Send_data[11]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10])%256;
  }
   GenericApp_applicationbuf=0;
#endif 
#if((SENSOR_TYPE =='S')||(SENSOR_TYPE ==0X02)||(SENSOR_TYPE ==0X05)||(SENSOR_TYPE ==0X40)||\
  (SENSOR_TYPE ==0X06)||(SENSOR_TYPE ==0X09)||(SENSOR_TYPE ==0X67)||(SENSOR_TYPE ==0X30)||(SENSOR_TYPE ==0X29))
 Send_data[6]=SENSOR_TYPE;//传感器类型 
 Send_datalend=0x05;  
 if(GenericApp_applicationbuf==0)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XAA;}
  if(GenericApp_applicationbuf==1)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XBB;}
  if(GenericApp_applicationbuf==2)
 {Send_data[7]=Send_data[9]=0XBB;}
 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
  GenericApp_applicationbuf=0;
#endif 
#if((SENSOR_TYPE ==0X65)||(SENSOR_TYPE ==0X66)||(SENSOR_TYPE ==0X6C)||(SENSOR_TYPE ==0X6E))
  Send_data[6]=SENSOR_TYPE;//传感器类型 
 if(GenericApp_switch==0)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XAA;}
  if(GenericApp_switch==1)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XBB;}
 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
  GenericApp_switch=0;
#endif 
  
#if(SENSOR_TYPE ==0X76)
  Send_data[6]=SENSOR_TYPE;//传感器类型 
  
Send_data[7]=0X00;
 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
#endif 
#if((SENSOR_TYPE ==0X6F)||(SENSOR_TYPE ==0X28))
  Send_data[6]=SENSOR_TYPE;//传感器类型 
 if(GenericApp_switch==0)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XAA;}
  if(GenericApp_switch==1)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XBB;}
 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
  GenericApp_switch=0;
#endif 
#if(SENSOR_TYPE ==0X6A)
  Send_data[6]=SENSOR_TYPE;//传感器类型 
 if(GenericApp_switch==0)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XAA;}
 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
  GenericApp_switch=0;
#endif 
#if((SENSOR_TYPE =='P')||(SENSOR_TYPE ==0X6D))
Send_data[6]=SENSOR_TYPE;//传感器类型
Send_datalend=0x05;
 if(GenericApp_applicationbuf==0)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XAA;}
  if(GenericApp_applicationbuf==1)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XBB;}

 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
  GenericApp_applicationbuf=0;
#endif
 #if((SENSOR_TYPE ==0XF0))
Send_data[6]=SENSOR_TYPE;//传感器类型
Send_datalend=0x05;
 if(GenericApp_applicationbuf==0)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XAA;}
  if(GenericApp_applicationbuf==1)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XBB;}

 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
  GenericApp_applicationbuf=0;
#endif
#if(SENSOR_TYPE ==0X6B)
Send_data[6]=SENSOR_TYPE;//传感器类型
Send_datalend=0x05;
 if(GenericApp_applicationbuf==0)
 {Send_data[7]=Send_data[8]=Send_data[9]=0XAA;}
  if(GenericApp_applicationbuf==1)
 {Send_data[8]=Send_data[9]=0XBB;}
#if defined(LEDA8)
     Send_data[7]=0XDE;
#endif
 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
  GenericApp_applicationbuf=0;
#endif
#if(SENSOR_TYPE ==0X25)
Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[7]=0X01;
 Send_data[Send_datalend+6]=0X0F;//校验和
 Send_datalend+=1;  //+校验和
#endif
 #if(SENSOR_TYPE ==0X7C)
Send_data[6]=SENSOR_TYPE;//传感器类型
#endif
#if(SENSOR_TYPE ==0X7B)
Send_data[6]=SENSOR_TYPE;//传感器类型
 //Send_data[7]=0X00;
 Send_data[12]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11])%256;
#endif
 #if(SENSOR_TYPE ==0X71)
Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[7]=0X01;
 Send_data[Send_datalend+6]=0X0F;//校验和
 Send_datalend+=1;  //+校验和
#endif
 #if(SENSOR_TYPE ==0X70)
Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[7]=0X01;
 Send_data[Send_datalend+6]=0X0F;//校验和
 Send_datalend+=1;  //+校验和
#endif
 #if(SENSOR_TYPE ==0X86)
Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[7]=0X01;
 Send_data[Send_datalend+6]=0X0F;//校验和
#endif
 #if(SENSOR_TYPE ==0X87)
Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[7]=0X01; Send_data[8]=0X01;
 Send_data[Send_datalend+6]=0X0F;//校验和
#endif
#if(SENSOR_TYPE ==0X26)
Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[7]=0X01;
 Send_data[Send_datalend+6]=0X0F;//校验和
 Send_datalend+=1;
#endif
 #if(SENSOR_TYPE ==0X27)
Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[8]=0X0F;//校验和
#endif
#if(SENSOR_TYPE ==0X07)  
Send_data[6]=SENSOR_TYPE;//传感器类型
#if defined(UHF)
Send_data[7]=0x01; //UHF-  01:命令读卡。B1:主动上传
if(AB==0X0B)
Send_data[7]=0xB1; //01:命令读卡。B1:主动上传
//AB=0X0A;
#endif
#if defined(RLM100)
Send_data[7]=0x02; //RLM100-  02:命令读卡。B2:主动上传
#if defined(AB)
Send_data[7]=0xB2; //02:命令读卡。B2:主动上传
#endif
#endif
//Send_data[13]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11]+Send_data[12])%256;
#endif

#if(SENSOR_TYPE ==0X0E)  
Send_data[6]=SENSOR_TYPE;//传感器类型

//Send_data[13]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11]+Send_data[12])%256;
#endif
#if(SENSOR_TYPE ==0X08)  
Send_data[6]=SENSOR_TYPE;//传感器类型
//Send_data[7]=0X01;
//Send_data[13]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11]+Send_data[12])%256;
#endif
#if(SENSOR_TYPE ==0X0D)  
Send_data[6]=SENSOR_TYPE;//传感器类型
#endif
#if((SENSOR_TYPE ==0X0A)||(SENSOR_TYPE ==0X0B)||(SENSOR_TYPE ==0X12)||(SENSOR_TYPE ==0X18)\
  ||(SENSOR_TYPE ==0X14)||(SENSOR_TYPE ==0X74))
Send_data[6]=SENSOR_TYPE;//传感器类型
#endif
#if(SENSOR_TYPE ==0X13)
#if defined(HUABANG) 
 // Send_data[15]=0;
float datieee754=0;
long dat=0;
  datieee754=ieee754()*100;
  dat= (long) datieee754;
  Send_data[15]=dat>>24;Send_data[16]=dat>>16;Send_data[17]=dat>>8;Send_data[18]=dat;
   Send_data[8]=Send_data[9]=Send_data[10]=Send_data[11]=Send_data[12]=0;
   Send_data[13]=Send_data[7];
   Send_data[7]=0XAA;Send_data[14]=0XBB;
#endif
Send_data[6]=SENSOR_TYPE;//传感器类型
#endif


#if(SENSOR_TYPE =='G')  
Send_data[6]=SENSOR_TYPE;//传感器类型
Send_data[13]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11]+Send_data[12])%256;
#endif

#if((SENSOR_TYPE =='I')||(SENSOR_TYPE ==0X64)||(SENSOR_TYPE ==0X77))
Send_data[7]=GenericApp_appIO;
 Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;

#endif
#if((SENSOR_TYPE ==0X10)||(SENSOR_TYPE ==0X0F)||(SENSOR_TYPE ==0X17)||(SENSOR_TYPE ==0X21)||(SENSOR_TYPE ==0X16))
 Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[8]=(Send_data[6]+Send_data[7])%256;
#endif
 #if((SENSOR_TYPE ==0X11)||(SENSOR_TYPE ==0X1A))
 Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[9]=(Send_data[6]+Send_data[7]+Send_data[8])%256;
#endif
#if(SENSOR_TYPE ==0X20)
 Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[7]=Send_data[8];
 Send_data[8]=(Send_data[10]*10+Send_data[11]);
 Send_data[9]=Send_data[12];
 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
#endif
#if(SENSOR_TYPE ==0X24)
  Send_data[6]=SENSOR_TYPE;
   Send_data[7]=0xa1;
  Send_data[8]=CSB_Len_H;
  Send_data[9]=CSB_Len_L;
  Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
#endif
  #if(SENSOR_TYPE ==0X80)
  Send_data[6]=SENSOR_TYPE;
   Send_data[7]=0xA1;
  Send_data[8]=0xA2;
  Send_data[9]=0xA3;
   Send_data[10]=CSB_Len_H;
  Send_data[11]=CSB_Len_L;
  Send_data[12]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11])%256;
#endif
#if(SENSOR_TYPE ==0X31)
  Send_data[6]=SENSOR_TYPE;//传感器类型 
  if(GenericApp_applicationbuf==0)
  {Send_datalend=0x05;Send_data[7]=Send_data[8]=Send_data[9]=0XAA;
  Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256; }
  if(GenericApp_applicationbuf==1)
  {Send_datalend=0x05;Send_data[7]=Send_data[8]=Send_data[9]=0XBB;
  Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256; }
  if(GenericApp_applicationbuf==2)
  {Send_datalend=0x09;Send_data[8]=Uart_Rev_Buff_1[0];Send_data[9]=Uart_Rev_Buff_1[1];Send_data[10]=Uart_Rev_Buff_1[2];
 Send_data[11]=Uart_Rev_Buff_1[3];Send_data[12]=Uart_Rev_Buff_1[4];Send_data[13]=Uart_Rev_Buff_1[5];
  Send_data[14]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11]+Send_data[12]+Send_data[13])%256; }
  if(GenericApp_applicationbuf==3)
  {Send_datalend=0x05;
  Send_data[10]=(Send_data[7]+Send_data[8]+Send_data[9])%256; }
  if(GenericApp_applicationbuf==4)
  {Send_datalend=0x09;
  Send_data[14]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11]+Send_data[12]+Send_data[13])%256; }
  GenericApp_applicationbuf=0;
#endif  
  
 #if(SENSOR_TYPE ==0X03)
 Send_data[6]=SENSOR_TYPE;//传感器类型
 
  Send_data[7]=GenericApp_appIO;
 Send_data[7]=(Send_data[7]|0XA0);
 Send_data[8]=(Send_data[6]+Send_data[7])%256;
#endif
#if(SENSOR_TYPE =='T')
  Send_data[6]=SENSOR_TYPE;//传感器类型
//不要校验和 Send_data[14]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11]+Send_data[12]+Send_data[13])%256;
#endif
#if(SENSOR_TYPE =='O')
 Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[14]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11]+Send_data[12]+Send_data[13])%256;
#endif
#if(SENSOR_TYPE =='D')
        float OUTRGB = 0.0;
        unsigned int RGBTEM;
                OUTRGB=TCS3200(1);
		//RGBTEM=(uint)(OUTR*OUTRGB);			//转换R	
                RGBTEM=(uint)(0.1795*OUTRGB);			//转换R	
                Send_data[7]=RGBTEM;
//-------------------------------------
		OUTRGB=TCS3200(2);
		//RGBTEM= (uint)(OUTG*OUTRGB);
               RGBTEM= (uint)(0.2614*OUTRGB);
                Send_data[8]=RGBTEM;
//--------------------------------
		OUTRGB=TCS3200(3);
		//RGBTEM= (uint)(OUTB*OUTRGB);
                RGBTEM= (uint)(0.1097*OUTRGB);
                Send_data[9]=RGBTEM;
                Send_data[6]=SENSOR_TYPE;//传感器类型
                Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
                P1_3=0;
#endif
#if(SENSOR_TYPE =='Q')
  Send_data[6]=SENSOR_TYPE;//传感器类型
#endif

#if(SENSOR_TYPE =='U')
 unsigned int  data;						    
    float Temperature;
        Temperature = ReadTemperature();        //返回16位二进制数
        Temperature *= CelsiurPerLSB;		//转换成浮点型温度值(摄氏度)
Send_data[6]=SENSOR_TYPE;//传感器类型
 data=(unsigned int)(100*Temperature);   
 Send_data[7]=(uint8)(data/ 100);
 Send_data[8]=(uint8)(data % 100);
 Send_data[9]=(Send_data[6]+Send_data[7]+Send_data[8])%256;
#endif
 
 #if(SENSOR_TYPE =='C')
 Single_Write_(write_1710 ,power_on );
 Single_Write_(write_1710 ,0x52 );
 Single_Write_(write_1710 ,0x78 );
 Single_Write_(write_1710 ,h_resolution_mod );
 halMcuWaitMs(500); //延时
 Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[7]=Single_Read_(read_1710);
 //Send_data[8]=Single_Read_(read_1710+1);
 Send_data[8]=(Send_data[6]+Send_data[7])%256;
halMcuWaitMs(50);//延时
 //data1[0]*256+data1[1];
 Single_Write_(write_1710 ,reset );
 WrToROM(power_down,write_1710,1);
#endif
#if(SENSOR_TYPE ==0X33)
 Send_data[6]=SENSOR_TYPE;//传感器类型
  Send_data[7]=0xD1;
 Send_datalend=0x07;
//B_LUX30_Multiple_read(0X00,&Send_data[8],4);
Send_data[12]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11])%256;
#endif
 
 #if(SENSOR_TYPE =='B')
 for(char i=0;i<RawDataLength;i++)
  {
  MMA7660_XYZ_Read_and_Filter();
  }
Send_data[7]=Xavg8;
Send_data[8]=Yavg8;
Send_data[9]=Zavg8;
mma_delay(30000);
 for(char i=0;i<RawDataLength;i++)
  {
  MMA7660_XYZ_Read_and_Filter();
  }
  if((Send_data[7]-Xavg8>5)||(Send_data[7]-Xavg8<-5))
     Send_data[7]=0;
  if((Send_data[8]-Yavg8>5)||(Send_data[8]-Yavg8<-5))
 Send_data[8]=0;
  if((Send_data[9]-Zavg8>5)||(Send_data[9]-Zavg8<-5))
 Send_data[9]=0;

//Xavg8>0x1f (正负)     
//(Xavg8&0x1f) (大小)   
//3(Xavg8&0x1f)/62 (加速度) 
//6(Xavg8&0x1f)/62 (高度)  
//6(Xavg8&0x1f)/62 (角度sin)
Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
 Send_data[6]=SENSOR_TYPE;//传感器类型
#endif
#if(SENSOR_TYPE =='F')
 #if defined(BMP085) 
      float temperature=0;
	float pressure=0;
        unsigned int data;

      XCLR=1; 
       mDelay(50); //延时
       BMP085_sample(0);
      temperature=T;
	pressure=p;
	temperature=temperature/10;	//单位：摄氏度
	pressure=pressure/1000;		//单位：kPa    //30 ... 110KPa（海拔9000米...-500米）
	mDelay(50); //延时
  Send_data[6]=SENSOR_TYPE;//传感器类型
  data=(uint)(100*temperature);     
  Send_data[7]=(uint8)(data/100);
  Send_data[8]=(uint8)(data%100);
  
  data=(uint)(100*pressure);      

  Send_data[9]=(uint8)(data/ 100);
  Send_data[10]=(uint8)(data % 100);
  Send_data[11]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10])%256;
#endif
  
#if defined(BMP180) 
        uint16_t data=0;
  Multiple_Read_BMP180();
   Send_data[6]=SENSOR_TYPE;//传感器类型  
  Send_data[7]=(uint8)(temperature/0x0a);;
  Send_data[8]=(uint8)(temperature%0x0a);
  data=(pressure/0x0a);
  Send_data[9]=(uint8)(data/0x64);
  Send_data[10]=(uint8)(data%0x64);
  Send_data[11]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10])%256;
#endif
#endif
#if(SENSOR_TYPE =='E')
  Send_data[11]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10])%256;
#endif
#if(SENSOR_TYPE ==0X85)
  Send_datalend=0x07;
  Send_data[6]=SENSOR_TYPE;//传感器类型
    Send_data[12]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11])%256;
#endif
#if(SENSOR_TYPE ==0X82)
    Send_datalend=0x09;  
   Send_data[6]=SENSOR_TYPE;//传感器类型
  Send_data[14]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11]+Send_data[12]+Send_data[13])%256;
#endif
#if((SENSOR_TYPE =='A')||(SENSOR_TYPE =='J')||(SENSOR_TYPE =='L')||(SENSOR_TYPE =='M')\
  ||(SENSOR_TYPE =='N')||(SENSOR_TYPE ==0X0C)||(SENSOR_TYPE ==0X19)\
  ||(SENSOR_TYPE ==0X62)||(SENSOR_TYPE ==0X63) ||(SENSOR_TYPE ==0X34)||(SENSOR_TYPE ==0X35)\
    ||(SENSOR_TYPE ==0X36)||(SENSOR_TYPE ==0X37)||(SENSOR_TYPE ==0X38)||(SENSOR_TYPE ==0X39)\
      ||(SENSOR_TYPE ==0X3A)||(SENSOR_TYPE ==0X3E)||(SENSOR_TYPE ==0X3F)||(SENSOR_TYPE ==0X75))

 Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[9]=(Send_data[6]+Send_data[7]+Send_data[8])%256;
#endif
 
#if((SENSOR_TYPE ==0X22)||(SENSOR_TYPE ==0X22))
Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
#endif
 
 #if((SENSOR_TYPE ==0X61)||(SENSOR_TYPE ==0X3C))
  Send_datalend=0x04;   
 Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[9]=(Send_data[6]+Send_data[7]+Send_data[8])%256;
#endif
#if(SENSOR_TYPE ==0X84)
  Send_datalend=0x05;   
 Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
#endif
 #if(SENSOR_TYPE ==0X3B)
   Send_datalend=0x05;   
 Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
#endif
#if(SENSOR_TYPE ==0X78)
 Send_data[6]=SENSOR_TYPE;//传感器类型
#endif
#if((SENSOR_TYPE ==0X72)||(SENSOR_TYPE ==0X81)||(SENSOR_TYPE ==0X3D))
 Send_data[6]=SENSOR_TYPE;//传感器类型
#if defined( SDS011 )
 Send_data[13]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11]+Send_data[12])%256;
#else
 Send_data[11]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10])%256;
#endif
 
#endif
#if((SENSOR_TYPE ==0X04))
 Send_data[6]=SENSOR_TYPE;//传感器类型
 Send_data[8]=(Send_data[6]+Send_data[7])%256;
 GenericApp_BX=0;
#endif

#if defined( ZigBee_C_R_E_IEEE )
ZigBee_C_R_E_Engineering_H();
#else
Send_data[1]=Send_datalend;
#endif
#if defined( CC2530_V30 )
for(char i=(Send_data[1]+7);i>5;i--)
Send_data[i+5]=Send_data[i];
Send_data[6]=Send_data[7]=Send_data[8]=Send_data[9]=0x0f;
Send_data[10]=T_MGSbit;
T_MGSbit&=~0X21; //xx0x xxx0 主动  正常 状态位
   if ( AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       (Send_data[1]+11),
                       Send_data,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
     { 
     AF_OK=1;
   }
   else  //失败
   { AF_OK=0;
   }
#else
   if ( AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       (Send_data[1]+6),
                       Send_data,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
   { 
     AF_OK=1;
   }
   else  //失败
   { AF_OK=0;
   }

#endif
#if defined( CC2530_RF_433M )  
   Send_data[(Send_data[1]+6)]=0x0D;  Send_data[(Send_data[1]+6+1)]=0x0A;
    HalUARTWrite(HAL_UART_PORT_0, Send_data,(Send_data[1]+6+2));
#endif
#endif
  HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
  HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
}
/*********************************************************************
*********************************************************************/
#if defined( SENSOR_TYPE_Coord )
void GenericAppCoordEB(); //C 事件处理
void GenericAppCoordEB() //C 事件处理
{if(AF_RFn)
{  
     if(GenericApp_ON_OFF>0)GenericApp_ON_OFF--;
         if(GenericApp_ON_OFF<1)
         {  
         GenericApp_DstAddr.addr.shortAddr=0XFFFF;
        if ( AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       chnnd[AF_RFnd][2]+6,
                       &chnnd[AF_RFnd][1], 
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
              {
                //AF_RFn+=16;              
               // HalUARTWrite(HAL_UART_PORT_0,  &chnnd[AF_RFnd][1],chnnd[AF_RFnd][2]+6);
                //HalUARTWrite(HAL_UART_PORT_0,  &AF_RFn,1);
                
                
               AF_RFn=0;
                chnnd[AF_RFnd][0]=0;
                
               // AF_RFn--; 
             //  if(AF_RFn==0){ chnnd[AF_RFnd][0]=0;}//10次全部失败
              //  HalUARTWrite(HAL_UART_PORT_0,  &chnnd[AF_RFnd][1],chnnd[AF_RFnd][2]+6);
              // GenericApp_ON_OFF=(4+(10-AF_RFn)*2);
               
               // if(AF_RFnds>0)AF_RFnds+1;
                }
           else//失败
           {AF_RFn--; 
           if(AF_RFn==0){ chnnd[AF_RFnd][0]=0;}//10次全部失败
            // HalUARTWrite(HAL_UART_PORT_0,  &AF_RFn,1);
          // halMcuWaitMs(1000+((10-AF_RF)*50));
             // HalUARTWrite(HAL_UART_PORT_0,  &chnnd[AF_RFnd][1],chnnd[AF_RFnd][2]+6);
            GenericApp_ON_OFF=(5+(10-AF_RFn)*2);
           }
         }
}
if( AF_RFn==0)
{
             AF_RFnds=0;
           if(chnnd[1][0]||chnnd[2][0]||chnnd[3][0]||chnnd[4][0])
             {  if(chnnd[4][0])
                {AF_RFnds=4;
                 if(chnnd[3][0])
                   {AF_RFnds=3;
                    if(chnnd[2][0])
                   {AF_RFnds=2;
                     if(chnnd[1][0])
                      {AF_RFnds=1;}
                   }
                   }
                }
             }

  
 if(chnnd[0][0])
 {  AF_RFnd=0;
  if(AF_RFnds!=0)AF_RFnd=AF_RFnds;
  AF_RFn=10;//发送10次
 }
 else
 { if(chnnd[1][0])
    {  AF_RFnd=1; AF_RFn=10;//发送5次
    }
    else
    { if(chnnd[2][0])
        {  AF_RFnd=2; AF_RFn=10;//发送5次
        }
        else
        { if(chnnd[3][0])
          {  AF_RFnd=3; AF_RFn=10;//发送5次
          }
          else
          { if(chnnd[4][0])
            {  AF_RFnd=4; AF_RFn=10;//发送5次
            }
          }
        }
    }
 }
}
   if(AF_RF==1)//失败标志存储失败数据
   {AF_RF=0;//以开始存储，清0
 
         if(!chnnd[0][0])
          {chnnd[0][0]=1;
           for(char i=0;i<(Send_data[1]+6);i++)
             {chnnd[0][1+i]=Send_data[i];
             } 
          }
         else
         {if(!chnnd[1][0])
           {chnnd[1][0]=1;
           for(char i=0;i<(Send_data[1]+6);i++)
             {chnnd[1][1+i]=Send_data[i];
             } 
          }
          else
          {if(!chnnd[2][0])
           {chnnd[2][0]=1;
           for(char i=0;i<(Send_data[1]+6);i++)
             {chnnd[2][1+i]=Send_data[i];
             } 
          }
           else
           {if(!chnnd[3][0])
             {chnnd[3][0]=1;
           for(char i=0;i<(Send_data[1]+6);i++)
             {chnnd[3][1+i]=Send_data[i];
             } 
          }
            else
            {if(!chnnd[4][0])
              {chnnd[4][0]=1;
           for(char i=0;i<(Send_data[1]+6);i++)
             {chnnd[4][1+i]=Send_data[i];
             } 
          }
             else
              {//不理会 
              }
            }
            
           }
          }
         }
 }
}

#endif

void GenericAppEndDeviceEB(void) //100MS
{
  if(openoff==0xaa)
  {
#if((SENSOR_TYPE =='E')||(SENSOR_TYPE =='B')||(SENSOR_TYPE =='C')||(SENSOR_TYPE =='D')||(SENSOR_TYPE =='F')\
  ||(SENSOR_TYPE =='U')||(SENSOR_TYPE ==0X10)||(SENSOR_TYPE ==0X21)||(SENSOR_TYPE ==0X11)\
    ||(SENSOR_TYPE ==0X0F)||(SENSOR_TYPE ==0X13)||(SENSOR_TYPE ==0X14)||(SENSOR_TYPE ==0X17)\
      ||(SENSOR_TYPE ==0X24)||(SENSOR_TYPE ==0X5A)||(SENSOR_TYPE ==0X5B)||(SENSOR_TYPE ==0X6A)||(SENSOR_TYPE =='A')\
        ||(SENSOR_TYPE =='J')||(SENSOR_TYPE =='L')||(SENSOR_TYPE =='M')||(SENSOR_TYPE =='N')\
  ||(SENSOR_TYPE ==0X04)||(SENSOR_TYPE ==0X22)||(SENSOR_TYPE ==0X0C)||(SENSOR_TYPE ==0X16)||(SENSOR_TYPE ==0X19)\
    ||(SENSOR_TYPE ==0X61)||(SENSOR_TYPE ==0X62)||(SENSOR_TYPE ==0X63)||(SENSOR_TYPE ==0X1A)||(SENSOR_TYPE ==0X33)\
      ||(SENSOR_TYPE ==0X72) ||(SENSOR_TYPE ==0X34)||(SENSOR_TYPE ==0X35)||(SENSOR_TYPE ==0X36)||(SENSOR_TYPE ==0X37)\
        ||(SENSOR_TYPE ==0X38)||(SENSOR_TYPE ==0X39)||(SENSOR_TYPE ==0X3A)||(SENSOR_TYPE ==0X80)||(SENSOR_TYPE ==0X3D)\
          ||(SENSOR_TYPE ==0X81)||(SENSOR_TYPE ==0X82)||(SENSOR_TYPE ==0X3B)||(SENSOR_TYPE ==0X3C)||(SENSOR_TYPE ==0X3E)\
            ||(SENSOR_TYPE ==0X3F)||(SENSOR_TYPE ==0X84)||(SENSOR_TYPE ==0X85)||(SENSOR_TYPE ==0X75)||(SENSOR_TYPE ==0X78))

#if defined( POWER_SAVING ) //终端使用计数
    GenericApp_applicationbuf+=GenericApp_time; 
#else
       if(GenericApp_time==1) //路由启动定时
        {/* 使能定时器3的溢出中断 */
        TIMER34_INIT(3);
         halSetTimer3Period(100);  //100MS
             T3_t1Val=(10*25);//25s //定时3
        IEN1 |= (0x01 << 3);             // 使能Timer3的中断T3IE
        TIMER3_RUN(TRUE); 
        GenericApp_time=0;
        }
#endif 
#endif   
   
           
#if(SENSOR_TYPE ==0XA3)  
      if(AF_RF)
       {AF_RF=0;Send_data[1]=Send_datalend+8; Send_data[6]=SENSOR_TYPE;//传感器类型 
        for(char i=0;i<(Send_data[1]+6);i++)
             {chnndr[i]=Send_data[i];
             } 
        AF_RFn=10;//发送10次
        
       }
       if(AF_RFn)
       { 
       if(GenericApp_ON_OFF>0)GenericApp_ON_OFF--;
         if(GenericApp_ON_OFF<1)
         {
          GenericApp_DstAddr.addr.shortAddr=0XFFFF;
      if ( AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       (chnndr[1]+6),
                       chnndr, 
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
          { AF_RFn=0; 
           // HalUARTWrite(HAL_UART_PORT_0, "0K",2);
          }
          else
           {AF_RFn--; 
           //HalUARTWrite(HAL_UART_PORT_0, chnndr,chnndr[1]+6);
            GenericApp_ON_OFF=(6+(10-AF_RFn)*3);
           }
         }
       }   
       
       
       
#endif 
       
#if(SENSOR_TYPE ==0X31) 
        if(AF_RF)
       {AF_RF=0;Send_data[1]=Send_datalend+8; Send_data[6]=SENSOR_TYPE;//传感器类型 
        for(char i=0;i<(Send_data[1]+6);i++)
             {chnndr[i]=Send_data[i];
             } 
        AF_RFn=10;//发送10次
        
       }
       if(AF_RFn)
       { 
       if(GenericApp_ON_OFF>0)GenericApp_ON_OFF--;
         if(GenericApp_ON_OFF<1)
         {
          GenericApp_DstAddr.addr.shortAddr=0XFFFF;
      if ( AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       (chnndr[1]+6),
                       chnndr, 
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
          { AF_RFn=0; 
           // HalUARTWrite(HAL_UART_PORT_0, chnndr,chnndr[1]+6);
          }
          else
           {AF_RFn--; 
          // HalUARTWrite(HAL_UART_PORT_0, chnndr,chnndr[1]+6);
            GenericApp_ON_OFF=(6+(10-AF_RFn)*3);
           }
         }
       }
      
#endif 
       
#if(SENSOR_TYPE ==0X05)
if(GenericApp_BX>1)
  GenericApp_BX--;
   else
   {P1_6=0;
    P1_7=0;
   }
#endif  

#if(SENSOR_TYPE ==0X40)
if(T_MG==0X01)
{LED_START++;
if(LED_START==5){P1_3=0; P1_4=1; }//LED1 红色 点亮
if(LED_START>10){LED_START=0;P1_3=1; P1_4=0; } //LED1 红色 熄灭
}
#endif   
#if(SENSOR_TYPE ==0X12)
if(GenericApp_ON_OFF>20)
   {GenericApp_BX++;
   GenericApp_ON_OFF=0;
   }
   else GenericApp_ON_OFF++;
   if(GenericApp_BX>5)
   { P1_0=1;
     P1_6=1;
    P1_7=0;
    halMcuWaitMs(280);
    P1_6=1;
    P1_7=1;
   }
#endif 
#if(SENSOR_TYPE ==0X07)
if(AB==0x0A)
{
#if defined(UHF)
if(GenericApp_ON_OFF>2)
  { //uint8  hr[6]={0XA5,0XFF,0X03,0X92,0X06,0XC0};
    // uint8  hr[6]={0XA5,0XFF,0X03,0XC1,0X04,0X93};
      uint8  hr[6]={0XA5,0XFF,0X03,0XC1,0X06,0X91};
   GenericApp_ON_OFF=0;
   GenericApp_applicationdata=0;
     HalUARTWrite(HAL_UART_PORT_0, hr,6);
   } 
   else GenericApp_ON_OFF++;
#endif
#if defined(RLM100)
if(GenericApp_ON_OFF>0)
  { uint8  hr[11]={0XAA,0X09,0X20,0X00,0X00,0X00,0X00,0X02,0x00,0x04,0x55};
   GenericApp_ON_OFF=0;
   GenericApp_applicationdata=0;
     HalUARTWrite(HAL_UART_PORT_0, hr,11);
   } 
   else GenericApp_ON_OFF++;
#endif
}
#endif
#if(SENSOR_TYPE ==0X31)
   if(car_op>1) ////可连续读3次时，开始累计计数
   {car_op++;
   if(car_op>50)//7秒后。清零，可继续读卡
     car_op=0;
   } 
   if((holzer==0)&&(P1_0==0))holzer=1; //做霍尔定位标志
   if((holzer==2)&&(P1_0==0))//第二次检测到定位标志
     {holzer=3; 
      uint8 Car_Stop_Buff[3]={0xAA,0x2B,0xBB};
     HalUARTWrite(HAL_UART_PORT_0,Car_Stop_Buff,3);
     }
    if((holzer==3)&&(P1_0==1))//第二次检测到定位标志结束
    { holzer=0;}
#endif
#if(SENSOR_TYPE ==0X16)
  if(GenericApp_ON_OFF>50)
   {  GenericApp_ON_OFF=0;
   if((Send_data[7]-GenericApp_appIO>0)||(GenericApp_appIO-Send_data[7]>0)||((GenericApp_appIO==0)&&(Send_data[7])))
   { Send_data[7]=GenericApp_appIO;
    GenericApp_SendTheMessage(); //应用函数
   }GenericApp_appIO=0;
   } 
   else GenericApp_ON_OFF++;
#endif
#if(SENSOR_TYPE ==0X08)
   #if defined(TRF7970A) 
    if(GenericApp_ON_OFF==20)
    printf("010A0003041001210000\n"); 
#endif
   if(GenericApp_ON_OFF==23)
   printf("010C00030410002101060000\n"); //设置 15693卡
   
    if(GenericApp_ON_OFF>30)
    { GenericApp_ON_OFF=25;
   if(GenericApp_BX)
        printf("0109000304A0010000\n"); //读卡 14443A
    else
      printf("010B000304140701000000\n"); //读卡  15693 
    GenericApp_applicationdata=0;
     //printf("010C00030410002101090000\n"); //设置 14443A卡
    // printf("0109000304A0010000\n"); //读卡
    }
 GenericApp_ON_OFF++;
#endif
#if(SENSOR_TYPE ==0X0D)
#if defined(TGRD15693) 
 
    if(GenericApp_ON_OFF>5)
   {if(!GenericApp_appIO)
   {GenericApp_appIO=1;
   printf(":1601020027EE43\n"); //设置 15693卡
   }
   GenericApp_ON_OFF=0;
   if(GenericApp_BX)
        printf(":1601020027EE43\n"); //读卡 14443A
    else
      printf(":1601020027EE43\n"); //读卡  15693 
    GenericApp_applicationdata=0;
   } 
   else GenericApp_ON_OFF++;
#endif
#if defined(PN532) 
if(GenericApp_ON_OFF>5)
   {uint8  hr[24]={0X55, 0X55,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0Xff,0X03,0Xfd,0Xd4,0X14,0X01,0X17,0}; 
    uint8 hr1[11]={0,0,0XFF,0X04,0XFC,0XD4,0X4A,0X02,0,0XE0,0};
     if(!GenericApp_appIO)
     {
     GenericApp_appIO=1;
   HalUARTWrite(HAL_UART_PORT_0, hr,24);//初始化PN532到normal状态
   HalUARTWrite(HAL_UART_PORT_0, hr,24);//初始化PN532到normal状态
   }
     GenericApp_ON_OFF=0;
     //  HalUARTWrite(HAL_UART_PORT_0, hr,24);//初始化PN532到normal状态
      HalUARTWrite(HAL_UART_PORT_0, hr1,11); //读卡  M1
    GenericApp_applicationdata=0;
   } 
   else GenericApp_ON_OFF++;
#endif
#endif
#if(SENSOR_TYPE ==0X0B)
   if(GenericApp_ON_OFF>5)
   { uint8  hr[4]={0XF6,0XB9,0X16,0XFE};
   GenericApp_ON_OFF=0;
   GenericApp_applicationdata=0;
     HalUARTWrite(HAL_UART_PORT_0, hr,4);
   } 
   else GenericApp_ON_OFF++;
#endif
#if(SENSOR_TYPE ==0X78)
   
if(GenericApp_applicationbuf>6000)  
 { GenericApp_applicationbuf=0;
 GenericApp_SendTheMessage(); //应用函数
}
#endif
#if(SENSOR_TYPE ==0X72)
 if(((Send_data[8]-T_MUMPT1)>0X05)||((T_MUMPT1-Send_data[8])>0X05)||((Send_data[8]==0)&&(T_MUMPT1!=0)))
  {T_MUMPT1=Send_data[8];
   GenericApp_SendTheMessage(); //应用函数
  }

if(GenericApp_applicationbuf>6000)  
 { GenericApp_applicationbuf=0;
 GenericApp_SendTheMessage(); //应用函数
}
#endif

#if((SENSOR_TYPE =='A')||(SENSOR_TYPE =='J')||(SENSOR_TYPE =='L')||(SENSOR_TYPE =='M')||(SENSOR_TYPE =='N')\
  ||(SENSOR_TYPE ==0X22)||(SENSOR_TYPE ==0X0C)||(SENSOR_TYPE ==0X19)\
    ||(SENSOR_TYPE ==0X62)||(SENSOR_TYPE ==0X63) ||(SENSOR_TYPE ==0X34)||(SENSOR_TYPE ==0X35)||(SENSOR_TYPE ==0X36)\
      ||(SENSOR_TYPE ==0X37)||(SENSOR_TYPE ==0X38)||(SENSOR_TYPE ==0X39)||(SENSOR_TYPE ==0X3A)||(SENSOR_TYPE ==0X3E)\
        ||(SENSOR_TYPE ==0X3F)||(SENSOR_TYPE ==0X75))//开关量定时反馈
if(GenericApp_ON_OFF>10)
 { GenericApp_ON_OFF=0;
 Send_data[8]=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
 #if((SENSOR_TYPE =='J') ||(SENSOR_TYPE ==0X34)||(SENSOR_TYPE ==0X35)||(SENSOR_TYPE ==0X36)||(SENSOR_TYPE ==0X37)\
   ||(SENSOR_TYPE ==0X38)||(SENSOR_TYPE ==0X39)||(SENSOR_TYPE ==0X3A)||(SENSOR_TYPE ==0X3E)||(SENSOR_TYPE ==0X75))
  if(Send_data[8]<0X0F)
    Send_data[8]=0;
  else
  if(Send_data[8]<0X15)
    Send_data[8]-=0X0D;
 #endif
  
#if(SENSOR_TYPE ==0X22)
  //  if(Send_data[8]> T_MUMPTZ1)
   //   T_MUMPTZ1=Send_data[8];
  // Send_data[8]= T_MUMPTZ1-Send_data[8];
#endif
#if(SENSOR_TYPE ==0X3F)
 //if(((Send_data[8]-T_MUMPT1)>0X04)||((T_MUMPT1-Send_data[8])>0X04)||((Send_data[8]==0)&&(T_MUMPT1!=0)))
if((((Send_data[8]>T_MUMPT1)? Send_data[8]-T_MUMPT1:T_MUMPT1-Send_data[8])>0X04) ||((Send_data[8]==0)&&(T_MUMPT1!=0)))
  {T_MUMPT1=Send_data[8];
   GenericApp_SendTheMessage(); //应用函数
  }
#endif
#if((SENSOR_TYPE =='J')||(SENSOR_TYPE ==0X62)||(SENSOR_TYPE ==0X63) ||(SENSOR_TYPE ==0X34)\
  ||(SENSOR_TYPE ==0X35)||(SENSOR_TYPE ==0X36)||(SENSOR_TYPE ==0X37)||(SENSOR_TYPE ==0X38)||(SENSOR_TYPE ==0X39)\
    ||(SENSOR_TYPE ==0X3A)||(SENSOR_TYPE ==0X3E)||(SENSOR_TYPE ==0X0C)||(SENSOR_TYPE ==0X75))//开关量定时反馈
//  if(((Send_data[8]>T_MUMPT1)? Send_data[8]-T_MUMPT1:T_MUMPT1-Send_data[8])>0X04) ||((Send_data[8]==0)&&(T_MUMPT1!=0)))
  if((((Send_data[8]>T_MUMPT1)? Send_data[8]-T_MUMPT1:T_MUMPT1-Send_data[8])>0X0A)||((Send_data[8]==0)&&(T_MUMPT1!=0)))
  {T_MUMPT1=Send_data[8];
   GenericApp_SendTheMessage(); //应用函数
  }
#endif
#if((SENSOR_TYPE ==0X22))//开关量定时反馈
 if((((Send_data[8]>T_MUMPT1)? Send_data[8]-T_MUMPT1:T_MUMPT1-Send_data[8])>0X0A)||((Send_data[8]==0)&&(T_MUMPT1!=0)))
  {T_MUMPT1=Send_data[8];
  T_MUMPTS=T_MUMPT1;
   T_MUMPTS=498+(T_MUMPTS*199);
  Send_data[8]=(T_MUMPTS/256);
  Send_data[9]=(T_MUMPTS%256);
  
   GenericApp_SendTheMessage(); //应用函数
  }
#endif


 if( GenericApp_sbum!=P2_0)
  {
    GenericApp_sbum=P2_0;
    //Send_data[8]=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
  if(GenericApp_sbum)Send_data[7]=1;
  else Send_data[7]=0;
   Send_data[8]=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
  T_MUMPTS=Send_data[8];
    T_MUMPTS=498+(T_MUMPTS*199);
  Send_data[8]=(T_MUMPTS/256);
  Send_data[9]=(T_MUMPTS%256);
  GenericApp_SendTheMessage(); //应用函数
 //  halMcuWaitMs(1000);  //延时 1000ms
  }
 }else GenericApp_ON_OFF++; 
if(GenericApp_applicationbuf>6000)  
 { GenericApp_applicationbuf=0;
    if(P2_0)
      Send_data[7]=1;
      else Send_data[7]=0;
 Send_data[8]=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
  T_MUMPTS=Send_data[8];
     T_MUMPTS=498+(T_MUMPTS*199);
  Send_data[8]=(T_MUMPTS/256);
  Send_data[9]=(T_MUMPTS%256);
 GenericApp_SendTheMessage(); //应用函数
}

#endif

#if((SENSOR_TYPE ==0X61))//开关量定时反馈
if(LMOUT)
{ if(T_MUMPT1<2) GenericApp_applicationbuf=6001;
GenericApp_ON_OFF=2;
 Send_data[7]=1; 
T_MUMPT1=2;
}
else 
if(GenericApp_ON_OFF!=2)
  {GenericApp_ON_OFF=0;}

if(GenericApp_sbum>200)
 { GenericApp_sbum=0;
 if((GenericApp_ON_OFF==0)&&(T_MUMPT1>0))
 { GenericApp_applicationbuf=6001;
   Send_data[7]=0; 
   if(T_MUMPT1>0)T_MUMPT1--;
 }
 if(GenericApp_ON_OFF==2)
 { GenericApp_applicationbuf=6001;
   Send_data[7]=1; 
   GenericApp_ON_OFF=0;
 }
 }else GenericApp_sbum++;
if(GenericApp_applicationbuf>6000)  
 { GenericApp_applicationbuf=0;
 GenericApp_SendTheMessage(); //应用函数
}
#endif
#if((SENSOR_TYPE ==0X04))
  if( GenericApp_sbum!=P2_0)
  {
    GenericApp_sbum=P2_0; 
    GenericApp_BX++;
  } else
      {
        if(GenericApp_applicationbuf>0)
        {GenericApp_applicationbuf+=100;
        }
      }
  if(GenericApp_BX>5)
  { Send_data[7]=1; 
   GenericApp_applicationbuf=1;
  GenericApp_SendTheMessage(); //应用函数
  }
if(GenericApp_applicationbuf>6000)  
 { GenericApp_applicationbuf=0;
   Send_data[7]=0; 
 GenericApp_SendTheMessage(); //应用函数
}
#endif 
#if(SENSOR_TYPE ==0X77)
if(P1_0!=GenericApp_applicationdata)
{GenericApp_applicationdata=P1_0;
    GenericApp_appIO=0X01;
}
  if(GenericApp_appIO!=0)
  {
    GenericApp_SendTheMessage(); //应用函数
    GenericApp_appIO=0;
  }
#endif 
#if(SENSOR_TYPE =='I')
unsigned int key=0;
key=((key+P1)<<4);
key+=((P0>>1)&0X08);
key+=(P2&0X07);
if(key!=GenericApp_applicationdata)
{GenericApp_applicationdata=key;
    if(P1_0==0)
    GenericApp_appIO=0X03;
    if(P1_1==0)
    GenericApp_appIO=0X0a;
    if(P1_2==0)
    GenericApp_appIO=0X05;
    if(P1_3==0)
    GenericApp_appIO=0X06;
    if(P1_4==0)
    GenericApp_appIO=0X07;
    if(P1_5==0)
    GenericApp_appIO=0X08;
    if(P1_6==0)
    GenericApp_appIO=0X09;
    if(P1_7==0)
    GenericApp_appIO=0X04;
    if(P2_0==0)
    GenericApp_appIO=0X0b;
    if(P2_1==0)
    GenericApp_appIO=0X0c;
    if(P2_2==0)
    GenericApp_appIO=0X0d;
    if(P0_4==0)
    GenericApp_appIO=0X0e;
  } 
    Send_data[8]=0;
    Send_data[8]+=(P1_0+P1_1+P1_2+P1_3+P1_4+P1_5);
  if((GenericApp_appIO!=0)||( Send_data[8]!=Send_data[9]))
  { //人流量统计数量
    Send_data[9]= Send_data[8];
    //人流量统计数量
    GenericApp_SendTheMessage(); //应用函数
    GenericApp_appIO=0;
  }
#endif
#if(SENSOR_TYPE ==0X24)
if(GenericApp_applicationbuf>6000)
{GenericApp_applicationbuf=0;
  uint8 h=0,l=0,cs,flag=0; //定义变量
  float timer=0,len=0;
  for(cs=0;cs<5;cs++) //连续测五次
  {timer=0;h=0;l=0;len=0;flag=0;       //初始化变量
    P1_2=1;           //拉高触发位
    halMcuWaitUs(20);  //延时 20us
    P1_2=0;           //拉低触发位此时已经发出测试波
    IRCON=0;         //初始化T1定时器
    T1CNTH=0;
    T1CNTL=0;          //写入任意值  H L 清零 0x0000
    while(P1_1==0);  //等待回响信号
    while(P1_1==1)  //收到回响信号时开启定时器
    { if(flag==0)
      {  flag=1;
        T1CTL = 0x0D;    //设置定时器计数  1/128分频   0X0000-0XFFFF循环计数
        T1CNTH=0;
        T1CNTL=0;
       // T1STAT = 0x21;
      }}
    h=T1CNTH; //回响信号结束时取出定时器的值
    l=T1CNTL; //读取 H 之前必须读取 L ; 
    timer+=(h*256); 
    timer+=l;
    len=(timer*128/(58*2*16))*10; //毫米为单位  计算距离
    halMcuWaitMs(60); //每次测量需要间隔60ms
  } 
  CSB_Len_H=(uint8)(len/256);
  CSB_Len_L=(uint8)((uint16)len%256); 
    GenericApp_SendTheMessage(); //应用函数
  }
#endif 
#if(SENSOR_TYPE ==0X80)
if(P2_0==0)
{      int pVc=0;
       pVc=0;
        IRCON=0;         //初始化T1定时器
        T1CTL = 0x05;    //设置定时器计数  1/8   0X0000-0XFFFF   一个循环 16ms   1/128=262ms
        T1CNTL=0;
        CSB_Len_L=0;  
    while((pVc<400)&&(CSB_Len_L==0))  //测速结束   测试时间超过6s 退出
   {  if(T1IF)
      {pVc++;T1IF=0;}
       if(P2_0==1)CSB_Len_L=0X01;
   };
   if(pVc<400)  //测试时间超过6s 退出
   {
   pVc=16*pVc;  //单位 ms
    CSB_Len_H=(uint8)(pVc/256);
    CSB_Len_L=(uint8)((uint16)pVc%256); 
    ////////////////////////////////////////////////////////////////////////////////////////////
//OLED屏显示内容
//OLED_Init(); //OLED初始化 
char *s=0;
sprintf(s, "%05d", pVc); 
OLED_P8x16Str(0,6,s);//第四行 -- 8x16的显示单元显示ASCII码
OLED_P8x16Str(40,6,"mS");//第四行 -- 8x16的显示单元显示ASCII码
    GenericApp_SendTheMessage(); //应用函数
   }
}
if(GenericApp_applicationbuf>6000)
{GenericApp_applicationbuf=0;
GenericApp_SendTheMessage(); //应用函数
}
#endif 
#if(SENSOR_TYPE ==0X64) 
P1_0=0;P1_2=1;P1_4=1;P1_6=1;
if(P1_1==0){while(P1_1==0);GenericApp_applicationdata=0X01;}
if(P1_3==0){while(P1_3==0);GenericApp_applicationdata=0X02;}
if(P1_5==0){while(P1_5==0);GenericApp_applicationdata=0X03;}
if(P1_7==0){while(P1_7==0);GenericApp_applicationdata=0X04;}
P1_0=1;P1_2=0;P1_4=1;P1_6=1;
if(P1_1==0){while(P1_1==0);GenericApp_applicationdata=0X05;}
if(P1_3==0){while(P1_3==0);GenericApp_applicationdata=0X06;}
if(P1_5==0){while(P1_5==0);GenericApp_applicationdata=0X07;}
if(P1_7==0){while(P1_7==0);GenericApp_applicationdata=0X08;}
P1_0=1;P1_2=1;P1_4=0;P1_6=1;
if(P1_1==0){while(P1_1==0);GenericApp_applicationdata=0X09;}
if(P1_3==0){while(P1_3==0);GenericApp_applicationdata=0X0A;}
if(P1_5==0){while(P1_5==0);GenericApp_applicationdata=0X0B;}
if(P1_7==0){while(P1_7==0);GenericApp_applicationdata=0X0C;}
P1_0=1;P1_2=1;P1_4=1;P1_6=0;
P1_1=1;
if(P1_1==0){while(P1_1==0);GenericApp_applicationdata=0X0D;}
if(P1_3==0){while(P1_3==0);GenericApp_applicationdata=0X0E;}
if(P1_5==0){while(P1_5==0);GenericApp_applicationdata=0X0F;}
if(P1_7==0){while(P1_7==0);GenericApp_applicationdata=0X10;}
  if(GenericApp_applicationdata!=0)
  {GenericApp_SendTheMessage(); //应用函数   
    GenericApp_applicationdata=0;
  }
#endif

/*#if(SENSOR_TYPE =='K')
 if(((~P1)&0X06)!=GenericApp_appIO)
  {
   GenericApp_appIO=((~P1)&0X06);
   GenericApp_ON_OFF=(GenericApp_appIO>>0X01);
   GenericApp_applicationbuf=2;
   GenericApp_SendTheMessage(); //应用函数
  }
#endif
*/

#if(SENSOR_TYPE =='K')
 if(T_MG>0X01)
  {T_MG--;
  if(T_MG==0X01){P1_0=1; T_MG=0X00;}
  }
#endif
#if(SENSOR_TYPE ==0X79)
if((PDI==0XD1)||(PDI==0XD2))GenericApp_ON_OFF=11;
if(GenericApp_ON_OFF>10)
 { GenericApp_ON_OFF=0;
 if(ERR_TEMP<5)
 {
 uint  data;
  valuen humi_val,temp_val;							    
  unsigned char error,checksum; 
 error=0;  
           error+=s_measure( &Send_data[7],&checksum,HUMI);  //measure humidity 
           error+=s_measure( &Send_data[9],&checksum,TEMP);  //measure temperature 
           if(error!=0){ s_connectionreset(); ERR_TEMP++;}                 //in case of an error: connection  
          else 
          { humi_val.i=Send_data[7]*256+Send_data[8];
            temp_val.i=Send_data[9]*256+Send_data[10];
            humi_val.f=(float)humi_val.i;                   //converts integer to float
            temp_val.f=(float)temp_val.i;                   //converts integer to float
            calc_dht90(&humi_val.f,&temp_val.f);            //calculate humidity, temperature
  data=(uint)(100*humi_val.f);   
 Send_data[10]=(uint8)(data/ 100);
  data=(uint)(100*temp_val.f);              
  Send_data[9]=(uint8)(data / 100);
          }
 
if((((Send_data[9]>T_TEMP)? Send_data[9]-T_TEMP:T_TEMP-Send_data[9])>0X01)||(PDI==0XD1)) //温度值 
  {T_TEMP=Send_data[9];
  Send_data[7]=0X03;
   Send_data[8]=0XD1;
Send_datalend=0x05;
GenericApp_SendTheMessage(); //应用函数
  }
  }
  Send_data[9]=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
if((((Send_data[9]>T_ADC)? Send_data[9]-T_ADC:T_ADC-Send_data[9])>0X0A)||(PDI==0XD2)) //AD采集数值
  {T_ADC=Send_data[9];
  Send_data[9]=(uint8)(((float)(Send_data[9]*11))/17);
    Send_data[7]=0X03;
   Send_data[8]=0XD2;
   Send_data[10]=(165-Send_data[9]);
Send_datalend=0x05;
GenericApp_SendTheMessage(); //应用函数
  }
  PDI=0;
 }else GenericApp_ON_OFF++;

#endif  
#if(SENSOR_TYPE ==0X83)
if(GenericApp_ON_OFF>0)
{GenericApp_ON_OFF=0;
 if(!P0_5)
  {P1_0=~P1_0;//P1_2=~P1_2; 
  while(!P0_5);
  }
}GenericApp_ON_OFF++;
#endif
#if(SENSOR_TYPE ==0X03)
   if(P1_4)
   {
   // if((P1&0X0F)!=GenericApp_appIO)
    {
    GenericApp_appIO=(P1&0X0F);
    GenericApp_SendTheMessage();
     halMcuWaitMs(1000); 
     }
   }
 else 
   if(!(P1&0X0F))
   GenericApp_appIO=0XFF;
#endif 
#if(SENSOR_TYPE ==0X10)
if(GenericApp_applicationbuf>6000)
{GenericApp_applicationbuf=0;
GenericApp_sbum=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
GenericApp_sbum=(3233/(GenericApp_sbum+3))-11;//GP2Y0A02YKOF (20-150)cm
Send_data[7]=GenericApp_sbum;
GenericApp_SendTheMessage();
} 
#endif 
#if(SENSOR_TYPE ==0X0F)
if(GenericApp_applicationbuf>6000)
{GenericApp_applicationbuf=0;
GenericApp_sbum=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);

GenericApp_sbum=((200*GenericApp_sbum)/255);//(0-200)(0-200CM)

Send_data[7]=GenericApp_sbum;
GenericApp_SendTheMessage();
} 
#endif 
#if(SENSOR_TYPE ==0X17)
if(GenericApp_ON_OFF>5)
{GenericApp_ON_OFF=0;
GenericApp_switch=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);//-0x68;
if(GenericApp_switch>0x44)
Send_data[7]=3;
if(GenericApp_switch<0x45)
Send_data[7]=2;
if(GenericApp_switch<0x35)
Send_data[7]=1;
if(GenericApp_switch<0x32)
Send_data[7]=0;
if(Send_data[7]!=GenericApp_appIO)
{GenericApp_appIO=Send_data[7];
GenericApp_switchk2=0;
}
else GenericApp_switchk2++;
if(Send_data[7]==0)
{
  if(GenericApp_switchk2==10)
GenericApp_SendTheMessage();
}
else
{
if(GenericApp_switchk2==2)
GenericApp_SendTheMessage();
}
//Send_data[7]=GenericApp_switch;
//GenericApp_SendTheMessage();


} 
else GenericApp_ON_OFF++;
#endif
#if(SENSOR_TYPE ==0X11) 
if(GenericApp_applicationbuf>6000)
{GenericApp_applicationbuf=0;
GenericApp_sbum=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
GenericApp_sbum=((0X15*GenericApp_sbum)/0X17);
Send_data[7]=GenericApp_sbum;
Send_data[8]=0XAA;
GenericApp_SendTheMessage();
}
#endif 
#if((SENSOR_TYPE ==0X1A)) 
if(GenericApp_ON_OFF>10)
 { GenericApp_ON_OFF=0;
 GenericApp_sbum=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
  if(((GenericApp_sbum-T_MUMPT1)>0X0A)||((T_MUMPT1-GenericApp_sbum)>0X0A)||((GenericApp_sbum==0)&&(T_MUMPT1!=0)))
  {T_MUMPT1=GenericApp_sbum;
   GenericApp_applicationbuf=6001;
  }
 }else GenericApp_ON_OFF++;
if(GenericApp_applicationbuf>6000)
{GenericApp_applicationbuf=0;
#if(SENSOR_TYPE ==0X1A) 
GenericApp_sbum=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
//GenericApp_sbum=((0X15*GenericApp_sbum)/0X17);
Send_data[7]=GenericApp_sbum;
Send_data[8]=0XAA;
#endif 
GenericApp_SendTheMessage();
}
#endif 
#if((SENSOR_TYPE ==0X3D)||(SENSOR_TYPE ==0X3B)||(SENSOR_TYPE ==0X3C))
if(GenericApp_ON_OFF>10)
 { GenericApp_ON_OFF=0;
 GenericApp_sbum=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
if(((GenericApp_sbum>T_MUMPT1)? GenericApp_sbum-T_MUMPT1:T_MUMPT1-GenericApp_sbum)>0X0A) 
  {
    T_MUMPT1=GenericApp_sbum;
   GenericApp_applicationbuf=6001;
  }
 }else GenericApp_ON_OFF++;
if(GenericApp_applicationbuf>6000)
{GenericApp_applicationbuf=0;
GenericApp_sbum=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
#if(SENSOR_TYPE ==0X3B)            // 不分压直连  UVM-30紫外线 0-1V 
Send_data[9]=(GenericApp_sbum/4); //单位 UV index
#endif 
#if(SENSOR_TYPE ==0X3C)   //SCA60C:N1000060 0-180  0.5-4.5  10K 10K分压  0-180  0.25-2.25V  对应08H-68H 中间平衡34H
if(GenericApp_sbum>0X5E) Send_data[8]=180; //最大角度180度
else if(GenericApp_sbum<0X0A) Send_data[8]=0; //最小角度0度
else if(((GenericApp_sbum>0X34)? GenericApp_sbum-0X34:0X34-GenericApp_sbum)<3) Send_data[8]=90; //平衡点 90度
else Send_data[8]=((int)(((float)GenericApp_sbum-0x08)*2.1));
if(Send_data[8]>180)Send_data[8]=180;
#endif 
#if(SENSOR_TYPE ==0X3D)
if(GenericApp_sbum>0x34){Send_data[8]=0X01;GenericApp_sbum=130*(GenericApp_sbum-0X34);}  //正向电流   5V输出 10k 10K分压到2.5V   0X34相当于 0A 1250mV   1=130ma=24mV  185mv=1A
else {Send_data[8]=0;GenericApp_sbum=130*(0X34-GenericApp_sbum);}
Send_data[9]=GenericApp_sbum>>8;
Send_data[10]=GenericApp_sbum;
#endif 
GenericApp_SendTheMessage();
}
#endif 
#if(SENSOR_TYPE ==0X84)
if(GenericApp_ON_OFF>10)
 { GenericApp_ON_OFF=0;
  GenericApp_sbum=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
   //SCA100T:-D01 0-60度  -D02 0-180度   0.5-4.5  10K 10K分压  0-180  0.25-2.25V  对应00H-63H 中间平衡31H
if(GenericApp_sbum>0X60) Send_data[8]=60; //最大角度180度
else if(GenericApp_sbum<0X02) Send_data[8]=0; //最小角度0度
else if(((GenericApp_sbum>0X31)? GenericApp_sbum-0X31:0X31-GenericApp_sbum)<3) Send_data[8]=30; //平衡点 30度
else Send_data[8]=((int)((((float)GenericApp_sbum)*60)/0X60));
if(Send_data[8]>60)Send_data[8]=60;
//Send_data[8]=GenericApp_sbum;

GenericApp_sbum=HalAdcRead (HAL_ADC_CHN_AIN5, HAL_ADC_RESOLUTION_8);
    //SCA100T:-D01 0-60度  -D02 0-180度   0.5-4.5  10K 10K分压  0-180  0.25-2.25V  对应00H-63H 中间平衡31H
if(GenericApp_sbum>0X60) Send_data[9]=60; //最大角度180度
else if(GenericApp_sbum<0X02) Send_data[9]=0; //最小角度0度
else if(((GenericApp_sbum>0X31)? GenericApp_sbum-0X31:0X31-GenericApp_sbum)<3) Send_data[9]=30; //平衡点 30度
else Send_data[9]=((int)((((float)GenericApp_sbum)*60)/0X60));
if(Send_data[9]>60)Send_data[9]=60;
//Send_data[9]=GenericApp_sbum;

if((Send_data[8]==30)&&(Send_data[9]==30)){P1_1=1; P1_2=1;P1_3=1; P1_4=1;}
if((Send_data[8]==30)&&(Send_data[9]<30)){P1_1=0; P1_2=0;P1_3=1; P1_4=1;}
if((Send_data[8]==30)&&(Send_data[9]>30)){P1_1=1; P1_2=1;P1_3=0; P1_4=0;}
if((Send_data[8]<30)&&(Send_data[9]==30)){P1_1=0; P1_2=1;P1_3=1; P1_4=0;}
if((Send_data[8]<30)&&(Send_data[9]<30)){P1_1=0; P1_2=0;P1_3=1; P1_4=0;}
if((Send_data[8]<30)&&(Send_data[9]>30)){P1_1=0; P1_2=1;P1_3=0; P1_4=0;}
if((Send_data[8]>30)&&(Send_data[9]==30)){P1_1=1; P1_2=0;P1_3=0; P1_4=1;}
if((Send_data[8]>30)&&(Send_data[9]<30)){P1_1=0; P1_2=0;P1_3=0; P1_4=1;}
if((Send_data[8]>30)&&(Send_data[9]>30)){P1_1=1; P1_2=0;P1_3=0; P1_4=0;}

GenericApp_sbum=Send_data[8]+Send_data[9];
if(((GenericApp_sbum>T_MUMPT1)? GenericApp_sbum-T_MUMPT1:T_MUMPT1-GenericApp_sbum)>0X0A) 
  {
    T_MUMPT1=GenericApp_sbum;
   GenericApp_applicationbuf=6001;
  }
 }else GenericApp_ON_OFF++;

if(GenericApp_applicationbuf>6000)
{GenericApp_applicationbuf=0;
GenericApp_SendTheMessage();
}
#endif 
#if(SENSOR_TYPE ==0X18)
 if(GenericApp_uart_485==1)
   P1_0=0; 
  if(GenericApp_uart_485==0)
   GenericApp_uart_485=1; 
 if(GenericApp_applicationbuf>6000)
{ P1_0=1; 
 GenericApp_uart_485=0; 
  GenericApp_applicationbuf=0;
 uint8  hr[8]={0xA1,0X04,0X01,0X74,0XD0,0X16,0X0A,0X0D};
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
 hr[2]=0x02;hr[4]=0xD3;hr[5]=0x12;
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
 hr[2]=0x03;hr[4]=0xd2;hr[5]=0x12;
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
  hr[2]=0x04;hr[4]=0xd5;hr[5]=0x0e;
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
  hr[2]=0x05;hr[4]=0xD4;hr[5]=0x0E;
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
  hr[2]=0x06;hr[4]=0xd7;hr[5]=0x0a;
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
  hr[2]=0x07;hr[4]=0xd6;hr[5]=0x0a;
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
 hr[2]=0x08;hr[4]=0xd9;hr[5]=0x06;
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
  hr[2]=0x09;hr[4]=0xd8;hr[5]=0x06;
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
   hr[2]=0x0A;hr[4]=0xdB;hr[5]=0x02;
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
  hr[2]=0x0B;hr[4]=0xdA;hr[5]=0x02;
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
   hr[2]=0x0C;hr[4]=0xdD;hr[5]=0xFE;
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
   hr[2]=0x0D;hr[4]=0xdC;hr[5]=0xFE;
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
   hr[2]=0x0E;hr[4]=0xdF;hr[5]=0xFA;
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
   hr[2]=0x0F;hr[4]=0xdE;hr[5]=0xFA;
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
   GenericApp_SendTheMessage();
} 
#endif
#if((SENSOR_TYPE ==0X5A)||(SENSOR_TYPE ==0X5B))
 if(GenericApp_applicationbuf>6000)
{
  GenericApp_applicationbuf=0;
   GenericApp_SendTheMessage();
} 
if(GenericApp_ON_OFF>10)
 { GenericApp_ON_OFF=0; 
  GenericApp_applicationbuf=0;
 uint8  hr[8]={0x02,0X03,0X00,0X2A,0X00,0X01,0XA5,0XF1};//DMA模式 RS485发送
 HalUARTWrite(HAL_UART_PORT_0, hr,8);//DMA模式 RS485发送最后两字节会丢掉。所以多添加两字节
}else GenericApp_ON_OFF++;
#endif
#if(SENSOR_TYPE ==0X70)
 if(GenericApp_uart_485==1)
   P1_0=0; 
  if(GenericApp_uart_485==0)
   GenericApp_uart_485=1; 
#endif
#if(SENSOR_TYPE ==0X13)
if(GenericApp_applicationbuf>6000)
{
  GenericApp_applicationbuf=0;
#if defined(HUABANG) 
 uint8  hr[8]={0xFF,0X03,0X00,0X00,0X00,0X02,0XD1,0XD5};
 HalUARTWrite(HAL_UART_PORT_0, hr,8); 
#endif
#if defined(ammeter) 
  uint8  hr[15]={0xFE,0X68,0X99,0X99,0X99,0X99,0X99,0X99,0X68,0X01,0X02,0X43,0XC3,0X6F,0X16};
 HalUARTWrite(HAL_UART_PORT_0, hr,15);    
#endif
} 
#endif
#if(SENSOR_TYPE ==0X74)
if(GenericApp_applicationbuf>6000)
{GenericApp_applicationbuf=0;
 {Send_data[7]=Send_data[8]=Send_data[9]=0XBB;}
 Send_data[10]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9])%256;
GenericApp_SendTheMessage();
} 
#endif
#if(SENSOR_TYPE ==0X14)
if(GenericApp_applicationbuf>6000)
{
  GenericApp_applicationbuf=0;
#if defined(PH_A)
 //uint8  hr[8]={0x01,0x03,0x00,0x00,0x00,0x0C,0xA5,0xE2}; 旧的
 uint8  hr[8]={0x01,0x03,0x00,0x00,0x00,0x0C,0x45,0xCF};// 新的
 HalUARTWrite(HAL_UART_PORT_0, hr,8);
#endif
#if defined(PH_ORP)
uint8  hr[8]={0x00,0x03,0x00,0x00,0x00,0x02,0xC5,0xDA};
HalUARTWrite(HAL_UART_PORT_0, hr,8);
#endif
#if defined(PH_E201C)
  float PH_A2;
unsigned int  PH;	
GenericApp_sbum=HalAdcRead (HAL_ADC_CHN_AIN5, HAL_ADC_RESOLUTION_8); //环境温度采集
Send_data[10]=((GenericApp_sbum*345)/4)>>8;   //LM35 输出0-150mv 硬件放大，软件还原
Send_data[11]=((GenericApp_sbum*345)/4);
GenericApp_sbum=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8); //PH采集
/***********************液体温度18B20采集**************************/
 unsigned int  data;						    
    float Temperature;
        Temperature = ReadTemperature();        //返回16位二进制数
        Temperature *= CelsiurPerLSB;		//转换成浮点型温度值(摄氏度)
         
        // data=(unsigned int)(100*Temperature);   
         data=(unsigned int)(Temperature);  
 /***********************液体温度18B20采集**************************/
 /***********************PH温度数据处理**************************/
 PH_A2=((float)(GenericApp_sbum*330)/127.0);  //实际采集数值对应电压值  需实际调整
 PH_A2=PH_A2-87.90;              //采集电压值减去87.90  需实际调整
 PH_A2=1400.0-PH_A2/0.089;              //计算PH值
 PH=(unsigned int)(PH_A2);      //PH值
 if(data<100){if(data>25)PH=PH+(data-25); else PH=PH-(25-data);}//  PH值进行温度补偿
 Send_data[8]=(uint8)(PH>>8);
 Send_data[9]=(uint8)(PH);
 Send_data[7]=0XCC;
 Send_data[12]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11])%256;
Send_datalend=0X07;
GenericApp_SendTheMessage();
 /***********************液PH温度数据处理*************************/
 
#endif 
} 
#endif
#if(SENSOR_TYPE ==0X21)
if(GenericApp_ON_OFF>10)
 { GenericApp_ON_OFF=0;
 Send_data[7]=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
  if(((Send_data[7]-T_MUMPT1)>0X1A)||((T_MUMPT1-Send_data[7])>0X1A)||((Send_data[7]==0)&&(T_MUMPT1!=0)))
  {T_MUMPT1=Send_data[7];
   GenericApp_applicationbuf=6001;
  }
 }else GenericApp_ON_OFF++;
if(GenericApp_applicationbuf>6000)
{GenericApp_applicationbuf=0;
//GenericApp_sbum=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
Send_data[7]=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
GenericApp_SendTheMessage();
}
#endif 
#if(SENSOR_TYPE ==0X66)
LEDtime++;
if(LEDtime>0)
{LEDtime=0;
 switch(LED_HL)					  
  {case 0:LED_HL=1;break; 
  case 1:LED_HL=0;break;
  default:LED_HL=0;break;
  }
 display();      //数码管扫描程序
}
#endif 
#if(SENSOR_TYPE ==0X76)
if(GenericApp_applicationbuf>300)
{
if((!P2_0)&&(0==p))
{p=1;lens++;
GenericApp_applicationbuf=0; 
GenericApp_SendTheMessage();
}
halMcuWaitMs(50); 
if(P2_0){p=0;}
Send_data[8]=lens/256;
Send_data[9]=lens%256;
}


if(GenericApp_applicationbuf>10000)
{GenericApp_applicationbuf=0;
lens=0;
}else GenericApp_applicationbuf++;
#endif 

#if(SENSOR_TYPE =='G')
if(GenericApp_ON_OFF>90)
 { GenericApp_ON_OFF=0;
 switch(Coil_AB)
  { 
    case 1:Coil_A1;Coil_AB++;break;//通电次序：+A,+B,-A,-B
    case 2:Coil_A2;Coil_AB++;break;
    case 3:Coil_B1;Coil_AB++;break;
    case 4:Coil_B2;Coil_AB=1;break;
    
    case 10:Coil_B2;Coil_AB++;break;//通电次序：+A,+B,-A,-B
    case 11:Coil_B1;Coil_AB++;break;
    case 12:Coil_A2;Coil_AB++;break;
    case 13:Coil_A1;Coil_AB=10;break;
     default:break;
  }		
 }else GenericApp_ON_OFF++;
#endif 
#if(SENSOR_TYPE ==0X30)
if(c68>1)c68--;
if(c68==1)
{c68=0;a68=0xab;b68=B01;}
static unsigned char i;	
if(P2_0)
while(b68)
{
  if(P2_0==0)
  {b68=0;
   switch(a68)
  { 
  case 0XAB:for(int t=0;t<10;t++){Coil_B2;halMcuWaitMs(10);Coil_A2;halMcuWaitMs(10);Coil_B1;halMcuWaitMs(10);Coil_A1;halMcuWaitMs(10);}break;
  case 0XBA:for(int t=0;t<10;t++){Coil_A1;halMcuWaitMs(10);Coil_B1;halMcuWaitMs(10);Coil_A2;halMcuWaitMs(10);Coil_B2;halMcuWaitMs(10);}break;
     default:break;
  }			
  }
  
halMcuWaitMs(20);
if((a68==0XAB)&&b68)	//正转标志
{
  switch(i)
  { 
    case 0:Coil_A1;i++;break;//通电次序：+A,+B,-A,-B
    case 1:Coil_B1;i++;break;
    case 2:Coil_A2;i++;break;
    case 3:Coil_B2;i=0;b68--;break;
     default:break;
  }			
}  
else    
  if((a68==0XBA)&&b68)	//反转
  { 
    switch(i)
    {						
    case 0:Coil_B2;i++;break;
    case 1:Coil_A2;i++;break;
    case 2:Coil_B1;i++;break;
    case 3:Coil_A1;i=0;b68--;break;
    default:break;
    }			
  }		
}
if(b68==0)Coil_OFF ;
#endif
#if(SENSOR_TYPE ==0X73)
if(P2_0)
{if(b68)C68=1;
if((a68==0XAB)&&b68)	//正转标志
{
 DIR=0;b68-=CP;{CP=~CP; halMcuWaitUs(800);}
}  
else    
if((a68==0XBA)&&b68)	//反转
  { 
 DIR=1;b68-=CP;{CP=~CP; halMcuWaitUs(800);}
  }		
}
//else{b68=0;}
 if(P2_0==0)
  {b68=0;
   if(C68)
  {
    halMcuWaitMs(5);
   switch(a68)
  { 
  case 0XAB: DIR=1; for(int i=300;i>0;i--){CP=~CP; halMcuWaitUs(1500);}Send_data[10]=0XAB; GenericApp_applicationbuf=1; GenericApp_SendTheMessage(); break;
  case 0XBA: DIR=0; for(int i=300;i>0;i--){CP=~CP; halMcuWaitUs(1500);}Send_data[10]=0XBA;  GenericApp_applicationbuf=1;GenericApp_SendTheMessage(); break;
     default:break;
  }C68=0;		
  }
  }
if(b68==0) 
{DIR=1;CP=1;}
#endif
#if(SENSOR_TYPE ==0X6A)
#if defined(HX711P) 
if(GenericApp_ON_OFF>10)
 { GenericApp_ON_OFF=0;
 AD_filter();//AD和滤波
 if((AD_Compar&0x800000)!=0x800000)
 {
         AD_Compar=  AD_Compar/2;//求体重//壳体测试
         AD_Compar=  AD_Compar-Send_data[7];
        
 }
 else AD_Compar=0;
 
  Send_data[8]=AD_Compar/256;
  Send_data[9]=AD_Compar%256;

if(((Send_data[9]-T_MUMPT1)>0X50)||((T_MUMPT1-Send_data[9])>0X50)||((Send_data[9]==0)&&(T_MUMPT1!=0)))//
  {T_MUMPT1=Send_data[9];
   GenericApp_applicationbuf=6001;
  }
 }else GenericApp_ON_OFF++;
#endif 
if(GenericApp_applicationbuf>6000)
{GenericApp_applicationbuf=1;
GenericApp_switch=2;
#if defined(ADCP) 
Send_data[8]=0;
//GenericApp_sbum=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8);
Send_data[9]=HalAdcRead (HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_8)-Send_data[7];
#endif 
GenericApp_SendTheMessage();
}
#endif 
#if(SENSOR_TYPE ==0X23)
   if(GenericApp_ON_OFF>1)
   {GenericApp_ON_OFF=0;
     if(SFG_R30X)
   { char DD[17]={0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x08,0x04,0x01,0x00,0x00,0x00,0xFF,0x01,0x0D}; 
       SFG_R30X=0;
     if(SFG_R30XA[0]==0x01)
       { 
           DD[8]=0x03;DD[9]=0X01;DD[10]=0X00 ;DD[11]=0X05;       //录入图像
           HalUARTWrite(HAL_UART_PORT_0, (uint8*)DD,12);
        }
          if(SFG_R30XA[0]==0x02)
       { 
           DD[8]=0x04;DD[9]=0X02;DD[10]=0X01 ;DD[11]=0X00;DD[12]=0X08; //生成特征1
         HalUARTWrite(HAL_UART_PORT_0, (uint8*)DD,13);
       }
       if(SFG_R30XA[0]==0x03)
       {  
           DD[8]=0x03;DD[9]=0X01;DD[10]=0X00 ;DD[11]=0X05;       //录入图像
           HalUARTWrite(HAL_UART_PORT_0, (uint8*)DD,12);
        }
        if(SFG_R30XA[0]==0x04)
       { 
           DD[8]=0x04;DD[9]=0X02;DD[10]=0X02 ;DD[11]=0X00; DD[12]=0X09; //生成特征2
           HalUARTWrite(HAL_UART_PORT_0, (uint8*)DD,13);
       }
        if(SFG_R30XA[0]==0x05)
       {
           DD[8]=0x03;DD[9]=0X05;DD[10]=0X00 ;DD[11]=0X09;  //合并特征
            HalUARTWrite(HAL_UART_PORT_0, (uint8*)DD,12);
       }
        if(SFG_R30XA[0]==0x06)
       { 
           DD[8]=0x06;DD[9]=0X06;DD[10]=0X02 ;DD[11]=SFG_R30XA[1]; DD[12]=SFG_R30XA[2]; //DD[13]=0X00;DD[14]=0X10;//存储特征
           DD[13]=(1+DD[8]+DD[9]+DD[10]+DD[11]+DD[12])/256;  DD[14]=(1+DD[8]+DD[9]+DD[10]+DD[11]+DD[12])%256;
           HalUARTWrite(HAL_UART_PORT_0, (uint8*)DD,15);
         }
         if(SFG_R30XA[0]==0xA1)//搜索
         { DD[8]=0x03;DD[9]=0X01;DD[10]=0X00 ;DD[11]=0X05;       //录入图像
           HalUARTWrite(HAL_UART_PORT_0, (uint8*)DD,12);
         }
          if(SFG_R30XA[0]==0xA2)
          { DD[8]=0x04;DD[9]=0X02;DD[10]=0X01 ;DD[11]=0X00;DD[12]=0X08; //生成特征1
           HalUARTWrite(HAL_UART_PORT_0, (uint8*)DD,13);
          }
           if(SFG_R30XA[0]==0xA3)
           {
           DD[8]=0x08;DD[9]=0X04;DD[10]=0X01 ;DD[11]=0X00; DD[12]=0X00; DD[13]=0X00;DD[14]=0XFF;DD[15]=0X01;DD[16]=0X0D;//对比图像
           HalUARTWrite(HAL_UART_PORT_0, (uint8*)DD,17);
           }
        if(SFG_R30XA[0]==0x21)
        {
          if((SFG_R30XA[1]==0xFF)&&(SFG_R30XA[2]==0xFF))//清空指纹库
          {
          DD[8]=0x03;DD[9]=0X0D;DD[10]=0X00 ;DD[11]=0X11;       
           HalUARTWrite(HAL_UART_PORT_0, (uint8*)DD,12);
           }
          else
          {DD[8]=0x07;DD[9]=0X0c;DD[10]=SFG_R30XA[1] ;DD[11]=SFG_R30XA[2]; DD[12]=0x00;DD[13]=0x01; //清空指纹库
          DD[14]=(2+DD[8]+DD[9]+DD[10]+DD[11]+DD[12])/256;  DD[15]=(2+DD[8]+DD[9]+DD[10]+DD[11]+DD[12])%256;
          HalUARTWrite(HAL_UART_PORT_0, (uint8*)DD,16);
          }
        }
        if(SFG_R30XA[0]==0xBB)
       {GenericApp_applicationbuf=1;
        Send_data[7]=0XAA;
       Send_data[8]=SFG_R30XA[1];Send_data[9]=SFG_R30XA[2];
         GenericApp_SendTheMessage();
         }
      if(SFG_R30XA[0]==0xDD)
       {GenericApp_applicationbuf=1;
        Send_data[7]=0XCC;
       Send_data[8]=SFG_R30XA[1];Send_data[9]=SFG_R30XA[2];
         GenericApp_SendTheMessage();
         }
          if(SFG_R30XA[0]==0xCC)
       {  GenericApp_applicationbuf=1;
        Send_data[7]=0XBB;
         GenericApp_SendTheMessage();
         SFG_R30XA[0]=0xA1;SFG_R30X=1;
         }
         }
   }
   else GenericApp_ON_OFF++;
#endif
#if(SENSOR_TYPE =='Q')
 if(GenericApp_ON_OFF>2)
 { GenericApp_ON_OFF=0;
 if( HXS>0X00)//屏幕固定数值闪烁
 {        // for(char p=0;p<3;p++)
    HXtime++;
           if(HXS>0X02)
            {
             if(HXtime==1)
              Ht1621WrAllData(0,Ht1621Tab,9);//清除1621寄存器数据，暨清屏 //SEG0～SEG17 COM0-3=0X00 0000 全灭
             if(HXtime>2)
             { {HXtime=0;HXS--;}
              P1_0=0;
#if defined(SHINING75MIL) 
              Ht1621WrOneData(9,0X01); //KG
            if(((AD_Compar/100)%10)!=0)
            { if(AD_Compar/1000)
               Ht1621WrOneData(1,(table2[(AD_Compar/100)%10])|0x01);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9  |百位1
               else
               Ht1621WrOneData(1,table2[(AD_Compar/100)%10]);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
               
               Ht1621WrOneData(0,table1[(AD_Compar/100)%10]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
            }
              Ht1621WrOneData(2,table1[(AD_Compar/10)%10]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
              Ht1621WrOneData(3,table2[(AD_Compar/10)%10]|0X01);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
              Ht1621WrOneData(4,table1[(AD_Compar%10)]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
              if(PAN_ID_Success==2)
              Ht1621WrOneData(5,(table2[(AD_Compar%10)])|0X01);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9 | M 添加网络显示M
              else
              Ht1621WrOneData(5,table2[(AD_Compar%10)]);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
              
              Ht1621WrOneData(6,table1[0]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
              Ht1621WrOneData(7,table2[0]);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
#endif
#if defined(SHINING100MIL)  
Ht1621WrOneData(8,0X0C); //公斤
 if(PAN_ID_Success==2)
              {
              Ht1621WrOneData(0,table1[AD_Compar/1000]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
              Ht1621WrOneData(1,table2[AD_Compar/1000]);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
              }
              Ht1621WrOneData(2,table1[(AD_Compar/100)%10]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
              Ht1621WrOneData(3,table2[(AD_Compar/100)%10]);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
              Ht1621WrOneData(4,table1[(AD_Compar/10)%10]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
              Ht1621WrOneData(5,table2[(AD_Compar/10)%10]|0X01);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
              Ht1621WrOneData(6,table1[(AD_Compar%10)]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
              Ht1621WrOneData(7,table2[(AD_Compar%10)]);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
#endif
          }
           
 }
 else
 {
           if(HXtime>8)
           {HXtime=0;HXS=0;
            Send_datalend=0x03;  
            Send_data[7]=RS;
            Send_data[8]=(Send_data[6]+Send_data[7])%256;
            GenericApp_SendTheMessage(); //应用函数
            Ht1621WrAllData(0,Ht1621Tab,9);//清除1621寄存器数据，暨清屏 //SEG0～SEG17 COM0-3=0X00 0000 全灭
             P1_0=1;
             RSn=0;
             T_MUMPT1=21;
           }
 }
 }
 else 
 {          AD_filter();//AD和滤波
          // AD_Compar=  AD_Compar/80;//求体重 //参照
            AD_Compar=  10*AD_Compar;//求体重//壳体测试
            AD_Compar=  AD_Compar/AD_SHINING;//求体重//壳体测试
           if((AD_Compar<10)&&(AD_Compar>(-5)))AD_Compar=0;
           RS=AD_Compar/10;
           if(T_MUMPT1<20)
            {
              P1_0=0;
#if defined(SHINING75MIL) 
              Ht1621WrOneData(9,0X01); //KG
            if(((AD_Compar/100)%10)!=0)
            { if(AD_Compar/1000)
               Ht1621WrOneData(1,(table2[(AD_Compar/100)%10])|0x01);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9  |百位1
               else
               Ht1621WrOneData(1,table2[(AD_Compar/100)%10]);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
               
               Ht1621WrOneData(0,table1[(AD_Compar/100)%10]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
            }
              Ht1621WrOneData(2,table1[(AD_Compar/10)%10]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
              Ht1621WrOneData(3,table2[(AD_Compar/10)%10]|0X01);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
              Ht1621WrOneData(4,table1[(AD_Compar%10)]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
              if(PAN_ID_Success==2)
              Ht1621WrOneData(5,(table2[(AD_Compar%10)])|0X01);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9 | M 添加网络显示M
              else
              Ht1621WrOneData(5,table2[(AD_Compar%10)]);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
              
              Ht1621WrOneData(6,table1[0]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
              Ht1621WrOneData(7,table2[0]);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
#endif
#if defined(SHINING100MIL)  
Ht1621WrOneData(8,0X0C); //公斤
 if(PAN_ID_Success==2)
              {
              Ht1621WrOneData(0,table1[AD_Compar/1000]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
              Ht1621WrOneData(1,table2[AD_Compar/1000]);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
              }
              Ht1621WrOneData(2,table1[(AD_Compar/100)%10]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
              Ht1621WrOneData(3,table2[(AD_Compar/100)%10]);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
              Ht1621WrOneData(4,table1[(AD_Compar/10)%10]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
              Ht1621WrOneData(5,table2[(AD_Compar/10)%10]|0X01);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
              Ht1621WrOneData(6,table1[(AD_Compar%10)]);//SEG1 COM0-3=table1[9]=0X0E 1110 数字9
              Ht1621WrOneData(7,table2[(AD_Compar%10)]);//SEG2 COM0-3=table2[9]=0X0D 1101 数字9
          
#endif
         }  
           if(((RSn>RS)?(RSn-RS):(RS-RSn))>1)
              {T_MUMPT1=0;
              RSn=RS;
              }
          else   
        {
          RSn=RS;
          if(T_MUMPT1<8)T_MUMPT1++;
          else
          { 
           if(AD_Compar<50)
           { 
            if(AD_Compar==0)
            {
             Ht1621WrAllData(0,Ht1621Tab,9);//清除1621寄存器数据，暨清屏 //SEG0～SEG17 COM0-3=0X00 0000 全灭
             P1_0=1;
             T_MUMPT1=21;
            }
            else 
             {T_MUMPT1=0;
             Tozero();//自动归零
            }
           }
           else 
           {HXS=0X05;
           Send_datalend=0x03;  
            Send_data[7]=RS;
            Send_data[8]=(Send_data[6]+Send_data[7])%256;
            GenericApp_SendTheMessage(); //应用函数
           }
           
          }
        }
  }
}else GenericApp_ON_OFF++;
#endif
#if(SENSOR_TYPE =='D')
 if(GenericApp_applicationbuf>6000)
  { P1_3=1;
    GenericApp_applicationbuf=0;
    for(unsigned int i=0;i<50000;i++);
    GenericApp_SendTheMessage(); //应用函数
  }
#endif
#if(SENSOR_TYPE ==0X33)
 if(GenericApp_ON_OFF>10)
 { GenericApp_ON_OFF=0;
B_LUX30_Multiple_read(0X00,&Send_data[8],4);

if(((Send_data[9]-T_MUMPT1)>0X04)||((T_MUMPT1-Send_data[9])>0X04)||((Send_data[9]==0)&&(T_MUMPT1!=0)))//327 Lux
  {T_MUMPT1=Send_data[9];
   GenericApp_SendTheMessage(); //应用函数
  }
 }else GenericApp_ON_OFF++;
 
if(GenericApp_applicationbuf>6000)
  {GenericApp_applicationbuf=0;
    GenericApp_SendTheMessage(); //应用函数
  }
#endif
#if(SENSOR_TYPE ==0X85)
if(GenericApp_ON_OFF>10)
 { GenericApp_ON_OFF=0;
     unsigned char     slaveaddress;	 			  
     unsigned long int    DATA;  
     unsigned char tempbat[2];
     
    slaveaddress=MEM_READ(0x00,0x10);
         //读取存于MLX90615 EEPROM "00h"地址中的SMBus地址
         DATA=MEM_READ(slaveaddress,0x27);
         //基于上述地址由MLX90615的内存07h中读取物体温度
          CALTEMP(DATA,tempbat);	
         //基于所得的十六进制温度格式计算实际温度
         // UartTX_Send_String(tempbat,5);	
           int temp;
          temp=(tempbat[0]*100+tempbat[0]);
            data[4]=temp>>8;
            data[5]=temp;
            data[8]= data[0]+data[1]+data[2]+data[3]+data[4]+data[5]+data[6]+data[7];

            Send_data[7]=0xAA;
            Send_data[8]=tempbat[0];//整数部分
            Send_data[9]=tempbat[1];//小数部分
            Send_data[10]=0xAA;    //保留
            Send_data[11]=0xAA;    //保留
         if(((Send_data[8]>T_MUMPT1)?( Send_data[8]-T_MUMPT1):(T_MUMPT1-Send_data[8]))>0X01) //大于1
         {T_MUMPT1=Send_data[8];
         GenericApp_applicationbuf=6001;
         }

 }else GenericApp_ON_OFF++;
 if(GenericApp_applicationbuf>6000)
  {GenericApp_applicationbuf=0;
  HalUARTWrite(HAL_UART_PORT_0,data,9);
    GenericApp_SendTheMessage(); //应用函数
  }
#endif
#if defined (NB_IOT_S10)
 
 if(NBjishu>5)
 { 
 //{"Nbiot-up":{"IMEI":"0123456789ABCDEF","HUMITURE":"25-36"}}
 //{"Nbiot-up" //"7b224e62696f742d757022"
 //:{"IMEI"// "3a7b22494d454922"
 //:"0123456789ABCDEF"//"3a223031323334353637383941424344454622"
 //,"HUMITURE" //"2c2248554d495455524522"
 //:"25-36"}}//"3a2232352d3336227d7d"
  uint8 tmp[32]={0};
   NBjishu=0;
 switch(NBliucheng)
 {case 0: HalUARTWrite(HAL_UART_PORT_0,"AT+NRB\r\n",8);
          HalUARTWrite(HAL_UART_PORT_0,"AT+NCONFIG=AUTOCONNECT,TRUE\r\n",29);break;
  case 1: HalUARTWrite(HAL_UART_PORT_0,"AT+NBAND=5\r\n",12);break;
  case 2: HalUARTWrite(HAL_UART_PORT_0,"AT+CFUN=1\r\n",11);break;
  case 3: HalUARTWrite(HAL_UART_PORT_0,"AT+CIMI\r\n",9);break;
  case 4: HalUARTWrite(HAL_UART_PORT_0,"AT+CGDCONT=1,\"IP\",\"CTNB\"\r\n",26);break;
  case 5: HalUARTWrite(HAL_UART_PORT_0,"AT+CGATT=1\r\n",12);break;
  case 6: HalUARTWrite(HAL_UART_PORT_0,"AT+COPS=1,2,\"46011\"\r\n",21);break;
  case 7: HalUARTWrite(HAL_UART_PORT_0,"AT+CGSN=1\r\n",11);break; //读取IMEI号
  case 15: HalUARTWrite(HAL_UART_PORT_0,"AT+CGATT?\r\n",11);break; //入网成功
  case 16: HalUARTWrite(HAL_UART_PORT_0,"AT+NSOCR=DGRAM,17,5050,1\r\n",26);break;//创建Socket 5050
  case 17: //strcat((char*)tmp, (char const*)tmp1);
     for(int i=0;i<16;i++)
    {  tmp[2*i]='3';tmp[2*i+1]=NB_IMEI[i];}
    tmp[31]='0';
    
    HalUARTWrite(HAL_UART_PORT_0,"AT+NSOST=0,139.196.218.156,5050,56,",35);//发送数据
    HalUARTWrite(HAL_UART_PORT_0,"7b224e622d757022",16);//发送数据  //{"Nb-up" //"7b224e622d757022"
    HalUARTWrite(HAL_UART_PORT_0,"3a7b22494d4549223a22",20);//发送数据  //:{"IMEI":"// "3a7b22494d4549223a22"
    //HalUARTWrite(HAL_UART_PORT_0,"30313233343536373839414243444546",32);//发送数据  //0123456789ABCDEF//"30313233343536373839414243444546"
    HalUARTWrite(HAL_UART_PORT_0,tmp,32);//发送数据  //0123456789ABCDEF//"30313233343536373839414243444546"
    NBjishu=4;NBliucheng=18;break;
 case 18: //Send_data[7]=34; Send_data[9]=60;
     tmp[0]='3';
     if((Send_data[9])<100){tmp[1]=(Send_data[9]/10)+0X30;}else tmp[1]='0'; //整数位
     tmp[2]='3';
     if((Send_data[9])<100){tmp[3]=(Send_data[9]%10)+0X30;}else tmp[3]='0';
     tmp[4]='2';
     tmp[5]='d';
     tmp[6]='3';
     if((Send_data[7])<100){tmp[7]=(Send_data[7]/10)+0X30;}else tmp[7]='0'; //整数位
     tmp[8]='3';
     if((Send_data[7])<100){tmp[9]=(Send_data[7]%10)+0X30;}else tmp[9]='0';
     HalUARTWrite(HAL_UART_PORT_0,"222c2248554d4954555245223a22",28);//发送数据   //:,"HUMITURE":" //"222c2248554d4954555245223a22"
   // HalUARTWrite(HAL_UART_PORT_0,"32352d3336",10);//发送数据  //25-36//"32352d3336"
     HalUARTWrite(HAL_UART_PORT_0,tmp,10);//发送数据  //25-36//"32352d3336"
    HalUARTWrite(HAL_UART_PORT_0,"227d7d\r\n",8);NBliucheng=100;break;//发送数据  //}} //227d7d//结束符
    
  case 19: HalUARTWrite(HAL_UART_PORT_0,"AT+NSORF=0,100\r\n",16);break;//读取数据
  case 30: HalUARTWrite(HAL_UART_PORT_0,"AT+NSORF=0,100\r\n",16);break;//读取数据
  case 40: HalUARTWrite(HAL_UART_PORT_0,"AT+NSORF=0,100\r\n",16);break;//读取数据
  case 100:break;//退出
  default:break;//退出

 }
 //NBliucheng++;
 
  if(NBliucheng>50)NBliucheng--;
  if(NBliucheng==55)NBliucheng=17;
 }
 else NBjishu++;

#endif
 
#if(SENSOR_TYPE =='E')
if(GenericApp_ON_OFF>10)
 { GenericApp_ON_OFF=0;
 uint  data;
  valuen humi_val,temp_val;							    
  unsigned char error,checksum; 
 error=0;  
           error+=s_measure( &Send_data[7],&checksum,HUMI);  //measure humidity 
           error+=s_measure( &Send_data[9],&checksum,TEMP);  //measure temperature 
          if(error!=0) s_connectionreset();                  //in case of an error: connection  
          else 
          { humi_val.i=Send_data[7]*256+Send_data[8];
            temp_val.i=Send_data[9]*256+Send_data[10];
            humi_val.f=(float)humi_val.i;                   //converts integer to float
            temp_val.f=(float)temp_val.i;                   //converts integer to float
            calc_dht90(&humi_val.f,&temp_val.f);            //calculate humidity, temperature
 Send_data[6]=SENSOR_TYPE;//传感器类型
  data=(uint)(100*humi_val.f);   
 Send_data[7]=(uint8)(data/ 100);
  Send_data[8]=(uint8)(data % 100);
  data=(uint)(100*temp_val.f);              
  Send_data[9]=(uint8)(data / 100);
  Send_data[10]=(uint8)(data % 100);
          }
if(((Send_data[9]-T_MUMPT1)>0X01)||((T_MUMPT1-Send_data[9])>0X01))//温度值
  {T_MUMPT1=Send_data[9];
  GenericApp_applicationbuf=6001;
  }
 }else GenericApp_ON_OFF++;


  if(GenericApp_applicationbuf>6000)
  {GenericApp_applicationbuf=0;

#if defined (NB_IOT_S10)
   if(NBliucheng>50)NBliucheng=17;
 //  P0_4=~P0_4;
#endif
    GenericApp_SendTheMessage(); //应用函数
  }

#endif
#if(SENSOR_TYPE ==0X82)
if(GenericApp_ON_OFF>10)
 { GenericApp_ON_OFF=0;
  Multiple_Read_HMC5883();      //连续读出数据，存储在BUF中
    //---------显示X轴
   // HMC5883_x=(BUF[0] << 8 | BUF[1]); //Combine MSB and LSB of X Data output register
   // HMC5883_z=(BUF[2] << 8 | BUF[3]); //Combine MSB and LSB of Z Data output register
   // HMC5883_y=(BUF[4] << 8 | BUF[5]); //Combine MSB and LSB of Y Data output register

  //HMC5883_x = atan2((float)(0.92* (BUF[4] << 8 | BUF[5])),(float)(0.92* (BUF[0] << 8 | BUF[1])));
  //HMC5883_y = atan2((float)(0.92* (BUF[2] << 8 | BUF[3])),(float)(0.92* (BUF[0] << 8 | BUF[1])));
  //HMC5883_z= atan2((float)(0.92* (BUF[2] << 8 | BUF[3])),(float)(0.92* (BUF[4] << 8 | BUF[5])));
  //HMC5883_x =((HMC5883_x < 0)?(HMC5883_x += 2*PI):(HMC5883_x -= 2*PI)) * 180/M_PI;
  //HMC5883_y =((HMC5883_y < 0)?(HMC5883_y += 2*PI):(HMC5883_y -= 2*PI)) * 180/M_PI;
 // HMC5883_z =((HMC5883_z < 0)?(HMC5883_z += 2*PI):(HMC5883_z -= 2*PI)) * 180/M_PI;
  
    HMC5883_x= atan2((float)(BUF[4] << 8 | BUF[5]),(float)(BUF[0] << 8 | BUF[1])) * (180 / 3.14159265) + 180; // angle in degrees
    HMC5883_y= atan2((float)(BUF[2] << 8 | BUF[3]),(float)(BUF[0] << 8 | BUF[1])) * (180 / 3.14159265) + 180; // angle in degrees
    HMC5883_z= atan2((float)(BUF[2] << 8 | BUF[3]),(float)(BUF[4] << 8 | BUF[5])) * (180 / 3.14159265) + 180; // angle in degrees
   
   Send_data[8]=(int)HMC5883_x>>8;
   Send_data[9]=(int)HMC5883_x;
   Send_data[10]=(int)HMC5883_y>>8;
   Send_data[11]=(int)HMC5883_y;
   Send_data[12]=(int)HMC5883_z>>8;
   Send_data[13]=(int)HMC5883_z;
if(((Send_data[9]>T_MUMPT1)?( Send_data[9]-T_MUMPT1):(T_MUMPT1-Send_data[9]))>0X05) //x角度大于5
  {T_MUMPT1=Send_data[9];
  GenericApp_applicationbuf=6001;
  }
if(((Send_data[11]>T_MUMPT_y)?( Send_data[11]-T_MUMPT_y):(T_MUMPT_y-Send_data[11]))>0X05) //y角度大于5
  {T_MUMPT_y=Send_data[11];
  GenericApp_applicationbuf=6001;
  }
if(((Send_data[13]>T_MUMPT_z)?( Send_data[13]-T_MUMPT_z):(T_MUMPT_z-Send_data[13]))>0X05) //z角度大于5
  {T_MUMPT_z=Send_data[13];
  GenericApp_applicationbuf=6001;
  }
 }else GenericApp_ON_OFF++;

  if(GenericApp_applicationbuf>6000)
  {GenericApp_applicationbuf=0;
    GenericApp_SendTheMessage(); //应用函数
  }
#endif
  #if(SENSOR_TYPE ==0X81)
if(GenericApp_ON_OFF>10)
 { GenericApp_ON_OFF=0;
   GenericApp_sbum=read_max6675();
   if(GenericApp_sbum==0X8001){Send_data[8]=1;Send_data[9]=0;Send_data[10]=0;}//1 开路
   else  { GenericApp_sbum=(GenericApp_sbum/4);
   Send_data[8]=0;//正常
   Send_data[9]=(GenericApp_sbum>>8);
   Send_data[10]=(GenericApp_sbum);
if(((Send_data[10]>T_MUMPT1)?( Send_data[10]-T_MUMPT1):(T_MUMPT1-Send_data[10]))>0X20) //温度变化超过 32度
  {T_MUMPT1=Send_data[10];
  GenericApp_applicationbuf=6001;
  }
}
 }else GenericApp_ON_OFF++;
  if(GenericApp_applicationbuf>6000)
  {GenericApp_applicationbuf=0;
    GenericApp_SendTheMessage(); //应用函数
  }
#endif
#if((SENSOR_TYPE =='B')||(SENSOR_TYPE =='C')||(SENSOR_TYPE =='F')||(SENSOR_TYPE =='U'))
  if(GenericApp_applicationbuf>6000)
  {GenericApp_applicationbuf=0;
    GenericApp_SendTheMessage(); //应用函数
  }
#endif
#if(SENSOR_TYPE =='S')
  if((P1&0XE0)!=GenericApp_appIO)
  {
     GenericApp_appIO=(P1&0XE0);
   if(P1_5)
    {
      if(GenericApp_ON_OFF<0x71)
      {
       if(GenericApp_ON_OFF>0x1F)
       GenericApp_ON_OFF+=0x05;
     else
       GenericApp_ON_OFF=0x20;
      }
      else
        GenericApp_ON_OFF=0x75;
     DimmingAppControl(GenericApp_ON_OFF,0X01);
       GenericApp_applicationbuf=1;
         GenericApp_SendTheMessage();
    }
      if(P1_7)
    {
       if(GenericApp_ON_OFF>0x20)
       GenericApp_ON_OFF-=0x05;
     else
       GenericApp_ON_OFF=0x00;
     DimmingAppControl(GenericApp_ON_OFF,0X01);
     GenericApp_applicationbuf=1;
         GenericApp_SendTheMessage();
    }
   if(P1_6)
   { DimmingAppControl(0x00,0X01);
     halMcuWaitMs(3000);
        if(P1_6)
        {  uint16 nv_data=0X0000;
            osal_nv_item_init( ZCD_NV_APP_R_E_KEY_P07,sizeof(nv_data), &nv_data );
            osal_nv_write( ZCD_NV_APP_R_E_KEY_P07, 0,sizeof(nv_data),&nv_data);  
           WDCTL = 0x00;WDCTL |= 0x09;  //0B  1.9MS  09  0.25秒
	}
   }
  } 
#endif
/*
#if(SENSOR_TYPE =='S')
  if((P1&0XE0)!=GenericApp_appIO)
  {
     GenericApp_appIO=(P1&0XE0);
   if(P1_6)
    {
      if(GenericApp_ON_OFF<0x71)
       GenericApp_ON_OFF+=0x05;
     else
       GenericApp_ON_OFF=0x75;
     DimmingAppControl(GenericApp_ON_OFF,0X01);
          GenericApp_KSH_H=1;
      if(GenericApp_KSH>2)
         { 
            uint16 nv_data=0X0000;
            osal_nv_item_init( ZCD_NV_APP_R_E_KEY_P07,sizeof(nv_data), &nv_data );
            osal_nv_write( ZCD_NV_APP_R_E_KEY_P07, 0,sizeof(nv_data),&nv_data);
            WDCTL = 0x00;
	    WDCTL |= 0x0B;
         }
    }
      if(P1_7)
    {  
       if(GenericApp_ON_OFF>0x07)
       GenericApp_ON_OFF-=0x05;
     else
       GenericApp_ON_OFF=0x00;
     DimmingAppControl(GenericApp_ON_OFF,0X01);
     if(GenericApp_KSH_H)
     {GenericApp_KSH86=0;
       GenericApp_KSH++;
      GenericApp_KSH_H=0;
     }
     }
  } 
#endif 
*/
  }
#if !defined (SENSOR_TYPE_Coord)
#if((SENSOR_TYPE ==0X66)||(SENSOR_TYPE ==0X73)||(SENSOR_TYPE ==0X76)||(SENSOR_TYPE =='G'))
if(GenericApp_ON_OFF>100)
{GenericApp_ON_OFF=0;
  ZigBee_C_R_E_Engineering_I();
}
else GenericApp_ON_OFF++;
#else
  ZigBee_C_R_E_Engineering_I();
#endif
#endif
}
