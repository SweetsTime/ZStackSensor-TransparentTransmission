/**************************************************************************************************
  Filename:       GenericApp.h
  Revised:        $Date: 2007-10-27 17:22:23 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15795 $

  Description:    This file contains the Generic Application definitions.
*/
/***********************************************************************
 * FT/ZIGBEE/传感器编译设置
 ***********************************************************************/
/*
ZDO_COORDINATOR=0             //0 主机路由程序  1 网络启动协调器程序
SENSOR_TYPE='K'               //传感器类型 例如 SENSOR_TYPE='K' 或 SENSOR_TYPE=0X03
CHANLIST_C_R_E=11             //信道  11-26
ZDAPP_CONFIG_PAN_ID=0xFFFF    // 0XFFFF   PAN_ID随机分配    不是0XFFFF时，PAN_ID设多少 是多少 
ZigBee_C_R_E_Engineering      //广播模式， 加x 点对点模式
ZigBee_C_R_E_IEEE             //IEEE长地址数据格式 加x 不带IEEE长地址数据格式
SENSOR_TYPE_R_E=0X01          //传感器短地址设置， 加x 短地址父节点分配
SENSOR_TYPE_POWER_ON         //不掉电寻找网络 ，  加x 寻找不到网络掉电
*/
#if(SENSOR_TYPE =='E')
#define NB_IOT_S10   //如果 打开此定义 及开启NBIOT检测

#endif

#if defined (NB_IOT_S10)
extern   unsigned char NBliucheng;
extern unsigned char NB_IMEI[16];//IMEI号
extern unsigned char NB_S;//内容长度
#endif
/***********************************************************************
 * FT/ZIGBEE/传感器编译设置
 ***********************************************************************/
/*
A//雨滴          A
J//可燃气体      J
L//光敏	        L
M//雨滴-结露     M
N//火焰          N
0X03//315M传感器  0X03 
0X04 //震动    0X04 
0X22 //二氧化碳  0x22
以上开关量检测

G//RFID          G
T//条形码        T****************************8
O//凌阳语音控制  O*************************88
0X20//噪声测量      0X20
0X23//指纹    0X23
0X02//直流电机调速  0X02 

P//红外学习遥控  P
0X06//智能小车    0X06****************************8
0X09//多功能控制板 0X09*****************************
0X0A//智能货架ISO15693  0X0A************************
0X0B//  125K低频 0x0B
串口控制

B//三轴加速度    B
C//光强          C
D//颜色          D
E//温湿度        E
F//气压海拔      F
I//遥控器	I
Q//电子称        Q****************************
U//18B20         U**************************8

0x01//电子称        0x01****************************

0X10 //红外测距  0X10
0x11 //氧气传感器 0X11
0X21 //光线   0X21
AD采集




K//R无线控制      K
0X12 //电磁锁     0x12
S//R调光模块      S****************************
H//R无线窗帘控制  H*****************************	
0X05// 雨棚控制  0x05
0X07// 超高频  0x07
0X08//  1443A 0x08


控制盒全都是 R 路由
接收控制模块
//ZCD_NV_APP_PANID_H   //PAN
**************************************************************************************************/

#ifndef GENERICAPP_H
#define GENERICAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h" 

/*********************************************************************
 * CONSTANTS 
 */

//#define CC2530_V30    //发射添加5字节 标志位版本
//#define CC2530_RF_433M    //数据串口传输给RF_433M模块发送
// These constants are only for example and should be changed to the
// device's needs
#define GENERICAPP_ENDPOINT           10

#define GENERICAPP_PROFID             0x0F04
#define GENERICAPP_DEVICEID           0x0001
#define GENERICAPP_DEVICE_VERSION     0
#define GENERICAPP_FLAGS              0

#define GENERICAPP_MAX_CLUSTERS       1
#define GENERICAPP_CLUSTERID          1

// Send Message Timeout
#define GENERICAPP_SEND_MSG_TIMEOUT   500     // Every 0.5 seconds

// Application Events (OSAL) - These are bit weighted definitions.
#define GENERICAPP_SEND_MSG_EVT       0x0001

 #if(SENSOR_TYPE ==0X0D) 
  #define TGRD15693   //如果 是泰格瑞德高频15693模块打开此定义
 //#define PN532   //如果 是PN532-NFC模块打开此定义
#endif 
#if(SENSOR_TYPE ==0X0E)  
 //#define STM2_4G   //如果 
 #define NRF2_4G   //如果 NRF
#endif
  

#if(SENSOR_TYPE =='F')  
//#define BMP085 //定义 
#define BMP180 //定义 
#endif
  
#if(SENSOR_TYPE =='Q')  
//#define SHINING100MIL //定义 第一版 电子秤 100mil
#define SHINING75MIL //定义 第二版 电子秤  75mil
#endif
  
#if(SENSOR_TYPE ==0X6A) 
  //#define ADCP //AD 采集
#define HX711P //数据采集HX711
                    
#endif 
  
#if(SENSOR_TYPE ==0X07)  
  extern unsigned char AB;
//#define AB //定义 主动上传。屏蔽被动上传   读写命令时 主动上次打开次定义
//#define UHF //定义 新力量
#define RLM100 //定义 RLM100
#endif
  
#if(SENSOR_TYPE ==0X72)
//#define SDS011 //定义 SDS011激光PM2.5传感器  //夏普PM2.5屏蔽此定义
#endif
#if(SENSOR_TYPE ==0X78)
extern  unsigned char data_buf[9];
extern  unsigned char data_RPY;
#endif
#if(SENSOR_TYPE ==0X13)
#define HUABANG  //定义华邦电表 IEEE754
//#define ammeter  //定义老式电表
#endif

#if(SENSOR_TYPE ==0X14) 
//#define PH_A   //上海诺博环保PH ORP检测仪    如果 是之前版本的PH计打开此定义
#define PH_ORP   //杭州美控自动化技术有限公司(MIK-PH160)  如果 是PH/ORP测试仪，打开此定义
//#define PH_E201C   //直接雷磁PH复合电极经信号处理采集 打开此定义
//以上宏定义 只能有一项打开
#endif   
/*********************************************************************
 * MACROS
 */
extern uint8 AF_OK;
extern uint8 AF_RF;
extern uint8 AF_RFn;

extern unsigned int GenericApp_applicationdata;
extern unsigned int  GenericApp_applicationbuf;
extern unsigned int GenericApp_uart_data;
extern unsigned char  GenericApp_CR;
extern uint8 openoff;
extern unsigned char  GenericApp_appIO;
extern unsigned int GenericApp_KSH;  
extern unsigned int GenericApp_KSH86;
extern uint8 Send_data[];
extern uint8 Send_datalend;
extern  unsigned int GenericApp_ON_OFF;
extern  unsigned char GenericApp_BX;
extern unsigned char OSAL_SET_CPU_INTO_SLEEP;

#if(ZDO_COORDINATOR==2)  //ZIGBEE  AT命令模式 透传模式
extern uint8 FT_AT;//0 命令模式//1 透传模式
#endif

#if(SENSOR_TYPE ==0X23)
extern unsigned char  SFG_R30X;
extern unsigned char  SFG_R30XA[3];
#endif 
#if(SENSOR_TYPE ==0X31)
#define RFID125K   //如果 是低频卡打开此定义
//#define RFID_NFC   //如果 是NFC高频卡打开此定义

extern  uint8 Uart_Rev_Buff_1[6];
extern  uint8 Uart_Rev_Buff_2[6];
extern uint8 car_stata;
extern uint8 car_op;
extern uint8 car_sum;
extern uint8  holzer;
#endif

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void GenericApp_Init( byte task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events );
extern void GenericApp_SendTheMessage(void);
extern void GenericApp_SendFT(void);
extern void GenericAppEndDeviceEB(void);
extern void GenericAppCoordEB(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GENERICAPP_H */
