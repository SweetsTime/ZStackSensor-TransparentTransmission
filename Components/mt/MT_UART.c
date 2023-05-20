/***************************************************************************************************
  Filename:       MT_UART.c
  Revised:        $Date: 2009-03-12 16:25:22 -0700 (Thu, 12 Mar 2009) $
  Revision:       $Revision: 19404 $

  Description:  This module handles anything dealing with the serial port.

  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

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

***************************************************************************************************/

/***************************************************************************************************
 * INCLUDES
 ***************************************************************************************************/
#include "ZComDef.h"
#include "OSAL.h"
#include "hal_uart.h"
#include "MT.h"
#include "MT_UART.h"
#include "OSAL_Memory.h"
#include "GenericApp.h"
#include "ZDApp.h"
#include "OSAL_Nv.h"
#include "delay.h"
#include "nwk_globals.h"
/***************************************************************************************************
 * MACROS
 ***************************************************************************************************/

/***************************************************************************************************
 * CONSTANTS
 ***************************************************************************************************/
/* State values for ZTool protocal */
#define SOP_STATE      0x00
#define CMD_STATE1     0x01
#define CMD_STATE2     0x02
#define LEN_STATE      0x03
#define DATA_STATE     0x04
#define FCS_STATE      0x05

/***************************************************************************************************
 *                                         GLOBAL VARIABLES
 ***************************************************************************************************/
/* Used to indentify the application ID for osal task */
byte App_TaskID;

/* ZTool protocal parameters */
uint8 state;
uint8 state0;
uint8  CMD_Token[2];

uint8  chnn[0X40];

uint8  LEN_Token;
uint8  FSC_Token;
mtOSALSerialData_t  *pMsg;
uint8  tempDataLen;
#if defined (ZAPP_P1) || defined (ZAPP_P2)
uint16  MT_UartMaxZAppBufLen;
bool    MT_UartZAppRxStatus;
#endif
/***************************************************************************************************
 *                                          LOCAL FUNCTIONS
 ***************************************************************************************************/

/***************************************************************************************************
 * @fn      MT_UartInit
 *
 * @brief   Initialize MT with UART support
 *
 * @param   None
 *
 * @return  None
***************************************************************************************************/
void MT_UartInit ()
{
  halUARTCfg_t uartConfig;
  /* Initialize APP ID */
  App_TaskID = 0;
  /* UART Configuration */
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = MT_UART_DEFAULT_BAUDRATE;
  uartConfig.flowControl          = MT_UART_DEFAULT_OVERFLOW;
  uartConfig.flowControlThreshold = MT_UART_DEFAULT_THRESHOLD;
  uartConfig.rx.maxBufSize        = MT_UART_DEFAULT_MAX_RX_BUFF;
  uartConfig.tx.maxBufSize        = MT_UART_DEFAULT_MAX_TX_BUFF;
  uartConfig.idleTimeout          = MT_UART_DEFAULT_IDLE_TIMEOUT;
  uartConfig.intEnable            = TRUE;
#if defined (ZTOOL_P1) || defined (ZTOOL_P2)
  uartConfig.callBackFunc         = MT_UartProcessZToolData;
#elif defined (ZAPP_P1) || defined (ZAPP_P2)
  uartConfig.callBackFunc         = MT_UartProcessZAppData;
#else
  uartConfig.callBackFunc         = NULL;
#endif

  /* Start UART */ 
#if defined (MT_UART_DEFAULT_PORT)
  HalUARTOpen (MT_UART_DEFAULT_PORT, &uartConfig);
#else
  /* Silence IAR compiler warning */  
  (void)uartConfig; 
#endif 
  /* Initialize for ZApp */
#if defined (ZAPP_P1) || defined (ZAPP_P2)
  /* Default max bytes that ZAPP can take */
  MT_UartMaxZAppBufLen  = 1;
  MT_UartZAppRxStatus   = MT_UART_ZAPP_RX_READY;
#endif
}
//串口1的初始化
#if(SENSOR_TYPE ==0X31)
void MT_Uart1Init(void)
{ 
  halUARTCfg_t uartConfig;
  
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = MT_UART1_DEFAULT_BAUDRATE;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 32; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = 32;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = 32;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout             = 6;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable                = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc          = MT_UartProcessZToolData;  //指定串口回调函数
  HalUARTOpen (HAL_UART_PORT_1, &uartConfig);
}
#endif
/***************************************************************************************************
 * @fn      MT_SerialRegisterTaskID
 *
 * @brief   This function registers the taskID of the application so it knows
 *          where to send the messages whent they come in.
 *
 * @param   void
 *
 * @return  void
 ***************************************************************************************************/
void MT_UartRegisterTaskID( byte taskID )
{
  App_TaskID = taskID;
}

/***************************************************************************************************
 * @fn      SPIMgr_CalcFCS
 *
 * @brief   Calculate the FCS of a message buffer by XOR'ing each byte.
 *          Remember to NOT include SOP and FCS fields, so start at the CMD field.
 *
 * @param   byte *msg_ptr - message pointer
 * @param   byte len - length (in bytes) of message
 *
 * @return  result byte
 ***************************************************************************************************/
byte MT_UartCalcFCS( uint8 *msg_ptr, uint8 len )
{
  byte x;
  byte xorResult;

  xorResult = 0;

  for ( x = 0; x < len; x++, msg_ptr++ )
    xorResult = xorResult ^ *msg_ptr;

  return ( xorResult );
}


/***************************************************************************************************
 * @fn      MT_UartProcessZToolData
 *
 * @brief   | SOP | Data Length  |   CMD   |   Data   |  FCS  |
 *          |  1  |     1        |    2    |  0-Len   |   1   |
 *
 *          Parses the data and determine either is SPI or just simply serial data
 *          then send the data to correct place (MT or APP)
 *
 * @param   port     - UART port
 *          event    - Event that causes the callback
 *
 *
 * @return  None
 ***************************************************************************************************/
void MT_UartProcessZToolData ( uint8 port, uint8 event )
{  
  uint8  ch;
  uint8  bytesInRxBuffer;
  (void)event;  // Intentionally unreferenced parameter
  while (Hal_UART_RxBufLen(port))
  {
   HalUARTRead (port, &ch, 1);
#if defined( SENSOR_TYPE_Coord )
   
#if(ZDO_COORDINATOR==2)  //ZIGBEE  AT命令模式 透传模式
   if(FT_AT==0) // 命令模式
   {
   switch (state) 
   { case 0X00:if (ch == 'F')state = 0X01;/*
                                          else{
                                         bytesInRxBuffer = Hal_UART_RxBufLen(port);
                                         if(bytesInRxBuffer<100)
                                         { Send_datalend=bytesInRxBuffer;
                                         HalUARTRead (port, Send_data, bytesInRxBuffer);
                                         HalUARTWrite(HAL_UART_PORT_0, Send_data,10);
                                         GenericApp_SendFT();
                                         }
                                  }*/
                                  break;
      case 0X01:if (ch == 'T'){state = 0X02;tempDataLen=0;}else state = 0X00; break;
      case 0X02:if (ch =='\r')state = 0X03;else{chnn[tempDataLen++]=ch;}break;
      case 0X03:if (ch =='\n')
                 {HalUARTCLER (port,bytesInRxBuffer);
                   if(tempDataLen==0){HalUARTWrite(HAL_UART_PORT_0, "\r\nOK\r\n",6);}
                 else{
                      if((osal_memcmp(&chnn[0], "+RST", 4))&&(tempDataLen==4)){HalUARTWrite(HAL_UART_PORT_0, "\r\nREADY\r\n",9);
                                                          WDCTL = 0x00;WDCTL |= 0x09;  //0B  1.9MS  09  0.25秒
                                                          }
                      if((osal_memcmp(&chnn[0], "+CSCAL?", 7))&&(tempDataLen==7)){uint16 nv_data;uint8 cscal[2];osal_nv_read(ZCD_NV_APP_CHANLIST,0,sizeof(nv_data),&nv_data);
                                                              FSC_Token=(nv_data&0x00ff);
                                                              cscal[0]=((FSC_Token/10)+'0'); //十进制
                                                              cscal[1]=((FSC_Token%10)+'0');//十进制
                                                              HalUARTWrite(HAL_UART_PORT_0, "\r\n+CSCAL:\"",10);
                                                              HalUARTWrite(HAL_UART_PORT_0, cscal,2);
                                                              HalUARTWrite(HAL_UART_PORT_0, "\"\r\n",3);
                                                              }
                      if((osal_memcmp(&chnn[0], "+CSCAL=\"", 8))&&(tempDataLen==11)){uint16 nwk_data;
                                                               nwk_data=0xaa00+(((chnn[8]-'0')*10)+(chnn[9]-'0'));//十进制
                                                             // nwk_data=0xaa00;nwk_data+=chnn[8]; 
                                                              if((nwk_data>0XAA0A)&&(nwk_data<0XAA1B)){
                                                              osal_nv_item_init( ZCD_NV_APP_CHANLIST,sizeof(nwk_data), &nwk_data ); 
                                                              osal_nv_write( ZCD_NV_APP_CHANLIST, 0, sizeof(nwk_data), &nwk_data );}
                                                              HalUARTWrite(HAL_UART_PORT_0, "\r\nOK\r\n",6);}
                      if((osal_memcmp(&chnn[0], "+CSPID?", 7))&&(tempDataLen==7)){uint16 nv_data;uint8 cscal[4];osal_nv_read(ZCD_NV_APP_PANID_H,0,sizeof(nv_data),&nv_data); 
                                                              FSC_Token=((nv_data>>12)&0X0F);if(FSC_Token>9)cscal[0]=FSC_Token+0X37;else cscal[0]=FSC_Token+0X30;
                                                              FSC_Token=((nv_data>> 8)&0X0F);if(FSC_Token>9)cscal[1]=FSC_Token+0X37;else cscal[1]=FSC_Token+0X30;
                                                              FSC_Token=((nv_data>> 4)&0X0F);if(FSC_Token>9)cscal[2]=FSC_Token+0X37;else cscal[2]=FSC_Token+0X30;
                                                              FSC_Token=((nv_data>> 0)&0X0F);if(FSC_Token>9)cscal[3]=FSC_Token+0X37;else cscal[3]=FSC_Token+0X30;
                                                              HalUARTWrite(HAL_UART_PORT_0, "\r\n+CSPID:",9);//十六进制  无双引号 大写
                                                              HalUARTWrite(HAL_UART_PORT_0, cscal,4);
                                                              HalUARTWrite(HAL_UART_PORT_0, "\r\n",2);}
                      if((osal_memcmp(&chnn[0], "+CSPID=", 7))&&(tempDataLen==11)){uint16 nwk_data;//十六进制  无双引号 大写
                                                                if(chnn[7]>'9')nwk_data=(chnn[7]-0X37)<<12;else nwk_data=(chnn[7]-0x30)<<12;//十六进制
                                                                if(chnn[8]>'9')nwk_data+=(chnn[8]-0X37)<<8;else nwk_data+=(chnn[8]-0x30)<<8;//十六进制
                                                                if(chnn[9]>'9')nwk_data+=(chnn[9]-0X37)<<4;else nwk_data+=(chnn[9]-0x30)<<4;//十六进制
                                                                if(chnn[10]>'9')nwk_data+=(chnn[10]-0X37);else nwk_data+=(chnn[10]-0x30);//十六进制
                                                                
                                                             // nwk_data=0xaa00;nwk_data+=chnn[8]; 
                                                                if((nwk_data!=0)&&(nwk_data!=0XFFFF))
                                                                {
                                                                osal_nv_item_init( ZCD_NV_APP_PANID_H,sizeof(nwk_data), &nwk_data ); 
                                                                osal_nv_write( ZCD_NV_APP_PANID_H, 0, sizeof(nwk_data), &nwk_data );
                                                                osal_nv_item_init( ZCD_NV_APP_PANID_L,sizeof(nwk_data), &nwk_data ); 
                                                                osal_nv_write( ZCD_NV_APP_PANID_L, 0, sizeof(nwk_data), &nwk_data );
                                                                HalUARTWrite(HAL_UART_PORT_0, "\r\nOK\r\n",6);
                                                                }
                                                                }
                      if((osal_memcmp(&chnn[0], "+CSMODE?",8))&&(tempDataLen==8)){uint16 nwk_data;osal_nv_read(ZCD_NV_APP_C_R_C,0,sizeof(nwk_data),&nwk_data);
                                                            if((nwk_data&0XF000)==0X5000) HalUARTWrite(HAL_UART_PORT_0, "\r\n+CSMODE:0\r\n",13); //路由器模式 
                                                            else HalUARTWrite(HAL_UART_PORT_0, "\r\n+CSMODE:1\r\n",13); //协调器模式
                                                            }
                      if((osal_memcmp(&chnn[0], "+CSMODE=0",9))&&(tempDataLen==9)){uint16 nwk_data;nwk_data=0X5000; //路由器模式 
                                                                osal_nv_item_init( ZCD_NV_APP_C_R_C,sizeof(nwk_data), &nwk_data ); 
                                                                osal_nv_write( ZCD_NV_APP_C_R_C, 0, sizeof(nwk_data), &nwk_data );
                                                              HalUARTWrite(HAL_UART_PORT_0, "\r\nOK\r\n",6);}
                       if((osal_memcmp(&chnn[0], "+CSMODE=1",9))&&(tempDataLen==9)){uint16 nwk_data;nwk_data=0X0000;//协调器模式
                                                                osal_nv_item_init( ZCD_NV_APP_C_R_C,sizeof(nwk_data), &nwk_data ); 
                                                                osal_nv_write( ZCD_NV_APP_C_R_C, 0, sizeof(nwk_data), &nwk_data );
                                                              HalUARTWrite(HAL_UART_PORT_0, "\r\nOK\r\n",6);}
                      if(osal_memcmp(&chnn[0], "+CZSEND", 7)){FT_AT=1;HalUARTWrite(HAL_UART_PORT_0, "\r\n>\r\n",5);}
                      if(osal_memcmp(&chnn[0], "+CZQSEND",8)){FT_AT=0;HalUARTWrite(HAL_UART_PORT_0, "\r\nOK\r\n",6);}
                     }
                 }state = 0X00;break;
      default: break;
    }  
   }
   else  //透传模式
   {
                                         bytesInRxBuffer = Hal_UART_RxBufLen(port);
                                         if(bytesInRxBuffer<128)
                                         { Send_datalend=bytesInRxBuffer+1;//加以取出的一字节长度
                                         Send_data[0]=ch;//存放已取出的数据
                                         HalUARTRead (port, &Send_data[1], bytesInRxBuffer);
                                        // HalUARTWrite(HAL_UART_PORT_0, Send_data,bytesInRxBuffer);
                                        if(osal_memcmp(&Send_data[0], "FT+CZQSEND\r\n",12))
                                            {FT_AT=0;HalUARTWrite(HAL_UART_PORT_0, "\r\nOK\r\n",6);
                                            }
                                          else GenericApp_SendFT();
                                         }

   }
#else
switch (state) 
    { case 0X00: if (ch == 0XFD)state = 0X01;if (ch == 0XCC)state = 0X04; break;
      case 0X01: LEN_Token = ch+0X04;tempDataLen = 0; if (LEN_Token){Send_data[1] = ch;state = 0X02;}else{state = 0X00;return;}break;
      case 0X02:Send_data[2 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[2 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
        {HalUARTRead (port, &Send_data[2 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== LEN_Token){ state = 0X00;Send_data[0] =0XFD;Send_datalend=Send_data[1];
        if(AF_RF==0)
        {
         GenericApp_SendTheMessage();
         if( AF_OK==0) 
         {
        AF_RF=1;//发送失败标志
         }
        }
        }break;  
      case 0X04: if(ch == 0XEE)state = 0X10;else if(ch == 0XBB)state = 0X20;else if(ch == 0XAA)state = 0X30;else state = 0X00; break;
      case 0X10: if(ch == 0XEE)state = 0X11; else state = 0X00;break;
      case 0X20: if(ch == 0XBB)state = 0X21; else state = 0X00;break;
      case 0X30: if(ch == 0XAA)state = 0X31;else if(ch == 0XBB){state = 0X41;LEN_Token=0X04;tempDataLen = 0;}else state = 0X00;break;  
      case 0X11: if(ch == 0XDD){ chnn[0]=0XCC,chnn[1]=0XEE,chnn[2]=0XEE,chnn[3]=0XDD,HalUARTWrite(HAL_UART_PORT_0, chnn,4);uint16 nv_data=0X0000;
            osal_nv_item_init( ZCD_NV_APP_R_E_KEY_P07,sizeof(nv_data), &nv_data );
            osal_nv_write( ZCD_NV_APP_R_E_KEY_P07, 0,sizeof(nv_data),&nv_data);WDCTL = 0x00;WDCTL |= 0x09;} state = 0X00;break;//发送主机短地址
      case 0X21: if(ch == 0XDD) {uint16 nv_data;osal_nv_read(ZCD_NV_APP_CHANLIST,0,sizeof(nv_data),&nv_data);
            chnn[0]=0XCC,chnn[1]=(nv_data&0x00ff),chnn[2]=0XBB,osal_nv_read(ZCD_NV_APP_PANID_H,0,sizeof(nv_data),&nv_data);   
            chnn[3]=(nv_data>>8),chnn[4]=nv_data,chnn[5]=0XDD;
            if (PAN_ID_Success!=2) chnn[3]=chnn[4]=0; nv_data=ZDAPP_CONFIG_PAN_ID;
            if(nv_data!=0XFFFF){chnn[3]=nv_data>>8;chnn[4]=nv_data;} HalUARTWrite(HAL_UART_PORT_0, chnn,6);}state = 0X00;break;//发送信道 PAN_ID
      case 0X31: if(ch == 0XDD){ chnn[0]=0XCC,chnn[1]=0XAA,chnn[2]=0XAA,chnn[3]=0XDD,HalUARTWrite(HAL_UART_PORT_0, chnn,4);uint16 nwk_data=0X0000;
            osal_nv_item_init( ZCD_NV_APP_C_R,sizeof(nwk_data), &nwk_data ); 
            osal_nv_write( ZCD_NV_APP_C_R, 0, sizeof(nwk_data), &nwk_data );WDCTL = 0x00;WDCTL |= 0x09;}state = 0X00;break;//重启 协调器C启动 
      case 0X41: chnn[3 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
               if (bytesInRxBuffer <= LEN_Token - tempDataLen)
                { HalUARTRead (port, &chnn[3 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
                else
                {HalUARTRead (port, &chnn[3 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
                if (tempDataLen== LEN_Token){if(chnn[6] == 0XDD){uint16 nwk_data; nwk_data=0xaa00;nwk_data+=chnn[3]; 
                if((nwk_data>0XAA0A)&&(nwk_data<0XAA1B)){
                osal_nv_item_init( ZCD_NV_APP_CHANLIST,sizeof(nwk_data), &nwk_data ); 
                 osal_nv_write( ZCD_NV_APP_CHANLIST, 0, sizeof(nwk_data), &nwk_data );}
                if(chnn[4]||chnn[5]){nwk_data=(uint16)chnn[4]<<8;nwk_data+=chnn[5];
                   osal_nv_item_init( ZCD_NV_APP_PANID_H,sizeof(nwk_data), &nwk_data ); 
            osal_nv_write( ZCD_NV_APP_PANID_H, 0, sizeof(nwk_data), &nwk_data );
                 osal_nv_item_init( ZCD_NV_APP_PANID_L,sizeof(nwk_data), &nwk_data ); 
            osal_nv_write( ZCD_NV_APP_PANID_L, 0, sizeof(nwk_data), &nwk_data );
                } chnn[0]=0XCC;chnn[1]=0XAA;chnn[2]=0XBB;chnn[3]=0XDD;
                 HalUARTWrite(HAL_UART_PORT_0, chnn,4); WDCTL = 0x00;WDCTL |= 0x09;  //0B  1.9MS  09  0.25秒
                  } state = 0X00;}break;  
     default: break;}  
#endif 

#else
#if((SENSOR_TYPE =='P')||(SENSOR_TYPE ==0X02))
#else


#if defined (NB_IOT_S10)

if(NBliucheng<7)
{
 switch (state) 
     {
      case 0X00:if (ch =='O')state = 0X01; break;
      case 0X01:if (ch =='K')state = 0X02;else{state = 0X00;return;}break;
      case 0X02:if (ch =='\r')state = 0X03;else{state = 0X00;return;}break;
      case 0X03:if (ch =='\n'){HalUARTCLER (port,bytesInRxBuffer);if(NBliucheng<7)NBliucheng++;}state = 0X00;break;
      default: break;
     }
 } 
if(NBliucheng==7)//读取IMEI号
{
 switch (state) 
     {
      case 0X00:if (ch =='+')state = 0X01; break;
      case 0X01:if (ch =='C')state = 0X02;else{state = 0X00;return;}break;
      case 0X02:if (ch =='G')state = 0X03;else{state = 0X00;return;}break;
      case 0X03:if (ch =='S')state = 0X04;else{state = 0X00;return;}break;
      case 0X04:if (ch =='N')state = 0X05;else{state = 0X00;return;}break;
      case 0X05:if (ch ==':'){tempDataLen = 0; state = 0X06;}else{state = 0X00;return;}break;
      case 0X06:if (ch =='\r')state = 0X09;else{NB_IMEI[tempDataLen++]=ch;}break;
      case 0X09:if (ch =='\n'){NB_IMEI[15]='\"';HalUARTCLER (port,bytesInRxBuffer);NBliucheng=15;}state = 0X00;break;
      default: break;
     }
 } 
if(NBliucheng==15)//入网成功判断
{
 switch (state) 
     {
      case 0X00:if (ch =='+')state = 0X01; break;
      case 0X01:if (ch =='C')state = 0X02;else{state = 0X00;return;}break;
      case 0X02:if (ch =='G')state = 0X03;else{state = 0X00;return;}break;
      case 0X03:if (ch =='A')state = 0X04;else{state = 0X00;return;}break;
      case 0X04:if (ch =='T')state = 0X05;else{state = 0X00;return;}break;
      case 0X05:if (ch =='T')state = 0X06;else{state = 0X00;return;}break;
      case 0X06:if (ch ==':')state = 0X07;else{state = 0X00;return;}break;
      case 0X07:if (ch =='1')state = 0X08;else{state = 0X00;return;}break;
      case 0X08:if (ch =='\r')state = 0X09;else{state = 0X00;return;}break;
      case 0X09:if (ch =='\n'){HalUARTCLER (port,bytesInRxBuffer);NBliucheng=16;}state = 0X00;break;
      default: break;
     }
 } 

if(NBliucheng==16) //创建Socket号
{
 switch (state) 
     {
      case 0X00:if ((ch >='0')&&(ch <('0'+10)))state = 0X01;
      case 0X01:if (ch =='\r')state = 0X02;else{state = 0X00;return;}break;
      case 0X02:if (ch =='\n')state = 0X03;else{state = 0X00;return;}break;
      case 0X03:if (ch =='\r')state = 0X04;else{state = 0X00;return;}break;
      case 0X04:if (ch =='\n')state = 0X05;else{state = 0X00;return;}break;
      case 0X05:if (ch =='O')state = 0X06;else{state = 0X00;return;}break;
      case 0X06:if (ch =='K')state = 0X07;else{state = 0X00;return;}break;
      case 0X07:if (ch =='\r')state = 0X08;else{state = 0X00;return;}break;
      case 0X08:if (ch =='\n'){P0_4=1;/*入网创建Socket成功*/HalUARTCLER (port,bytesInRxBuffer);NBliucheng=100;}state = 0X00;break;
      default: break;
     }
 } 
if((NBliucheng>22)&&(NBliucheng!=30)&&(NBliucheng!=40)) //有请求数据
{
 switch (state) 
     {
      case 0X00:if (ch =='+')state = 0X01; break;
      case 0X01:if (ch =='N')state = 0X02;else{state = 0X00;return;}break;
      case 0X02:if (ch =='S')state = 0X03;else{state = 0X00;return;}break;
      case 0X03:if (ch =='O')state = 0X04;else{state = 0X00;return;}break;
      case 0X04:if (ch =='N')state = 0X05;else{state = 0X00;return;}break;
      case 0X05:if (ch =='M')state = 0X06;else{state = 0X00;return;}break;
      case 0X06:if (ch =='I')state = 0X07;else{state = 0X00;return;}break;
      case 0X07:if (ch ==':')state = 0X08;else{state = 0X00;return;}break;
      case 0X08:if ((ch >='0')&&(ch <('0'+10)))state = 0X09;else{state = 0X00;return;}break;
      case 0X09:if (ch ==',')state = 0X0A;else{state = 0X00;return;}break;
      case 0X0A:if (ch ==ch){NB_S=(ch-0X30);state = 0X0B;}else{state = 0X00;return;}break;
      case 0X0B:if (ch =='\r')state = 0X0e;else {NB_S=(NB_S*10)+(ch-0X30);state = 0X0C;}break;
      case 0X0C:if (ch =='\r')state = 0X0e;else {NB_S=(NB_S*10)+(ch-0X30);state = 0X0D;}break;
      case 0X0D:if (ch =='\r')state = 0X0e;else{state = 0X00;return;}break;
      case 0X0e:if (ch =='\n'){ 
        HalUARTCLER (port,bytesInRxBuffer);if(NB_S>20){ NBliucheng=30;}else {NBliucheng=40;}}state = 0X00;break;
      default: break;
     }
 }
if(NBliucheng==30) //读取请求数据 //命令判断  LED 
{
  switch (state) 
     {
      case 0X00:if (ch =='1')state = 0X01;break;
      case 0X01:if (ch =='3')state = 0X02;else{state = 0X00;return;}break;
      case 0X02:if (ch =='9')state = 0X03;else{state = 0X00;return;}break;
      case 0X03:if (ch =='.')state = 0X04;else{state = 0X00;return;}break;
      case 0X04:if (ch =='1')state = 0X05;else{state = 0X00;return;}break;
      case 0X05:if (ch =='9')state = 0X06;else{state = 0X00;return;}break;
      case 0X06:if (ch =='6')state = 0X07;else{state = 0X00;return;}break;
      case 0X07:if (ch =='.')state = 0X08;else{state = 0X00;return;}break;
      case 0X08:if (ch =='2')state = 0X09;else{state = 0X00;return;}break;
      case 0X09:if (ch =='1')state = 0X0a;else{state = 0X00;return;}break;
      case 0X0a:if (ch =='8')state = 0X0b;else{state = 0X00;return;}break;
      case 0X0b:if (ch =='.')state = 0X0c;else{state = 0X00;return;}break;
      case 0X0c:if (ch =='1')state = 0X0d;else{state = 0X00;return;}break;
      case 0X0d:if (ch =='5')state = 0X0e;else{state = 0X00;return;}break;
      case 0X0e:if (ch =='6')state = 0X0f;else{state = 0X00;return;}break;
      case 0X0f:if (ch ==','){state = 0X10;}else{state = 0X00;return;}break;
      case 0X10:if (ch ==ch)state = 0X11;else{state = 0X00;return;}break;
      case 0X11:if (ch ==ch)state = 0X12;else{state = 0X00;return;}break;
      case 0X12:if (ch ==ch)state = 0X13;else{state = 0X00;return;}break;
      case 0X13:if (ch ==ch)state = 0X14;else{state = 0X00;return;}break;
      case 0X14:if (ch ==',')state = 0X15;else{state = 0X00;return;}break;
      case 0X15:if (ch ==ch){NB_S=ch-0x30;state = 0X16;}else{state = 0X00;return;}break;
      case 0X16:if (ch ==','){tempDataLen = 0;state = 0X19;if(NB_S<20){HalUARTCLER (port,bytesInRxBuffer);state = 0X0;}}else {NB_S=(NB_S*10)+(ch-0x30);state = 0X17;}break;
      case 0X17:if (ch ==','){tempDataLen = 0;state = 0X19;if(NB_S<20){HalUARTCLER (port,bytesInRxBuffer);state = 0X0;}}else {NB_S=(NB_S*10)+(ch-0x30);state = 0X18;}break;
      case 0X18:if (ch ==','){tempDataLen = 0;state = 0X19;if(NB_S<20){HalUARTCLER (port,bytesInRxBuffer);state = 0X0;}}else{HalUARTCLER (port,bytesInRxBuffer);state = 0X0;}break;
      case 0X19:if (ch ==','){state = 0X25;}else {if(ch>'9')FSC_Token=(ch-0x37);else FSC_Token=(ch-0x30);state = 0X20;}break;
      case 0X20:if (ch ==','){state = 0X25; }else {chnn[tempDataLen++]=FSC_Token*16+((ch>'9')?( ch-0x37):(ch-0x30));state = 0X19;}break;
      case 0X25:   //   HalUARTWrite(HAL_UART_PORT_0, chnn,NB_S);//"{"Nb-down":{"LED":"00"}}"
                 if(osal_memcmp(&chnn[2], "Nb-down", 7))
                  {
                    if(osal_memcmp(&chnn[13], "LED", 3))
                     {if(osal_memcmp(&chnn[19], "00", 2))P0_4=0;
                      if(osal_memcmp(&chnn[19], "01", 2))P0_4=1;
                      if(osal_memcmp(&chnn[19], "03", 2))P0_4=1;
                     }
                  }
                    HalUARTCLER (port,bytesInRxBuffer);NBliucheng=40;state = 0X00; break;
     default: break;
     }
 }
if(NBliucheng==40) //
{
  switch (state) 
     {
      case 0X00:if (ch =='O')state = 0X01; break;
      case 0X01:if (ch =='K')state = 0X02;else{state = 0X00;return;}break;
      case 0X02:if (ch =='\r')state = 0X03;else{state = 0X00;return;}break;
      case 0X03:if (ch =='\n'){ 
      HalUARTCLER (port,bytesInRxBuffer);NBliucheng=100;}state = 0X00;break;
      default: break;
     }
 } 
#endif      
#if(SENSOR_TYPE ==0X25) 
switch (state) 
    { case 0X00: if (ch == 0XF7)state = 0X01;if (ch == 0XF0)state = 0X05; break;
      case 0X01:LEN_Token = ch;tempDataLen = 0; if (LEN_Token){Send_data[9] = LEN_Token;state = 0X02;}else{state = 0X00;return;}break;
      case 0X02:Send_data[10 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
           if (bytesInRxBuffer <= LEN_Token - tempDataLen)
           { HalUARTRead (port, &Send_data[10 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
           else
           {HalUARTRead (port, &Send_data[10 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
           if (tempDataLen== LEN_Token){ state = 0X00;Send_data[8] =0XF7;
           Send_datalend=LEN_Token+4;GenericApp_SendTheMessage();}break;   

     case 0X05:LEN_Token = ch;tempDataLen = 0; if (LEN_Token){Send_data[9] = LEN_Token;state = 0X06;}else{state = 0X00;return;}break;
     case 0X06:Send_data[10 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
           if (bytesInRxBuffer <= LEN_Token - tempDataLen)
           { HalUARTRead (port, &Send_data[10 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
           else
           {HalUARTRead (port, &Send_data[10 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
           if (tempDataLen== LEN_Token){ state = 0X00;Send_data[8] =0XF1;
           
           if(Send_data[10]==0XBB){uint16 nv_data;osal_nv_read(ZCD_NV_APP_CHANLIST,0,sizeof(nv_data),&nv_data);
            Send_data[9] =0X05;Send_data[10]=(nv_data&0x00ff),Send_data[11]=0XBB,osal_nv_read(ZCD_NV_APP_PANID_H,0,sizeof(nv_data),&nv_data);
            Send_data[12]=(nv_data>>8),Send_data[13]=nv_data,Send_data[14]=0XFE;
            if (PAN_ID_Success!=2) Send_data[12]=Send_data[13]=0;HalUARTWrite(HAL_UART_PORT_0,&Send_data[8],7);}
            
           if(Send_data[10]==0XEE){Send_data[9] =0X02;Send_data[11]=0XFE; HalUARTWrite(HAL_UART_PORT_0,&Send_data[8],4);
           uint16 nv_data=0X0000;osal_nv_item_init( ZCD_NV_APP_R_E_KEY_P07,sizeof(nv_data), &nv_data );
            osal_nv_write( ZCD_NV_APP_R_E_KEY_P07, 0,sizeof(nv_data),&nv_data);WDCTL = 0x00;WDCTL |= 0x09;} 
             
           }break;   
       default: break;
     } 
#endif    
#if(SENSOR_TYPE ==0X7C) //F7  03  A1  51 00 
switch (state) 
    { case 0X00: if (ch == 0XF7)state = 0X01;break;
      case 0X01:if(LEN_Token)LEN_Token = ch; Send_data[10]=ch; tempDataLen = 0; if (LEN_Token<10){state = 0X02;}else{state = 0X00;return;}break;
      case 0X02:Send_data[11 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
           if (bytesInRxBuffer <= LEN_Token - tempDataLen)
           { HalUARTRead (port, &Send_data[11 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
           else
           {HalUARTRead (port, &Send_data[11 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
           if (tempDataLen== LEN_Token){ state = 0X00; Send_data[9]=0XF7; 
           Send_datalend=5+ Send_data[10]; GenericApp_SendTheMessage();}break; 
       default: break;
     } 
#endif 
#if(SENSOR_TYPE ==0X7B) //F7  03  00  50 67 
switch (state) 
    { case 0X00: if (ch == 0XF7)state = 0X01;break;
      case 0X01:LEN_Token = ch;tempDataLen = 0; if (LEN_Token){state = 0X02;}else{state = 0X00;return;}break;
      case 0X02:Send_data[9 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
           if (bytesInRxBuffer <= LEN_Token - tempDataLen)
           { HalUARTRead (port, &Send_data[9 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
           else
           {HalUARTRead (port, &Send_data[9 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
           if (tempDataLen== LEN_Token){ state = 0X00; if(Send_data[7]<10) Send_data[7]++;else Send_data[7]=0;
           Send_datalend=6; if(Send_data[7]>9) GenericApp_SendTheMessage();}break; 
       default: break;
     } 
#endif      
#if(SENSOR_TYPE ==0X71) 
switch (state) 
{     case 0X00: if((ch == 0XF1)||(ch == 0XF7)){state = 0X01;Send_data[8] =ch;} break;
      case 0X01:LEN_Token = ch;tempDataLen = 0; if (LEN_Token){Send_data[9] = LEN_Token;state = 0X02;}else{state = 0X00;return;}break;
      case 0X02:Send_data[10 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[10 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
        {HalUARTRead (port, &Send_data[10 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== LEN_Token){ state = 0X00;
       Send_datalend=LEN_Token+4;GenericApp_SendTheMessage();}break;   
       default: break;} 
#endif      

#if(SENSOR_TYPE ==0X73) 
switch (state) 
{ case 0X00: if((ch == 0X05)||(ch == 0X03)){LEN_Token = ch;tempDataLen = 0; state = 0X01;} break;
  case 0X01:if((ch == 0XA8)||(ch == 0XAC)){Send_data[8 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[8 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
        {HalUARTRead (port, &Send_data[8 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== LEN_Token){ state = 0X00;Send_data[7] =0X00;//Send_data[8] =0XA8;
        Send_datalend=LEN_Token+4;GenericApp_applicationbuf=3;GenericApp_SendTheMessage();}}else state = 0X00; break;   
       default: break;} 
#endif 
#if(SENSOR_TYPE ==0X78)
switch (state) 
    { case 0X00: if (ch == 0X5A)state = 0X01; break;
    case 0X01: if (ch == 0X5A)state = 0X02;else {state = 0X00;HalUARTCLER (port,bytesInRxBuffer);} break;
      case 0X02: if (1){data_RPY=ch;state = 0X03;} else {state = 0X00;HalUARTCLER (port,bytesInRxBuffer);} break;
      case 0X03: if ((ch == 0X06)||(ch == 0X08)){LEN_Token = ch+1;tempDataLen = 0;state = 0X04;}else{state = 0X00;HalUARTCLER (port,bytesInRxBuffer);return;}break;
      case 0X04: data_buf[0 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &data_buf[0 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
        {HalUARTRead (port, &data_buf[0 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== LEN_Token){  state = 0X00;Send_data[7] =0XAA;
       Send_datalend=34;//HalUARTCLER (port,bytesInRxBuffer);
      switch(data_RPY)//判断输出数据类型
		{
			case 0x15:{//加数度数据输出
                         data_RPY=(data_buf[0]>Send_data[8])? (data_buf[0]-Send_data[8]):(Send_data[8]-data_buf[0]);
		if((data_RPY>0X0A)&&(data_RPY<0XE0)) { GenericApp_applicationbuf=60001;}
               // else
                 for(char n=0;n<6;n++)
                  { Send_data[8+n]= data_buf[0+n];}
				        // Acc[0]=(data_buf[0]<<8)|data_buf[1];
					//Acc[1]=(data_buf[2]<<8)|data_buf[3];
					//Acc[2]=(data_buf[4]<<8)|data_buf[5];
			}
			break;
			case 0x25:{//陀螺仪数据输出
				  for(char n=0;n<6;n++)
                          {Send_data[14+n]= data_buf[0+n];}
                                      // Gyr[0]=(data_buf[0]<<8)|data_buf[1];
					//Gyr[1]=(data_buf[2]<<8)|data_buf[3];
					//Gyr[2]=(data_buf[4]<<8)|data_buf[5];
					//send_out(Gyr,3,0x25);
			}
				break;
			case 0x35:{//磁场数据输出
                                     for(char n=0;n<6;n++)
                             {Send_data[20+n]= data_buf[0+n];}
				        //Mag[0]=(data_buf[0]<<8)|data_buf[1];
					//Mag[1]=(data_buf[2]<<8)|data_buf[3];
					//Mag[2]=(data_buf[4]<<8)|data_buf[5];
					//send_out(Mag,3,0x35);
			}
				break;
			case 0x45:{//欧拉角数据输出
                           for(char n=0;n<6;n++)
                             {Send_data[26+n]= data_buf[0+n];}
				          //ROLL=(data_buf[0]<<8)|data_buf[1];
					//PITCH=(data_buf[2]<<8)|data_buf[3];
					//YAW=(data_buf[4]<<8)|data_buf[5];
					//rpy[0]=ROLL;
				 // rpy[1]=PITCH;
				//  rpy[2]=YAW;
					//send_out(rpy,3,0x45);
			}
				break;
			case 0x65:{//四元数数据输出
                           for(char n=0;n<8;n++)
                             {Send_data[32+n]= data_buf[0+n];}
				//  Q[0]=(data_buf[0]<<8)|data_buf[1];
					//Q[1]=(data_buf[2]<<8)|data_buf[3];
					//Q[2]=(data_buf[4]<<8)|data_buf[5];
					//Q[3]=(data_buf[6]<<8)|data_buf[7];
					//send_out(Q,4,0x65);
			}//GenericApp_SendTheMessage();
				break;
			default:break;

		}
        }break;   
       default: break;} 


#endif   
#if(SENSOR_TYPE ==0X85) //>>>>>>>>>>>>>>>>>>>>>>
    switch (state) 
    { case 0X00: if (ch == 0XA5)state = 0X01; else HalUARTCLER (port,bytesInRxBuffer);break;
      case 0X01:if (ch == 0X15)state = 0X02; else state = 0X00; break;
      case 0X02:if (ch == 0XBA) GenericApp_applicationbuf=6001;state = 0X00;HalUARTCLER (port,bytesInRxBuffer); break;
      default: break;} 

#endif



#if(SENSOR_TYPE ==0X72) 
#if defined( SDS011 )
 switch (state) 
    { case 0X00: if (ch == 0XAA)state = 0X01; break;
    case 0X01:if (ch == 0XC0){LEN_Token = 0X08;tempDataLen = 0; state = 0X02;}else{state = 0X00;return;}break;
    case 0X02:Send_data[9 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[9 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
        {HalUARTRead (port, &Send_data[9 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== LEN_Token){ state = 0X00;Send_data[7] =0XA0;Send_data[8] =0X55;
       Send_datalend=LEN_Token;
      if(GenericApp_ON_OFF>10){ GenericApp_ON_OFF=0; GenericApp_SendTheMessage();}
       GenericApp_ON_OFF++;       
        }break;   
    default: break;}    
#else
  chnn[GenericApp_uart_data]=ch;
  if(GenericApp_uart_data<=0X18)GenericApp_uart_data++;
if((chnn[GenericApp_uart_data-7]==0xAA)&&(chnn[GenericApp_uart_data-1]==0xFF))
   {  
  if(GenericApp_ON_OFF>10)
 {  
   GenericApp_ON_OFF=0;
 unsigned int pm25,abc;
 pm25=chnn[1]*256+chnn[2];
 abc=((pm25*5000)/1024);//  电压扩大1000倍  
  Send_data[8]=abc/10;
 pm25=(abc*79);//修改 79数值调节
 if(pm25<1000)pm25=3300;
Send_data[9]=pm25/100;
Send_data[10]=pm25%100;

 }
   GenericApp_ON_OFF++;
  GenericApp_uart_data=0;
  for(char y=0;y<7;y++)
      chnn[y]=0X00;
  }
#endif 
#endif   
#if(SENSOR_TYPE ==0X86) 
switch (state) 
{     case 0X00: if(ch){state = 0X01;LEN_Token = ch;} break;
      case 0X01:if(ch == 0XF7){tempDataLen = 0; Send_data[8] = LEN_Token;state = 0X02;}else{state = 0X00;return;}break;
      case 0X02:Send_data[10 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[10 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
        {HalUARTRead (port, &Send_data[10 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== (LEN_Token-1)){ state = 0X00;Send_data[9] =0XF7;
       Send_datalend=LEN_Token+3;GenericApp_SendTheMessage();}break;   
       default: break;} 
#endif 
#if(SENSOR_TYPE ==0X87) 
      bytesInRxBuffer = Hal_UART_RxBufLen(port);
      if(bytesInRxBuffer<128)
      { Send_datalend=bytesInRxBuffer+1;//加以取出的一字节长度
        Send_data[10]=ch;//存放已取出的数据
         if(Send_data[10]=='{')
        {HalUARTRead (port, &Send_data[11], bytesInRxBuffer);
        if(Send_data[10+Send_datalend-1]=='}')
        {Send_data[9]=Send_datalend;
        Send_datalend+=5;
          GenericApp_SendTheMessage();
         }
        }
         else //清空数据
         {HalUARTCLER (port,bytesInRxBuffer);
         }
        
     }
      else
        {HalUARTCLER (port,bytesInRxBuffer);
         }
#endif 
#if(SENSOR_TYPE ==0X26) 
switch (state) 
    { case 0X00: if (ch == 0XF7)state = 0X01; break;
      case 0X01:LEN_Token = ch;tempDataLen = 0; if (LEN_Token){Send_data[9] = LEN_Token;state = 0X02;}else{state = 0X00;return;}break;
      case 0X02:Send_data[10 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[10 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
        {HalUARTRead (port, &Send_data[10 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== LEN_Token){ state = 0X00;Send_data[8] =0XF7;
       Send_datalend=LEN_Token+4;GenericApp_SendTheMessage();}break;   
       default: break;} 
#endif      
#if(SENSOR_TYPE ==0X27) 
Send_data[7]=ch; if((ch==0x41)||(ch==0x45)) GenericApp_SendTheMessage(); 
#endif   
#if(SENSOR_TYPE =='O')
    switch (state) 
    { case 0X00: if (ch == 0XCE)state = 0X01; break;
      case 0X01:LEN_Token = ch;tempDataLen = 0; if (LEN_Token){Send_data[10] = LEN_Token;state = 0X02;}else{state = 0X00;return;}break;
      case 0X02:Send_data[11 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[11 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
        {HalUARTRead (port, &Send_data[11 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== LEN_Token){ state = 0X00;Send_data[9] =0XCE;
       Send_datalend=LEN_Token+6;GenericApp_SendTheMessage();}break;   
       default: break;} 
#endif
#if(SENSOR_TYPE =='T')
   Send_data[GenericApp_applicationdata+7]=ch;
        if(ch==0x0d)
        { Send_datalend=3+GenericApp_applicationdata;
       // HalUARTWrite(HAL_UART_PORT_0, Send_data,14);
        GenericApp_SendTheMessage();
        GenericApp_applicationdata=0;
        }
         else GenericApp_applicationdata+=1;
#endif
#if(SENSOR_TYPE ==0X20)
       
        if(ch>=0xA0)
        GenericApp_appIO=1; 
        if(GenericApp_appIO)
        { HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
        Send_data[GenericApp_applicationdata+7]=ch;
       if(GenericApp_applicationdata>=6)  
        { 
        Send_datalend=0x05; 
        // HalUARTWrite(HAL_UART_PORT_0, Send_data,14);
         GenericApp_appIO=0;
         GenericApp_applicationdata=0;
         GenericApp_ON_OFF++;
        }
        else GenericApp_applicationdata+=1;
        }
       if(GenericApp_ON_OFF>10)
       { GenericApp_SendTheMessage();
          GenericApp_ON_OFF=0;
       }
#endif
#if(SENSOR_TYPE ==0X31)
   if(port == HAL_UART_PORT_1) //低频模块独到6位卡号自动通过串口发送
   {
#if defined(RFID_NFC)
     if(PAN_ID_Success==2)//组网成功
   {
      switch (state) 
      {case 0X00: if (ch == 0X55)state = 0X01; break;
      case 0X01: if (ch == 0XAA){state = 0X02;Uart_Rev_Buff_2[0]=ch;tempDataLen = 0; }else state = 0X00;break;
      case 0X02:Uart_Rev_Buff_2[1 + tempDataLen++] = ch; 
              if(tempDataLen== 0X05)
             { state = 0X00;Send_data[7]=0X02;
        uint8 Car_Stop_Buff[3]={0xAA,0x2B,0xBB};
        if(!osal_memcmp( Uart_Rev_Buff_1, Uart_Rev_Buff_2,6))
        { if(!car_stata) 
          {
           if(holzer==1) //==1时 第一次检测到定位 读卡时不停止
           {holzer=2;}
           else 
           {HalUARTWrite(HAL_UART_PORT_0,Car_Stop_Buff,3);
           holzer=0;}
          }
          osal_memcpy( Uart_Rev_Buff_1, Uart_Rev_Buff_2,6);

         GenericApp_applicationbuf=2;
         GenericApp_SendTheMessage();
           if( AF_OK==0) 
            {AF_RF=1;
            }
        }
             }break;
      default: break;
   } 
   }
     else
    { HalUARTCLER (port,bytesInRxBuffer); }  //清空缓存
#endif                                                 
#if defined(RFID125K)
   
   if(PAN_ID_Success==2)//组网成功
   {
       Uart_Rev_Buff_2[car_sum]=ch;
       if(car_sum>=5)   
        {Send_data[7]=0X02;
        uint8 Car_Stop_Buff[3]={0xAA,0x2B,0xBB};
        if(!osal_memcmp( Uart_Rev_Buff_1, Uart_Rev_Buff_2,6))
        { if(!car_stata) 
          HalUARTWrite(HAL_UART_PORT_0,Car_Stop_Buff,3);
          osal_memcpy( Uart_Rev_Buff_1, Uart_Rev_Buff_2,6);
         GenericApp_applicationbuf=2;
         GenericApp_SendTheMessage();
           if( AF_OK==0) 
            {AF_RF=1;
            }
        
        }
         car_sum=0;
        }
        else car_sum+=1;
   }else
    { HalUARTCLER (port,bytesInRxBuffer); }  //清空缓存
#endif  
}
   else if(port == HAL_UART_PORT_0)
   {
      
      switch (state0) 
      {case 0X00: if (ch == 0XCC)state0 = 0X01; break;
      case 0X01: state0 = 0X02;Send_data[11]=ch;break;
      case 0X02:Send_data[12] = ch;state0 = 0X00; 
        GenericApp_applicationbuf=4;
        Send_datalend=0x08; 
       Send_data[7]=0X01;Send_data[8]=0XAA;Send_data[9]=0XBB;Send_data[10]=0XCC;Send_data[13]=0XDD;
        GenericApp_SendTheMessage();
      break;
      default: break;
      }
   }
#endif

#if(SENSOR_TYPE ==0XA3)
      
        chnn[GenericApp_applicationdata]=ch;
        if(GenericApp_applicationdata<=6)GenericApp_applicationdata++;
        else GenericApp_applicationdata=0;
     if((chnn[GenericApp_applicationdata-1]==0xDD)&&(chnn[GenericApp_applicationdata-6]==0xCC))
     { 
      Send_data[7]=chnn[1];Send_data[8]=chnn[2];Send_data[9]=chnn[3];Send_data[10]=chnn[4];

       GenericApp_applicationdata=0;
       GenericApp_applicationbuf=3;
       GenericApp_SendTheMessage();
          if( AF_OK==0) 
            {AF_RF=1;
            }
     }
#endif
#if(SENSOR_TYPE =='G')
     Send_data[GenericApp_applicationdata+7]=ch;
       if(GenericApp_applicationdata>=5)  
        { 
       Send_datalend=0x08; 
        // HalUARTWrite(HAL_UART_PORT_0, Send_data,14);
         GenericApp_SendTheMessage();
         GenericApp_applicationdata=0;
        }
        else GenericApp_applicationdata+=1;
#endif
#if(SENSOR_TYPE ==0X79)
     Send_data[GenericApp_applicationdata+9]=ch;
       if(GenericApp_applicationdata>=5)  
        { 
            Send_data[7]=0X07;
            Send_data[8]=0XD3;
            Send_datalend=0x09;
        // HalUARTWrite(HAL_UART_PORT_0, Send_data,14);
         GenericApp_SendTheMessage();
         GenericApp_applicationdata=0;
        }
        else GenericApp_applicationdata+=1;
#endif
#if(SENSOR_TYPE ==0X07)
#if defined(UHF)
if(AB==0X0A)
{
switch (state) 
     {case 0X00: if (ch == 0XE5)state = 0X01; else {bytesInRxBuffer = Hal_UART_RxBufLen(port);HalUARTCLER (port,bytesInRxBuffer);}break;
      case 0X01: if (ch == 0X00)state = 0X02;else state = 0X00; break;
      case 0X02: tempDataLen=0;LEN_Token=ch-4;state = 0X03;break;
      case 0X03: state = 0X04;break;
      case 0X04: state = 0X05;break;
      case 0X05: state = 0X06;break;
      case 0X06: state = 0X07;break;
     case 0X07:if(ch==0xE2){Send_data[8 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[8 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
       {HalUARTRead (port, &Send_data[8 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== LEN_Token){state = 0X00;
        Send_datalend=LEN_Token+4;GenericApp_SendTheMessage();}}else state = 0X00; break;   
       default: break;} 
  
/* chnn[GenericApp_applicationdata]=ch;
        if(GenericApp_applicationdata<=0X1F)GenericApp_applicationdata++;
     if((chnn[GenericApp_applicationdata-13]==0xE5)&&(chnn[GenericApp_applicationdata-11]==0x0B))
     {
       for(char y=0;y<0X0A;y++)
       Send_data[y+7]=chnn[y+4];
      GenericApp_applicationdata=0;
      Send_datalend=0X0B;
      GenericApp_SendTheMessage();
      for(char y=0;y<12;y++)
      chnn[y]=0X00;
     }
       */
}
else
{
switch (state) 
{     case 0X00: if ((ch == 0XE5)||(ch == 0XE9)){state = 0X01;Send_data[9]=ch;}else {bytesInRxBuffer = Hal_UART_RxBufLen(port);HalUARTCLER (port,bytesInRxBuffer);}break;
      case 0X01: if (ch == 0X00)state = 0X02;else state = 0X00; break;
      case 0X02: tempDataLen=0;LEN_Token=ch;Send_data[11]=ch; state = 0X03;break;
      case 0X03:Send_data[12 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[12 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
       {HalUARTRead (port, &Send_data[12 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== LEN_Token){state = 0X00;Send_data[10]=0; Send_data[8]=Send_data[11]+0X03; 
       Send_datalend=LEN_Token+6;GenericApp_SendTheMessage();} break;   
       default: break;} 
}
#endif
#if defined(RLM100)
     switch (state) 
     {case 0X00: if (ch == 0XAA)state = 0X01;else {bytesInRxBuffer = Hal_UART_RxBufLen(port);HalUARTCLER (port,bytesInRxBuffer);} break;
      case 0X01: if (ch > 0X10)state = 0X02;else state = 0X00; break;
      case 0X02: if (ch == 0X20)state = 0X03;else state = 0X00; break;
      case 0X03: if (ch == 0X00){tempDataLen=0;LEN_Token=8;state = 0X04;}else state = 0X00; break;
      case 0X04:Send_data[8 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[8 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
       {HalUARTRead (port, &Send_data[8 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== LEN_Token){state = 0X00;
       Send_datalend=LEN_Token+2;GenericApp_SendTheMessage();} break;   
       default: break;} 
#endif
#endif  
#if(SENSOR_TYPE ==0X0E)
    // 02 03 04 05 00 00 00 65 41 25 01 00 00 02 DC
#if defined(STM2_4G) 
     switch (state) 
     { case 0X00: if (ch == 0X02)state = 0X01; else {bytesInRxBuffer = Hal_UART_RxBufLen(port);HalUARTCLER (port,bytesInRxBuffer);}break;
      case 0X01: if (ch == 0X03)state = 0X02;else state = 0X00; break;
      case 0X02: if (ch == 0X04)state = 0X03;else state = 0X00; break;
      case 0X03: if (ch == 0X05)state = 0X04;else state = 0X00; break;
      case 0X04: if (ch == 0X00){tempDataLen=0;LEN_Token=10;state = 0X05;}else state = 0X00; break;
      case 0X05:Send_data[13 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[13 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
       {HalUARTRead (port, &Send_data[13 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== LEN_Token){state = 0X00;Send_data[7]=0X01;Send_data[8] =0X02;Send_data[9] =0X03;Send_data[10] =0X04;Send_data[11] =0X05;Send_data[12] =0X00;
       Send_datalend=LEN_Token+7;GenericApp_SendTheMessage();} break;   
       default: break;} 
#endif
//55 06 12 13 0F FF 0C BB
#if defined(NRF2_4G) 
     switch (state) 
     { case 0X00: if (ch == 0X55)state = 0X01; else {bytesInRxBuffer = Hal_UART_RxBufLen(port);HalUARTCLER (port,bytesInRxBuffer);}break;
      case 0X01: if (ch == 0X06){tempDataLen=0;LEN_Token=6;state = 0X02;}else state = 0X00; break;
      case 0X02:Send_data[10 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[10 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
       {HalUARTRead (port, &Send_data[10 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== LEN_Token){state = 0X00;Send_data[7]=0X02;Send_data[8] =0X55;Send_data[9] =0X06;
       Send_datalend=LEN_Token+4;GenericApp_SendTheMessage();} break;   
       default: break;} 
#endif
#endif
#if(SENSOR_TYPE ==0X08)
     if(ch=='[')GenericApp_applicationdata=0;
       chnn[GenericApp_applicationdata]=ch;
       if(GenericApp_applicationdata<=0X1F)GenericApp_applicationdata++;
     if(ch==']')
     {if(GenericApp_applicationdata>0X0A)
     {if(GenericApp_applicationdata<=0X0C)
      {   for(char y=2;y<0X0A;y+=2)
        Send_data[y+6]=chnn[0X09-y];
        for(char y=1;y<0X0A;y+=2)
         Send_data[y+8]=chnn[0X09-y];
      GenericApp_applicationdata=0;
     Send_datalend=0X0B;
      GenericApp_SendTheMessage();
      for(char y=0;y<0X10;y++)
      chnn[y]=0X00;
     }
       else
       {
      if(GenericApp_applicationdata>0x0D)
      {
       for(char y=2;y<0X12;y+=2)
        Send_data[y+6]=chnn[0X11-y];
        for(char y=1;y<0X12;y+=2)
         Send_data[y+8]=chnn[0X11-y];
      GenericApp_applicationdata=0;
      Send_datalend=0X13;
      GenericApp_SendTheMessage();
      for(char y=0;y<0X10;y++)
      chnn[y]=0X00;
      }
       }
     }
      GenericApp_applicationdata=0;
     }
#endif
#if(SENSOR_TYPE ==0X0D)
#if defined(TGRD15693) 
     if(ch==':')GenericApp_applicationdata=0;
       chnn[GenericApp_applicationdata]=ch;
       if(GenericApp_applicationdata<=0X25)GenericApp_applicationdata++;
        if((chnn[GenericApp_applicationdata-1]==0X0A)&&(chnn[GenericApp_applicationdata-2]==0X0D))
     {if(GenericApp_applicationdata>0X15)
      {
       for(char y=2;y<0X12;y+=2)
        Send_data[y+6]=chnn[(GenericApp_applicationdata-4)-y];
        for(char y=1;y<0X12;y+=2)
         Send_data[y+8]=chnn[(GenericApp_applicationdata-4)-y];
      GenericApp_applicationdata=0;
      Send_datalend=0X13;
      GenericApp_SendTheMessage();
      for(char y=0;y<0X1F;y++)
      chnn[y]=0X00;
      }
      GenericApp_applicationdata=0;
     }
#endif
#if defined(PN532) 
      switch (state) 
     { case 0X00:  if (ch == 0X00)state = 0X01; break;//else {bytesInRxBuffer = Hal_UART_RxBufLen(port);HalUARTCLER (port,bytesInRxBuffer);}
      case 0X01:  if (ch == 0XFF)state = 0X02;else if(ch != 0X00)state = 0X00; break;
      case 0X02: if (ch >= 0X0C){tempDataLen=0;LEN_Token=ch+3;state = 0X03;}else state = 0X00; break;
      case 0X03: Send_data[8 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[8 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
       {HalUARTRead (port, &Send_data[8 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
       
        if (tempDataLen== LEN_Token)
        {state = 0X00; for(char i=0;i<4;i++)Send_data[8+i]=Send_data[17+i];
       Send_datalend=6;GenericApp_SendTheMessage();} break;   
       default: break;} 
       
#endif  
#endif
#if(SENSOR_TYPE ==0X0A)
       chnn[GenericApp_applicationdata]=ch;
      if(GenericApp_applicationdata<=0X1F)GenericApp_applicationdata++;
      else GenericApp_applicationdata=0;
      if((chnn[GenericApp_applicationdata-1]==0X0A)&&(chnn[GenericApp_applicationdata-2]==0X0D))
     {if(GenericApp_applicationdata>0X10)
     { 
      for(char y=0;y<0X14;y++)
       Send_data[y+8]=chnn[y];
    Send_datalend=0X14;
      GenericApp_SendTheMessage();
      for(char y=0;y<0X10;y++)
      chnn[y]=0X00;
     }
      GenericApp_applicationdata=0;
     }
#endif
#if((SENSOR_TYPE ==0X5A)||(SENSOR_TYPE ==0X5B))
     if(GenericApp_uart_data<=0X09)GenericApp_uart_data++;
      else GenericApp_uart_data=0;
     chnn[GenericApp_uart_data]=ch;
    if((GenericApp_uart_data==0x07)&&(chnn[GenericApp_uart_data-5]==0x03))
     {
     Send_data[8]=chnn[4];
    // Send_data[9]=chnn[5];
     if((((chnn[5]>Send_data[9])? chnn[5]-Send_data[9]:Send_data[9]-chnn[5])>0X0A)||((Send_data[9]!=0)&&(chnn[5]==0))) 
  {Send_data[9]=chnn[5];
      GenericApp_SendTheMessage();
    }
      for(char y=0;y<0X11;y++)
      chnn[y]=0X00;
      GenericApp_uart_data=0;
     }
#endif
#if(SENSOR_TYPE ==0X70) 
switch (state) 
    { case 0X00: if (ch == 0XF1){state = 0X01;Send_data[8] =ch;} break;
      case 0X01:LEN_Token = ch;tempDataLen = 0; if (LEN_Token){Send_data[9] = LEN_Token;state = 0X02;}else{state = 0X00;return;}break;
      case 0X02:Send_data[10 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[10 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
        {HalUARTRead (port, &Send_data[10 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== LEN_Token){ state = 0X00;
       Send_datalend=LEN_Token+4;GenericApp_SendTheMessage();}break;   
      default: break;} 
#endif      
#if(SENSOR_TYPE ==0X0B)
       if(GenericApp_applicationdata<=0X1F)GenericApp_applicationdata++;
      else GenericApp_applicationdata=0;
     chnn[GenericApp_applicationdata]=ch;
     if((chnn[GenericApp_applicationdata-16]=='S')&&(chnn[GenericApp_applicationdata-15]=='N')&&(chnn[GenericApp_applicationdata-14]==':'))
     {
      for(char y=0;y<0X14;y++)
       Send_data[y+8]=chnn[GenericApp_applicationdata-(16-y)];
    Send_datalend=0X14;
      GenericApp_SendTheMessage();
      for(char y=0;y<0X10;y++)
      chnn[y]=0X00;
      GenericApp_applicationdata=0;
     }
#endif
#if(SENSOR_TYPE ==0X13) 
#if defined(HUABANG) 
     switch (state) 
    { case 0X00:if(ch==0X03){state = 0X02;}else{state = 0X01;Send_data[7] =ch;} break;
      case 0X01:if(ch==0X03){state = 0X02;}else{Send_data[7] =ch;state = 0X00;return;}break;
      case 0X02:if(ch==0X04){LEN_Token = 6;tempDataLen = 0;state = 0X03;}else{state = 0X00;return;}break;
      case 0X03:Send_data[8 + tempDataLen++] = ch; bytesInRxBuffer = Hal_UART_RxBufLen(port);
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        { HalUARTRead (port, &Send_data[8 + tempDataLen], bytesInRxBuffer);tempDataLen += bytesInRxBuffer; }
        else
        {HalUARTRead (port, &Send_data[8 + tempDataLen], LEN_Token - tempDataLen);tempDataLen += (LEN_Token - tempDataLen);}
        if (tempDataLen== LEN_Token){ state = 0X00;
        //Send_data[7]=0XAA;Send_data[14]=0XBB;
         // Send_data[15]=0;
         // Send_data[8]=Send_data[9]=Send_data[10]=Send_data[11]=Send_data[12]=0;
          Send_datalend=0X0E;GenericApp_SendTheMessage();}break;   
      default: break;} 
#endif
#if defined(ammeter) 
 
     if(GenericApp_uart_data<=0X1F)GenericApp_uart_data++;
      else GenericApp_uart_data=0;
     chnn[GenericApp_uart_data]=ch;
    if((chnn[GenericApp_uart_data]==0x16)&&(chnn[GenericApp_uart_data-10]==0x68)&&(chnn[GenericApp_uart_data-17]==0x68))
     {Send_data[7]=0XAA;
      for(char y=0;y<0X06;y++)
       Send_data[y+8]=chnn[GenericApp_uart_data-(11+y)];
      Send_data[14]=0XBB;
       for(char y=0;y<0X04;y++)
       Send_data[y+15]=(chnn[GenericApp_uart_data-(2+y)]-0X33);
    Send_datalend=0X0E;
      GenericApp_SendTheMessage();
      for(char y=0;y<0X11;y++)
      chnn[y]=0X00;
      GenericApp_uart_data=0;
     }
#endif
#endif
#if(SENSOR_TYPE ==0X14)
#if defined(PH_A)
     if(GenericApp_uart_data<=0X1F)GenericApp_uart_data++;
      else GenericApp_uart_data=0;
     chnn[GenericApp_uart_data]=ch;
    if((chnn[GenericApp_uart_data-16]==0x01)&&(chnn[GenericApp_uart_data-15]==0x03)&&(chnn[GenericApp_uart_data-14]==0x0C))
     {Send_data[7]=0XCC;
      for(char y=0;y<0X04;y++)
      Send_data[y+8]=chnn[GenericApp_uart_data-(13-y)];
      uint16 len;
      len=(Send_data[8]*256+Send_data[9]);
      Send_data[8]=len/1000;
      Send_data[9]=(len%1000)/10;
      len=(Send_data[10]*256+Send_data[11]);
      Send_data[10]=len/100;
      Send_data[11]=len%100;
       Send_data[12]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11])%256;
      Send_datalend=0X07;
      GenericApp_SendTheMessage();
      for(char y=0;y<0X11;y++)
      chnn[y]=0X00;
      GenericApp_uart_data=0;
     }
#endif
#if defined(PH_ORP)
  if(GenericApp_uart_data<=0X09)GenericApp_uart_data++;
      else GenericApp_uart_data=0;
     chnn[GenericApp_uart_data]=ch;
if((chnn[GenericApp_uart_data-7]==0x00)&&(chnn[GenericApp_uart_data-6]==0x03)&&(chnn[GenericApp_uart_data-5]==0x04))
     {Send_data[7]=0XCC;
      for(char y=0;y<0X04;y++)
      Send_data[y+8]=chnn[GenericApp_uart_data-(5-y)];
       Send_data[12]=(Send_data[6]+Send_data[7]+Send_data[8]+Send_data[9]+Send_data[10]+Send_data[11])%256;
      Send_datalend=0X07;
      GenericApp_SendTheMessage();
      for(char y=0;y<0X11;y++)
      chnn[y]=0X00;
      GenericApp_uart_data=0;
     }
#endif
#endif
#if(SENSOR_TYPE ==0X23)
       if(GenericApp_applicationdata<=0X1F)GenericApp_applicationdata++;
      else GenericApp_applicationdata=0;
     chnn[GenericApp_applicationdata]=ch;
     if((chnn[GenericApp_applicationdata-11]==0XEF)&&(chnn[GenericApp_applicationdata-10]==0X01)&&(chnn[GenericApp_applicationdata-5]==0X07))
     {if((chnn[GenericApp_applicationdata-2]==0X00))
        {
          switch(SFG_R30XA[0])
          {
         case 0x01: SFG_R30XA[0]=0x02;break;
         case 0x02: SFG_R30XA[0]=0x03;break;
         case 0x03: SFG_R30XA[0]=0x04;break;
         case 0x04: SFG_R30XA[0]=0x05;break;
         case 0x05: SFG_R30XA[0]=0x06;break;
         case 0x06: SFG_R30XA[0]=0xBB;break;
         case 0xA1: SFG_R30XA[0]=0xA2;break;
         case 0xA2: SFG_R30XA[0]=0xA3;break;//注意接收时多4位，
         case 0xA3: SFG_R30XA[0]=0xCC;break;
         case 0x21: SFG_R30XA[0]=0xDD;break;
          default :break;
          } 
          Send_data[8]=chnn[GenericApp_applicationdata-1];
          Send_data[9]=chnn[GenericApp_applicationdata-0];
        }
     else{ switch(SFG_R30XA[0])
          {
         case 0x01: SFG_R30XA[0]=0x01;break;
         case 0x02: SFG_R30XA[0]=0x01;break;
         case 0x03: SFG_R30XA[0]=0x01;break;
         case 0x04: SFG_R30XA[0]=0x01;break;
         case 0x05: SFG_R30XA[0]=0x01;break;
         case 0x06: SFG_R30XA[0]=0x01;break;
         case 0xA1: SFG_R30XA[0]=0xA1;break;
         case 0xA2: SFG_R30XA[0]=0xA1;break;//注意接收时多4位，
         case 0xA3: SFG_R30XA[0]=0xA1;break;
         case 0x21: SFG_R30XA[0]=0x21;break;
          default :break;
          } 
          }
          
       SFG_R30X=1;
     //  else
   //   SFG_R30X=3;
      for(char y=0;y<0X10;y++)
      chnn[y]=0X00;
      GenericApp_applicationdata=0;
      
     }
#endif
#endif
#endif
  /*  switch (state) 
    { 
      case SOP_STATE: 
        if (ch == MT_UART_SOF)
        state = LEN_STATE; 
        break;
 
      case LEN_STATE:
        LEN_Token = ch;

        tempDataLen = 0;

        // Allocate memory for the data //
        pMsg = (mtOSALSerialData_t *)osal_msg_allocate( sizeof ( mtOSALSerialData_t ) +
                                                        MT_RPC_FRAME_HDR_SZ + LEN_Token );

        if (pMsg)
        {
          // Fill up what we can //
          pMsg->hdr.event = CMD_SERIAL_MSG;
          pMsg->msg = (uint8*)(pMsg+1);
          pMsg->msg[MT_RPC_POS_LEN] = LEN_Token;
          state = CMD_STATE1;
        }
        else
        {
          state = SOP_STATE;
          return;
        }
        break;

      case CMD_STATE1:
        pMsg->msg[MT_RPC_POS_CMD0] = ch;
        state = CMD_STATE2;
        break;

      case CMD_STATE2:
        pMsg->msg[MT_RPC_POS_CMD1] = ch;
        // If there is no data, skip to FCS state //
        if (LEN_Token)
        {
          state = DATA_STATE;
        }
        else
        {
          state = FCS_STATE;
        } 
        break;

      case DATA_STATE:

        // Fill in the buffer the first byte of the data //
        pMsg->msg[MT_RPC_FRAME_HDR_SZ + tempDataLen++] = ch;

        // Check number of bytes left in the Rx buffer //
        bytesInRxBuffer = Hal_UART_RxBufLen(port);

        // If the remain of the data is there, read them all, otherwise, just read enough //
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        {
          HalUARTRead (port, &pMsg->msg[MT_RPC_FRAME_HDR_SZ + tempDataLen], bytesInRxBuffer);
          tempDataLen += bytesInRxBuffer;
        }
        else
        {
          HalUARTRead (port, &pMsg->msg[MT_RPC_FRAME_HDR_SZ + tempDataLen], LEN_Token - tempDataLen);
          tempDataLen += (LEN_Token - tempDataLen);
        }

        // If number of bytes read is equal to data length, time to move on to FCS //
        if ( tempDataLen == LEN_Token )
            state = FCS_STATE;

        break;

      case FCS_STATE:

        FSC_Token = ch;

       // Make sure it's correct //
        if ((MT_UartCalcFCS ((uint8*)&pMsg->msg[0], MT_RPC_FRAME_HDR_SZ + LEN_Token) == FSC_Token))
        {
          osal_msg_send( App_TaskID, (byte *)pMsg );
        }
        else
        {
         // deallocate the msg //
          osal_msg_deallocate ( (uint8 *)pMsg );
        }

        // Reset the state, send or discard the buffers at this point //
        state = SOP_STATE;

        break;

      default:
       break;
    }*/
  }HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
}

#if defined (ZAPP_P1) || defined (ZAPP_P2)
/***************************************************************************************************
 * @fn      MT_UartProcessZAppData
 *
 * @brief   | SOP | CMD  |   Data Length   | FSC  |
 *          |  1  |  2   |       1         |  1   |
 *
 *          Parses the data and determine either is SPI or just simply serial data
 *          then send the data to correct place (MT or APP)
 *
 * @param   port    - UART port
 *          event   - Event that causes the callback
 *
 *
 * @return  None
 ***************************************************************************************************/
void MT_UartProcessZAppData ( uint8 port, uint8 event )
{

  osal_event_hdr_t  *msg_ptr;
  uint16 length = 0;
  uint16 rxBufLen  = Hal_UART_RxBufLen(MT_UART_DEFAULT_PORT);

  /*
     If maxZAppBufferLength is 0 or larger than current length
     the entire length of the current buffer is returned.
  */
  if ((MT_UartMaxZAppBufLen != 0) && (MT_UartMaxZAppBufLen <= rxBufLen))
  {
    length = MT_UartMaxZAppBufLen;
  }
  else
  {
    length = rxBufLen;
  }

  /* Verify events */
  if (event == HAL_UART_TX_FULL)
  {
    // Do something when TX if full
    return;
  }

  if (event & ( HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT))
  {
    if ( App_TaskID )
    {
      /*
         If Application is ready to receive and there is something
         in the Rx buffer then send it up
      */
      if ((MT_UartZAppRxStatus == MT_UART_ZAPP_RX_READY ) && (length != 0))
      {
        /* Disable App flow control until it processes the current data */
         MT_UartAppFlowControl (MT_UART_ZAPP_RX_NOT_READY);

        /* 2 more bytes are added, 1 for CMD type, other for length */
        msg_ptr = (osal_event_hdr_t *)osal_msg_allocate( length + sizeof(osal_event_hdr_t) );
        if ( msg_ptr )
        {
          msg_ptr->event = SPI_INCOMING_ZAPP_DATA;
          msg_ptr->status = length;

          /* Read the data of Rx buffer */
          HalUARTRead( MT_UART_DEFAULT_PORT, (uint8 *)(msg_ptr + 1), length );

          /* Send the raw data to application...or where ever */
          osal_msg_send( App_TaskID, (uint8 *)msg_ptr );
        }
      }
    }
  }
}

/***************************************************************************************************
 * @fn      SPIMgr_ZAppBufferLengthRegister
 *
 * @brief
 *
 * @param   maxLen - Max Length that the application wants at a time
 *
 * @return  None
 *
 ***************************************************************************************************/
void MT_UartZAppBufferLengthRegister ( uint16 maxLen )
{
  /* If the maxLen is larger than the RX buff, something is not right */
  if (maxLen <= MT_UART_DEFAULT_MAX_RX_BUFF)
    MT_UartMaxZAppBufLen = maxLen;
  else
    MT_UartMaxZAppBufLen = 1; /* default is 1 byte */
}

/***************************************************************************************************
 * @fn      SPIMgr_AppFlowControl
 *
 * @brief
 *
 * @param   status - ready to send or not
 *
 * @return  None
 *
 ***************************************************************************************************/
void MT_UartAppFlowControl ( bool status )
{

  /* Make sure only update if needed */
  if (status != MT_UartZAppRxStatus )
  {
    MT_UartZAppRxStatus = status;
  }

  /* App is ready to read again, ProcessZAppData have to be triggered too */
  if (status == MT_UART_ZAPP_RX_READY)
  {
    MT_UartProcessZAppData (MT_UART_DEFAULT_PORT, HAL_UART_RX_TIMEOUT );
  }

}

#endif //ZAPP

/***************************************************************************************************
***************************************************************************************************/
