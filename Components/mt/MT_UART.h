/***************************************************************************************************
  Filename:       MT_UART.h
  Revised:        $Date: 2009-07-02 15:24:39 -0700 (Thu, 02 Jul 2009) $
  Revision:       $Revision: 20269 $

  Description:    This header describes the functions that handle the serial port.

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
#ifndef MT_UART_H
#define MT_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************************************
 *                                               INCLUDES
 ***************************************************************************************************/
#include "Onboard.h"
#include "OSAL.h"
#include "GenericApp.h"

/***************************************************************************************************
 *                                             CONSTANTS
 ***************************************************************************************************/

/* Start-of-frame delimiter for UART transport */
#define MT_UART_SOF                     0xFE

/* UART frame overhead for SOF and FCS bytes */
#define MT_UART_FRAME_OVHD              2

/* Default values */
#if defined (ZTOOL_P1) || defined (ZTOOL_P2)
  #define MT_UART_DEFAULT_PORT           ZTOOL_PORT
#elif defined (ZAPP_P1) || defined (ZAPP_P2)
  #define MT_UART_DEFAULT_PORT           ZAPP_PORT
#endif

#if !defined( MT_UART_DEFAULT_OVERFLOW )
  #define MT_UART_DEFAULT_OVERFLOW       FALSE
#endif
  
#if defined( SENSOR_TYPE_Coord )
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_38400
#else
#if((SENSOR_TYPE =='G')||(SENSOR_TYPE =='T')||(SENSOR_TYPE =='O')||(SENSOR_TYPE =='P')||\
  (SENSOR_TYPE ==0X02)||(SENSOR_TYPE ==0X06)||(SENSOR_TYPE ==0X09)\
    ||(SENSOR_TYPE ==0X14)||(SENSOR_TYPE ==0X18)||(SENSOR_TYPE ==0X29)||(SENSOR_TYPE ==0X73)||(SENSOR_TYPE ==0X79))
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_9600
#endif
 
#if defined (NB_IOT_S10)
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_9600
#endif
  
#if(SENSOR_TYPE ==0X07)
#if defined(UHF)
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_9600
#endif
#if defined(RLM100)
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_57600
#endif
#endif
  
#if defined( CC2530_RF_433M )  
#if((SENSOR_TYPE ==0X39)||(SENSOR_TYPE ==0X63))
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_9600
#endif
#endif
  
#if((SENSOR_TYPE ==0X20)||(SENSOR_TYPE ==0X74))
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_2400
#endif
#if(SENSOR_TYPE ==0X72)
#if defined( SDS011 )
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_9600
#else
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_2400
#endif
#endif
#if((SENSOR_TYPE ==0X08)||(SENSOR_TYPE ==0X0A)||(SENSOR_TYPE ==0X25)||(SENSOR_TYPE ==0X26)||(SENSOR_TYPE ==0X78))
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_115200
#endif
  
#if(SENSOR_TYPE ==0X0E)
#if defined(STM2_4G) 
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_115200
#endif
  
#if defined(NRF2_4G) 
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_9600
#endif
#endif
  
#if(SENSOR_TYPE ==0X0D)
#if defined(TGRD15693) 
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_115200
#endif
#if defined(PN532) 
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_115200
#endif
#endif
#if(SENSOR_TYPE ==0X7B)
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_9600
#endif
  
#if(SENSOR_TYPE ==0X7C)                 //无线无源
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_9600
#endif
  
#if(SENSOR_TYPE ==0X85)
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_115200
#endif
  
#if(SENSOR_TYPE ==0X13)
#if defined(HUABANG) 
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_9600
#endif
#if defined(ammeter) 
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_1200
#endif
#endif
#if((SENSOR_TYPE ==0X0B)||(SENSOR_TYPE ==0XA3)||(SENSOR_TYPE ==0X5A)||(SENSOR_TYPE ==0X5B)||(SENSOR_TYPE ==0X27)||(SENSOR_TYPE ==0X6F)||(SENSOR_TYPE ==0X70)\
  ||(SENSOR_TYPE ==0X71)||(SENSOR_TYPE ==0X86)||(SENSOR_TYPE ==0X87))
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_9600
#endif
#if(SENSOR_TYPE ==0X31)
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_9600 //串口0 和小车通讯
#define MT_UART1_DEFAULT_BAUDRATE        HAL_UART_BR_9600 //串口1 和RFID低频通讯
#endif
#if(SENSOR_TYPE ==0X6B)
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_38400
#endif
#if(SENSOR_TYPE ==0X6D)
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_115200
#endif
#if(SENSOR_TYPE ==0XF0)
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_9600
#endif
#if((SENSOR_TYPE ==0X23)||(SENSOR_TYPE ==0X05))
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_9600
#endif
#endif

#if !defined MT_UART_DEFAULT_BAUDRATE
#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_38400
#endif
  

  
#define MT_UART_DEFAULT_THRESHOLD        MT_UART_THRESHOLD
#define MT_UART_DEFAULT_MAX_RX_BUFF      MT_UART_RX_BUFF_MAX
#if !defined( MT_UART_DEFAULT_MAX_TX_BUFF )
  #define MT_UART_DEFAULT_MAX_TX_BUFF    MT_UART_TX_BUFF_MAX
#endif
#define MT_UART_DEFAULT_IDLE_TIMEOUT     MT_UART_IDLE_TIMEOUT

/* Application Flow Control */
#define MT_UART_ZAPP_RX_NOT_READY         0x00
#define MT_UART_ZAPP_RX_READY             0x01
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8             *msg;
} mtOSALSerialData_t;

/*
 * Initialization
 */
extern void MT_UartInit (void);
#if(SENSOR_TYPE ==0X31)
extern void MT_Uart1Init (void);
#endif
/*
 * Process ZTool Rx Data
 */
void MT_UartProcessZToolData ( uint8 port, uint8 taskId );

/*
 * Process ZApp Rx Data
 */
void MT_UartProcessZAppData ( uint8 port, uint8 event );

/*
 * Calculate the check sum
 */
extern uint8 MT_UartCalcFCS( uint8 *msg_ptr, uint8 length );

/*
 * Register TaskID for the application
 */
extern void MT_UartRegisterTaskID( uint8 taskID );

/*
 * Register max length that application can take
 */
extern void MT_UartZAppBufferLengthRegister ( uint16 maxLen );

/*
 * Turn Application flow control ON/OFF
 */
extern void MT_UartAppFlowControl ( uint8 status );

/***************************************************************************************************
***************************************************************************************************/

#endif  /* MT_UART_H */
