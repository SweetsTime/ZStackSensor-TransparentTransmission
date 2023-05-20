#include <SHT1x.h>
/*-------------------------------------- 
;模块名称:s_transstart(); 
;功    能:启动传输函数
;占用资源:--
;参数说明:--
;创建日期:2008.08.15 
;版    本:FV1.0(函数版本Function Version)
;修改日期:--
;修改说明:--
;-------------------------------------*/  
 
void s_transstart(void) 
// generates a transmission start  
//       _____         ________ 
// DATA:      |_______| 
//           ___     ___ 
// SCK : ___|   |___|   |______ 
{   SCK=0;   
  SHT_DATAOUT;
   DATA=1; SCK=0;                   //Initial state 
   _nop_(); 
   SCK=1; 
   _nop_(); 
   DATA=0; 
   _nop_(); 
   SCK=0;   
   _nop_();_nop_();
   SCK=1; 
   _nop_(); 
   DATA=1;        
   _nop_(); 
   SCK=0;    
   _nop_();
} 

/*-------------------------------------- 
;模块名称:s_connectionreset(); 
;功    能:连接复位函数
;占用资源:--
;参数说明:--
;创建日期:2008.08.15 
;版    本:FV1.0(函数版本Function Version)
;修改日期:--
;修改说明:--
;-------------------------------------*/ 
void s_connectionreset(void) 
// communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart 
//       _____________________________________________________         ________ 
// DATA:                                                      |_______| 
//          _    _    _    _    _    _    _    _    _        ___     ___ 
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______ 
{   
  unsigned char i;  
  SCK=0;   
   SHT_DATAOUT;_nop_();
  DATA=1;_nop_(); SCK=0;                    //Initial state 
  for(i=0;i<9;i++)                  //9 SCK cycles 
  {_nop_(); 
    SCK=1;
    _nop_(); 
    SCK=0; 
  } 
  _nop_();
  s_transstart();                   //transmission start 
} 

/*-------------------------------------- 
;模块名称:s_write_byte(); 
;功    能:DHT90写函数
;占用资源:--
;参数说明:--
;创建日期:2008.08.15 
;版    本:FV1.0(函数版本Function Version)
;修改日期:--
;修改说明:--
;-------------------------------------*/ 
char s_write_byte(unsigned char value) 
//---------------------------------------------------------------------------------- 
// writes a byte on the Sensibus and checks the acknowledge  
{ 
  unsigned char y,error=0;   
   _nop_();   _nop_(); 
  for (y=0x80;y>0;y/=2)             //shift bit for masking 
  {  
    if (y & value) DATA=1;          //masking value with i , write to SENSI-BUS 
    else DATA=0;      
    SCK=1;                          //clk for SENSI-BUS 
    _nop_();_nop_();_nop_();        //pulswith approx. 3 us    
    SCK=0; 
  }
  DATA=1;                          //release DATA-line 
  SHT_DATAIN;
  _nop_(); 
  SCK=1;                            //clk #9 for ack  
  _nop_();
 
  error=DATA;                       //check ack (DATA will be pulled down by DHT90),DATA在第9个
  _nop_();_nop_();_nop_();  
  SCK=0;
  SHT_DATAOUT;
  DATA=1;                         //release DATA-line 
  _nop_(); 
  return error;                     //error=1 in case of no acknowledge //返回：0成功，1失败
} 
 

/*-------------------------------------- 
;模块名称:s_read_byte(); 
;功    能:DHT90读函数
;占用资源:--
;参数说明:--
;创建日期:2008.08.15 
;版    本:FV1.0(函数版本Function Version)
;修改日期:--
;修改说明:--
;-------------------------------------*/ 
char s_read_byte(unsigned char ack)  
// reads a byte form the Sensibus and gives an acknowledge in case of "ack=1"  
{   unsigned char y,val=0; 
  for (y=0x80;y>0;y/=2)             //shift bit for masking 
  { SCK=1;                          //clk for SENSI-BUS 
  _nop_();
    if (DATA) val=(val | y);        //read bit   
	_nop_();_nop_();_nop_();    //pulswith approx. 3 us
    SCK=0;        
  } 
    SHT_DATAOUT;_nop_();
  if(ack==1)DATA=0;                 //in case of "ack==1" pull down DATA-Line 
  else DATA=1;                      //如果是校验(ack==0)，读取完后结束通讯
  _nop_();_nop_();_nop_();          //pulswith approx. 3 us 
  SCK=1;                            //clk #9 for ack 
  _nop_();_nop_();_nop_();          //pulswith approx. 3 us  
  SCK=0;                 
  _nop_();_nop_();_nop_();          //pulswith approx. 3 us 
 DATA=1;                           //release DATA-line 
  SHT_DATAIN;
  return val; 
} 
 
/*-------------------------------------- 
;模块名称:s_measure(); 
;功    能:测量温湿度函数
;占用资源:--
;参数说明:--
;创建日期:2008.08.15 
;版    本:FV1.0(函数版本Function Version)
;修改日期:--
;修改说明:--
;-------------------------------------*/ 
char s_measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode) 
// makes a measurement (humidity/temperature) with checksum 
{ 
  unsigned error=0; 
  unsigned long int i; 
 
  s_transstart();                   //transmission start 
  switch(mode){                     //send command to sensor 
    case TEMP  : error+=s_write_byte(MEASURE_TEMP); break; 
    case HUMI  : error+=s_write_byte(MEASURE_HUMI); break; 
    default     : break;    
  } 
   SHT_DATAIN;
  for (i=0;i<125535;i++) if(DATA==0) break; //wait until sensor has finished the measurement 
  if(DATA) error+=1;                // or timeout (~2 sec.) is reached 
  *(p_value)  =s_read_byte(ACK);    //read the first byte (MSB) 
  *(p_value+1)=s_read_byte(ACK);    //read the second byte (LSB) 
  *p_checksum =s_read_byte(noACK);  //read checksum 
  return error; 
} 
 
/*-------------------------------------- 
;模块名称:calc_dht90(); 
;功    能:温湿度补偿函数
;占用资源:--
;参数说明:--
;创建日期:2008.08.15 
;版    本:FV1.0(函数版本Function Version)
;修改日期:--
;修改说明:--
;-------------------------------------*/ 
/*
void calc_dht90(unsigned int *p_humidity ,unsigned int *p_temperature)
// calculates temperature [C] and humidity [%RH] 
// input :  humi [Ticks] (12 bit) 
//          temp [Ticks] (14 bit)
// output:  humi [%RH]
//          temp [C]
{ const unsigned int C1=400;              // for 12 Bit
  const unsigned int C2=405;           // for 12 Bit
  const unsigned int C3=28;        // for 12 Bit
  const unsigned int T1=1000;             // for 14 Bit @ 5V
  const unsigned int T2=8;           // for 14 Bit @ 5V 

  unsigned long int rh=*p_humidity;             // rh:      Humidity [Ticks] 12 Bit 
  unsigned long int t=*p_temperature;           // t:       Temperature [Ticks] 14 Bit
  unsigned long int rh_lin;                     // rh_lin:  Humidity linear
  unsigned long int rh_true;                    // rh_true: Temperature compensated humidity
  unsigned int t_C;                        // t_C   :  Temperature [C]

  t_C=(t - 4000);                  //calc. temperature from ticks to [C]
  rh_lin=(C2*rh)/100-(C3*rh*rh)/100000 -C1;     //calc. humidity from ticks to [%RH]
  rh_true=(((t_C-2500)*(T1+T2*rh))/100000)+rh_lin;   //calc. temperature compensated humidity [%RH]
  if(rh_true>10000)rh_true=10000;       //cut if the value is outside of
  if(rh_true<10)rh_true=10;       //the physical possible range

  *p_temperature=t_C;               //return temperature [C]
  *p_humidity=rh_true;              //return humidity[%RH]
}*/
void calc_dht90(float *p_humidity ,float *p_temperature)
// calculates temperature [C] and humidity [%RH] 
// input :  humi [Ticks] (12 bit) 
//          temp [Ticks] (14 bit)
// output:  humi [%RH]
//          temp [C]
{ const float C1=-4.0;              // for 12 Bit
  const float C2=+0.0405;           // for 12 Bit
  const float C3=-0.0000028;        // for 12 Bit
  const float T1=+0.01;             // for 14 Bit @ 5V
  const float T2=+0.00008;           // for 14 Bit @ 5V 

  float rh=*p_humidity;             // rh:      Humidity [Ticks] 12 Bit 
  float t=*p_temperature;           // t:       Temperature [Ticks] 14 Bit
  float rh_lin;                     // rh_lin:  Humidity linear
  float rh_true;                    // rh_true: Temperature compensated humidity
  float t_C;                        // t_C   :  Temperature [C]

  t_C=t*0.01 - 40;                  //calc. temperature from ticks to [C]
  rh_lin=C3*rh*rh + C2*rh + C1;     //calc. humidity from ticks to [%RH]
  rh_true=(t_C-25)*(T1+T2*rh)+rh_lin;   //calc. temperature compensated humidity [%RH]
  if(rh_true>100)rh_true=100;       //cut if the value is outside of
  if(rh_true<0.1)rh_true=0.1;       //the physical possible range

  *p_temperature=t_C;               //return temperature [C]
  *p_humidity=rh_true;              //return humidity[%RH]
}
//*********************第二部分DHT90设置   END****************************************
