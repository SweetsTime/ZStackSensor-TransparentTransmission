
#include  "mma7660.h"


int MMA7660_SPCNT_Value;
int MMA7660_INTSU_Value;
int MMA7660_MODE_Value;
int MMA7660_SR_Value;
int MMA7660_PDET_Value;
int MMA7660_PD_Value;
void mma_delay(int time) ;
void IIC_OneClk(void);
int IIC_ReadByteNCK(void)  ;



void mma_delay(int time)                      //Time delay function
{             
    int i;
    for (i=0; i<time; i++) ;                //Software loop for time delay
}

void IIC_OneClk(void)                           //I2C CLK pin output one clock: CLK pin rises up before falls down
{                
    mma_delay(Delay5us);
    I2C_SCL=1;		        //I2C CLK pin output high(1)		
    mma_delay(Delay10us);                   //Delay 10uS
    I2C_SCL=0;                        //I2C CLK pin output low(0)
    mma_delay(Delay5us);	
}

void IIC_Start(void) 
{                    
    DATAOUT;                   //Set I2C DATA pin as output
   I2C_SCL=1;                    //I2C CLK pin output high(1)
    I2C_SDA=1; 	            //I2C DATA pin output high(1)
    mma_delay(Delay20us);                   //Delay 20uS
    I2C_SDA=0;                  //I2C DATA pin output low(0)
    mma_delay(Delay10us);                   //Delay 10uS
    I2C_SCL=0;                  //I2C CLK pin output low(0)
    mma_delay(Delay10us);                   //Delay 10uS
}

void IIC_Stop (void)                            //I2C Stop signal generation: Data pin rises up when clock in is high
{                   
    mma_delay(Delay10us);                   //Delay 10uS
    I2C_SCL=1;		           //I2C CLK pin output high(1)
    mma_delay(Delay10us);                   //Delay 10uS
    I2C_SDA=1;	                    //I2C DATA pin output high(1)
}
 
void IIC_SendByte(int sData)                 //I2C send one byte out
{        
    int i;
    for (i=7; i>=0; i--)                        //Loop 8 times to send 8 bits
    {                  
	if ((sData>>i)&0x01)                    //Judge output 1 or 0
        {                
	  I2C_SDA=1;	                //I2C DATA pin output high(1) if output 1
	} 
        else 
        { 
	 I2C_SDA=0;                   //I2C DATA pin output low(0) if output 0
	}
	  IIC_OneClk();                         //Output one clock pulse after data pin is ready
    }		
}

int  IIC_ChkAck(void)                     //Check I2C Acknowledgement signal
{     
    DATAIN;                      //Set I2C DATA pin as input
    mma_delay(Delay5us);
     I2C_SCL=1;		        //I2C CLK pin output high(1)	
    mma_delay(Delay5us);                 //Delay 10uS
    if (I2C_SDA)
    {                 //Read I2C DATA pin
      mma_delay(Delay5us);                  //Delay 5uS
     I2C_SCL=0;                     //I2C CLK pin output low(0)
      mma_delay(Delay5us);                   //Delay 5us again
      DATAOUT;                   //Set I2C DATA pin as output
      I2C_SDA=0;                    //I2C DATA pin output low(0)
      return 1;                             //Return 1 if read 1 from I2C DATA pin
    } 
    else 
    {                              //If I2C DATA pin is invalid for acknowledgement signal
      mma_delay(Delay5us);                  //Delay 5uS
      I2C_SCL=0;                     //I2C CLK pin output low(0)
      mma_delay(Delay5us);                  //Delay 5uS again
     DATAOUT;                    //Set I2C DATA pin as output
      I2C_SDA=0;                    //I2C DATA pin output low(0)
      return 0;                             //Return 0 if read 0 from I2C DATA pin
    }			
    	
    
}

void IIC_RepeatedStart(void) 
{            //I2C Repeat Start signal generation: Data pin falls down when clock is high
    mma_delay(Delay20us);                   //Delay 20uS
    mma_delay(Delay20us);                   //Delay 20uS
    I2C_SDA=1;	                     //I2C DATA pin output high(1)
    mma_delay(Delay10us);                   //Delay 10uS
    I2C_SCL=1;	                    //I2C CLK pin output high(1)
    mma_delay(Delay20us);                   //Delay 20uS
    mma_delay(Delay20us);                   //Delay 20uS
    I2C_SDA=0;                       //I2C DATA pin output low(0)
    mma_delay(Delay10us);                   //Delay 10uS
    I2C_SCL=0;                       //I2C CLK pin output low(0)
    mma_delay(Delay10us);                   //Delay 10uS	
}

int IIC_ReadByteNCK(void)               //Read one byte but do not send acknowledgement signal
{            
    int i;
    int data;
    DATAIN;                    //Set I2C DATA pin as input
    data = 0;                               //Prepare to receive data
    for (i=7; i>=0; i--) 
    {                  //Loop 8 times to receive 8 bits
      if (I2C_SDA) 
        data |= (0x01<<i);    //If read a 1, set to data bit
      IIC_OneClk();
    }			                                //Output one clock pulse after a bit is read

    DATAOUT;                    //Set I2C DATA pin as output
    I2C_SDA=1;                    //I2C DATA pin output high(1): no acknowledge
    IIC_OneClk();                           //Output one clock pulse after data pin is ready
    I2C_SDA=0;                      //I2C DATA pin output low(0)
    return data;                            //Return received data
}


void FSL_MMA_IICWrite(int RegAdd, int Data)          //Write one byte to a sensor register via I2C
{   
    //Start
    IIC_Start();                                        //Output a START signal

    // Device hardware address
    IIC_SendByte(FSL_MMA_AddW);                         //Send one byte of sensor IIC address for writing
    if (IIC_ChkAck()) 
    {                                 //Check acknowledge signal
      #ifdef ACC_DEBUG	                                //Only for debug
        prompt_trace(MOD_MMA, "# Device Write Address Error #\r\n");   //Print error information
      #endif
        IIC_Stop();	                                      //Output a STOP signal
      return;                                           //If acknowledgement signal is read as 1, then return to end
    }
                                                      //If acknowledgement signal is read as 0, then go to next step
    // Register address to read                         
    IIC_SendByte(RegAdd);                               //Send one byte of register address in the sensor
    if (IIC_ChkAck()) 
     {                                 //Check acknowledgement signal
      #ifdef ACC_DEBUG	                                //Only for debug
        prompt_trace(MOD_MMA, "# Sensor Reg Address NACK #\r\n");   //Print error information
      #endif                                              
        IIC_Stop();                                       //Output a STOP signal
      return;    	                                      //If acknowledgement signal is read as 1, then return to end
     }                                                     //If acknowledgement signal is read as 0, then go to next step

    // Data to send
    IIC_SendByte(Data);                                 //Send one byte of data
    if (IIC_ChkAck())
    {                                 //Check acknowledgement signal
      #ifdef ACC_DEBUG	                                //Only for debug
        prompt_trace(MOD_MMA, "# Sensor Data NACK #\r\n");    //Print error information
      #endif
        IIC_Stop();	                                      //Output a STOP signal
      return;                                           //If acknowledgement signal is read as 1, then return to end
    }
                                                      //If acknowledgement signal is read as 0, then go to next step
    // Stop	
    IIC_Stop();	                                        //Output a STOP signal	

}


int FSL_MMA_IICRead(int RegAdd)              //Read a byte from sensor register via I2C
{                
    int Data;

    //Start
    IIC_Start();                                        //Output a START signal
                                                      
    // Device hardware address
    IIC_SendByte(FSL_MMA_AddW);                         //Send a byte of sensor IIC address for writing
    if (IIC_ChkAck()) 
    {                                 //Check acknowledge signal 
      #ifdef ACC_DEBUG	                                //Only for debug
        prompt_trace(MOD_MMA, "# Device Write Address Error #\r\n");   //Print error information
      #endif
	IIC_Stop();	                                      //Output a STOP signal	
	return 0;                                         //If acknowledgement signal is read as 1, then return to end
    }
                                                      //If acknowledgement signal is read as 0, then go to next step
    // Register address to read
    IIC_SendByte(RegAdd);                               //Send one byte of register address in the sensor
    if (IIC_ChkAck()) 
    {                                 //Check acknowledge signal 
    #ifdef ACC_DEBUG	                                //Only for debug
      prompt_trace(MOD_MMA, "# Sensor Reg Address NACK #\r\n");   //Print error information
    #endif
      IIC_Stop();	                                      //Output a STOP signal	
      return 0;                                         //If acknowledgement signal is read as 1, then return to end
    }
                                                      //If acknowledgement signal is read as 0, then go to next step
    // Repeated Start
    IIC_RepeatedStart();                                //Output a REPEAT START signal
	// IIC_Start(); 
    // Device hardware address                          
    IIC_SendByte(FSL_MMA_AddR);                         //Send the sensor IIC address for reading
    if (IIC_ChkAck()) 
    {                                 //Check acknowledge signal 
      #ifdef ACC_DEBUG	                                //Only for debug
        prompt_trace(MOD_MMA, "# Device Read Address Error #\r\n");    //Print error information
      #endif
    }                                                 //If acknowledgement signal is read as 1, do nothing
                                                      //If acknowledgement signal is read as 0, then go to next step
    Data = IIC_ReadByteNCK();                           //Read one byte but do not output acknowledgement
   // #ifdef ACC_DEBUG	                                //Only for debug
     //  prompt_trace(MOD_MMA, "# Sensor Read Data = %d #\r\n", Data);   //Print received data
    //#endif

    // Stop	
    IIC_Stop();	                                        //Output a STOP signal
    
    return Data;                                        //Return received data
}
void MMA7660_Init(void)
{
  FSL_MMA_IICWrite(MMA7660_MODE, 0);              //Make 7660 enter standby mode to set registers

  FSL_MMA_IICWrite(MMA7660_SPCNT, MMA7660_SPCNT_Value);//Sleep Counter Register: SPCNT
//                                                  b00000000 :Default
//                                                  if SCPS = 0, Sleep clock is 64Hz~1Hz, maximum sleep time is 4s~256s
//                                                  if SCPS = 1, Sleep clock is 4Hz~1/16Hz, maximum sleep time is 64s~4096s
  FSL_MMA_IICWrite(MMA7660_INTSU, MMA7660_INTSU_Value);//Interrupt Setup Register: INTSU
//                                                  b00000000 :Default
//                                                   ||||||||
//                                                   |||||||+- FBINT: Frount/Back position causes an interrupt or not
//                                                   ||||||+-- PLINT: Up/Down/Right/Left position causes an interrupt or not
//                                                   |||||+--- PDINT: Successful pulse detection causes an interrupt or not
//                                                   ||||+---- ASINT: Exiting Auto-Sleep causes an interrupt or not
//                                                   |||+----- GINT:  Every measurement causes an interrupt or not
//                                                   ||+------ SHINTX:Shake on X axis causes an interrupt and set the Shake bit in TILT register or not
//                                                   |+------- SHINTY:Shake on Y axis causes an interrupt and set the Shake bit in TILT register or not
//                                                   +-------- SHINTZ:Shake on Z axis causes an interrupt and set the Shake bit in TILT register or not
  FSL_MMA_IICWrite(MMA7660_SR, MMA7660_SR_Value); //Sample Rates Register: SR
//                                                  b00000001 :Default
//                                                   ||||||||
//                                                   |||||+++- AMSR[2:0]: Sample rates in Active mode and Auto-Sleep mode
//                                                   |||++---- AWSR[1:0]: Sample rates in Auto-Wake mode
//                                                   +++------ FILT[2:0]: Tilt debounce filter
  FSL_MMA_IICWrite(MMA7660_PDET, MMA7660_PDET_Value);//Pulse Detection Register: PDET
//                                                  b00000000 :Default
//                                                   ||||||||
//                                                   |||+++++- PDTH[4:0]: Pulse detection threshold
//                                                   ||+------ XDA: 0-X axis is enabled for pulse detection; 1-disabled
//                                                   |+------- YDA: 0-Y axis is enabled for pulse detection; 1-disabled
//                                                   +-------- ZDA: 0-Z axis is enabled for pulse detection; 1-disabled
  FSL_MMA_IICWrite(MMA7660_PD, MMA7660_PD_Value); //Pulse Debounce Count: PD
//                                                  b00000000 :Default
//                                                   ||||||||
//                                                   ++++++++- PD[7:0]: Pulse detection debounce filter
  FSL_MMA_IICWrite(MMA7660_MODE, MMA7660_MODE_Value);//Mode Register: MODE
//                                                  b00000000 :Default
//                                                   ||||||||
//                                                   |||||||+- MODE:0-Standby or test mode; 1-Active mode
//                                                   ||||||+-- Reserved: 0
//                                                   |||||+--- TON: 0-Normal mode; 1-Test mode
//                                                   ||||+---- AWE: 0-Auto-Wake disabled; 1-Enabled
//                                                   |||+----- ASE: 0-Auto-Sleep disabled; 1-Enabled
//                                                   ||+------ SCPS:0-Sleep counter prescaler is divided by 1; 1-divided by 16
//                                                   |+------- IPP: 0-Interrupt output is open-drain; 1-push-pull
//                                                   +-------- IAH: 0-Interrupt is active low; 1-active high
}

void MMA7660_Startup(void)
{
  MMA7660_SPCNT_Value = 240;     //Sleep delay = 60/16*16 = 60s
  MMA7660_INTSU_Value = 0x10;   //Only Front/Back position change and Up/Down/Right/Left Position change cause interrupts
  MMA7660_SR_Value = 0xF1;      //FILT[2:0] = 111 - 8 samples per filter
                                //AWSR[1:0] = 11  - 1 samples/second on auto-wake mode
                                //AMSR[2:0] = 100 - 8 samples/second on active mode
  MMA7660_MODE_Value = 0x39;    //Interrupt output high active
                                //Interrupt output open drain
                                //Sleep counter clock divided by 16
                                //Auto-Sleep enabled
                                //Auto-Wake enabled
                                //Test Mode off
                                //Active Mode
  MMA7660_Init();
}

int mma7660_IICRead_Alert(int RegAdd)
{

  int temp8u;
  
  do{
    temp8u = FSL_MMA_IICRead(RegAdd);
  } while (temp8u&0x40);
  return temp8u;
}

void IIC_Read_MMA7660_XYZ6(int *pX, int *pY, int *pZ)
{
    *pX = (int)mma7660_IICRead_Alert(MMA7660_XOUT);
    //Sign extend
   // if (*pX&0x20) *pX |= 0xC0;  
    *pY = (int)mma7660_IICRead_Alert(MMA7660_YOUT);  
    //Sign extend
    //if (*pY&0x20) *pY |= 0xC0;  
    *pZ = (int)mma7660_IICRead_Alert(MMA7660_ZOUT);  
    //Sign extend
    //if (*pZ&0x20) *pZ |= 0xC0;  
}

