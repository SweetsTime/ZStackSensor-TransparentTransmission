#include <HX711.h>
unsigned long  INzero; //零点值
unsigned long  AD_val;	//AD值绝对值
long AD_Compar;//AD相对值
unsigned long ReadCount(char AB)
{ 
unsigned long Count;    
unsigned char i; 
CLR_ADSK() ; 
Count=0; 
while(!DATA); 
asm("NOP");
while(DATA); 
for (i=0;i<24;i++){
SET_ADSK() ; 
Count=Count<<1; 
CLR_ADSK() ; 
if(DATA) Count++; 
} 
SET_ADSK() ; 
Count=Count^0x800000; 
CLR_ADSK() ;
if(AB==1)
{
SET_ADSK() ;
asm("NOP");
CLR_ADSK() ;
} 
SET_ADSK(); 
return(Count); 
} 

void AD_filter() ///////////////////////自动滤波////////////////////////////
{
AD_val= ReadCount(0);
AD_Compar=AD_val;
AD_Compar>>=8; 
AD_val>>=6;
AD_Compar=AD_Compar-INzero;//------要
}

 void Tozero()/////////////////////////开机自动归零/////////////////////////
{AD_val= ReadCount(0);
AD_Compar=AD_val;
AD_Compar>>=8; 
if(AD_Compar<80000)
{	
INzero=AD_Compar;
}
} 