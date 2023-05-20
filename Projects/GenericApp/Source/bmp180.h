#ifndef BMP180_H
#define BMP180_h

#define uint16_t unsigned short
#define int16_t short
#define uint8_t unsigned char
#define uint32_t unsigned int
#define int32_t int

#define BMP180_ADDRESS        0xEE

/*************大气压力传感器模拟IIC协议引脚***********************/


#define SCL_H         P1_3 = 1
#define SCL_L         P1_3 = 0   
#define SDA_H         P1_4 = 1
#define SDA_L         P1_4 = 0
#define SCL_read      P1_3
#define SDA_read      P1_4
//#endif
void Init_BMP180(void);
void Multiple_Read_BMP180(void);

extern long pressure;
extern long temperature;
#endif