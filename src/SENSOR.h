#ifndef SENSOR_H
#define SENSOR_H

#include "mbed.h"
//*************************************************************
// definiciones de registros
//*************************************************************
//****************************************************
//sensor uv(veml6070)
//****************************************************
#define UV_ADDR_ARA (0x18)// para leer la parte alta
#define UV_ADDR_MO (0x70)// para leer la parte baja
#define UV_ADDR_H (0x39 << 1)// para leer la parte alta
#define UV_ADDR_L (0x38 << 1)// para leer la parte baja


//****************************************************
//sensor magnetometro(hmc5883l)
//****************************************************
#define mag_IDENT_A             0x0A // In this case the identification register A is used to identify the devide. ASCII value H
#define mag_I2C                 0x0D // 7-bit address. 0x3C write, 0x3D read.
#define mag_I2C_W               0x1A // Same as (& 0xFE), ensure that the MSB bit is being set to zero (RW=0 -> Writing)
#define mag_I2C_R               0x1B // Same as (| 0x01), ensure that the MSB bit is being set to one  (RW=1 -> Reading)

#define mag_CRA                 0x00 //configuracion de registro A
#define mag_CRB                 0x01 //configuracion de registro B
#define mag_MR                  0x02 //registro de modo del sensor
#define mag_SR                  0x09 //registro de estado

#define mag_DXRA                0x03 //registro alto de datos de X
#define mag_DXRB                0x04 //registro bajo de datos X
#define mag_DYRA                0x05 //registro alto de datos de Y
#define mag_DYRB                0x06 //registro bajo de datos Y
#define mag_DZRA                0x07 //registro alto de datos de Z
#define mag_DZRB                0x08 //registro bajo de datos Z
/*
//****************************************************
// sensor imu(mpu6050)
//****************************************************

//****************************************************
// conversor analogo/digital (ads1115)
//****************************************************
*/
class SENSOR
{
    public:
    SENSOR(PinName sda, PinName scl);  // definicion de los pines i2c por los cuales se accede.
    
//************************************************************
// definiciones de funciones publicas, a las que se puede acceder desde otro programa
//************************************************************

//****************************************************
//sensor uv(veml6070)
//****************************************************
    uint16_t getUV(void);
    void inicial(void);    

//****************************************************
//sensor magnetometro(hmc5883l)
//****************************************************
    uint16_t getMx(void);
    uint16_t getMy(void);
    uint16_t getMz(void);
/*
//****************************************************
// sensor imu(mpu6050)
//****************************************************
    uint16_t getGx(void);
    uint16_t getGy(void);
    uint16_t getGz(void);
    uint16_t getAx(void);
    uint16_t getAy(void);
    uint16_t getAz(void);
    uint16_t getTEM(void);
//****************************************************
// conversor analogo/digital (ads1115)
//****************************************************

*/
private:
//*************************************************************
// definiciones de funciones privada, solo para la libreria
//*************************************************************
char cmd[2];
I2C i2c;
char Read(char data);
void Write(char reg_address, char data);
//****************************************************
//sensor uv(veml6070)
//****************************************************

//****************************************************
//sensor magnetometro(hmc5883l)
//****************************************************

//****************************************************
// sensor imu(mpu6050)
//****************************************************

//****************************************************
// conversor analogo/digital (ads1115)
//****************************************************
};
#endif