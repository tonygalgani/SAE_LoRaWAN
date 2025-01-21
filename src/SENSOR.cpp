# include "SENSOR.h"
# include "mbed.h"
SENSOR::SENSOR(PinName sda, PinName scl): i2c(sda, scl)
{      i2c.frequency(100000);  // configuracion de la velocidad del i2c para modo standart para los casos de uv y magnetometro
                               // para fast mode utilizar la freuencia de 400000 Hz, esto esta especificado en los datasheetZ de uv y magnetometro
                               
       inicial();
    }
void SENSOR::inicial()
{
    //******************************************************************************************************
    //INICIALIZACION DE UV, RESET AL ACK Y CONFIGURACION DEL MODO DE USO.
    //******************************************************************************************************
    int address;
    i2c.read(UV_ADDR_ARA,cmd,1);
    //printf("GGGG%dHHH",cmd);
    cmd[0]=0x06;            //reserva=00, ack=0, ackthr=0, tiempo de integracion=01,reserva=1, shhutdown=0 ==00000110=0x6h 
    i2c.write(UV_ADDR_MO,cmd,1,false);
    wait_ms(200);
    // configuracion del magnetometro
    //cmd[0]=mag_IDENT_A;
    
    cmd[0]=Read(mag_IDENT_A);
    printf("...%d...",cmd[0]);
    int iden=cmd[0];
    if(iden != 0x48){         
      printf("magnetometro no identificado %d\n",iden);
      }
//  else{
                //configuracion de registro A( velocidad de datos y medicion)
        Write(mag_CRA,0x70);
          //configuracion del registro B(ganancia del sensor)
        Write(mag_CRB,0x20);
               // configuracion del modo de funcionamiento
        Write(mag_MR,0x00);
        printf("magnetometro listo\n");       
  //    }
  
  
    //******************************************************************************************************
}
uint16_t SENSOR::getUV()
{
    uint16_t uvi;
    i2c.read(UV_ADDR_H, cmd,1);
    uvi = cmd[0]<<8;
    //printf(".....%d:::::",uvi);
    i2c.read(UV_ADDR_L, cmd,1,false);
   // printf("&&%d....",cmd);
    uvi |= cmd[0];
    return uvi;
}


char SENSOR::Read(char data)
{
    char tx = data;
    char rx;

    i2c.write(mag_I2C_W, &tx, 1);
    i2c.read(mag_I2C_R, &rx, 1);
    return rx;
}
void SENSOR::Write(char reg_address, char data)
{
    char tx[2];
    tx[0]=reg_address;
    tx[1]=data;

    i2c.write(mag_I2C_W,tx,2);
}
//MAGNETOMETRO EN TRES EJES
uint16_t SENSOR::getMx()
{
    uint16_t magx;
    //cmd[0]=mag_DXRA;
    cmd[0]=Read(mag_DXRA);
    magx=cmd[0]<<8;
    cmd[0]=Read(mag_DXRB);
    magx |=cmd[0];
    return magx;
}
uint16_t SENSOR::getMy()
{
    uint16_t magy;
    cmd[0]= Read(mag_DYRA);
    magy=cmd[0]<<8;
    cmd[0]= Read(mag_DYRB);
    magy |=cmd[0];
    return magy;
}
uint16_t SENSOR::getMz()
{
    uint16_t magz;
    cmd[0]=Read(mag_DZRA);
    magz=cmd[0]<<8;
    cmd[0]=Read(mag_DZRB);
    magz |=cmd[0];
    return magz;
}
