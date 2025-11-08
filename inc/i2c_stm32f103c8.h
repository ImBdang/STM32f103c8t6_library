/*RM - rm0008, also ImBdang*/
#ifndef I2C_BDANG
#define I2C_BDANG

#include <stdint.h>

/*===============I2C REGISTER==============*/
typedef struct{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
} I2C_TypeDef;

#define I2C1 ((volatile I2C_TypeDef*)0x40005400)
#define I2C2 ((volatile I2C_TypeDef*)0x40005800)

#endif