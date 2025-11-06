/*RM - rm0008, also ImBdang*/
#ifndef SPI_BDANG
#define SPI_BDANG

#include <stdint.h>

/*===============SPI REGISTER==============*/
typedef struct{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
} SPI_TypeDef;

#define SPI1 ((volatile SPI_TypeDef*)0x40013000)
#define SPI2 ((volatile SPI_TypeDef*)0x40003800)
#define SPI3 ((volatile SPI_TypeDef*)0x40003C00)

#endif