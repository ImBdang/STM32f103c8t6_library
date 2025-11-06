/*RM - rm0008, also ImBdang*/
#ifndef AFIO_BDANG
#define AFIO_BDANG

#include <stdint.h>

/*===============SPI REGISTER==============*/
typedef struct{
    volatile uint32_t EVCR;
    volatile uint32_t MAPR;
    volatile uint32_t EXTICR1;
    volatile uint32_t EXTICR2;
    volatile uint32_t EXTICR3;
    volatile uint32_t EXTICR4;
    volatile uint32_t MAPR2;
} AFIO_TypeDef;

#define AFIO ((volatile AFIO_TypeDef*)0x40010000)

#endif