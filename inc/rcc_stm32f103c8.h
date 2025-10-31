/*RM - rm0008, also ImBdang*/
#ifndef RCC_BDANG
#define RCC_BDANG

#include <stdint.h>

/*===============RCC REGISTER==============*/
typedef struct{
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
} RCC_TypeDef;

#define RCC ((volatile RCC_TypeDef*)0x40021000)

#endif