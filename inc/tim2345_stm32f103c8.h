/*RM - rm0008, also ImBdang*/
#ifndef TIM2345_BDANG
#define TIM2345_BDANG

#include <stdint.h>

/*===============TIM6/7 REGISTER==============*/
typedef struct{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RESERVED1;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t RESERVED2;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;    
} TIM2345_TypeDef;

#define TIM2 ((volatile TIM2345_TypeDef*)0x40000000)
#define TIM3 ((volatile TIM2345_TypeDef*)0x40000400)
#define TIM4 ((volatile TIM2345_TypeDef*)0x40000800)
#define TIM5 ((volatile TIM2345_TypeDef*)0x40000C00)

#endif