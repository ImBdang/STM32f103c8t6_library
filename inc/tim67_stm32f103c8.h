/*RM - rm0008, also ImBdang*/
#ifndef TIM67_BDANG
#define TIM67_BDANG

#include <stdint.h>

/*===============TIM6/7 REGISTER==============*/
typedef struct{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t RESERVED0;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t RESERVED1;
    volatile uint32_t RESERVED2;
    volatile uint32_t RESERVED3;
    volatile uint32_t CNT;
    volatile uint32_t PCS;
    volatile uint32_t ARR;
} TIM67_TypeDef;

#define TIM6 ((volatile TIM67_TypeDef*)0x40001000)
#define TIM7 ((volatile TIM67_TypeDef*)0x40001400)

#endif