/*RM - rm0008, also ImBdang*/
#ifndef GPIO_BDANG
#define GPIO_BDANG

#include <stdint.h>

/*===============GPIO REGISTER==============*/
typedef struct{
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;
} GPIO_TypeDef;

#define GPIOA ((volatile GPIO_TypeDef*)0x40010800)
#define GPIOB ((volatile GPIO_TypeDef*)0x40010C00)
#define GPIOC ((volatile GPIO_TypeDef*)0x40011000)
#define GPIOD ((volatile GPIO_TypeDef*)0x40011400)
#define GPIOE ((volatile GPIO_TypeDef*)0x40011800)
#define GPIOF ((volatile GPIO_TypeDef*)0x40011C00)
#define GPIOG ((volatile GPIO_TypeDef*)0x40012000)

#endif