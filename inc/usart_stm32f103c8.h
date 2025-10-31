/*RM - rm0008, also ImBdang*/
#ifndef USART_BDANG
#define USART_BDANG

#include <stdint.h>

/*===============RCC REGISTER==============*/
typedef struct{
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

#define USART1 ((volatile USART_TypeDef*)0x40013800)
#define USART2 ((volatile USART_TypeDef*)0x40004400)
#define USART3 ((volatile USART_TypeDef*)0x40004800)
#define UART4 ((volatile USART_TypeDef*)0x40004C00)
#define UART5 ((volatile USART_TypeDef*)0x40005000)

#endif