#include "rcc_stm32f103c8.h"

void RCC_HSE_On(void){
    RCC->CR |= (1 << 16);
}

void RCC_HSE_Off(void){
    RCC->CR |= (0 << 16);
}

void RCC_HSI_On(void){
    RCC->CR |= (1 << 0);
}

void RCC_HSI_Off(void){
    RCC->CR |= (1 << 0);
}

void RCC_PLL_On(void){
    RCC->CR |= (1 << 24);
}
void RCC_PLL_Off(void){
    RCC->CR |= (0 << 24);
}

uint8_t RCC_HSE_Ready(void){
    return (RCC->CR & (1 << 17));
}

uint8_t RCC_HSI_Ready(void){
    return (RCC->CR & (1 << 1));
}

uint8_t RCC_PLL_Ready(void){
    return (RCC->CR & (1 << 25));
}

void RCC_Select_SYSCLK_HSE(void){
    RCC->CFGR &= ~(0x3);
    RCC->CFGR |= (1 << 0);
}

void RCC_Select_SYSCLK_HSI(void){
    RCC->CFGR &= ~(0x3);
    RCC->CFGR |= (0 << 0);
}

void RCC_Select_SYSCLK_PLL(void){
    RCC->CFGR &= ~(0x3);
    RCC->CFGR |= (2 << 0);
}

uint8_t RCC_Get_SYSCLK_Source(void){
    return (RCC->CFGR >> 2) & 0x3;
}

void RCC_APB2_Reset(uint32_t mask){
    RCC->APB2RSTR |= mask;
    RCC->APB2RSTR &= ~mask;

}

void RCC_APB1_Reset(uint32_t mask){
    RCC->APB1RSTR |= mask;
    RCC->APB1RSTR &= ~mask;
}