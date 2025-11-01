#include "gpio_stm32f103c8.h"
#include "rcc_stm32f103c8.h"
#include "interact_bits.h"

void main(void){
    RCC->APB2ENR = 0x0;

    //HSE ON
    RCC->CR |= (1 << 16);
    while (!(RCC->CR & (1 << 17)));

    //HSE as PLL SRC
    RCC->CFGR |= (1 << 16);

    //PLL MUL
    RCC->CFGR |= (7 << 18);

    //PLL ON
    RCC->CR |= (1 << 24);
    while (!(RCC->CR & (1 << 25)));

    //PLL as SYSCLK
    RCC->CFGR |= (2 << 0);


    RCC->CR &= ~(1 << 0);
    while (RCC->CR & (1 << 1));

}