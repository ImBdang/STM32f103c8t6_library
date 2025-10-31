#include "gpio_stm32f103c8.h"
#include "rcc_stm32f103c8.h"
#include "interact_bit.h"

void main(void){
    RCC->APB2ENR |= (3 << 3);

    GPIOC->CRH &= ~(0xF << 20);
    GPIOC->CRH |= (1 << 20);
    GPIOC->ODR |= (0 << 13);


    GPIOB->CRH &= ~(0xF << 8);
    GPIOB->CRH |= (1 << 8);
    GPIOB->ODR |= (0 << 10);
    setbit(&GPIOB->ODR, 10, 0);
    while (1){

    }
}