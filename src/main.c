#include "gpio_stm32f103c8.h"
#include "rcc_stm32f103c8.h"
#include "tim67_stm32f103c8.h"
#include "nvic.h"

void delay_1s(void){
    TIM6->CNT = 0;
    TIM6->SR &= ~(1 << 0);
    while (!(TIM6->SR & (1 << 0)));
    TIM6->SR &= ~(1 << 0);
}

void init(void){
    RCC->APB1ENR = 0x0;
    RCC->APB2ENR = 0x0;
    RCC->CFGR &= ~(0xF << 18);

    //HSE ON
    RCC->CR |= (1 << 16);
    while (!(RCC->CR & (1 << 17)));

    //Make sure that HSE is not devided by 2
    RCC->CFGR &= ~(1 << 17);

    //HSE as PLL SRC
    RCC->CFGR |= (1 << 16);

    //PLL MUL
    RCC->CFGR |= (7 << 18);

    //PLL ON
    RCC->CR |= (1 << 24);
    while (!(RCC->CR & (1 << 25)));

    //PLL as SYSCLK
    RCC->CFGR |= (2 << 0);

    //Set 36MHz for APB1
    RCC->CFGR |= (4 << 8);

    //Enable GPIOC
    RCC->APB2ENR |= (1 << 4);

    //set mode PC13, blink led on board
    GPIOC->CRH &= ~(0xF << 20); 
    GPIOC->CRH |= (2 << 20);
    GPIOC->BRR = (1 << 13);

    //Enable Timer6
    RCC->APB1ENR |= (1 << 4);

    //config TIM6
    TIM6->CR1 = 0;
    TIM6->CR2 = 0;
    TIM6->CR1 &= ~(1 << 7); //not buffered for ARR
    TIM6->CR1 &= ~(1 << 3); //one pulse mode off
    TIM6->CR1 |= (1 << 2); //only counter over/under flow generate event
    TIM6->CR1 &= ~(1 << 1); //UDIS disable
    TIM6->DIER |= (1 << 0); //enable UIE for interupt
    TIM6->EGR |= (1 << 0);
    TIM6->PSC = 3599;
    TIM6->ARR = 9999;
}

void TIM6_IRQHandler(void) {
    if(TIM6->SR & 1) { // UIF flag
        TIM6->SR &= ~(1 << 0); // xóa flag
        GPIOC->ODR ^= (1 << 13);
    }
}

void initInterupt(void){
    NVIC->ISER1 |= (1 << 22);
    NVIC->IPR13 &= ~(0xFF << 16); //xoa byte offset 2 cua IPR13
    NVIC->IPR13 |= (10 << 20);
}

void main(void){
    init();
    initInterupt();
    void (*check)(void) = TIM6_IRQHandler;

    //TIM6 interupt id = 54
    //54/32 = 1 (ISER1)
    //54%32 = 22 (bit 22 trong ISER1)
    
    //54/4 = 13 (IPR13)
    //54%4 = 2 (bit 2 trong IPR13) (byte offset)
    //shift = byte_offset*8 + 4 = 2*8 + 4 = 20
    //thanh ghi IPR la 32bit, chi dung 4 bit high

    TIM6->CR1 |= (1 << 0); //enable counter
    __asm volatile ("cpsie i");
    while (1){

    }


}