/* SPI1 community with ESP32 */

#include "nvic.h"
#include "rcc_stm32f103c8.h"
#include "gpio_stm32f103c8.h"
#include "spi_stm32f103c8.h"

/* DUMMY IRS handler */
void TIM6_IRQHandler(void){}
void USART1_IRQHandler(void){}


/*
PLLMUL = 9 
HSE ON
72MHz
*/
void initCLOCK(){
    /* HSE enable */
    RCC->CR |= (1 << 16);
    while (!(RCC->CR & (1 << 17)));

    /* CFGR PLLSRC use HSE */
    RCC->CFGR |= (1 << 16);
    RCC->CFGR &= ~(1 << 17);

    /* CFGR PLLMUL */
    RCC->CFGR |= (7 << 18);

    /* PLL enable */
    RCC->CR |= (1 << 24);
    while (!(RCC->CR & (1 << 25)));

    /* SW choose PLL as CLK */
    RCC->CFGR &= ~(0x3 << 0);  
    RCC->CFGR |= (0x2 << 0);
    while ((RCC->CFGR & (0x3 << 2)) != (0x2 << 2));

    /* Enable clock for SPI1 and GPIOA */
    RCC->APB2ENR |= (1 << 12);
    RCC->APB2ENR |= (1 << 2 );
}

/*
PA4 - NSS - AF output push pull
PA5 - SCK - AF output push pull
PA6 - MISO - input floating 
PA7 - MOSI - AF output push pull 
*/
void initGPIO(){

}




void main(){
     
}