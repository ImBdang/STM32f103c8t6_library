#include "nvic.h"
#include "rcc_stm32f103c8.h"
#include "gpio_stm32f103c8.h"
#include "tim2345_stm32f103c8.h"

/*
module color sensor GY-31
S2 - A15
S3 - A12
OUT -A0 TIMER2 INPUT CAPTURE
ID TIM2 INTERUPT 28
*/

/* Interupt func */

volatile uint32_t last_capture = 0;
volatile uint32_t pulse_width = 0;
volatile uint32_t freq_hz = 0;

void USART1_IRQHandler(void) {}
void TIM6_IRQHandler(void) {}

char* get_color(uint32_t freq){
    if(freq < 15000) return "RED";
    else if(freq < 25000) return "GREEN";
    else return "BLUE";
}

void TIM2_IRQHandler(void){
    if (TIM2->SR & (1 << 1)) {
        uint32_t current_capture = TIM2->CCR1;
        TIM2->SR &= ~(1 << 1); 
        pulse_width = current_capture - last_capture;
        last_capture = current_capture;

        if (pulse_width != 0)
            freq_hz = 72000000 / pulse_width;

        char* color = get_color(freq_hz);
        __asm__("nop"); 
    }
}


void initCLOCK(){
    /* khoi tao va set PLLSRC 72Mhz */
    RCC->CR |= (1 << 16);
    while (!(RCC->CR & (1 << 17)));
    RCC->CFGR |= (7 << 18);
    RCC->CFGR |= (1 << 16);
    RCC->CR |= (1 << 24); //PLL on
    while (!(RCC->CR & (1 << 25))); //cho pll 
    RCC->CFGR = (RCC->CFGR & ~3) | 2;  //SYSCLK = PLL
    while (((RCC->CFGR >> 2) & 3) != 2); 

    /* Enable bus */
    RCC->APB2ENR |= (1 << 2);//GPIOA
    RCC->APB1ENR |= (1 << 0);//TIMER2
}

void initGPIO(){
    GPIOA->CRL &= ~(0xF << 0);   

    GPIOA->CRH &= ~(0xF << 16); // clear A12
    GPIOA->CRH |= (0x1 << 16);  // output mode

    GPIOA->CRH &= ~(0xF << 28); // clear A15
    GPIOA->CRH |= (0x1 << 28);  // output mode

    /* A0 */
    GPIOA->CRL &= ~(0xF << 0);
    GPIOA->CRL |= (0x4 << 0);  
}


void initTIMER2(){
    /* clear */
    TIM2->CCMR1 = 0;
    TIM2->CCER = 0;

    /* Config */
    TIM2->CCMR1 &= ~(3 << 0); // clear CC1S 
    TIM2->CCMR1 |= (1 << 0);  // CC1S = 01 (TI1)
    TIM2->CCMR1 |= (0x5 << 4); // IC1F 0101
    TIM2->CCER |= (1 << 0); //CC1E = 1, CC1P = 0;

    TIM2->CCMR1 |= (0 << 2); //IC1PSC = 00

    TIM2->DIER |= (1 << 1);

    TIM2->ARR = 0xFFFFFFFF; // tránh overflow sớm
    TIM2->PSC = 0;           // prescaler = 0 (full speed)

    TIM2->CR1 |= (1 << 0);
}

void initINTERTUP(){
    NVIC->ISER0 |= (1 << 28);
    NVIC->IPR7 &= ~(0xFF << 0); //xoa byte offset 2 cua IPR13
    NVIC->IPR7 |= (10 << 0);
}

// 0x4000000C - DIER
// 0x40000010 - SR

void main(){
    initCLOCK();
    initGPIO();
    initTIMER2();

    initINTERTUP();
    __asm volatile ("cpsie i");
    while (1) {
        if (TIM2->SR & (1 << 1)) {
            uint32_t current_capture = TIM2->CCR1; //while func
            TIM2->SR &= ~(1 << 1); 
            pulse_width = current_capture - last_capture;
            last_capture = current_capture;

            if (pulse_width != 0)
                freq_hz = 72000000 / pulse_width;

            char* color = get_color(freq_hz); 
            __asm__("nop"); 
        }
    }
}