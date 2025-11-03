#include "buffer_queu_uart.h"
#include "rcc_stm32f103c8.h"
#include "usart_stm32f103c8.h"
#include "gpio_stm32f103c8.h"
#include "interact_bits.h"
#include "tim67_stm32f103c8.h"
#include "nvic.h"

volatile Queu rx_queu;
volatile Queu tx_queu;

//USARTDIV = 39.0625, baud 115200, PCLK2 = 72MHz
//8N1

/* USART Polling style */
uint8_t usart1_receive(void) {
    while (!(USART1->SR & (1 << 5)));  
    return USART1->DR;                
}

void USART1_sendChar(char c) {
    while (!(USART1->SR & (1 << 7))); //TXE ok
    USART1->DR = c;
}

void USART1_sendString(const char *s) {
    while (*s) {
        USART1_sendChar(*s++);
    }
}
/* ========================================== */

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
    RCC->APB2ENR |= (1 << 14); //USART1
    RCC->APB2ENR |= (1 << 4); //GPIOC
    RCC->APB2ENR |= (1 << 2);//GPIOA
}

void initUART(){
    /* RESET register */
    USART1->BRR = 0;
    USART1->CR1 = 0;   
    USART1->CR2 = 0;
    USART1->CR3 = 0;  
    USART1->GTPR = 0; 

    /* SET baudrate 115200 */
    USART1->BRR = (39 << 4);
    USART1->BRR |= 1; //0.0625*16=1

    /* CR1 CFG */    
    USART1->CR1 |= (1 << 13); //enable usart1
    USART1->CR1 |= (0 << 12); //1 bit start, 8 bit data
    USART1->CR1 |= (0 << 11); //WAKE method Idle line
    USART1->CR1 |= (0 << 10); //parity disable
    USART1->CR1 |= (1 << 5); //RXNEIE
    USART1->CR1 |= (1 << 7); //TXEIE
    USART1->CR1 |= (1 << 3); //trans enba
    USART1->CR1 |= (1 << 2); //re enb

    /* CR2 CFG */
    USART1->CR2 |= (0 << 12); //1 bit stop

    //set mode alternate PA9 10
    //PA9 TX
    GPIOA->CRH &= ~(0xF << 4); 
    GPIOA->CRH |=  (0xB << 4);//1011
    
    //PA10 RX
    GPIOA->CRH &= ~(0xF << 8); 
    GPIOA->CRH |=  (0x4 << 8);//0100
}

void initLED13(){
    //config led PC13
    GPIOC->CRH &= ~(0xF << 20);
    GPIOC->CRH |= (1 << 20);
    setbit(&GPIOC->ODR, 13, 1); //tat led PC13 (active low)
}

/* USART1 id 37 */
void initINTERTUP(){
    NVIC->ISER1 |= (1 << 5);
    NVIC->IPR9 &= ~(0xFF << 8); //xoa byte offset 2 cua IPR13
    NVIC->IPR9 |= (10 << 8);
}

// void USART1_IRQHandler(void) {
//     GPIOC->ODR ^= (1 << 13); 
//     if (USART1->SR & (1 << 5)) {  
//         uint8_t temp = USART1->DR;
//         if (!isFull(&rx_queu)){
//             enqueu(&rx_queu, temp);
//         }
//     }
// }


void USART1_IRQHandler(void) {
    /* Check neu co du lieu nhan */
    if (USART1->SR & (1 << 5)) {
        uint8_t data = USART1->DR;
        enqueu(&rx_queu, data);
    }
    /* Check neu co du lieu can gui */
    if ((USART1->SR & (1 << 7)) && (USART1->CR1 & (1 << 7))) {
        if (!isEmpty(&tx_queu)) {
            uint8_t byte;
            uint8_t status = dequeu(&tx_queu, &byte);
            if (status){
                USART1->DR = byte;
            }
        } else {
            USART1->CR1 &= ~(1 << 7); 
        }
    }
}

void TIM6_IRQHandler(void) {
    if (TIM6->SR & 1) { 
        TIM6->SR &= ~(1 << 0);
        GPIOC->ODR ^= (1 << 13);
    }
    TIM6->SR &= ~(1 << 0);
}


void USART_sendcharInterupt(char c){
    enqueu(&tx_queu, c);
    USART1->CR1 |= (1 << 7); // Bật TXE interrupt
}



void main(){
    initCLOCK();
    initUART();
    initINTERTUP();
    initLED13();

    tx_queu.head=0;
    tx_queu.tail=0;

    rx_queu.head=0;
    rx_queu.tail=0;
    __asm volatile ("cpsie i");

    /* Send chuoi string co ban */
    const char *msg = "hello\n";
    while (*msg) {
        while (isFull(&tx_queu));
        USART_sendcharInterupt(*msg++);
    }
    
    while (1){

    }
}