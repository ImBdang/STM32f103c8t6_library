#include <stdint.h>

#define STACK_TOP 0x20005000  // SRAM_START + SRAM_SIZE

extern int main(void);

void Reset_Handler(void);
void Default_Handler(void);
void TIM6_IRQHandler(void);
void USART1_IRQHandler(void);
void TIM2_IRQHandler(void);

__attribute__((section(".isr_vector")))
void (* const vector_table[])(void) = {
    (void (*)(void))STACK_TOP,    // 0: MSP
    Reset_Handler,                // 1: Reset
    Default_Handler,              // 2: NMI
    Default_Handler,              // 3: HardFault
    Default_Handler,              // 4: MemManage (if present)
    Default_Handler,              // 5: BusFault
    Default_Handler,              // 6: UsageFault
    0,0,0,0,                      // reserved
    Default_Handler,              // 11: SVCall
    Default_Handler,              // 12: DebugMonitor
    0,                            // reserved
    Default_Handler,              // 14: PendSV
    Default_Handler,              // 15: SysTick
    /* External IRQs start here: index = 16 + IRQn */
    Default_Handler, // IRQ0 WWDG
    Default_Handler, // IRQ1 PVD
    Default_Handler, // IRQ2 TAMPER
    Default_Handler, // IRQ3 RTC
    Default_Handler, // IRQ4 FLASH
    Default_Handler, // IRQ5 RCC
    Default_Handler, // IRQ6 EXTI0
    Default_Handler, // IRQ7 EXTI1
    Default_Handler, // IRQ8 EXTI2
    Default_Handler, // IRQ9 EXTI3
    Default_Handler, // IRQ10 EXTI4
    Default_Handler, // IRQ11 DMA1_Channel1
    Default_Handler, // IRQ12 DMA1_Channel2
    Default_Handler, // IRQ13 DMA1_Channel3
    Default_Handler, // IRQ14 DMA1_Channel4
    Default_Handler, // IRQ15 DMA1_Channel5
    Default_Handler, // IRQ16 DMA1_Channel6
    Default_Handler, // IRQ17 DMA1_Channel7
    Default_Handler, // IRQ18 ADC1_2
    Default_Handler, // IRQ19 USB_HP_CAN_TX
    Default_Handler, // IRQ20 USB_LP_CAN_RX0
    Default_Handler, // IRQ21 CAN_RX1
    Default_Handler, // IRQ22 CAN_SCE
    Default_Handler, // IRQ23 EXTI9_5
    Default_Handler, // IRQ24 TIM1_BRK
    Default_Handler, // IRQ25 TIM1_UP
    Default_Handler, // IRQ26 TIM1_TRG_COM
    Default_Handler, // IRQ27 TIM1_CC
    TIM2_IRQHandler, // IRQ28 TIM2
    Default_Handler, // IRQ29 TIM3
    Default_Handler, // IRQ30 TIM4
    Default_Handler, // IRQ31 I2C1_EV
    Default_Handler, // IRQ32 I2C1_ER
    Default_Handler, // IRQ33 I2C2_EV
    Default_Handler, // IRQ34 I2C2_ER
    Default_Handler, // IRQ35 SPI1
    Default_Handler, // IRQ36 SPI2
    USART1_IRQHandler, // IRQ37 USART1
    Default_Handler, // IRQ38 USART2
    Default_Handler, // IRQ39 USART3
    Default_Handler, // IRQ40 EXTI15_10
    Default_Handler, // IRQ41 RTCAlarm
    Default_Handler, // IRQ42 USBWakeup
    Default_Handler, // IRQ43 TIM8_BRK
    Default_Handler, // IRQ44 TIM8_UP
    Default_Handler, // IRQ45 TIM8_TRG_COM
    Default_Handler, // IRQ46 TIM8_CC
    Default_Handler, // IRQ47 ADC3
    Default_Handler, // IRQ48 FSMC
    Default_Handler, // IRQ49 SDIO
    Default_Handler, // IRQ50 TIM5
    Default_Handler, // IRQ51 SPI3
    Default_Handler, // IRQ52 UART4
    Default_Handler, // IRQ53 UART5
    TIM6_IRQHandler, // IRQ54 TIM6 (or TIM6_DAC)
    /* ... tiếp tục cho đủ số IRQ phần cứng MCU đó */
};

extern uint32_t _sdata, _edata, _etext;
extern uint32_t _sbss, _ebss;

void Default_Handler(void) { while(1); }

void Reset_Handler(void) {
    uint32_t *src = &_etext;
    uint32_t *dst = &_sdata;
    while(dst < &_edata) *dst++ = *src++;

    dst = &_sbss;
    while(dst < &_ebss) *dst++ = 0;

    main();

    while(1);
}
