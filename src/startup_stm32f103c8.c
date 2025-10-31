#include <stdint.h>

#define STACK_TOP 0x20005000  // SRAM_START + SRAM_SIZE

extern int main(void);

void Reset_Handler(void);
void Default_Handler(void);

__attribute__((section(".isr_vector")))
uint32_t *vector_table[] = {
    (uint32_t *)STACK_TOP,
    (uint32_t *)Reset_Handler,
    (uint32_t *)Default_Handler, // NMI
    (uint32_t *)Default_Handler, // HardFault
    // ... các handler khác có thể thêm sau
};

void Default_Handler(void) { while(1); }

extern uint32_t _sdata, _edata, _etext;
extern uint32_t _sbss, _ebss;

void Reset_Handler(void) {
    uint32_t *src = &_etext;
    uint32_t *dst = &_sdata;
    while(dst < &_edata) *dst++ = *src++;

    dst = &_sbss;
    while(dst < &_ebss) *dst++ = 0;

    main();

    while(1);
}
