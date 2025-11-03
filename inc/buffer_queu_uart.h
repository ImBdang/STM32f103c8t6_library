//static queu for stm32f103c8t6
#ifndef BUFFER_QUEU_UART_H
#define BUFFER_QUEU_UART_H

#define SIZE_QUEU 16

#include <stdint.h>

typedef struct Queu{
    volatile uint8_t data[SIZE_QUEU];
    volatile uint8_t head;
    volatile uint8_t tail;
} Queu;

uint8_t isFull(volatile Queu *queu);
uint8_t isEmpty(volatile Queu *q);
void enqueu(volatile Queu *queu, uint8_t data);
uint8_t dequeu(volatile Queu *queu, uint8_t *hold);

#endif
