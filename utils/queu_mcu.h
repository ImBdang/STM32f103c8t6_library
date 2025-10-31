//static queu for stm32f103c8t6
#ifndef QUEU_MCU
#define QUEU_MCU

/* 
Test queu chua du lieu gui tu UART
Baudrate 9600, data 8N1 (10 bit)
SIZE_QUEU = baud_rate / 10 * t_process_max
Gia du t_process_max la 10ms ~ 0.01s vay thi size queu can 
de khong bi tran SRAM la 9600/10 * 0.01 ~= 9.6 bytes -> 10bytes
nhung ma 16 bytes cho dep
*/
#define SIZE_QUEU 16

#include <stdint.h>

typedef struct Queu{
    uint8_t data[SIZE_QUEU];
    volatile uint8_t head;
    volatile uint8_t tail;
} Queu;

uint8_t isFull(Queu *queu);
uint8_t isEmpty(Queu *q);
void enqueu(Queu *queu, uint8_t data);
void dequeu(Queu *queu, uint8_t *hold);
#endif
