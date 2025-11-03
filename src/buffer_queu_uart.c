#include "buffer_queu_uart.h"

uint8_t isFull(volatile Queu *queu){
    return ((queu->head + 1) % SIZE_QUEU) == queu->tail;
}

uint8_t isEmpty(volatile Queu *q){
    return q->head == q->tail;
}

void enqueu(volatile Queu *queu, uint8_t data){
    if (isFull(queu) != 0)
        return;
    queu->data[queu->head] = data;
    queu->head = (queu->head + 1) % SIZE_QUEU;
}

uint8_t dequeu(volatile Queu *queu, uint8_t *hold){
    if (isEmpty(queu) == 1)
        return 0;
    *hold = queu->data[queu->tail];
    queu->tail = (queu->tail + 1) % SIZE_QUEU;
    return 1;
}