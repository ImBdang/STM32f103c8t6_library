#include "queu_mcu.h"


uint8_t isFull(Queu *queu){
    return ((queu->head + 1) % SIZE_QUEU) == queu->tail;
}

uint8_t isEmpty(Queu *q){
    return q->head == q->tail;
}

void enqueu(Queu *queu, uint8_t data){
    if (isFull(queu) != 0)
        return;
    queu->data[queu->head] = data;
    queu->head = (queu->head + 1) % SIZE_QUEU;
}

void dequeu(Queu *queu, uint8_t *hold){
    if (isEmpty(queu) == 1)
        return;
    *hold = queu->data[queu->tail];
    queu->tail = (queu->tail + 1) % SIZE_QUEU;
}