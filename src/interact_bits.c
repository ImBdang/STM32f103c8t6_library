#include "stdint.h"
void setbit(volatile uint32_t *reg, uint8_t bit, uint8_t mode){
    if (mode == 0)
        *reg &= ~(1 << bit);
    else
        *reg |= (1 << bit);
}