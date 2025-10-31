/*RM - rm0008, also ImBdang*/
#ifndef INTERACT_BDANG
#define INTERACT_BDANG

#include <stdint.h>

//ham nay set bit duoc chon len 1
void setbit(volatile uint32_t *reg, uint8_t bit, uint8_t mode);

#endif