/*RM - rm0008, also ImBdang*/
#ifndef FLASH_BDANG
#define FLASH_BDANG

#include <stdint.h>

/*===============FLASH REGISTER==============*/
typedef struct{
    volatile uint32_t ACR;
} FLASH_TypeDef;

#define FLASH ((volatile FLASH_TypeDef*)0x40022000)

#endif