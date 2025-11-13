/*RM - rm0008, also ImBdang*/
#ifndef FLASH_BDANG
#define FLASH_BDANG

#include <stdint.h>

/*===============FLASH REGISTER==============*/
typedef struct {
    volatile uint32_t ACR;       
    volatile uint32_t KEYR;      
    volatile uint32_t OPTKEYR;   
    volatile uint32_t SR;       
    volatile uint32_t CR;   
    volatile uint32_t AR;       
    uint32_t RESERVED;           
    volatile uint32_t OBR;       
    volatile uint32_t WRPR;      
} FLASH_TypeDef;

#define FLASH ((volatile FLASH_TypeDef*)0x40022000)


#endif