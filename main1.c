/* I2C with tiny RTC ds1307 */

#include "i2c_stm32f103c.h"
#include "nvic.h"
#include "rcc_stm32f103c8.h"
#include "gpio_stm32f103c8.h"

/* DUMMY IRS handler */
void TIM6_IRQHandler(void){}
void USART1_IRQHandler(void){}
void TIM2_IRQHandler(void){}


#define DS1307_ADDR 0x68

void initI2C1(void) {
    I2C1->CR1 = 0;             // tắt để cấu hình
    I2C1->CR2 = 36;            // tần số bus APB1 = 36 MHz
    I2C1->CCR = 180;           // chế độ chuẩn (100 kHz)
    I2C1->TRISE = 37;          // = Freq + 1
    I2C1->CR1 |= (1 << 10);    // **BẬT ACK - QUAN TRỌNG!**
    I2C1->CR1 |= (1 << 0);     // bật I2C (PE)
}

// Thêm timeout để tránh treo
void i2c_sendAddr(uint8_t addr){
    I2C1->DR = addr;
    
    uint32_t timeout = 100000;
    while (!(I2C1->SR1 & (1 << 1)) && timeout--);  // đợi ADDR có timeout
    
    if (timeout == 0) {
        // Không có phản hồi - phát STOP
        I2C1->CR1 |= (1 << 9);
        return;
    }
    
    (void)I2C1->SR1;
    (void)I2C1->SR2;  // xóa cờ
}

void ds1307_readTime(uint8_t *buf){
    i2c_start();
    i2c_sendAddr(DS1307_ADDR << 1);  // chế độ ghi
    i2c_write(0x00);                 // thanh ghi bắt đầu 0x00
    
    i2c_start();                     // repeated start
    i2c_sendAddr((DS1307_ADDR << 1) | 1);  // chế độ đọc

    // Bật ACK cho tất cả byte trừ byte cuối
    I2C1->CR1 |= (1 << 10);
    
    for(int i=0; i<6; i++)
        buf[i] = i2c_read(1);   // đọc với ACK
    buf[6] = i2c_read(0);       // byte cuối NACK
    i2c_stop();
}


/*
PLLMUL = 9 
HSE ON
72MHz
*/
void initCLOCK(){
    /* HSE enable */
    RCC->CR |= (1 << 16);
    while (!(RCC->CR & (1 << 17)));

    /* CFGR PLLSRC use HSE */
    RCC->CFGR |= (1 << 16);
    RCC->CFGR &= ~(1 << 17);

    /* CFGR PLLMUL */
    RCC->CFGR |= (7 << 18);

    /* PLL enable */
    RCC->CR |= (1 << 24);
    while (!(RCC->CR & (1 << 25)));

    /* SW choose PLL as CLK */
    RCC->CFGR &= ~(0x3 << 0);  
    RCC->CFGR |= (0x2 << 0);
    while ((RCC->CFGR & (0x3 << 2)) != (0x2 << 2));

    /* Prescaler APB1, APB2, AHB*/
    RCC->CFGR |= (4 << 8);  //36MHz - APB1
    RCC->CFGR |= (0 << 11);  //72Mhz - APB2
    RCC->CFGR |= (0 << 4);  //72Mhz - AHB

    /* Enable clock for I2C and GPIOB */
    RCC->APB1ENR |= (1 << 21);
    RCC->APB2ENR |= (1 << 3 );
}

/*
PB6 - SCL
PB7 - SDA
*/
void initGPIO(){
    // //PB6, GPOIO output
    // GPIOB->CRL &= ~(0xF << (6 * 4));
    // GPIOB->CRL |=  (0x1 << (6 * 4)); //mode 01 (10Mhz), cnf 00

    // //PB7 input pull up/down
    // GPIOB->CRL &= ~(0xF << (7 * 4));
    // GPIOB->CRL |=  (0x8 << (7 * 4)); //mode 00, cnf 01
    // GPIOB->ODR |=  (1 << 7);         //pull up


    // PB6, PB7 -> AF open-drain, 50MHz
    GPIOB->CRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOB->CRL |=  ((0xB << (6 * 4)) | (0xB << (7 * 4))); 
    // 0xB = 1011b = MODE=11 (50MHz), CNF=10 (AF open-drain)
}

void initI2C1(void) {
    I2C1->CR1 = 0;             // disable để cấu hình
    I2C1->CR2 = 36;            // tần số bus APB1 = 36 MHz
    I2C1->CCR = 180;           // chế độ chuẩn (100 kHz)
    I2C1->TRISE = 37;          // = Freq + 1
    I2C1->CR1 |= (1 << 0);     // bật I2C (PE)
}


void main(){
    initCLOCK();
    initGPIO();
    initI2C1();


    uint8_t data[7];
    ds1307_readTime(data);

    volatile uint8_t sec  = bcd2dec(data[0]);
    // uint8_t min  = bcd2dec(data[1]);
    // uint8_t hour = bcd2dec(data[2]);
    // uint8_t day  = bcd2dec(data[3]);
    // uint8_t date = bcd2dec(data[4]);
    // uint8_t mon  = bcd2dec(data[5]);
    // uint8_t year = bcd2dec(data[6]);

    while (1){
        volatile uint8_t sec  = bcd2dec(data[0]);
    }

}