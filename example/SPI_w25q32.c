/* 
SPI1 communication with W25Q32 
STM32F103C8T6 - Bare metal basic example
Ghi chuỗi "bdang" vào địa chỉ 0x000000
*/

#include "rcc_stm32f103c8.h"
#include "gpio_stm32f103c8.h"
#include "spi_stm32f103c8.h"
#include "flash_stm32f103c8t6.h"

/* DUMMY IRQ handlers */
void TIM6_IRQHandler(void){}
void USART1_IRQHandler(void){}
void TIM2_IRQHandler(void){}

/* ============= W25Q32 Commands ============= */
#define W25Q_READ_DATA          0x03
#define W25Q_WRITE_ENABLE       0x06
#define W25Q_WRITE_DISABLE      0x04
#define W25Q_READ_STATUS_REG1   0x05
#define W25Q_PAGE_PROGRAM       0x02
#define W25Q_SECTOR_ERASE       0x20
#define W25Q_READ_JEDEC_ID      0x9F

/* ============= Delay Functions ============= */
void delay_ms(uint32_t ms) {
    for(uint32_t i = 0; i < ms * 8000; i++) {
        __asm("nop");
    }
}

void delay_us(uint32_t us) {
    for(uint32_t i = 0; i < us * 8; i++) {
        __asm("nop");
    }
}

/* ============= SPI Functions ============= */
// Gửi và nhận 1 byte qua SPI
uint8_t SPI1_Transfer(uint8_t data) {
    // Đợi TX buffer trống
    while(!(SPI1->SR & (1 << 1)));
    SPI1->DR = data;
    // Đợi nhận xong
    while(!(SPI1->SR & (1 << 0)));
    return SPI1->DR;
}

// Kéo CS xuống thấp (chọn chip)
void W25Q_CS_Low(void) {
    GPIOA->BRR = (1 << 4);
    delay_us(1);
}

// Kéo CS lên cao (bỏ chọn chip)
void W25Q_CS_High(void) {
    delay_us(1);
    GPIOA->BSRR = (1 << 4);
    delay_us(1);
}

/* ============= W25Q32 Basic Functions ============= */

// Đọc Status Register 1
uint8_t W25Q_ReadStatusReg1(void) {
    uint8_t status;
    W25Q_CS_Low();
    SPI1_Transfer(W25Q_READ_STATUS_REG1);
    status = SPI1_Transfer(0x00);
    W25Q_CS_High();
    return status;
}

// Đợi cho đến khi chip không bận (BUSY bit = 0)
void W25Q_WaitBusy(void) {
    while(W25Q_ReadStatusReg1() & 0x01) {
        delay_ms(1);
    }
}

// Gửi lệnh Write Enable
void W25Q_WriteEnable(void) {
    W25Q_CS_Low();
    SPI1_Transfer(W25Q_WRITE_ENABLE);
    W25Q_CS_High();
    delay_us(10);
}

// Đọc JEDEC ID (để kiểm tra kết nối)
void W25Q_ReadJEDEC_ID(uint8_t *manufacturer, uint8_t *device_id) {
    W25Q_CS_Low();
    SPI1_Transfer(W25Q_READ_JEDEC_ID);
    *manufacturer = SPI1_Transfer(0x00);
    SPI1_Transfer(0x00); // Memory Type
    *device_id = SPI1_Transfer(0x00);
    W25Q_CS_High();
}

// Xóa 1 sector (4KB) - BẮT BUỘC trước khi ghi
void W25Q_EraseSector(uint32_t address) {
    W25Q_WaitBusy();
    W25Q_WriteEnable();
    
    W25Q_CS_Low();
    SPI1_Transfer(W25Q_SECTOR_ERASE);
    SPI1_Transfer((address >> 16) & 0xFF); // A23-A16
    SPI1_Transfer((address >> 8) & 0xFF);  // A15-A8
    SPI1_Transfer(address & 0xFF);         // A7-A0
    W25Q_CS_High();
    
    W25Q_WaitBusy(); // Đợi xóa xong (thường mất 30-400ms)
}

// Ghi 1 trang (tối đa 256 bytes)
void W25Q_WritePage(uint32_t address, uint8_t *data, uint16_t size) {
    if(size > 256) size = 256; // Giới hạn 256 bytes/page
    
    W25Q_WaitBusy();
    W25Q_WriteEnable();
    
    W25Q_CS_Low();
    SPI1_Transfer(W25Q_PAGE_PROGRAM);
    SPI1_Transfer((address >> 16) & 0xFF);
    SPI1_Transfer((address >> 8) & 0xFF);
    SPI1_Transfer(address & 0xFF);
    
    // Ghi từng byte
    for(uint16_t i = 0; i < size; i++) {
        SPI1_Transfer(data[i]);
    }
    
    W25Q_CS_High();
    W25Q_WaitBusy(); // Đợi ghi xong
}

// Đọc dữ liệu từ địa chỉ bất kỳ
void W25Q_ReadData(uint32_t address, uint8_t *buffer, uint32_t size) {
    W25Q_WaitBusy();
    
    W25Q_CS_Low();
    SPI1_Transfer(W25Q_READ_DATA);
    SPI1_Transfer((address >> 16) & 0xFF);
    SPI1_Transfer((address >> 8) & 0xFF);
    SPI1_Transfer(address & 0xFF);
    
    // Đọc từng byte
    for(uint32_t i = 0; i < size; i++) {
        buffer[i] = SPI1_Transfer(0x00);
    }
    
    W25Q_CS_High();
}

/* ============= Clock & Peripheral Init ============= */
void initCLOCK(){
    RCC->CR |= (1 << 16);
    while (!(RCC->CR & (1 << 17)));
    RCC->CFGR &= ~(1 << 17);
    RCC->CFGR |= (1 << 16);
    RCC->CFGR |= (7 << 18);
    RCC->CR |= (1 << 24);
    while (!(RCC->CR & (1 << 25)));
    RCC->CFGR |= (4 << 8); 
    RCC->CFGR &= ~(0x3 << 0);  
    RCC->CFGR |= (0x2 << 0);
    while ((RCC->CFGR & (0x3 << 2)) != (0x2 << 2));
}

void initGPIO(){
    RCC->APB2ENR |= (1 << 2);
    GPIOA->CRL &= ~(0xF << 16);
    GPIOA->CRL |= (0x3 << 16);
    GPIOA->BSRR = (1 << 4);
    GPIOA->CRL &= ~(0xF << 20);
    GPIOA->CRL |= (0xB << 20);
    GPIOA->CRL &= ~(0xF << 24);
    GPIOA->CRL |= (0x4 << 24);
    GPIOA->CRL &= ~(0xF << 28);
    GPIOA->CRL |= (0xB << 28);
}

void initSPI1(){
    RCC->APB2ENR |= (1 << 12);
    SPI1->CR1 = 0;
    SPI1->CR1 |= (2 << 3);
    SPI1->CR1 &= ~(1 << 0);
    SPI1->CR1 &= ~(1 << 1);
    SPI1->CR1 &= ~(1 << 11);
    SPI1->CR1 &= ~(1 << 7);
    SPI1->CR1 |= (1 << 9);
    SPI1->CR1 |= (1 << 8);
    SPI1->CR1 |= (1 << 2);
    SPI1->CR1 |= (1 << 6);
}

void initFLASH(){
    FLASH->ACR &= ~(0x7 << 0);  
    FLASH->ACR |= (2 << 0);  
}

/* ============= MAIN ============= */
void main(){
    initFLASH();
    initCLOCK();
    initGPIO();
    initSPI1();
    
    delay_ms(100); // Đợi W25Q32 khởi động
    
    // Kiểm tra kết nối
    uint8_t manufacturer, device_id;
    W25Q_ReadJEDEC_ID(&manufacturer, &device_id);
    // W25Q32: manufacturer = 0xEF, device_id = 0x15
    
    // Chuỗi cần ghi
    uint8_t data_write[] = "bdang";
    uint8_t data_read[6] = {0}; // Buffer đọc (5 ký tự + null)
    
    uint32_t address = 0x000000; // Địa chỉ ghi
    
    // Bước 1: Xóa sector (BẮT BUỘC)
    W25Q_EraseSector(address);
    
    // Bước 2: Ghi dữ liệu
    W25Q_WritePage(address, data_write, 5);
    
    // Bước 3: Đọc lại để kiểm tra
    W25Q_ReadData(address, data_read, 5);
    
    // Vòng lặp vô hạn
    while(1) {
        delay_ms(1000);
        // Có thể đọc lại liên tục để debug
        // W25Q_ReadData(address, data_read, 5);
    }
}