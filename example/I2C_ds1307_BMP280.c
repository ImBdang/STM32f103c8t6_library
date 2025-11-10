#include "i2c_stm32f103c8.h"
#include "gpio_stm32f103c8.h"
#include "rcc_stm32f103c8.h"

/* BMP280 I2C Address (SDO connected to GND: 0x76, SDO to VCC: 0x77) */
#define BMP280_ADDR 0x76
#define ds1307_addr 0x68

/* BMP280 Registers */
#define BMP280_REG_ID           0xD0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_REG_PRESS_LSB    0xF8
#define BMP280_REG_PRESS_XLSB   0xF9
#define BMP280_REG_TEMP_MSB     0xFA
#define BMP280_REG_TEMP_LSB     0xFB
#define BMP280_REG_TEMP_XLSB    0xFC

/* Calibration registers */
#define BMP280_REG_CALIB_START  0x88

/* BMP280 Modes */
#define BMP280_MODE_SLEEP       0x00
#define BMP280_MODE_FORCED      0x01
#define BMP280_MODE_NORMAL      0x03

/* Oversampling settings */
#define BMP280_OVERSAMP_SKIP    0x00
#define BMP280_OVERSAMP_1X      0x01
#define BMP280_OVERSAMP_2X      0x02
#define BMP280_OVERSAMP_4X      0x03
#define BMP280_OVERSAMP_8X      0x04
#define BMP280_OVERSAMP_16X     0x05

/* DUMMY IRS handler */
void TIM6_IRQHandler(void){}
void USART1_IRQHandler(void){}
void TIM2_IRQHandler(void){}

/* Calibration data structure */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP280_CalibData;

BMP280_CalibData calib;
int32_t t_fine;

/* HSE on, PLL=9, SYSCLK = 72Mhz, APB1 = 36Mhz */
void init_CLOCK(){
    /* HSE ON */
    RCC->CR |= (1 << 16);
    while ( !(RCC->CR & (1 << 17)) );
    /* PLLMUL */
    RCC->CFGR |= (7 << 18);
    /* HSR not div */
    RCC->CFGR &= ~(1 << 17);
    /* PLL SRC */
    RCC->CFGR |= (1 << 16);
    /* PLL ON */
    RCC->CR |= (1 << 24);
    while ( !(RCC->CR & (1 << 25)) );
    /* APB1 PRE = 2 */
    RCC->CFGR |= (4 << 8);
    /* Switch to PLL as main CLK */
    RCC->CFGR &= ~(3 << 0);
    RCC->CFGR |= (2 << 0);
    while (((RCC->CFGR >> 2) & 0x3) != 0x2);
}

/* PB6 - SCL, PB7 -SDA, both pin mode opendrain */
void init_GPIO(){
    /* APB2ENR GPIOB */
    RCC->APB2ENR |= (1 << 3);
    /* MODE=11, CNF=11, output mode alternate open drain */
    GPIOB->CRL &= ~(0xF << 24);
    GPIOB->CRL &= ~(0xF << 28);
    GPIOB->CRL |= (0xF << 24);
    GPIOB->CRL |= (0xF << 28);
}

/* ============================== I2C ===================================*/

/* Init I2C1 */
void init_I2C1(){
    RCC->APB1ENR |= (1 << 21);
    /* Turn off to cfg */
    I2C1->CR1 &= ~(1 << 0);
    /* CR1 config */
    I2C1->CR1 &= ~(1 << 1);
    /* CR2 config */
    I2C1->CR2 = (36 << 0);
    /* OAR1 */
    I2C1->OAR1 &= ~(1 << 15);
    /* CCR */
    I2C1->CCR = 180;
    /* TRISE */
    I2C1->TRISE = 37;
    /* Turn on */
    I2C1->CR1 |= (1 << 0);
}

/* Slave address transmission */
void i2c1_sendAddr(uint8_t addr){
    I2C1->DR = addr;
    while ( !(I2C1->SR1 & (1 << 1)));
    (void) I2C1->SR1;
    (void) I2C1->SR2;
}

/* Start condition */
void i2c1_start(uint8_t addr){
    I2C1->CR1 |= (1 << 8);
    while (!((I2C1->SR1) & (1 << 0)));
    (void) I2C1->SR1;
    i2c1_sendAddr(addr);
}

/* Master transmitter */
void i2c1_write(uint8_t data){
    while ( !(I2C1->SR1 & (1 << 7)));
    I2C1->DR = data;
    while ( !(I2C1->SR1 & (1 << 7)));
}

/* Master receiver */
uint8_t i2c1_read(uint8_t ack){
    if (ack)
        I2C1->CR1 |= (1 << 10);
    else
        I2C1->CR1 &= ~(1 << 10);
    while ( !(I2C1->SR1 & (1 << 6)));
    return I2C1->DR;
}

/* Closing communication */
void i2c1_stop(){
    I2C1->CR1 |= (1 << 9);
}

/* ============================== BMP280 ================================*/

/* Write single byte to BMP280 register */
void bmp280_write_reg(uint8_t reg, uint8_t value){
    i2c1_start(BMP280_ADDR << 1);
    i2c1_write(reg);
    i2c1_write(value);
    i2c1_stop();
}

/* Read single byte from BMP280 register */
uint8_t bmp280_read_reg(uint8_t reg){
    uint8_t data;
    i2c1_start(BMP280_ADDR << 1);
    i2c1_write(reg);
    i2c1_start((BMP280_ADDR << 1) | 1);
    data = i2c1_read(0);
    i2c1_stop();
    return data;
}

/* Read multiple bytes from BMP280 */
void bmp280_read_regs(uint8_t reg, uint8_t *buffer, uint8_t len){
    i2c1_start(BMP280_ADDR << 1);
    i2c1_write(reg);
    i2c1_start((BMP280_ADDR << 1) | 1);
    
    for(uint8_t i = 0; i < len - 1; i++){
        buffer[i] = i2c1_read(1);  // ACK
    }
    buffer[len - 1] = i2c1_read(0);  // NACK for last byte
    i2c1_stop();
}

/* Read calibration data */
void bmp280_read_calibration(){
    uint8_t calib_data[24];
    bmp280_read_regs(BMP280_REG_CALIB_START, calib_data, 24);
    
    calib.dig_T1 = (calib_data[1] << 8) | calib_data[0];
    calib.dig_T2 = (calib_data[3] << 8) | calib_data[2];
    calib.dig_T3 = (calib_data[5] << 8) | calib_data[4];
    calib.dig_P1 = (calib_data[7] << 8) | calib_data[6];
    calib.dig_P2 = (calib_data[9] << 8) | calib_data[8];
    calib.dig_P3 = (calib_data[11] << 8) | calib_data[10];
    calib.dig_P4 = (calib_data[13] << 8) | calib_data[12];
    calib.dig_P5 = (calib_data[15] << 8) | calib_data[14];
    calib.dig_P6 = (calib_data[17] << 8) | calib_data[16];
    calib.dig_P7 = (calib_data[19] << 8) | calib_data[18];
    calib.dig_P8 = (calib_data[21] << 8) | calib_data[20];
    calib.dig_P9 = (calib_data[23] << 8) | calib_data[22];
}

/* Initialize BMP280 */
uint8_t bmp280_init(){
    /* Check chip ID */
    uint8_t chip_id = bmp280_read_reg(BMP280_REG_ID);
    if(chip_id != 0x58){
        return 0;  // Wrong chip ID
    }
    
    /* Read calibration data */
    bmp280_read_calibration();
    
    /* Configure BMP280 */
    /* CTRL_MEAS: osrs_t=16x, osrs_p=16x, mode=normal */
    uint8_t ctrl_meas = (BMP280_OVERSAMP_16X << 5) | 
                        (BMP280_OVERSAMP_16X << 2) | 
                        BMP280_MODE_NORMAL;
    bmp280_write_reg(BMP280_REG_CTRL_MEAS, ctrl_meas);
    
    /* CONFIG: standby=0.5ms, filter=16, spi disable */
    bmp280_write_reg(BMP280_REG_CONFIG, 0x10);
    
    return 1;  // Success
}

/* Compensate temperature (returns temperature in 0.01 DegC) */
int32_t bmp280_compensate_temperature(int32_t adc_T){
    int32_t var1, var2, T;
    
    var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * 
            ((int32_t)calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * 
              ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * 
            ((int32_t)calib.dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

/* Compensate pressure (returns pressure in Pa) - 32-bit version */
uint32_t bmp280_compensate_pressure(int32_t adc_P){
    int32_t var1, var2;
    uint32_t p;
    
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calib.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)calib.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)calib.dig_P4) << 16);
    var1 = (((calib.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + 
            ((((int32_t)calib.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)calib.dig_P1)) >> 15);
    
    if(var1 == 0){
        return 0;  // Avoid division by zero
    }
    
    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
    
    if(p < 0x80000000){
        p = (p << 1) / ((uint32_t)var1);
    } else {
        p = (p / (uint32_t)var1) * 2;
    }
    
    var1 = (((int32_t)calib.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)calib.dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + calib.dig_P7) >> 4));
    
    return p;
}

/* Read temperature and pressure */
void bmp280_read_data(int32_t *temperature, uint32_t *pressure){
    uint8_t data[6];
    
    /* Read pressure and temperature data (6 bytes) */
    bmp280_read_regs(BMP280_REG_PRESS_MSB, data, 6);
    
    /* Combine bytes to get raw ADC values (20-bit) */
    int32_t adc_P = ((int32_t)data[0] << 12) | 
                    ((int32_t)data[1] << 4) | 
                    ((int32_t)data[2] >> 4);
    
    int32_t adc_T = ((int32_t)data[3] << 12) | 
                    ((int32_t)data[4] << 4) | 
                    ((int32_t)data[5] >> 4);
    
    /* Compensate temperature and pressure */
    *temperature = bmp280_compensate_temperature(adc_T);
    *pressure = bmp280_compensate_pressure(adc_P);
}

/* Simple delay function */
void delay_ms(uint32_t ms){
    for(uint32_t i = 0; i < ms * 8000; i++){
        __asm("nop");
    }
}



/* ============================== ds1307 ===================================*/
uint8_t bcd2dec(uint8_t val){
    return ((val >> 4) * 10) + (val & 0x0F);
}
uint8_t dec2bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

/* Enable ds1307 (bit 7 0x00 reg) */
void ds1307_enable(){
    /* Write mode, write address pointer */
    uint8_t sec;
    i2c1_start(ds1307_addr << 1);
    i2c1_write(0x00);

    /* Read mode, get current reg */
    i2c1_start((ds1307_addr << 1) | 1);
    sec = i2c1_read(0);

    /* Write mode, write reg after edit */
    sec &= ~(1 << 7);
    i2c1_start(ds1307_addr << 1);
    i2c1_write(0x00);
    i2c1_write(sec);
    i2c1_stop();
}

/* Mode hour 24h */
void ds1307_sethour_mode24(){
    /* Write mode, write address pointer */
    uint8_t hour;
    i2c1_start(ds1307_addr << 1);
    i2c1_write(0x02);

    /* Read mode, get current reg */
    i2c1_start((ds1307_addr << 1) | 1);
    hour = i2c1_read(0);

    /* Write mode, write reg after edit */
    hour &= ~(1 << 6); 
    i2c1_start(ds1307_addr << 1);
    i2c1_write(0x00);
    i2c1_write(hour);
    i2c1_stop();
}

/* Set time for ds1307 */
void ds1307_set_time(uint8_t sec, uint8_t min, uint8_t hour){
    /* Change decimal to bcd */
    sec  = dec2bcd(sec);   
    min  = dec2bcd(min);
    hour = dec2bcd(hour);
    /* Write to ds1307 reg */
    i2c1_start(ds1307_addr << 1);
    i2c1_write(0x00);    
    i2c1_write(sec);
    i2c1_write(min);
    i2c1_write(hour);
    i2c1_stop();
}

/* Get time of ds1307 */
void ds1307_get_time(uint8_t *time_arr){
    /* Start conditon and set pointer */
    i2c1_start(ds1307_addr << 1);
    i2c1_write(0x00);
    /* Read data */
    i2c1_start(ds1307_addr << 1 | 1);
    for(int i=0; i<6; i++)
        time_arr[i] = i2c1_read(1);  
    time_arr[6] = i2c1_read(0);       
    i2c1_stop();
}

/* Hour mode 24h, all time and date set to 00*/
void init_ds1307(){
    ds1307_enable();
    ds1307_sethour_mode24();
}




/* ============================== MAIN ==================================*/
void main(){
    init_CLOCK();
    init_GPIO();
    init_I2C1();
    init_ds1307();
    uint8_t time_arr[7];
    
    /* Initialize BMP280 */
    if(!bmp280_init()){
        /* Sensor not found - handle error */
        while(1);
    }
    
    int32_t temperature;  // Temperature in 0.01 DegC
    uint32_t pressure;    // Pressure in Pa
    
    while(1){
        /* Read sensor data */
        bmp280_read_data(&temperature, &pressure);
        
        /* temperature = 2534 means 25.34 DegC */
        /* pressure = 101325 means 101325 Pa = 1013.25 hPa */
        
        /* Convert to useful units: */
        // float temp_celsius = temperature / 100.0f;
        // float pressure_hPa = pressure / 100.0f;
        ds1307_get_time(time_arr);

        uint8_t sec  = bcd2dec(time_arr[0] & 0x7F);  
        uint8_t min  = bcd2dec(time_arr[1]);
        uint8_t hour = bcd2dec(time_arr[2] & 0x3F);  
        uint8_t day  = time_arr[3];
        uint8_t date = bcd2dec(time_arr[4]);
        uint8_t mon  = bcd2dec(time_arr[5]);
        uint8_t year = bcd2dec(time_arr[6]);
        
        delay_ms(1000);  // Read every 1 second
    }
}