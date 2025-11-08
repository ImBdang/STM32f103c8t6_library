#include "i2c_stm32f103c8.h"
#include "gpio_stm32f103c8.h"
#include "rcc_stm32f103c8.h"

#define ds1307_addr 0x68

/* DUMMY IRS handler */
void TIM6_IRQHandler(void){}
void USART1_IRQHandler(void){}
void TIM2_IRQHandler(void){}


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
    /* MODE=11, CNF=11, output mode alertnate open drain */
    GPIOB->CRL &= ~(0xF << 24);
    GPIOB->CRL &= ~(0xF << 28);
    GPIOB->CRL |= (0xF << 24);
    GPIOB->CRL |= (0xF << 28);
}

uint8_t bcd2dec(uint8_t val){
    return ((val >> 4) * 10) + (val & 0x0F);
}
uint8_t dec2bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}



/* ============================== I2C ===================================*/

/* Init I2C1 */
void init_I2C1(){
    RCC->APB1ENR |= (1 << 21);
    /* Turn off to cfg */
    I2C1->CR1 &= ~(1 << 0);             //PE=0
    /* CR1 config */
    I2C1->CR1 &= ~(1 << 1);             //SMBUS=0 I2C mode
    /* CR2 config */           
    I2C1->CR2 = (36 << 0);              //PCLK1 = 36MHz
    /* OAR1 */
    I2C1->OAR1 &= ~(1 << 15);           //7 bit mode
    /* CCR */
    I2C1->CCR = 180;                    //f_SCL Sm 100Khz, CCR = PCLK1/ (2 * f_SCL)
    /* TRISE */
    I2C1->TRISE = 37;                   //TRISE = Freq + 1
    /* Turn on */
    I2C1->CR1 |= (1 << 0);              //PE = 1
}

/* Slave address tranmission */
void i2c1_sendAddr(uint8_t addr){
    I2C1->DR = addr;                    //send addr to DR reg
    while ( !(I2C1->SR1 & (1 << 1)));   //wait ADDR bit set
    /* read SR1 and SR2 to clear bit ADDR (EV6) */
    (void) I2C1->SR1;                
    (void) I2C1->SR2; 
}

/* Start condition */
void i2c1_start(uint8_t addr){
    I2C1->CR1 |= (1 << 8);              //Set bit START
    while (!((I2C1->SR1) & (1 << 0)));
    /* EV5 */
    (void) I2C1->SR1;
    // i2c1_sendAddr(addr);
}

/* Master transmitter (EV8_1, EV8, EV8_2)*/
void i2c1_write(uint8_t data){
    while ( !(I2C1->SR1 & (1 << 7)));
    I2C1->DR = data;
    while ( !(I2C1->SR1 & (1 << 7)));
}

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


/* ============================== ds1307 ===================================*/

/* Enable ds1307 (bit 7 0x00 reg) */
void ds1307_enable(){
    /* Write mode, write address pointer */
    uint8_t sec;
    i2c1_start(ds1307_addr);
    i2c1_sendAddr(ds1307_addr << 1);
    i2c1_write(0x00);

    /* Read mode, get current reg */
    i2c1_start(ds1307_addr);
    i2c1_sendAddr((ds1307_addr << 1) | 1);
    sec = i2c1_read(0);

    /* Write mode, write reg after edit */
    sec &= ~(1 << 7);
    i2c1_start(ds1307_addr);
    i2c1_sendAddr(ds1307_addr << 1);
    i2c1_write(0x00);
    i2c1_write(sec);
    i2c1_stop();
}

/* Mode hour 24h */
void ds1307_sethour_mode24(){
    /* Write mode, write address pointer */
    uint8_t hour;
    i2c1_start(ds1307_addr);
    i2c1_sendAddr(ds1307_addr << 1);
    i2c1_write(0x02);

    /* Read mode, get current reg */
    i2c1_start(ds1307_addr);
    i2c1_sendAddr((ds1307_addr << 1) | 1);
    hour = i2c1_read(0);

    /* Write mode, write reg after edit */
    hour &= ~(1 << 6); 
    i2c1_start(ds1307_addr);
    i2c1_sendAddr(ds1307_addr << 1);
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
    i2c1_start(ds1307_addr);
    i2c1_sendAddr(ds1307_addr << 1);
    i2c1_write(0x00);    
    i2c1_write(sec);
    i2c1_write(min);
    i2c1_write(hour);
    i2c1_stop();
}

/* Get time of ds1307 */
void ds1307_get_time(uint8_t *time_arr){
    /* Start conditon and set pointer */
    i2c1_start(ds1307_addr);
    i2c1_sendAddr(ds1307_addr << 1);
    i2c1_write(0x00);
    /* Read data */
    i2c1_start(ds1307_addr);
    i2c1_sendAddr(ds1307_addr << 1 | 1);
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

//0x40005400 - CR1
//0x40005414 - SR1
//0x40005410 - DR
void main(){
    init_CLOCK();
    init_GPIO();
    init_I2C1();
    ds1307_set_time(12, 12, 13);
    init_ds1307();

    uint8_t time_arr[7];
    while (1){
        ds1307_get_time(time_arr);

        uint8_t sec  = bcd2dec(time_arr[0] & 0x7F);  
        uint8_t min  = bcd2dec(time_arr[1]);
        uint8_t hour = bcd2dec(time_arr[2] & 0x3F);  
        uint8_t day  = time_arr[3];
        uint8_t date = bcd2dec(time_arr[4]);
        uint8_t mon  = bcd2dec(time_arr[5]);
        uint8_t year = bcd2dec(time_arr[6]);
    }
}