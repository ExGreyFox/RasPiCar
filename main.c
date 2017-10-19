// Author: sradanov
// File Name: main.c
// Date: 10-01-2017

// feature-1 branch pull request test


#include <bcm2835.h>
#include <stdio.h>
#include "main.h"

#define MAX_LEN 32	//I2C write buffer max length

/* Compiler Switches */
//#define GPIO_TEST
#define I2C_TEST
//#define PWM_TEST

/* Plug P1 pin 11 (which is GPIO pin 17) */
#define PIN_11 RPI_GPIO_P1_11

// PWM output on RPi Plug P1 pin 12 (which is GPIO pin 18)
// in alt fun 5.
// Note that this is the _only_ PWM pin available on the RPi IO headers
#define PIN RPI_GPIO_P1_12
// and it is controlled by PWM channel 0
#define PWM_CHANNEL 0
// This controls the max range of the PWM signal
#define RANGE 1024

/*-----Global 32-bit flags-----*/
volatile uint32_t flags1 = 0;

/* I2C test vsriables */
char wbuf[MAX_LEN];
uint32_t len = 1;
uint8_t data;
uint16_t clk_div = BCM2835_I2C_CLOCK_DIVIDER_148;
uint8_t slave_address = 0x33;

int main(void)
{
	/* Initialize bcm2835 */
    if (!bcm2835_init())
		printf("BCM2835 init Failed\n");

#ifdef GPIO_TEST		
    /* Set the pin to be an output */
    bcm2835_gpio_fsel(PIN_11, BCM2835_GPIO_FSEL_OUTP);
#endif
#ifdef I2C_TEST		
    /* I2C begin */    
     if (!bcm2835_i2c_begin())
		printf("bcm2835_i2c_begin failed. Are you running as root??\n");
		
    bcm2835_i2c_setSlaveAddress(slave_address);
    bcm2835_i2c_setClockDivider(clk_div);
    wbuf[0] = 0x44; // Dump 0x44 on I2C bus
#endif
#ifdef PWM_TEST    
    // Set the output pin to Alt Fun 5, to allow PWM channel 0 to be output there
    bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_ALT5);

    // Clock divider is set to 16.
    // With a divider of 16 and a RANGE of 1024, in MARKSPACE mode,
    // the pulse repetition frequency will be
    // 1.2MHz/1024 = 1171.875Hz, suitable for driving a DC motor with PWM
    bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_16);
    bcm2835_pwm_set_mode(PWM_CHANNEL, 1, 1);
    bcm2835_pwm_set_range(PWM_CHANNEL, RANGE);
    
     // Vary the PWM m/s ratio between 1/RANGE and (RANGE-1)/RANGE
    // over the course of a a few seconds
    int direction = 1; // 1 is increase, -1 is decrease
    int data = 1;
#endif       

	for (;;)
	{
#ifdef GPIO_TEST
		// Turn it on
		bcm2835_gpio_write(PIN_11, HIGH);
		printf("Pin goes HIGH\n");
	
		// wait a bit
		bcm2835_delay(500);
	
		// turn it off
		bcm2835_gpio_write(PIN_11, LOW);
		printf("Pin goes LOW\n");
	
		// wait a bit
		bcm2835_delay(500);
#endif
#ifdef I2C_TEST
    	data = bcm2835_i2c_write(wbuf, len);
    	printf("Write Result = %d\n", wbuf[0]);
    	bcm2835_delay(1000);
#endif
#ifdef PWM_TEST
		if (data == 1)
			direction = 1;   // Switch to increasing
		else if (data == RANGE-1)
			direction = -1;  // Switch to decreasing
		data += direction;
		bcm2835_pwm_set_data(PWM_CHANNEL, data);
	bcm2835_delay(1);
#endif
    }
    
    bcm2835_close();
    return 0;
}

