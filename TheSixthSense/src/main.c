/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */


///////////////////////////////////////////// Source Version 1.0 //////////////////////////////////
////////////////////////////////////// Works with TheSixthSense PCB V1.1 ///////////////////////////

/*
 *
 * Created: 19.04.2015 12:36:47
 *  Author: Sebastian Foerster
 */ 
#include <asf.h>
#include <math.h>
#include "akku_adc.h"
#include "lsm303.h"

#define PI 3.14159265

#define TIMER1				TCC5
#define TIMER2				TCC4

#define TC_MOT1				TC45_CCB
#define TC_MOT2				TC45_CCA
#define TC_MOT3				TC45_CCD
#define TC_MOT4				TC45_CCC

#define TIMER_RESOLUTION		500 // x * 90 < 2^16 !!!

#define TIMER_RESOLUTION_MAX	300 //don't drive the motors with the highest current (its IMO to strong)

#define write_MOT1(x)			tc45_write_cc_buffer(&TIMER1, TC45_CCB, x)
#define write_MOT2(x)			tc45_write_cc_buffer(&TIMER1, TC45_CCA, x)
#define write_MOT3(x)			tc45_write_cc_buffer(&TIMER2, TC45_CCD, x)
#define write_MOT4(x)			tc45_write_cc_buffer(&TIMER2, TC45_CCC, x)

void test_program(void);
void motor_program(void);
void init_pwm(void);
void motor_test(void);
void calibrate_program(void);

float mag_direction(const vector_f *magData);

//you can use also a non calibartet and non accel. version, with this easy equation
float mag_direction(const vector_f *magData)
{
	return ((atan2 (magData->x,magData->z) * 180.0 / PI) + 180.0);
}


//later use? Test Hz
static void ovf_interrupt_callback(void)
{
	//gpio_toggle_pin(LED_GREEN_O);
	tc45_clear_overflow(&TIMER2);
}


//init PWM of two timers to supply 4 motors with 250 Hz
void init_pwm(void)
{
	
	/* Unmask clock for ... */
	tc45_enable(&TIMER1);
	tc45_enable(&TIMER2);

	/* Configure TC in normal mode */
	tc45_set_wgm(&TIMER1, TC45_WG_DS_T);
	tc45_set_wgm(&TIMER2, TC45_WG_DS_T);

	/* Configure period equal to resolution to obtain 250Hz */
	tc45_write_period(&TIMER1, TIMER_RESOLUTION);
	tc45_write_period(&TIMER2, TIMER_RESOLUTION);

	/* Enable channels */
	tc45_enable_cc_channels(&TIMER1, TC45_CCACOMP);
	tc45_enable_cc_channels(&TIMER1, TC45_CCBCOMP);
	tc45_enable_cc_channels(&TIMER2, TC45_CCCCOMP);
	tc45_enable_cc_channels(&TIMER2, TC45_CCDCOMP);
	
	
	//channels inverted... full gain => motors off
	write_MOT1(TIMER_RESOLUTION);
	write_MOT2(TIMER_RESOLUTION);
	write_MOT3(TIMER_RESOLUTION);
	write_MOT4(TIMER_RESOLUTION);

	tc45_set_overflow_interrupt_callback(&TIMER2, ovf_interrupt_callback);

	/*
	 * Enable TC interrupts
	 */
	tc45_set_overflow_interrupt_level(&TIMER2, TC45_INT_LVL_LO);

	/*
	 * Run 
	 */
	tc45_write_clock_source(&TIMER1, TC45_CLKSEL_DIV8_gc);
	tc45_write_clock_source(&TIMER2, TC45_CLKSEL_DIV8_gc);
}


int main (void)
{
	irq_initialize_vectors();
	cpu_irq_enable();

	// Initialize the sleep manager
	sleepmgr_init();

	sysclk_init();

	board_init();
	
	init_pwm();
	
	init_adc();
	
	uint8_t iam = LSM303_init();

	motor_program();
	//calibrate_program();
	//test_program();
	//motor_test();
}

//switch all motors on with full power
//then rotate the angle value an bring all motors to spin at the correct values
void motor_test(void)
{
	write_MOT1(0);
	delay_ms(1000);
	write_MOT1(TIMER_RESOLUTION);
	write_MOT2(0);
	delay_ms(1000);
	write_MOT2(TIMER_RESOLUTION);
	write_MOT3(0);
	delay_ms(1000);
	write_MOT3(TIMER_RESOLUTION);
	write_MOT4(0);
	delay_ms(1000);
	write_MOT4(TIMER_RESOLUTION);
	
	float angle = 0.0;
	
	while(1) {
		
		delay_ms(100);
		
		angle += 10.0;
		
		if(angle > 360.0) angle -= 360.0;
		
		float sector = angle / 90.0;
		
		uint16_t ang = (uint16_t)angle % 4;
		

		//act with 2 from 4 motors at the same time
		//but doesn't feel well... -> pwm not linear because of resonance frequenz of vibrators
		//use 8 motors is a better solution
		if(sector >= 0.0 && sector < 1.0) {
			write_MOT1((TIMER_RESOLUTION - TIMER_RESOLUTION_MAX) + (ang*TIMER_RESOLUTION_MAX)/90);
			write_MOT2(TIMER_RESOLUTION - (ang*TIMER_RESOLUTION_MAX)/90);
			write_MOT3(TIMER_RESOLUTION);
			write_MOT4(TIMER_RESOLUTION);
		} 
		else if(sector >= 1.0 && sector < 2.0) {
			write_MOT1(TIMER_RESOLUTION);
			write_MOT2((TIMER_RESOLUTION - TIMER_RESOLUTION_MAX) + (ang*TIMER_RESOLUTION_MAX)/90);
			write_MOT3(TIMER_RESOLUTION - (ang*TIMER_RESOLUTION_MAX)/90);
			write_MOT4(TIMER_RESOLUTION);
		}
		else if(sector >= 2.0 && sector < 3.0) {
			write_MOT1(TIMER_RESOLUTION);
			write_MOT2(TIMER_RESOLUTION);
			write_MOT3((TIMER_RESOLUTION - TIMER_RESOLUTION_MAX) + (ang*TIMER_RESOLUTION_MAX)/90);
			write_MOT4(TIMER_RESOLUTION - (ang*TIMER_RESOLUTION_MAX)/90);
		}
		else if(sector >= 3.0 && sector < 4.0) {
			write_MOT1(TIMER_RESOLUTION - (ang*TIMER_RESOLUTION_MAX)/90);
			write_MOT2(TIMER_RESOLUTION);
			write_MOT3(TIMER_RESOLUTION);
			write_MOT4((TIMER_RESOLUTION - TIMER_RESOLUTION_MAX) + (ang*TIMER_RESOLUTION_MAX)/90);
		}
	}
}

#define ADC_MEASURE_INTERVAL	50 //(uint16_t)
//normal operation program
//wait a second, read the values start the correct motor(s) for 0,3s
//flash short the green led
void motor_program(void)
{
	uint16_t adc_interval = 0;
	float angle= 0.0;
	
	//offset if needed
	float offset_ang = 0.0;
	
	vector_f magData = {0.0, 0.0, 0.0};;
	vector_f accelData = {0.0, 0.0, 0.0};;
	
	while(1) {

		delay_ms(1000);
		if(LSM303_read_accel(&accelData) == 0x00) {
			delay_ms(10);
		}
		if(LSM303_read_mag(&magData) == 0x00) {
			delay_ms(10);
		}
			
		//angle = mag_direction();
		angle = heading(&magData,&accelData);

		gpio_set_pin_high(LED_GREEN_O);
		
		if(angle > 360.0) angle = 360.0;
		
		angle = 360.0 - angle;
		
		//add offset position on the leg position
		angle += offset_ang;
		if(angle > 360.0) angle -= 360.0;
		
		float sector = angle / 90.0;
		
		uint16_t ang = (uint16_t)angle % 4;
		
		//get correct sector
		if(sector >= 0.0 && sector < 1.0) {
			write_MOT1((TIMER_RESOLUTION - TIMER_RESOLUTION_MAX) + (ang*TIMER_RESOLUTION_MAX)/90);
			write_MOT2(TIMER_RESOLUTION - (ang*TIMER_RESOLUTION_MAX)/90);
			write_MOT3(TIMER_RESOLUTION);
			write_MOT4(TIMER_RESOLUTION);
		}
		else if(sector >= 1.0 && sector < 2.0) {
			write_MOT1(TIMER_RESOLUTION);
			write_MOT2((TIMER_RESOLUTION - TIMER_RESOLUTION_MAX) + (ang*TIMER_RESOLUTION_MAX)/90);
			write_MOT3(TIMER_RESOLUTION - (ang*TIMER_RESOLUTION_MAX)/90);
			write_MOT4(TIMER_RESOLUTION);
		}
		else if(sector >= 2.0 && sector < 3.0) {
			write_MOT1(TIMER_RESOLUTION);
			write_MOT2(TIMER_RESOLUTION);
			write_MOT3((TIMER_RESOLUTION - TIMER_RESOLUTION_MAX) + (ang*TIMER_RESOLUTION_MAX)/90);
			write_MOT4(TIMER_RESOLUTION - (ang*TIMER_RESOLUTION_MAX)/90);
		}
		else if(sector >= 3.0 && sector < 4.0) {
			write_MOT1(TIMER_RESOLUTION - (ang*TIMER_RESOLUTION_MAX)/90);
			write_MOT2(TIMER_RESOLUTION);
			write_MOT3(TIMER_RESOLUTION);
			write_MOT4((TIMER_RESOLUTION - TIMER_RESOLUTION_MAX) + (ang*TIMER_RESOLUTION_MAX)/90);
		}
		
		gpio_set_pin_low(LED_GREEN_O);
		
		delay_ms(300);
		
		//all motors off
		write_MOT1(TIMER_RESOLUTION);
		write_MOT2(TIMER_RESOLUTION);
		write_MOT3(TIMER_RESOLUTION);
		write_MOT4(TIMER_RESOLUTION);
		
		//stop program if USB is pluged in for charging
		while(gpio_pin_is_high(USB_DETECT_I)) { }
			
		uint16_t adc_value = 0;
			
		if(adc_interval++ > ADC_MEASURE_INTERVAL) {
			adc_interval = 0;
			adc_value = read_bat();
			if(adc_value < 3900) // -> 3,10 Volt @ 10k || 10k
			{
				delay_ms(10000);
			}
		}
	
	}
}

//rotate the pcb in all directions, read the values with help of your PDI Debugger!
//Enter the calibration values in lsm303.c!
void calibrate_program(void)
{
	
	vector_f min_v = {32767.0, 32767.0, 32767.0};
	vector_f max_v = {-32768.0, -32768.0, -32768.0};
		
	vector_f magData = {0.0, 0.0, 0.0};
	
	while(1) {
		delay_ms(10);
		if(LSM303_read_mag(&magData) == 0x00) {
			delay_ms(10);
		}
		
		if(min_v.x > magData.x)	min_v.x = magData.x;
		if(min_v.y > magData.y)	min_v.y = magData.y;
		if(min_v.z > magData.z)	min_v.z = magData.z;
		
		if(max_v.x < magData.x)	max_v.x = magData.x;
		if(max_v.y < magData.y)	max_v.y = magData.y;
		if(max_v.z < magData.z)	max_v.z = magData.z;
		
	}
}

//blink led if not pointed to the north (+- 10°)
//led on if pointed to the north (+- 10°)
void test_program(void)
{
	float angle= 0.0;
	
	vector_f magData = {0.0, 0.0, 0.0};;
	vector_f accelData = {0.0, 0.0, 0.0};;
	
	while(1) {
		delay_ms(10);
		if(LSM303_read_accel(&accelData) == 0x00) {
			delay_ms(10);
		}
		if(LSM303_read_accel(&magData) == 0x00) {
			delay_ms(10);
		}
		
		angle = heading(&magData, &accelData);
		
		gpio_toggle_pin(LED_GREEN_O);
		
		
		if(angle > 350.0 || angle < 10.0) {
			gpio_set_pin_high(LED_GREEN_O);
			} else {
			gpio_set_pin_low(LED_GREEN_O);
		}
		angle = 0.0;
		
	}
}