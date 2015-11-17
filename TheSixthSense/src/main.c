/**
 * \file
 *
 * \brief TheSixthSense Project
 *
 */

/**
 * \mainpage User Application
 *
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */


///////////////////////////////////////////// Source Version 1.3 //////////////////////////////////
////////////////////////////////////// Works with TheSixthSense PCB V1.3 ///////////////////////////

/*
 *
 * Created: 19.04.2015 12:36:47
 *  Author: Sebastian Foerster
 
 TheSixthSense C (Atmel Studio 6.2) Source Code with ASF lib from Atmel
 Copyright (C) Atmel Corporation
 Copyright (C) Sebastian Foerster

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2
 of the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 
 */ 
#include <asf.h>
#include <math.h>
#include "string.h"
#include "akku_adc.h"
#include "lsm303.h"

#define MOTOR_NUM				8

#define MAX_MOTOR_FORCE			19

//some main program timings
#define CALI_PROGRAMM_ENTER		1500 //* 2 ms
#define WAIT_IDLE_MODE			1000 //* 2 ms
#define WAIT_VIBRATE_MODE		150  //* 2 ms

//mean value filter constant
#define MEAN_VALUE				128

#define ADC_MEASURE_INTERVAL	50 //(uint16_t) depending on the vibration intervals
	
#define PI						3.14159265

#define TIMER_RESOLUTION		500 // 250 HZ at 2 MHz and div = 8

volatile uint8_t motor_soft_bam[MOTOR_NUM] = {0};

#define SET_MOTOR_BAM(mot,bam)	{motor_soft_bam[mot-1]=bam;}
	
//software states of the main state machine (more states in the future?)
#define MAIN_VIBRATE			0
#define MAIN_IDLE				1
#define MAIN_CHECK_VBAT			2

//state machine
static uint8_t main_state = MAIN_CHECK_VBAT; 

void test_program(void);
void main_program(void);
void init_bam(void);
void motor_test(void);
void calibrate_program(void);
void soft_bam_process(uint8_t bitmask);

float mag_direction(const vector_f *magData);

//you can use also a non calibrated and non accel. version, with this easy equation
float mag_direction(const vector_f *magData)
{
	return ((atan2 (magData->x,magData->z) * 180.0 / PI) + 180.0);
}

void soft_bam_process(uint8_t bitmask)
{
	//error if adding motors without changing source
	#if(MOTOR_NUM != 8)
		#error (add more motors here)
	#endif
	
	if(motor_soft_bam[0] & bitmask) {
		gpio_set_pin_high(M1_O);
	} else {
		gpio_set_pin_low(M1_O);
	}
	if(motor_soft_bam[1] & bitmask) {
		gpio_set_pin_high(M2_O);
	} else {
		gpio_set_pin_low(M2_O);
	}
	if(motor_soft_bam[2] & bitmask) {
		gpio_set_pin_high(M3_O);
	} else {
		gpio_set_pin_low(M3_O);
	}
	if(motor_soft_bam[3] & bitmask) {
		gpio_set_pin_high(M4_O);
	} else {
		gpio_set_pin_low(M4_O);
	}
	if(motor_soft_bam[4] & bitmask) {
		gpio_set_pin_high(M5_O);
	} else {
		gpio_set_pin_low(M5_O);
	}
	if(motor_soft_bam[5] & bitmask) {
		gpio_set_pin_high(M6_O);
	} else {
		gpio_set_pin_low(M6_O);
	}
	if(motor_soft_bam[6] & bitmask) {
		gpio_set_pin_high(M7_O);
	} else {
		gpio_set_pin_low(M7_O);
	}
	if(motor_soft_bam[7] & bitmask) {
		gpio_set_pin_high(M8_O);
	} else {
		gpio_set_pin_low(M8_O);
	}
}

static void ovf_interrupt_callback(void)
{
	//gpio_toggle_pin(LED_GREEN_O);
	soft_bam_process(0x01);
	tc45_clear_overflow(&TCC4);
}

static void cca_interrupt_callback(void)
{
	//gpio_toggle_pin(LED_GREEN_O);
	soft_bam_process(0x02);
	tc45_clear_cc_interrupt(&TCC4, TC45_CCA);
}

static void ccb_interrupt_callback(void)
{
	//gpio_toggle_pin(LED_GREEN_O);
	soft_bam_process(0x04);
	tc45_clear_cc_interrupt(&TCC4, TC45_CCB);
}

static void ccc_interrupt_callback(void)
{
	//gpio_toggle_pin(LED_GREEN_O);
	soft_bam_process(0x08);
	tc45_clear_cc_interrupt(&TCC4, TC45_CCC);
}

static void ccd_interrupt_callback(void)
{
	//gpio_toggle_pin(LED_GREEN_O);
	soft_bam_process(0x10);
	tc45_clear_cc_interrupt(&TCC4, TC45_CCD);
}

//init BAM with one timer to supply 8 motors with 250 Hz
void init_bam(void)
{
	
	/* Unmask clock for ... */
	tc45_enable(&TCC4);

	/* Configure TC in normal mode */
	tc45_set_wgm(&TCC4, TC45_WG_NORMAL);

	/* Configure period equal to resolution to obtain 250Hz */
	tc45_write_period(&TCC4, TIMER_RESOLUTION);

	//no cc outputs 
	TCC4.CTRLE = 0;
	
	//bam timing -> 5 Bit possible with ovf vector
	tc45_write_cc_buffer(&TCC4, TC45_CCA, TIMER_RESOLUTION >> 4);
	tc45_write_cc_buffer(&TCC4, TC45_CCB, TIMER_RESOLUTION >> 3);
	tc45_write_cc_buffer(&TCC4, TC45_CCC, TIMER_RESOLUTION >> 2);
	tc45_write_cc_buffer(&TCC4, TC45_CCD, TIMER_RESOLUTION >> 1);

	tc45_set_overflow_interrupt_callback(&TCC4, ovf_interrupt_callback);
	tc45_set_cca_interrupt_callback(&TCC4, cca_interrupt_callback);
	tc45_set_ccb_interrupt_callback(&TCC4, ccb_interrupt_callback);
	tc45_set_ccc_interrupt_callback(&TCC4, ccc_interrupt_callback);
	tc45_set_ccd_interrupt_callback(&TCC4, ccd_interrupt_callback);

	/*
	 * Enable TC interrupts
	 */
	tc45_set_overflow_interrupt_level(&TCC4, TC45_INT_LVL_LO);
	tc45_set_cca_interrupt_level(&TCC4, TC45_INT_LVL_LO);
	tc45_set_ccb_interrupt_level(&TCC4, TC45_INT_LVL_LO);
	tc45_set_ccc_interrupt_level(&TCC4, TC45_INT_LVL_LO);
	tc45_set_ccd_interrupt_level(&TCC4, TC45_INT_LVL_LO);

	/*
	 * Run 
	 */
	tc45_write_clock_source(&TCC4, TC45_CLKSEL_DIV8_gc);
}

static void alarm(uint32_t time)
{
//gpio_toggle_pin(LED_GREEN_O);
	rtc_set_alarm(time);
}


int main (void)
{
	irq_initialize_vectors();
	cpu_irq_enable();

	sysclk_init();

	board_init();
	
	// Initialize the sleep manager
	sleepmgr_init();
	
	
	//start rtc
	rtc_init();
	rtc_set_callback(alarm);
	rtc_set_alarm_relative(0);
	
	init_bam();
	
	init_adc();
	
	uint8_t iam = LSM303_init();
	
	//wait for stable vcc
	delay_ms(10);
		
	//read mag calibration data
	nvm_eeprom_read_buffer(1 * EEPROM_PAGE_SIZE,&mag_cali,sizeof(mag_cali));
		
	//cali data available or is it fresh eeprom?
	if(isnan(mag_cali.m_min.x)) {
		mag_cali.m_min.x = -32768.0;
		mag_cali.m_min.y = -32768.0;
		mag_cali.m_min.z = -32768.0;
			
		mag_cali.m_max.x = 32767.0;
		mag_cali.m_max.y = 32767.0;
		mag_cali.m_max.z = 32767.0;
	}

	main_program();
	//test_program();
	//motor_test();
}

//normal operation program
//wait a second, read the values start the correct motor(s) for 0,3s
//flash short the green led
void main_program(void)
{
	//start with battery measure
	uint16_t	adc_interval = 0;
	float		angle= 0.0;
	
	//offset if needed (only positive allowed)
	float offset_ang = 0.0;
	
	vector_f magData = {0.0, 0.0, 0.0};
	vector_32 accelData = {0, 0, 0};
	vector_f accelData_f;
		
	uint16_t cnt_pushbutton = 0;
	
	uint32_t wait_idle = 0;
	uint32_t wait_vibrate = 0;
	
	while(1) {
		
		
		//working in time slices of about 2 ms ... see rtc settings (conf_rtc.h)
		sleepmgr_enter_sleep();
		
		//awake and check the stuff in this main loop
		
		//Test push button to enter the cali program
		if(gpio_pin_is_low(PUSH_BUTTON_I)) {
			if(cnt_pushbutton < UINT16_MAX) {
				cnt_pushbutton++;
			}
		}
		if(gpio_pin_is_high(PUSH_BUTTON_I)) {
			if(cnt_pushbutton > CALI_PROGRAMM_ENTER) {
				calibrate_program();
			}
			cnt_pushbutton = 0;
		}
		

		vector_32 accelData_temp;
			
		//if new data ready to read?		
		if(LSM303_new_accel_data()) {
			//read data
			if(LSM303_read_accel_32(&accelData_temp) == 0x00) {
				//error condition?
				gpio_set_pin_high(LED_GREEN_O);
				delay_ms(10);
				gpio_set_pin_low(LED_GREEN_O);
			} 
			else 
			{
				//mean value filter
				accelData.x += accelData_temp.x - (accelData.x / MEAN_VALUE);
				accelData.y += accelData_temp.y - (accelData.y / MEAN_VALUE);
				accelData.z += accelData_temp.z - (accelData.z / MEAN_VALUE);
			}
		}
		
		//gpio_toggle_pin(LED_GREEN_O);
		
		switch(main_state) {
			case MAIN_CHECK_VBAT:
			{
				uint16_t adc_value = 0;
				adc_interval = 0;
				adc_value = read_bat();
				if(adc_value < 3900) // -> 3,10 Volt @ 10k || 10k
				{
					
					// enter low power mode and never wake up,
					// do power cycle with main switch, if you want to wakeup the controller again
					// because
					// it's easy
					// and you didn't know how deep the voltage was dropping while in sleep mode and if the LSM303D needs a reset...
					
					//stop all motors an blinker fast, to say: "we are going to sleep"
					for(uint8_t i = 0;i<sizeof(motor_soft_bam);i++) {
						motor_soft_bam[i]=0;
						gpio_toggle_pin(LED_GREEN_O);
						delay_ms(100);
					}
					gpio_set_pin_low(LED_GREEN_O);
					
					//set LSM303D to sleep
					LSM303_set_sleep();
					
					//set ATXMega32E5 to sleep
					sleep_set_mode(SLEEP_SMODE_PDOWN_gc);
					sleep_enable();
					sleep_enter();
					
					//this section will never reached
					// ...
				}
				//switch to idle mode
				main_state = MAIN_IDLE;
				break;
			}
			case MAIN_IDLE:
			{
				if(wait_idle++ > WAIT_IDLE_MODE) {
					wait_idle = 0;
					main_state = MAIN_VIBRATE;
				}
				break;
			}
			case MAIN_VIBRATE:
			{
				//enter first--- set all settings
				wait_vibrate++;
				if(wait_vibrate == 1) {
					//read from mag sensor
					if(LSM303_read_mag(&magData) == 0x00) {
						gpio_set_pin_high(LED_GREEN_O);
						delay_ms(10);
						gpio_set_pin_low(LED_GREEN_O);
					}
		
					accelData_f.x = (float)accelData.x;
					accelData_f.y = (float)accelData.y;
					accelData_f.z = (float)accelData.z;
				
					//calc tilt compensated compass reading
					angle = heading(&magData,&accelData_f);

					gpio_set_pin_high(LED_GREEN_O);
				
					if(angle > 360.0) angle = 360.0;
				
					//add offset position on the leg position
					angle += offset_ang;
					if(angle > 360.0) angle -= 360.0;
				
					//turn the direction, uncomment if motors are not in the same direction like the pcb measure
					angle = 360.0 - angle;
				
					uint8_t sector = angle / (360.0 / MOTOR_NUM);
				
					//get correct motor
					if(sector < MOTOR_NUM) {
						motor_soft_bam[sector] = MAX_MOTOR_FORCE;
					} 
					else 
					{
						//gpio_toggle_pin(LED_GREEN_O);
					}
				
					gpio_set_pin_low(LED_GREEN_O);
					
				} else if(wait_vibrate > WAIT_VIBRATE_MODE) {
					wait_vibrate = 0;
				
				
					//stop all motors
					for(uint8_t i = 0;i<sizeof(motor_soft_bam);i++) {
						motor_soft_bam[i]=0;
					}
				
					//stop program if USB is plugged in for charging
					while(gpio_pin_is_high(USB_DETECT_I))
					{
						//blink LED in charge mode
						gpio_toggle_pin(LED_GREEN_O);
						delay_ms(300);
					}
				
					gpio_set_pin_low(LED_GREEN_O);
				
				
					//switch state
					if(adc_interval++ > ADC_MEASURE_INTERVAL) {
						main_state = MAIN_CHECK_VBAT;
					} 
					else 
					{
						main_state = MAIN_IDLE;
					}
				}
				
				break;
			}
		}
	}
}

//rotate the pcb in all directions!!!
void calibrate_program(void)
{
	
	vector_f min_v = {32767.0, 32767.0, 32767.0};
	vector_f max_v = {-32768.0, -32768.0, -32768.0};
		
	vector_f magData = {0.0, 0.0, 0.0};
		
	gpio_set_pin_high(LED_GREEN_O);
	
	delay_ms(500);
	
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
		
		if(gpio_pin_is_low(PUSH_BUTTON_I)) {
			break;
		}
		
	}
	
	memcpy(&mag_cali.m_min,&min_v,sizeof(vector_f));
	memcpy(&mag_cali.m_max,&max_v,sizeof(vector_f));
	
	//save new values to the eeprom
	nvm_eeprom_flush_buffer();
	nvm_eeprom_load_page_to_buffer((const uint8_t*)&mag_cali);
	nvm_eeprom_atomic_write_page(1);
	
	gpio_set_pin_low(LED_GREEN_O);
}

//////////////// Test function section /////////////////

///// insert a function to your main application and you could test your hardware

//blink led if not pointed to the north (+- 10°)
//led on if pointed to the north (+- 10°)
void test_program(void)
{
	float angle= 0.0;
	
	vector_f magData = {0.0, 0.0, 0.0};;
	vector_f accelData = {0.0, 0.0, 0.0};;
	
	while(1) {
		delay_ms(10);
		if(LSM303_read_accel_f(&accelData) == 0x00) {
			delay_ms(10);
		}
		if(LSM303_read_mag(&magData) == 0x00) {
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

//switch all motors on with full power
//then rotate the angle value an bring all motors to spin at the correct values
void motor_test(void)
{
	//turn all motors individual on
	for(uint8_t i = 0;i<sizeof(motor_soft_bam);i++) {
		motor_soft_bam[i]=16;
		delay_ms(1000);
		motor_soft_bam[i]=0;
	}
	
	float angle = 0.0;
	
	while(1) {
		
		delay_ms(100);
		
		angle += 10.0;
		
		if(angle > 360.0) angle -= 360.0;
		
		uint8_t sector = angle / (360.0 / MOTOR_NUM);
		
		//stop all motors
		for(uint8_t i = 0;i<sizeof(motor_soft_bam);i++) {
			motor_soft_bam[i]=0;
		}
		
		//set one active...
		if(sector < MOTOR_NUM) {
			motor_soft_bam[sector] = 16;
			} else {
			gpio_toggle_pin(LED_GREEN_O);
		}
	}
}