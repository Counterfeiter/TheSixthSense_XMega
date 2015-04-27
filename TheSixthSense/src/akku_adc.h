/*
 * akku_adc.h
 *
 * Created: 25.04.2015 14:32:00
 *  Author: Sebastian Foerster
 */ 


#ifndef AKKU_ADC_H_
#define AKKU_ADC_H_


#include <asf.h>

uint8_t init_adc(void);

volatile int16_t adc_value;


#endif /* AKKU_ADC_H_ */