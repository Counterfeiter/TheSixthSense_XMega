/*
 * akku_adc.h
 *
 * Created: 25.04.2015 14:32:00
 *  Author: Sebastian Foerster
 */ 


#ifndef AKKU_ADC_H_
#define AKKU_ADC_H_


#include <asf.h>

void		init_adc(void);
uint16_t	read_bat(void);


#endif /* AKKU_ADC_H_ */