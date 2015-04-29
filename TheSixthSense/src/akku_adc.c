/*
 * akku_adc.c
 *
 * Created: 25.04.2015 14:31:41
 *  Author: Sebastian Foerster
 */ 


#include "akku_adc.h"

#define XMEGA_FIXED_OFFSET		190
#define FILTER_DIS				4



/*static void adc_handler(ADC_t *adc, uint8_t ch_mask, adc_result_t result)
{
	static uint32_t filter_var = 0;
	
	filter_var = filter_var - (filter_var >> FILTER_DIS) +  (uint32_t)result;
	result = ((uint16_t)(filter_var >> FILTER_DIS));
	
	adc_value = result - XMEGA_FIXED_OFFSET - 0x07FF;
	//gpio_toggle_pin(LED_GREEN_O);
}*/

#define ADC_AVR		8

uint16_t read_bat(void)
{
	gpio_set_pin_high(VBAT_MEASURE_O);
	
	//wait 10 ms to charge the cap
	delay_ms(20);
	
	uint16_t adc_value = 0;
	/* Sum 8 samples */
	for (uint8_t i = 0; i < ADC_AVR; i++) {
		delay_ms(5);
		/* Do one conversion to find offset */
		adc_start_conversion(&ADCA, ADC_CH0);
		adc_wait_for_interrupt_flag(&ADCA, ADC_CH0);
		adc_clear_interrupt_flag(&ADCA, ADC_CH0);

		/* Accumulate conversion results */
		adc_value += adc_get_result(&ADCA, ADC_CH0);
	}
	
	gpio_set_pin_low(VBAT_MEASURE_O);
	
	return adc_value / ADC_AVR;
}

void init_adc(void)
{
	
	struct adc_config         adc_conf;
	struct adc_channel_config adcch_conf;
	
	// Initialize configuration structures.
	adc_read_configuration(&ADCA, &adc_conf);
	adcch_read_configuration(&ADCA, ADC_CH0, &adcch_conf);
	
	adcch_disable_interrupt(&adcch_conf);
	adcch_write_configuration(&ADCA, ADC_CH0, &adcch_conf);

	/* Configure the ADC module:
	 * - unsigned, 12-bit results
	 * - 10 kHz maximum clock rate
	 * - auto conversion triggering
	 */
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC); //   VCC/1.6 -> ref
	adc_set_clock_rate(&adc_conf, 10000UL);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);

	adc_write_configuration(&ADCA, &adc_conf);

	/* Configure ADC channel 1:
	 * - single-ended measurement from configured input pin
	 */
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN0, ADCCH_NEG_NONE,1);

	adcch_write_configuration(&ADCA, ADC_CH0, &adcch_conf);

	// Enable the ADC
	adc_enable(&ADCA);
}