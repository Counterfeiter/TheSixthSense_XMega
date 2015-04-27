/*
 * akku_adc.c
 *
 * Created: 25.04.2015 14:31:41
 *  Author: Sebastian Foerster
 */ 


#include "akku_adc.h"

#define XMEGA_FIXED_OFFSET		190
#define FILTER_DIS				4


volatile int16_t adc_value;

static void adc_handler(ADC_t *adc, uint8_t ch_mask, adc_result_t result)
{
	static uint32_t filter_var = 0;
	
	filter_var = filter_var - (filter_var >> FILTER_DIS) +  (uint32_t)result;
	result = ((uint16_t)(filter_var >> FILTER_DIS));
	
	adc_value = result - XMEGA_FIXED_OFFSET - 0x07FF;
	//gpio_toggle_pin(LED_GREEN_O);
}

uint8_t init_adc(void)
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
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12,
			ADC_REF_AREFA);
	adc_set_clock_rate(&adc_conf, 10000UL);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);

	adc_write_configuration(&ADCA, &adc_conf);

	/* Configure ADC channel 1:
	 * - single-ended measurement from configured input pin
	 * - interrupt flag set on completed conversion
	 */
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN1, ADCCH_NEG_NONE,1);
	adcch_set_interrupt_mode(&adcch_conf, ADCCH_MODE_COMPLETE);
	//adcch_enable_interrupt(&adcch_conf);
	adc_set_callback(&ADCA, &adc_handler);

	adcch_write_configuration(&ADCA, ADC_CH0, &adcch_conf);

	// Enable the ADC
	adc_enable(&ADCA);
	adc_start_conversion(&ADCA, ADC_CH0);
	
	uint16_t offset = 0;
	/* Sum 16 samples */
	for (uint8_t i = 0; i < 8; i++) {
		/* Do one conversion to find offset */
		adc_start_conversion(&ADCA, ADC_CH0);
		adc_wait_for_interrupt_flag(&ADCA, ADC_CH0);

		/* Accumulate conversion results */
		offset += adc_get_result(&ADCA, ADC_CH0);
	}
	
	adc_disable(&ADCA);
	
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_FREERUN, 1, 0);
	adc_write_configuration(&ADCA, &adc_conf);
	
	adcch_enable_interrupt(&adcch_conf);
	adcch_write_configuration(&ADCA, ADC_CH0, &adcch_conf);
	
	adc_enable(&ADCA);
	adc_start_conversion(&ADCA, ADC_CH0);
	
	if((offset/8) > 200) {
		return true;
	}
	
	return false;
}