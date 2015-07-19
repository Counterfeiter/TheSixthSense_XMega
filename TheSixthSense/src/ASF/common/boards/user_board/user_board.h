/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>

#define LED_GREEN_O		IOPORT_CREATE_PIN(PORTD,3)

#define PUSH_BUTTON_I	IOPORT_CREATE_PIN(PORTD,2)

#define	M8_O			IOPORT_CREATE_PIN(PORTD,4)
#define	M7_O			IOPORT_CREATE_PIN(PORTD,5)
#define	M6_O			IOPORT_CREATE_PIN(PORTC,6)
#define	M5_O			IOPORT_CREATE_PIN(PORTC,7)
#define	M4_O			IOPORT_CREATE_PIN(PORTC,2)
#define	M3_O			IOPORT_CREATE_PIN(PORTC,3)
#define	M2_O			IOPORT_CREATE_PIN(PORTC,4)
#define	M1_O			IOPORT_CREATE_PIN(PORTC,5)

#define	VBAT_MEASURE_O	IOPORT_CREATE_PIN(PORTR,1)

#define	USB_DETECT_I	IOPORT_CREATE_PIN(PORTA,6)
#define	CHARGE_DETECT_I	IOPORT_CREATE_PIN(PORTA,7)




#endif // USER_BOARD_H
