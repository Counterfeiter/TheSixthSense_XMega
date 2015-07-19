/**
 * \file
 *
 * \brief User board initialization template
 *
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>

void board_init(void)
{
	/* This function is meant to contain board-specific initialization code
	 * for, e.g., the I/O pins. The initialization can rely on application-
	 * specific board configuration, found in conf_board.h.
	 */
	
	ioport_configure_pin(LED_GREEN_O, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	
	ioport_configure_pin(USB_DETECT_I, IOPORT_DIR_INPUT | IOPORT_PULL_DOWN);
	ioport_configure_pin(CHARGE_DETECT_I, IOPORT_DIR_INPUT | IOPORT_PULL_DOWN);
	
	ioport_configure_pin(PUSH_BUTTON_I, IOPORT_DIR_INPUT | IOPORT_PULL_UP);
	
	ioport_configure_pin(VBAT_MEASURE_O, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	
	ioport_configure_pin(M1_O, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(M2_O, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(M3_O, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(M4_O, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(M5_O, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(M6_O, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(M7_O, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(M8_O, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	
	
}
