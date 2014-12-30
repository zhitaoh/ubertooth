/*
 * Copyright 2014 Zhitao He
 *
 * This file is NOT part of Project Ubertooth.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <string.h>

#include "ubertooth.h"
#include "../bluetooth_rxtx/ubertooth_usb.h"
#include "ubertooth_interface.h"

volatile u32 clkn;                       // clkn 3200 Hz counter

const char compile_info[] =
	"ubertooth " GIT_REVISION " (" COMPILE_BY "@" COMPILE_HOST ") " TIMESTAMP;
/* const char compile_info[] = */
/* 	"ubertooth zhitao"; */

static void clkn_init()
{
	/*
	 * Because these are reset defaults, we're assuming TIMER0 is powered on
	 * and in timer mode.  The TIMER0 peripheral clock should have been set by
	 * clock_start().
	 */

	/* stop and reset the timer to zero */
	T0TCR = TCR_Counter_Reset;
	clkn = 0;

	/*
	 * The peripheral clock has a period of 20ns.  5 pclk periods
	 * makes one CLK100NS period (100 ns).
	 */
	T0PR = 4;

	/* 3125 * 100 ns = 312.5 us, the Bluetooth clock (CLKN). */
	T0MR0 = 3124;
	T0MCR = TMCR_MR0R | TMCR_MR0I;
	ISER0 = ISER0_ISE_TIMER0;

	/* start timer */
	T0TCR = TCR_Counter_Enable;
}

/* Update CLKN. */
void TIMER0_IRQHandler()
{
	if (T0IR & TIR_MR0_Interrupt) {

		clkn++;
		T0IR = TIR_MR0_Interrupt;
	}
}

/* Sleep (busy wait) for 'millis' milliseconds. The 'wait' routines in
 * ubertooth.c are matched to the clock setup at boot time and can not
 * be used while the board is running at 100MHz. */
static void msleep(uint32_t millis)
{
	uint32_t stop_at = clkn + millis * 3125 / 1000;  // millis -> clkn ticks
	do { } while (clkn < stop_at);                   // TODO: handle wrapping
}

static int vendor_request_handler(u8 request, u16 *request_params, u8 *data, int *data_len)
{
	u32 command[5];
	u32 result[5];
	u64 ac_copy;
	int i; // loop counter
	u32 clock;
	int clock_offset;
	u8 length; // string length
	usb_pkt_rx *p = NULL;
	u16 reg_val;

	switch (request) {

	case UBERTOOTH_FLASH:
		bootloader_ctrl = DFU_MODE;
		reset();
		break;

	case UBERTOOTH_GET_COMPILE_INFO:
		length = (u8)strlen(compile_info);
		data[0] = length;
		memcpy(&data[1], compile_info, length);
		*data_len = 1 + length;
		break;

	case UBERTOOTH_GET_USRLED:
		data[0] = (USRLED) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_USRLED:
		if (request_params[0])
			USRLED_SET;
		else
			USRLED_CLR;
		break;

	case UBERTOOTH_GET_RXLED:
		data[0] = (RXLED) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_RXLED:
		if (request_params[0])
			RXLED_SET;
		else
			RXLED_CLR;
		break;

	case UBERTOOTH_GET_TXLED:
		data[0] = (TXLED) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_TXLED:
		if (request_params[0])
			TXLED_SET;
		else
			TXLED_CLR;
		break;

	case UBERTOOTH_GET_PARTNUM:
		command[0] = 54; /* read part number */
		iap_entry(command, result);
		data[0] = result[0] & 0xFF; /* status */
		data[1] = result[1] & 0xFF;
		data[2] = (result[1] >> 8) & 0xFF;
		data[3] = (result[1] >> 16) & 0xFF;
		data[4] = (result[1] >> 24) & 0xFF;
		*data_len = 5;
		break;

	default:
		return 0;
	}

	return 1;
}

int main()
{
	ubertooth_init(); // (in ubertooth.c): {gpio_init(); cc2420_init(); and clock_start();}
	clkn_init(); // needed by handle_usb()
	ubertooth_usb_init(vendor_request_handler);

	while (1) {
		handle_usb(clkn);
		TXLED_SET;
		msleep(500);
		RXLED_SET;
		msleep(500);
		USRLED_SET;
		msleep(500);
		TXLED_CLR;
		RXLED_CLR;
		USRLED_CLR;
		msleep(500);
	}

	return 0;
}
