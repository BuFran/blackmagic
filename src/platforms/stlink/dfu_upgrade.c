/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2013 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/scb.h>
#include "io.h"

#include "usbdfu.h"
uint32_t app_address = 0x08000000;

static uint8_t rev;
static uint32_t led_idlerun;
static uint32_t led2_state = 0;

void dfu_detach(void)
{
	/* Disconnect USB cable by resetting USB Device
	   and pulling USB_DP low*/
	rcc_periph_reset_pulse(RST_USB);
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	
	io_low(PA12);
	io_output_opendrain(PA12);
	/* Pull PB0 (T_NRST) low
	 */
	io_output_opendrain(PB0);
	io_low(PB0);
	
	SCB_VTOR = 0;
	scb_reset_core();
}

void stlink_set_rev(void)
{
	int i;

	/* First, get Board revision by pulling PC13/14 up. Read
	 *  11 for ST-Link V1, e.g. on VL Discovery, tag as rev 0
	 *  10 for ST-Link V2, e.g. on F4 Discovery, tag as rev 1
	 */
	rcc_periph_clock_enable(RCC_GPIOC);
	
	io_input_pullup(PC13);
	io_input_pullup(PC14);

	for (i = 0; i < 100; i++)
		rev = (~(gpio_get(GPIOC, GPIO14 | GPIO13)) >> 13) & 3;

	led_idlerun = (rev == 0) ? PA8 : PA9;
	
	io_output(led_idlerun);
}

int main(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	stlink_set_rev();

	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(900000);

	dfu_protect(UPD_MODE);

	/* Handle USB disconnect/connect */
	/* Just in case: Disconnect USB cable by resetting USB Device
	 * and pulling USB_DP low
	 * Device will reconnect automatically as Pull-Up is hard wired*/
	rcc_periph_reset_pulse(RST_USB);
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_GPIOA);
	
	io_low(PA12);
	io_output_opendrain(PA12);

	systick_interrupt_enable();
	systick_counter_enable();

	dfu_init(&stm32f103_usb_driver, UPD_MODE);

	dfu_main();
}

void sys_tick_handler(void)
{
	if (rev == 0) {
		io_toggle(led_idlerun);
	} else {
		if (led2_state & 1) {
			io_output(led_idlerun);
			io_high(led_idlerun);
		} else {
			io_input_analog(led_idlerun);
		}
		led2_state++;
	}
}
