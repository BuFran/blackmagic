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

uint32_t app_address = 0x08002000;

void dfu_detach(void)
{
        /* USB device must detach, we just reset... */
	scb_reset_system();
}

int main(void)
{
	/* Check the force bootloader pin*/
	rcc_periph_clock_enable(RCC_GPIOB);
	if(io_is_set(PB12))
		dfu_jump_app_if_valid();

	dfu_protect(DFU_MODE);

#if defined(FCLK_16MHZ)
	rcc_clock_setup_in_hse_16mhz_out_72mhz();
#else
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
#endif
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(900000);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USB);
	io_input(PA8);

	systick_interrupt_enable();
	systick_counter_enable();

	io_output(PB11);
	io_input(PB2);
	io_input(PB10);

	dfu_init(&stm32f103_usb_driver, DFU_MODE);

	io_high(PA8);
	io_output(PA8);

	dfu_main();
}

void sys_tick_handler(void)
{
	io_toggle(PB11); /* LED2 on/off */
}

