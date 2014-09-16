/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
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

/* This file implements the platform specific functions for the STM32
 * implementation.
 */

#include "platform.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/adc.h>

#include "jtag_scan.h"
#include <usbuart.h>

#include <ctype.h>

uint8_t running_status;
volatile uint32_t timeout_counter;

jmp_buf fatal_error_jmpbuf;

static void morse_update(void);

static void adc_init(void);

/* Pins PB[7:5] are used to detect hardware revision.
 * 000 - Original production build.
 * 001 - Mini production build.
 */
int platform_hwversion(void)
{
	static int hwversion = -1;
	if (hwversion == -1) {
		io_input_pulldown(PB5);
		io_input_pulldown(PB6);
		io_input_pulldown(PB7);

		hwversion = gpio_get(GPIOB, GPIO7 | GPIO6 | GPIO5) >> 5;
	}
	return hwversion;
}

int platform_init(void)
{
#if defined(FCLK_16MHZ)
	rcc_clock_setup_in_hse_16mhz_out_72mhz();
#else
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
#endif

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_CRC);

	/* Setup GPIO ports */
	io_input(PIN_USB_PU);
	io_output(PIN_TMS);
	io_output(PIN_TCK);
	io_output(PIN_TDI);

	/* This needs some fixing... */
	/* Toggle required to sort out line drivers... */
	gpio_port_write(GPIOA, 0x8100);
	gpio_port_write(GPIOB, 0x2000);

	gpio_port_write(GPIOA, 0x8180);
	gpio_port_write(GPIOB, 0x2002);
	
	io_output(PIN_LED_UART);
	io_output(PIN_LED_IDLERUN);
	io_output(PIN_LED_ERROR);

	/* FIXME: This pin in intended to be input, but the TXS0108 fails
	 * to release the device from reset if this floats. */
	io_output(PA7);
	
	/* Enable SRST output. Original uses a NPN to pull down, so setting the
	 * output HIGH asserts. Mini is directly connected so use open drain output
	 * and set LOW to assert.
	 */
	platform_srst_set_val(false);
	
	if (platform_hwversion() == 0)
		io_output(PIN_SRST);
	else
		io_output_opendrain(PIN_SRST);
		
	/* Enable internal pull-up on PWR_BR so that we don't drive
	   TPWR locally or inadvertently supply power to the target. */
	if (platform_hwversion () > 0) {
		io_input_pullup(PIN_PWR_BR);
	}

	/* Setup heartbeat timer */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(900000);	/* Interrupt us at 10 Hz */
	SCB_SHPR(11) &= ~((15 << 4) & 0xff);
	SCB_SHPR(11) |= ((14 << 4) & 0xff);
	systick_interrupt_enable();
	systick_counter_enable();

	if (platform_hwversion() > 0) {
		adc_init();
	} else {
		io_input_pullup(PB0);
	}

	SCB_VTOR = 0x2000;	// Relocate interrupt vector table here

	cdcacm_init();
	usbuart_init();

	jtag_scan(NULL);

	return 0;
}

void platform_srst_set_val(bool assert)
{
	io_set(PIN_SRST, (platform_hwversion() == 0) ? assert : !assert);
}

void platform_delay(uint32_t delay)
{
	timeout_counter = delay;
	while(timeout_counter);
}

void sys_tick_handler(void)
{
	if(running_status)
		io_toggle(PIN_LED_IDLERUN);

	if(timeout_counter)
		timeout_counter--;

	morse_update();
}


/* Morse code patterns and lengths */
static const struct {
	uint16_t code;
	uint8_t bits;
} morse_letter[] = {
	{        0b00011101,  8}, // 'A' .-
	{    0b000101010111, 12}, // 'B' -...
	{  0b00010111010111, 14}, // 'C' -.-.
	{      0b0001010111, 10}, // 'D' -..
	{            0b0001,  4}, // 'E' .
	{    0b000101110101, 12}, // 'F' ..-.
	{    0b000101110111, 12}, // 'G' --.
	{      0b0001010101, 10}, // 'H' ....
	{          0b000101,  6}, // 'I' ..
	{0b0001110111011101, 16}, // 'J' .---
	{    0b000111010111, 12}, // 'K' -.-
	{    0b000101011101, 12}, // 'L' .-..
	{      0b0001110111, 10}, // 'M' --
	{        0b00010111,  8}, // 'N' -.
	{  0b00011101110111, 14}, // 'O' ---
	{  0b00010111011101, 14}, // 'P' .--.
	{0b0001110101110111, 16}, // 'Q' --.-
	{      0b0001011101, 10}, // 'R' .-.
	{        0b00010101,  8}, // 'S' ...
	{          0b000111,  6}, // 'T' -
	{      0b0001110101, 10}, // 'U' ..-
	{    0b000111010101, 12}, // 'V' ...-
	{    0b000111011101, 12}, // 'W' .--
	{  0b00011101010111, 14}, // 'X' -..-
	{0b0001110111010111, 16}, // 'Y' -.--
	{  0b00010101110111, 14}, // 'Z' --..
};


const char *morse_msg;
static const char * volatile morse_ptr;
static char morse_repeat;

void morse(const char *msg, char repeat)
{
	morse_msg = morse_ptr = msg;
	morse_repeat = repeat;
	SET_ERROR_STATE(0);
}

static void morse_update(void)
{
	static uint16_t code;
	static uint8_t bits;

	if(!morse_ptr) return;

	if(!bits) {
		char c = *morse_ptr++;
		if(!c) {
			if(morse_repeat) {
				morse_ptr = morse_msg;
				c = *morse_ptr++;
			} else {
				morse_ptr = 0;
				return;
			}
		}
		if((c >= 'A') && (c <= 'Z')) {
			c -= 'A';
			code = morse_letter[c].code;
			bits = morse_letter[c].bits;
		} else {
			code = 0; bits = 4;
		}
	}
	SET_ERROR_STATE(code & 1);
	code >>= 1; bits--;
}

static void adc_init(void)
{
	rcc_periph_clock_enable(RCC_ADC1);

	io_input_analog(PB0);

	adc_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	for (int i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibration(ADC1);
}

const char *platform_target_voltage(void)
{
	if (platform_hwversion() == 0)
		return io_is_set(PB0) ? "OK" : "ABSENT!";

	static char ret[] = "0.0V";
	const uint8_t channel = 8;
	adc_set_regular_sequence(ADC1, 1, (uint8_t*)&channel);

	adc_start_conversion_direct(ADC1);

	/* Wait for end of conversion. */
	while (!adc_eoc(ADC1));

	uint32_t val = adc_read_regular(ADC1) * 99; /* 0-4095 */
	ret[0] = '0' + val / 81910;
	ret[2] = '0' + (val / 8191) % 10;

	return ret;
}

void assert_boot_pin(void)
{
	io_output(PB12);
	io_low(PB12);
}

void exti15_10_isr(void)
{
	if (io_is_set(PIN_USB_VBUS)) {
		/* Drive pull-up high if VBUS connected */
		io_output(PIN_USB_PU);
	} else {
		/* Allow pull-up to float if VBUS disconnected */
		io_input(PIN_USB_PU);
	}

	exti_reset_request(io_pin(PIN_USB_VBUS));
}

void setup_vbus_irq(void)
{
	nvic_set_priority(USB_VBUS_IRQ, IRQ_PRI_USB_VBUS);
	nvic_enable_irq(USB_VBUS_IRQ);

	io_high(PIN_USB_PU);
	io_input_pullup(PIN_USB_VBUS);
	
	/* Configure EXTI for USB VBUS monitor */
	exti_select_source(io_pin(PIN_USB_VBUS), io_port(PIN_USB_VBUS));
	exti_set_trigger(io_pin(PIN_USB_VBUS), EXTI_TRIGGER_BOTH);
	exti_enable_request(io_pin(PIN_USB_VBUS));

	exti15_10_isr();
}
