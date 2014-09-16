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
#ifndef __PLATFORM_H
#define __PLATFORM_H

#include <stdint.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/memorymap.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>

#include <setjmp.h>
#include <alloca.h>

#include "io.h"

#include "gdb_packet.h"

#define INLINE_GPIO
#define CDCACM_PACKET_SIZE 	64
#define BOARD_IDENT             "Black Magic Probe (STLINK), (Firmware 1.5" VERSION_SUFFIX ", build " BUILDDATE ")"
#define BOARD_IDENT_DFU		"Black Magic (Upgrade) for STLink/Discovery, (Firmware 1.5" VERSION_SUFFIX ", build " BUILDDATE ")"
#define BOARD_IDENT_UPD		"Black Magic (DFU Upgrade) for STLink/Discovery, (Firmware 1.5" VERSION_SUFFIX ", build " BUILDDATE ")"
#define DFU_IDENT               "Black Magic Firmware Upgrade (STLINK)"
#define DFU_IFACE_STRING	"@Internal Flash   /0x08000000/8*001Ka,56*001Kg"
#define UPD_IFACE_STRING	"@Internal Flash   /0x08000000/8*001Kg"

extern usbd_device *usbdev;
#define CDCACM_GDB_ENDPOINT	1
#define CDCACM_UART_ENDPOINT	3

/* Important pin mappings for STM32 implementation:
 *
 * LED0 = 	PB2	(Yellow LED : Running)
 * LED1 = 	PB10	(Yellow LED : Idle)
 * LED2 = 	PB11	(Red LED    : Error)
 *
 * TPWR = 	RB0 (input) -- analogue on mini design ADC1, ch8
 * nTRST = 	PB1
 * SRST_OUT = 	PA2
 * TDI = 	PA3
 * TMS = 	PA4 (input for SWDP)
 * TCK = 	PA5
 * TDO = 	PA6 (input)
 * nSRST = 	PA7 (input)
 *
 * USB cable pull-up: PA8
 * USB VBUS detect:  PB13 -- New on mini design.
 *                           Enable pull up for compatibility.
 * Force DFU mode button: PB12
 */

/* Hardware definitions... */
#define PIN_TDI			PA7
#define PIN_TDO			PA6
#define PIN_TCK			PA5
#define PIN_TMS			PB14

#define PIN_SWDIO		PIN_TMS
#define PIN_SWCLK		PIN_TCK

#define PIN_SRST_V1		PB1
#define PIN_SRST_V2		PB0
#define PIN_SRST		pin_srst

/* Use PC14 for a "dummy" uart led. So we can observere at least with scope*/
#define PIN_LED_UART		PC14

#define PIN_LED_IDLERUN_V1	PA8
#define PIN_LED_IDLERUN_V2	PA9
#define PIN_LED_IDLERUN		led_idlerun


#define USB_DRIVER      stm32f103_usb_driver
#define USB_IRQ         NVIC_USB_LP_CAN_RX0_IRQ
#define USB_ISR          usb_lp_can_rx0_isr
/* Interrupt priorities.  Low numbers are high priority.
 * For now USART2 preempts USB which may spin while buffer is drained.
 * TIM3 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB		(2 << 4)
#define IRQ_PRI_USBUSART	(1 << 4)
#define IRQ_PRI_USBUSART_TIM	(3 << 4)
#define IRQ_PRI_USB_VBUS	(14 << 4)
#define IRQ_PRI_TIM3		(0 << 4)

#define USBUSART		USART2
#define USBUSART_PIN_TX		PA2
#define USBUSART_CR1		USART2_CR1
#define USBUSART_IRQ		NVIC_USART2_IRQ
#define USBUSART_CLK		RCC_USART2
#define USBUSART_ISR		usart2_isr

#define USBUSART_TIM		TIM4
#define USBUSART_TIM_CLK	RCC_TIM4
#define USBUSART_TIM_IRQ	NVIC_TIM4_IRQ
#define USBUSART_TIM_ISR	tim4_isr


#define DEBUG(...)

extern uint8_t running_status;
extern volatile uint32_t timeout_counter;

extern jmp_buf fatal_error_jmpbuf;

extern const char *morse_msg;
extern uint32_t led_idlerun;
extern uint32_t pin_srst;

#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)	io_set(led_idlerun, state)

#define PLATFORM_SET_FATAL_ERROR_RECOVERY()	{setjmp(fatal_error_jmpbuf);}
#define PLATFORM_FATAL_ERROR(error)	do { 		\
	if(running_status) gdb_putpacketz("X1D");	\
		else gdb_putpacketz("EFF");		\
	running_status = 0;				\
	target_list_free();				\
	longjmp(fatal_error_jmpbuf, (error));		\
} while (0)

int platform_init(void);
void morse(const char *msg, char repeat);
const char *platform_target_voltage(void);
void platform_delay(uint32_t delay);
void platform_srst_set_val(bool assert);

/* <cdcacm.c> */
void cdcacm_init(void);
/* Returns current usb configuration, or 0 if not configured. */
int cdcacm_get_config(void);
int cdcacm_get_dtr(void);

/* <platform.h> */
void uart_usb_buf_drain(uint8_t ep);

/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define vasprintf vasiprintf

void disconnect_usb(void);
void assert_boot_pin(void);
void setup_vbus_irq(void);
void platform_srst_set_val(bool assert);

#endif