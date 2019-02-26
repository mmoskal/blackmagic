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

/* This file implements the platform specific functions for the ST-Link
 * implementation.
 */

#include "general.h"
#include "cdcacm.h"
#include "usbuart.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/adc.h>

uint8_t running_status;

static uint32_t rev;

int platform_hwversion(void)
{
	return rev;
}

#include <libopencm3/stm32/flash.h>

void rcc_clock_setup(void)
{
	/* Enable internal high-speed oscillator. */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Select HSI as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

	/* Enable external high-speed oscillator 16MHz. */
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);

	/*
	 * Set prescalers for AHB, ADC, ABP1, ABP2.
	 * Do this before touching the PLL (TODO: why?).
	 */
	rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);    /* Set. 72MHz Max. 72MHz */
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);  /* Set. 12MHz Max. 14MHz */
	rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);     /* Set. 36MHz Max. 36MHz */
	rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);    /* Set. 72MHz Max. 72MHz */

	/*
	 * Sysclk runs with 72MHz -> 2 waitstates.
	 * 0WS from 0-24MHz
	 * 1WS from 24-48MHz
	 * 2WS from 48-72MHz
	 */
	flash_set_ws(FLASH_ACR_LATENCY_2WS);

	/*
	 * Set the PLL multiplication factor to 9.
	 * 16MHz (external) * 9 (multiplier) / 2 (PLLXTPRE_HSE_CLK_DIV2) = 72MHz
	 */
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL9);

	/* Select HSI as PLL source. */
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);

	/*
	 * Divide external frequency by 2 before entering PLL
	 * (only valid/needed for HSE).
	 */
	rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK_DIV2);

	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);

	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

	/* Set the peripheral clock frequencies used */
	rcc_ahb_frequency = 72000000;
	rcc_apb1_frequency = 36000000;
	rcc_apb2_frequency = 72000000;
}


void platform_init(void)
{
	rev = detect_rev();
	SCS_DEMCR |= SCS_DEMCR_VC_MON_EN;
#ifdef ENABLE_DEBUG
	void initialise_monitor_handles(void);
	initialise_monitor_handles();
#endif
int i = 20000000;
while(i--) asm("nop");
	rcc_clock_setup();

	/* Setup GPIO ports */
	gpio_set_mode(TMS_PORT, GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_INPUT_FLOAT, TMS_PIN);
	gpio_set_mode(TCK_PORT, GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, TCK_PIN);
	gpio_set_mode(TDI_PORT, GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, TDI_PIN);

gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);


	platform_srst_set_val(false);

	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, LED_IDLE_RUN);

	/* Relocate interrupt vector table here */
	extern int vector_table;
	SCB_VTOR = (uint32_t)&vector_table;

	platform_timing_init();
	// if (rev > 1) /* Reconnect USB */
		gpio_set(GPIOA, GPIO15);
	cdcacm_init();
	/* Don't enable UART if we're being debugged. */
	if (!(SCS_DEMCR & SCS_DEMCR_TRCENA))
		usbuart_init();
}

void platform_srst_set_val(bool assert)
{
	if (assert) {
		gpio_set_mode(SRST_PORT, GPIO_MODE_OUTPUT_50_MHZ,
		              GPIO_CNF_OUTPUT_OPENDRAIN, SRST_PIN);
		gpio_clear(SRST_PORT, SRST_PIN);
		while (gpio_get(SRST_PORT, SRST_PIN)) {};
	} else {
		gpio_set_mode(SRST_PORT, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_PULL_UPDOWN, SRST_PIN);
		gpio_set(SRST_PORT, SRST_PIN);
		while (!gpio_get(SRST_PORT, SRST_PIN)) {};
	}
}

bool platform_srst_get_val()
{
	return gpio_get(SRST_PORT, SRST_PIN) == 0;
}

const char *platform_target_voltage(void)
{
	return "unknown";
}
