/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015 Gareth McMullin <gareth@blacksphere.co.nz>
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
#include "general.h"
#include "morse.h"

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>

bool running_status = false;
static volatile uint32_t time_ms = 0;
uint32_t swd_delay_cnt = 0;

static size_t morse_tick = 0;
#ifdef PLATFORM_HAS_POWER_SWITCH
static uint8_t monitor_ticks = 0;

/* Derived from calculating (1.2V / 3.0V) * 4096 */
#define ADC_VREFINT_MAX 1638U
/* Derived from calculating (1.2V / 3.6V) * 4096 */
#define ADC_VREFINT_MIN 1365U
#endif

void platform_timing_init(void)
{
	/* Setup heartbeat timer */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* Interrupt us at 100 Hz */
	systick_set_reload(rcc_ahb_frequency / (8U * SYSTICKHZ));
	/* SYSTICK_IRQ with low priority */
	nvic_set_priority(NVIC_SYSTICK_IRQ, 14U << 4U);
	systick_interrupt_enable();
	systick_counter_enable();
}

void platform_delay(uint32_t ms)
{
	platform_timeout_s timeout;
	platform_timeout_set(&timeout, ms);
	while (!platform_timeout_is_expired(&timeout))
		continue;
}

void sys_tick_handler(void)
{
	time_ms += SYSTICKMS;

	if (morse_tick >= MORSECNT) {
		if (running_status)
			gpio_toggle(LED_PORT, LED_IDLE_RUN);
		SET_ERROR_STATE(morse_update());
		morse_tick = 0;
	} else
		++morse_tick;

#ifdef PLATFORM_HAS_POWER_SWITCH
	/* First check if target power is presently enabled */
	if (platform_target_get_power()) {
		/*
		 * Every 10 systicks, set up an ADC conversion on the 9th tick, then
		 * read back the value on the 10th, checking the internal bandgap reference
		 * is still sat in the correct range. If it diverges down, this indicates
		 * backfeeding and that VCC is being pulled higher than 3.3V. If it diverges
		 * up, this indicates either backfeeding or overcurrent and that VCC is being
		 * pulled below 3.3V. In either case, for safety, disable tpwr and set
		 * a morse error of "TPWR ERROR"
		 */

		/* If we're on the 9th tick, start the bandgap conversion */
		if (monitor_ticks == 8U) {
			uint8_t channel = ADC_CHANNEL_VREF;
			adc_set_regular_sequence(ADC1, 1, &channel);
			adc_start_conversion_direct(ADC1);
		}

		/* If we're on the 10th tick, check the result of bandgap conversion */
		if (monitor_ticks == 9U) {
			const uint32_t ref = adc_read_regular(ADC1);
			/* Clear EOC bit. The GD32F103 does not automatically reset it on ADC read. */
			ADC_SR(ADC1) &= ~ADC_SR_EOC;
			monitor_ticks = 0;

			/* Now compare the reference against the known good range */
			if (ref > ADC_VREFINT_MAX || ref < ADC_VREFINT_MIN) {
				/* Something's wrong, so turn tpwr off and set the morse blink pattern */
				platform_target_set_power(false);
				morse("TPWR ERROR", true);
			}
		} else
			++monitor_ticks;
	} else
		monitor_ticks = 0;
#endif
}

uint32_t platform_time_ms(void)
{
	return time_ms;
}

/*
 * Assume some USED_SWD_CYCLES per clock and CYCLES_PER_CNT cycles
 * per delay loop count with 2 delay loops per clock
 */

/* Values for STM32F103 at 72 MHz */
#define USED_SWD_CYCLES 22
#define CYCLES_PER_CNT  10

void platform_max_frequency_set(uint32_t freq)
{
	uint32_t divisor = rcc_ahb_frequency - USED_SWD_CYCLES * freq;
	/* If we now have an insanely big divisor, the above operation wrapped to a negative signed number. */
	if (divisor >= 0x80000000U) {
		swd_delay_cnt = 0;
		return;
	}
	divisor /= 2U;
	swd_delay_cnt = divisor / (CYCLES_PER_CNT * freq);
	if (swd_delay_cnt * (CYCLES_PER_CNT * freq) < divisor)
		++swd_delay_cnt;
}

uint32_t platform_max_frequency_get(void)
{
	uint32_t ret = rcc_ahb_frequency;
	ret /= USED_SWD_CYCLES + CYCLES_PER_CNT * swd_delay_cnt;
	return ret;
}
