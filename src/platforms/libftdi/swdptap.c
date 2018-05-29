/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2018 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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

/* MPSSE bit-banging SW-DP interface over FTDI.
 * Slow, but usable.
 */

#include <stdio.h>
#include <assert.h>
#include <ftdi.h>

#include "general.h"
#include "swdptap.h"

static uint8_t olddir = 0;

#define MPSSE_MASK (MPSSE_TDI | MPSSE_TDO | MPSSE_TMS)
#define MPSSE_TD_MASK (MPSSE_TDI | MPSSE_TDO)
#define MPSSE_TMS_SHIFT (MPSSE_WRITE_TMS | MPSSE_LSB |\
						 MPSSE_BITMODE | MPSSE_WRITE_NEG)

#define MPSSE_TMS_IN_PORT GET_BITS_LOW
#define MPSSE_TMS_IN_PIN MPSSE_TMS

int swdptap_init(void)
{
	int err = ftdi_usb_purge_buffers(ftdic);
	if (err != 0) {
		fprintf(stderr, "ftdi_usb_purge_buffer: %d: %s\n",
			err, ftdi_get_error_string(ftdic));
		abort();
	}
	/* Reset MPSSE controller. */
	err = ftdi_set_bitmode(ftdic, 0,  BITMODE_RESET);
	if (err != 0) {
		fprintf(stderr, "ftdi_set_bitmode: %d: %s\n",
			err, ftdi_get_error_string(ftdic));
		return -1;;
	}
	/* Enable MPSSE controller. Pin directions are set later.*/
	err = ftdi_set_bitmode(ftdic, 0, BITMODE_MPSSE);
	if (err != 0) {
		fprintf(stderr, "ftdi_set_bitmode: %d: %s\n",
			err, ftdi_get_error_string(ftdic));
		return -1;;
	}
	uint8_t ftdi_init[9] = {TCK_DIVISOR, 0x01, 0x00, SET_BITS_LOW, 0,0,
				SET_BITS_HIGH, 0,0};
	ftdi_init[4]=  active_cable->dbus_data |  MPSSE_MASK;
	ftdi_init[5]= active_cable->dbus_ddr   & ~MPSSE_TD_MASK;
	ftdi_init[7]= active_cable->cbus_data;
	ftdi_init[8]= active_cable->cbus_ddr;
	platform_buffer_write(ftdi_init, 9);
	platform_buffer_flush();

	return 0;
}

static void swdptap_turnaround(uint8_t dir)
{
	if (dir == olddir)
		return;
	olddir = dir;
	uint8_t cmd[6];
	int index = 0;

	if(dir)	  { /* SWDIO goes to input */
		cmd[index++] = SET_BITS_LOW;
		cmd[index++] = active_cable->dbus_data |  MPSSE_MASK;
		cmd[index++] = active_cable->dbus_ddr  & ~MPSSE_MASK;
	}
	/* One clock cycle */
	cmd[index++] = MPSSE_TMS_SHIFT;
	cmd[index++] = 0;
	cmd[index++] = 0;
	if (!dir) {
		cmd[index++] = SET_BITS_LOW;
		cmd[index++] = active_cable->dbus_data |  MPSSE_MASK;
		cmd[index++] = active_cable->dbus_ddr  & ~MPSSE_TD_MASK;
	}
	platform_buffer_write(cmd, index);
}

bool swdptap_bit_in(void)
{
	swdptap_turnaround(1);
	uint8_t cmd[4];
	int index = 0;

	cmd[index++] = MPSSE_TMS_IN_PORT;
	cmd[index++] = MPSSE_TMS_SHIFT;
	cmd[index++] = 0;
	cmd[index++] = 0;
	platform_buffer_write(cmd, index);
	uint8_t data[1];
	platform_buffer_read(data, 1);
	return (data[0] &=  MPSSE_TMS_IN_PIN);
}

void swdptap_bit_out(bool val)
{
	swdptap_turnaround(0);
	uint8_t cmd[3];

	cmd[0] = MPSSE_TMS_SHIFT;
	cmd[1] = 0;
	cmd[2] = (val)? 1 : 0;
	platform_buffer_write(cmd, 3);
}

