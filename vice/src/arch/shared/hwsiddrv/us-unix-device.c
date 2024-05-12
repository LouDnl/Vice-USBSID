/*
 * us-unix-linux.c - Linux specific usbsid driver.
 *
 * Written by
 *  Simon White <sidplay2@yahoo.com>
 *  Marco van den Heuvel <blackystardust68@yahoo.com>
 *  LouDnl
 *
 * This file is part of VICE, modified from the other hardware sid sources.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 *  02111-1307  USA.
 *
 */

#include "vice.h"

#ifdef UNIX_COMPILE

#if defined(HAVE_USBSID)

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <libftdi1/ftdi.h>

#include "alarm.h"
#include "us-unix.h"
#include "usbsid.h"
#include "log.h"
#include "maincpu.h"
#include "sid-resources.h"
#include "types.h"

#include "usbsid-macros.h"

/* Approx 3 PAL screen updates */
// #define USBSID_DELAY_CYCLES 50000

/* FIXME: currently only 1 SID is supported */
#define MAXSID 2

#define VENDOR 0x0403
#define PRODUCT 0x6010

#define PORT1 INTERFACE_A // AD0 ~ AD7
#define PORT2 INTERFACE_B // BD0 ~ BD7

/* #define DEBUG_USBSIDFTDI */

#if defined(DEBUG_USBSIDFTDI)
#define DBG(...) printf(__VA_ARGS__)
#else
#define DBG(...)
#endif

static int sids_found = -1;
static int usid_dev = -1;
// static CLOCK usid_main_clk;
// static CLOCK usid_alarm_clk;
// static alarm_t *usid_alarm = 0;
struct ftdi_context *ftdi1;
struct ftdi_context *ftdi2;
int f_port;
unsigned char gpiobuff1[1];
unsigned char gpiobuff2[1];
unsigned char readbuff[1];

static int usbsid_init(void)
{
    /* Already open */
    if (usid_dev >= 0) {
        return -1;
    }

    /* Open device port 1*/
    if ((ftdi1 = ftdi_new()) == 0) {
        fprintf(stderr, "ftdi_new failed\n");
        return -1;
    }
    ftdi_set_interface(ftdi1, PORT1);
    f_port = ftdi_usb_open(ftdi1, VENDOR, PRODUCT);
    if (f_port < 0 && f_port != -5) {
        fprintf(stderr, "Unable to open ftdi device: %d (%s)\n", f_port, ftdi_get_error_string(ftdi1));
        ftdi_free(ftdi1);
        return -1;
    }
    /* Open device port 2*/
    if ((ftdi2 = ftdi_new()) == 0) {
        fprintf(stderr, "ftdi_new failed\n");
        return -1;
    }
    ftdi_set_interface(ftdi2, PORT2);
    f_port = ftdi_usb_open(ftdi2, VENDOR, PRODUCT);
    if (f_port < 0 && f_port != -5) {
        fprintf(stderr, "Unable to open ftdi device: %d (%s)\n", f_port, ftdi_get_error_string(ftdi2));
        ftdi_free(ftdi2);
        return -1;
    }
    /* PORT1 DATA ~ ALL PINS INPUT */
    ftdi_set_bitmode(ftdi1, 0x0, BITMODE_BITBANG);
    /* PORT2 RES,RW,CS & ADDR ~ ALL PINS OUTPUT */
    ftdi_set_bitmode(ftdi2, 0xFF, BITMODE_BITBANG);
    usid_dev = 0;  /* Placebo check */

    if (usid_dev < 0)
    {
        log_error(LOG_DEFAULT, "Could not open SID device USBSID.");
        return -1;
    }

    /* Make sure we have atleast sid */
    // f_port = ftdi_read_pins(ftdi2, gpiobuff2);

    /* WRITE PORT1 DATA */
    gpiobuff1[0] = 0x0;  /* PORT1 DATA ~ 0b00000000 ~ all LOW */
    f_port = ftdi_write_data(ftdi1, gpiobuff1, 1);
    /* WRITE PORT2 RES,RW,CS & ADDR */
    gpiobuff2[0] = 0x0;  /* PORT2 ADDR ~ 0b00000000 ~ all LOW */
    f_port = ftdi_write_data(ftdi2, gpiobuff2, 1);
    gpiobuff2[0] = 0x5;  /* PORT2 ADDR ~ 0b00000101 ~ CS & RES HIGH */
    f_port = ftdi_write_data(ftdi2, gpiobuff2, 1);
    return 0;
}

int us_device_open(void)
{
    if (!sids_found) {
        return -1;
    }

    if (sids_found > 0) {
        return 0;
    }

    sids_found = 0;

    log_message(LOG_DEFAULT, "Detecting Linux usbsid boards.");

    if (usbsid_init() < 0) {
        return -1;
    }
    // usid_alarm = alarm_new(maincpu_alarm_context, "usbsid", usbsid_alarm_handler, 0);
    sids_found = 1;
    usbsid_reset();
    log_message(LOG_DEFAULT, "Linux usbsid: opened.");
    return 0;
}

int us_device_close(void)
{
    /* Driver cleans up after itself */
    if (usid_dev >= 0) {

        ftdi_disable_bitbang(ftdi1);
        ftdi_disable_bitbang(ftdi2);
        ftdi_usb_close(ftdi1);
        ftdi_free(ftdi1);
        ftdi_usb_close(ftdi2);
        ftdi_free(ftdi2);

    }
    // alarm_destroy(usid_alarm);
    // usid_alarm = 0;
    sids_found = -1;
    log_message(LOG_DEFAULT, "Linux usbsid: closed.");
    return 0;
}

void us_device_reset(void)
{
    if (sids_found > 0) {
        // usid_main_clk  = maincpu_clk;
        // usid_alarm_clk = USBSID_DELAY_CYCLES;
        // alarm_set(usid_alarm, USBSID_DELAY_CYCLES);

        /* READ PORT2 RES,RW,CS & ADDR */
        f_port = ftdi_read_pins(ftdi2, gpiobuff2);
        /* WRITE PORT2 RES,RW,CS & ADDR */
        gpiobuff2[0] &= ~(1 << 0);  /* PORT2 ADDR ~ 0b00000000 ~ RES LOW */
        f_port = ftdi_write_data(ftdi2, gpiobuff2, 1);
        /* WRITE PORT2 RES,RW,CS & ADDR */
        gpiobuff2[0] |= 1 << 0;  /* PORT2 ADDR ~ 0b00000001 ~ RES HIGH */
        f_port = ftdi_write_data(ftdi2, gpiobuff2, 1);
        DBG("USBSID reset!\n");
    }
}

int us_device_read(uint16_t addr, int chipno)
{
    if (chipno < MAXSID && addr < 0x20 && usid_dev >= 0) {
        // CLOCK cycles = maincpu_clk - usid_main_clk - 1;
        // usid_main_clk = maincpu_clk;
        uint8_t value = 0;
        readbuff[0] = 0x0;

        DBG("START us_device_read\n");
        /* READ PORT2 RES,RW,CS & ADDR */
        f_port = ftdi_read_pins(ftdi2, gpiobuff2);
        DBG("us_device_read ftdi_read_pins ftdi2: dataport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addr: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addrport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 "\n",
            readbuff[0], PRINTF_BYTE_TO_BINARY_INT8(readbuff[0]), addr, PRINTF_BYTE_TO_BINARY_INT8(addr), gpiobuff2[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff2[0]));
        /* save only the first 3 pins ~ 0b00000111 mask */
        gpiobuff2[0] &= 0x5;
        /* merge buff and bitshift addr 3 left ~ 0b11111000 */
        gpiobuff2[0] = gpiobuff2[0] | (addr & 0x1F) << 3;
        /* WRITE PORT2 RES,RW,CS & ADDR */
        f_port = ftdi_write_data(ftdi2, gpiobuff2, 1);
        DBG("us_device_read ftdi_write_data ftdi2: dataport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addr: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addrport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 "\n",
            readbuff[0], PRINTF_BYTE_TO_BINARY_INT8(readbuff[0]), addr, PRINTF_BYTE_TO_BINARY_INT8(addr), gpiobuff2[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff2[0]));

        /* PORT2 CS LOW & RW HIGH (strobe) */
        gpiobuff2[0] &= ~(1 << 2);  // 0b11111001
        gpiobuff2[0] |= (1 << 1);   // 0b11111011
        f_port = ftdi_write_data(ftdi2, gpiobuff2, 1);
        DBG("us_device_read ftdi_write_data ftdi2: dataport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addr: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addrport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 "\n",
            readbuff[0], PRINTF_BYTE_TO_BINARY_INT8(readbuff[0]), addr, PRINTF_BYTE_TO_BINARY_INT8(addr), gpiobuff2[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff2[0]));

        /* READ PORT1 DATA */
        f_port = ftdi_read_data(ftdi1, readbuff, 1);
        value = readbuff[0];
        DBG("us_device_read ftdi_read_data ftdi1: dataport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addr: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addrport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 "\n",
            readbuff[0], PRINTF_BYTE_TO_BINARY_INT8(readbuff[0]), addr, PRINTF_BYTE_TO_BINARY_INT8(addr), gpiobuff2[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff2[0]));

        /* PORT2 CS HIGH & RW LOW (strobe) */
        gpiobuff2[0] |= (1 << 2);  // 0b01100101
        gpiobuff2[0] &= ~(1 << 1);  // 0b01100001
        f_port = ftdi_write_data(ftdi2, gpiobuff2, 1);

        DBG("us_device_read ftdi_write_data ftdi2: dataport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addr: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addrport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 "\n",
            readbuff[0], PRINTF_BYTE_TO_BINARY_INT8(readbuff[0]), addr, PRINTF_BYTE_TO_BINARY_INT8(addr), gpiobuff2[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff2[0]));
        DBG("END us_device_read\n");
        return value;
    }
    return 0;
}

void us_device_store(uint16_t addr, uint8_t val, int chipno)
{
    if (chipno < MAXSID && addr < 0x20 && usid_dev >= 0) {
        DBG("START us_device_store\n");
        /* PORT1 ~ ALL PINS OUTPUT */
        f_port = ftdi_set_bitmode(ftdi1, 0xFF, BITMODE_BITBANG);

        /* READ PORT1 DATA */
        // f_port = ftdi_read_pins(ftdi1, gpiobuff1);

        /* READ PORT2 RES,RW,CS & ADDR */
        f_port = ftdi_read_pins(ftdi2, gpiobuff2);
        DBG("us_device_store ftdi_read_pins ftdi2: dataport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addr: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addrport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 "\n",
            gpiobuff1[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff1[0]), addr, PRINTF_BYTE_TO_BINARY_INT8(addr), gpiobuff2[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff2[0]));

        /* WRITE PORT1 DATA */
        gpiobuff1[0] = val;
        f_port = ftdi_write_data(ftdi1, gpiobuff1, 1);
        DBG("us_device_store ftdi_write_data ftdi1: dataport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addr: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addrport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 "\n",
            gpiobuff1[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff1[0]), addr, PRINTF_BYTE_TO_BINARY_INT8(addr), gpiobuff2[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff2[0]));
        /* save only the first 3 pins ~ 0b00000111 mask */
        gpiobuff2[0] &= 0x5;
        /* merge buff and bitshift addr 3 left ~ 0b11111000 */
        gpiobuff2[0] = gpiobuff2[0] | (addr & 0x1F) << 3;

        /* WRITE PORT2 RES,RW,CS & ADDR */
        f_port = ftdi_write_data(ftdi2, gpiobuff2, 1);
        DBG("us_device_store ftdi_write_data ftdi2: dataport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addr: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addrport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 "\n",
            gpiobuff1[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff1[0]), addr, PRINTF_BYTE_TO_BINARY_INT8(addr), gpiobuff2[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff2[0]));

        /* PORT2 CS & RW LOW (strobe) */
        gpiobuff2[0] &= ~(1 << 1) & ~(1 << 2);  // 0b11111001
        f_port = ftdi_write_data(ftdi2, gpiobuff2, 1);
        DBG("us_device_store ftdi_write_data ftdi2: dataport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addr: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addrport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 "\n",
            gpiobuff1[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff1[0]), addr, PRINTF_BYTE_TO_BINARY_INT8(addr), gpiobuff2[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff2[0]));
        usleep(1);
        /* PORT2 CS & RW HIGH (strobe) */
        gpiobuff2[0] |= 1 << 1 | 1 << 2;  // 0b00000110
        f_port = ftdi_write_data(ftdi2, gpiobuff2, 1);
        DBG("us_device_store ftdi_write_data ftdi2: dataport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addr: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 " addrport: 0x%04x 0b" PRINTF_BINARY_PATTERN_INT8 "\n",
            gpiobuff1[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff1[0]), addr, PRINTF_BYTE_TO_BINARY_INT8(addr), gpiobuff2[0], PRINTF_BYTE_TO_BINARY_INT8(gpiobuff2[0]));

        /* PORT1 ~ ALL PINS INPUT */
        f_port = ftdi_set_bitmode(ftdi1, 0x0, BITMODE_BITBANG);
        DBG("END us_device_store\n");
    }
}

unsigned int us_device_available(void)
{
    return sids_found;
}

/* ---------------------------------------------------------------------*/

// void us_device_state_read(int chipno, struct sid_us_snapshot_state_s *sid_state)
// {
//     sid_state->usid_main_clk = (uint32_t)usid_main_clk;
//     sid_state->usid_alarm_clk = (uint32_t)usid_alarm_clk;
//     sid_state->lastaccess_clk = 0;
//     sid_state->lastaccess_ms = 0;
//     sid_state->lastaccess_chipno = 0;
//     sid_state->chipused = 0;
//     sid_state->device_map[0] = 0;
//     sid_state->device_map[1] = 0;
//     sid_state->device_map[2] = 0;
//     sid_state->device_map[3] = 0;
// }

// void us_device_state_write(int chipno, struct sid_us_snapshot_state_s *sid_state)
// {
//     usid_main_clk = (CLOCK)sid_state->usid_main_clk;
//     usid_alarm_clk = (CLOCK)sid_state->usid_alarm_clk;
// }
#endif /* HAVE_USBSID */
#endif /* UNIX_COMPILE */
