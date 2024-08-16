/*
 * us-unix-linux.c - Linux specific usbsid driver.
 *
 * Written by
 *  LouDnl
 *
 * Based on vice code written by
 *  Simon White <sidplay2@yahoo.com>
 *  Marco van den Heuvel <blackystardust68@yahoo.com>
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

#include <errno.h>
#include <libusb.h>

#include "alarm.h"
#include "us-unix.h"
#include "usbsid.h"
#include "log.h"
#include "maincpu.h"
#include "sid-resources.h"
#include "types.h"

#include <string.h>

#include "usbsid-macros.h"

#define MAXSID 4

/* Approx 3 PAL screen updates */
#define USBSID_DELAY_CYCLES 50000

static CLOCK usid_main_clk;
static CLOCK usid_alarm_clk;
static alarm_t *usid_alarm = 0;

#define VENDOR_ID      0xcafe
#define PRODUCT_ID     0x4011
#define ACM_CTRL_DTR   0x01
#define ACM_CTRL_RTS   0x02
static int ep_out_addr = 0x02;
static int ep_in_addr  = 0x82;
static struct libusb_device_handle *devh = NULL;

/* set line encoding: here 9600 8N1
 * 9600 = 0x2580 -> 0x80, 0x25 in little endian
 * 115200 = 0x1C200 -> 0x00, 0xC2, 0x01 in little endian
 * 921600 = 0xE1000 -> 0x00, 0x10, 0x0E in little endian
 */
static unsigned char encoding[] = { 0x00, 0x10, 0x0E, 0x00, 0x00, 0x00, 0x08 };
static int rc;
static int actual_length;

static int sids_found = -1;
static int usid_dev = -1;

#define DEBUG_USBSID
#ifdef DEBUG_USBSID
#define UDBG(...) printf(__VA_ARGS__)
// #define UDBG(...) log_message(LOG_DEFAULT, __VA_ARGS__)
#else
#define UDBG(...)
#endif

/* #define DEBUG_USBSIDMEM */
#ifdef DEBUG_USBSIDMEM
#define MDBG(...) printf(__VA_ARGS__)
uint8_t memory[65536];
#else
#define MDBG(...)
#endif

static void usbsid_alarm_handler(CLOCK offset, void *data);

static int usbsid_init(void)
{

    /* Already open */
    if (usid_dev >= 0) {
        return -1;
    }
    if (devh != NULL) {
        libusb_close(devh);
    }

    /* Initialize libusb */
    rc = libusb_init(NULL);
	if (rc != 0) {
        log_error(LOG_ERR, "Error initializing libusb: %s: %s\n",
        libusb_error_name(rc), libusb_strerror(rc));
        goto out;
    }

	/* Set debugging output to max level */
	libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, 0);
	// libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, 3);

	/* Look for a specific device and open it */
	devh = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
    if (!devh) {
        log_error(LOG_ERR, "Error opening USB device with VID & PID: %d\n", rc);
        rc = -1;
        goto out;
    }

	/* As we are dealing with a CDC-ACM device, it's highly probable that
     * Linux already attached the cdc-acm driver to this device.
     * We need to detach the drivers from all the USB interfaces. The CDC-ACM
     * Class defines two interfaces: the Control interface and the
     * Data interface.
     */
    for (int if_num = 0; if_num < 2; if_num++) {
        if (libusb_kernel_driver_active(devh, if_num)) {
            libusb_detach_kernel_driver(devh, if_num);
        }
        rc = libusb_claim_interface(devh, if_num);
        if (rc < 0) {
            log_error(LOG_ERR, "Error claiming interface: %d, %s: %s\n",
            rc, libusb_error_name(rc), libusb_strerror(rc));
            goto out;
        }
    }

    /* Start configuring the device:
     * - set line state */
    rc = libusb_control_transfer(devh, 0x21, 0x22, ACM_CTRL_DTR | ACM_CTRL_RTS, 0, NULL, 0, 0);
    if (rc != 0 && rc != 7) {
        log_error(LOG_ERR, "?Error configuring line state during control transfer: %d, %s: %s\n",
            rc, libusb_error_name(rc), libusb_strerror(rc));
        goto out;
    }

    /* - set line encoding here */
    rc = libusb_control_transfer(devh, 0x21, 0x20, 0, 0, encoding, sizeof(encoding), 0);
    if (rc != 0 && rc != 7) {
        log_error(LOG_ERR, "Error configuring line encoding during control transfer: %d, %s: %s\n",
        rc, libusb_error_name(rc), libusb_strerror(rc));
        goto out;
    }

    usid_dev = (rc == 0 || rc == 7) ? 0 : -1;

    if (usid_dev < 0)
    {
        log_error(LOG_ERR, "Could not open SID device USBSID.");
        goto out;
    }

	return usid_dev;
out:
    us_device_close();
    return rc;
}

void us_device_reset(void)
{
    if (sids_found > 0) {
        unsigned char data[4] = {0x3, 0x0, 0x0, 0x0};
        int size = sizeof(data);
        if (libusb_bulk_transfer(devh, ep_in_addr, data, size, &actual_length, 0) < 0)
        {
            log_message(LOG_ERR, "Error while sending reset to sid\n");
        }
        usid_main_clk  = maincpu_clk;
        usid_alarm_clk = USBSID_DELAY_CYCLES;
        alarm_set(usid_alarm, USBSID_DELAY_CYCLES);
        UDBG("USBSID reset!\n");
    }
}

int us_device_open(void)
{
    if (!sids_found) {
        return -1;
    }

    if (sids_found > 0) {
        return sids_found;
    }

    sids_found = 0;

    log_message(LOG_DEFAULT, "Detecting Linux usbsid boards.");

    if (usid_dev != 0) {
        rc = usbsid_init();
        if (rc != 0) {
            return -1;
        }
    }

    log_message(LOG_DEFAULT, "Linux usbsid boards detected [rc]%d [usid_dev]%d [sids_found]%d\n", rc, usid_dev, sids_found);

    usid_alarm = alarm_new(maincpu_alarm_context, "usbsid", usbsid_alarm_handler, 0);
    sids_found = 1;
    usbsid_reset(); /* eventually calls us_device_reset  */

    /* zero length read to clear any lingering data */
    int transferred = 0;
    unsigned char buffer[1];
    libusb_bulk_transfer(devh, ep_out_addr, buffer, 0, &transferred, 1);

    log_message(LOG_DEFAULT, "Linux usbsid: opened.");

    #ifdef DEBUG_USBSIDMEM
	for (unsigned int i = 0; i < 65536; i++)
	{
		if (i >= 0xD400 || i <= 0xD430 ) {
			/* do nothing */
		} else {
			memory[i] = 0x00; // fill with NOPs
		}
	}
    #endif

    return 0;
}

int us_device_close(void)
{
    /* Driver cleans up after itself */
    if (devh != NULL) {
        for (int if_num = 0; if_num < 2; if_num++) {
            libusb_release_interface(devh, if_num);
            if (libusb_kernel_driver_active(devh, if_num)) {
                libusb_detach_kernel_driver(devh, if_num);
            }
        }
        libusb_close(devh);
        libusb_exit(NULL);
    }
    alarm_destroy(usid_alarm);
    /* Clean up vars */
    usid_alarm = 0;
    sids_found = -1;
    usid_dev = -1;
    rc = -1;
    devh = NULL;
    log_message(LOG_DEFAULT, "Linux usbsid: closed.");
    return 0;
}
#include "maincpu.h"
int us_device_read(uint16_t addr, int chipno)
{
    if (chipno < MAXSID && usid_dev >= 0)
    {
        CLOCK cycles = maincpu_clk - usid_main_clk - 1;
        usid_main_clk = maincpu_clk;
        unsigned char data[4];  /* Read buffer where each byte should be the same value */
        memset(data, 0, sizeof data);
        addr &= 0x1F;                                 /* remove address limitation */
        addr = (addr + (chipno * 0x20));
        unsigned char wdata[4] = {0x1, 0xD4, addr, 0x0};  /* set addr write data for read */  /* NOTICE: addr is limited to $D400 range */
        int actual_lengthw;  /* Stores the actual length of the read data */
        if (libusb_bulk_transfer(devh, ep_out_addr, wdata, sizeof(wdata), &actual_lengthw, 0) < 0)
        {
            log_message(LOG_ERR, "Error while sending char\n");
        }
        UDBG("[S#]%d RW@[0x%02x] (0b"PRINTF_BINARY_PATTERN_INT8") ", chipno, addr, PRINTF_BYTE_TO_BINARY_INT8(addr));
        int actual_lengthr;  /* Stores the actual length of the read data */
        rc = 0;
        rc = libusb_bulk_transfer(devh, ep_in_addr, data, sizeof(data), &actual_lengthr, 1000);
        if (rc == LIBUSB_ERROR_TIMEOUT) {
            log_message(LOG_ERR, "Timeout (%d)\n", actual_lengthr);
            return -1;
        } else if (rc < 0) {
            log_message(LOG_ERR, "Error while waiting for char\n");
            return -1;
        }
        UDBG("RCV[0x%02x] (0b"PRINTF_BINARY_PATTERN_INT8") [C]%lu\r\n", data[0], PRINTF_BYTE_TO_BINARY_INT8(data[0]), cycles);
        return data[0];
    }
    return 0;
}

void us_device_store(uint16_t addr, uint8_t val, int chipno) /* max chipno = 1 */
{
    if (chipno < MAXSID && usid_dev >= 0) {  /* remove 0x20 address limitation */
        CLOCK cycles = maincpu_clk - usid_main_clk - 1;
        usid_main_clk = maincpu_clk;
        #ifdef DEBUG_USBSIDMEM
        memory[(0xD400 | addr)] = val;
        #endif
        addr &= 0x1F;
        addr = (addr + (chipno * 0x20));
        unsigned char data[4] = {0x0, 0xD4, addr, val}; /* NOTICE: addr is limited to $D400 range */
        int size = sizeof(data);
        if (libusb_bulk_transfer(devh, ep_out_addr, data, size, &actual_length, 0) < 0)
        {
            log_message(LOG_ERR, "Error while sending char\n");
        }
        UDBG("[S#]%d WR@[0x%02x] (0b"PRINTF_BINARY_PATTERN_INT8") DAT[0x%02x] (0b"PRINTF_BINARY_PATTERN_INT8") [C]%lu\r\n", chipno, addr, PRINTF_BYTE_TO_BINARY_INT8(addr), val, PRINTF_BYTE_TO_BINARY_INT8(val), cycles);
        #ifdef DEBUG_USBSIDMEM
        MDBG("one single memwrite ~ addr: %04x byte: %04x phyaddr: %04x | Synth 1: $%02X%02X %02X%02X %02X %02X %02X | Synth 2: $%02X%02X %02X%02X %02X %02X %02X | Synth 3: $%02X%02X %02X%02X %02X %02X %02X\n",
		addr, val, laddr,
		memory[0xD400], memory[0xD401], memory[0xD402], memory[0xD403], memory[0xD404], memory[0xD405], memory[0xD406],
		memory[0xD407], memory[0xD408], memory[0xD409], memory[0xD40A], memory[0xD40B], memory[0xD40C], memory[0xD40D],
		memory[0xD40E], memory[0xD40F], memory[0xD410], memory[0xD411], memory[0xD412], memory[0xD413], memory[0xD414]);
        #endif
    }
}

unsigned int us_device_available(void)
{
    return sids_found;
}

static void usbsid_alarm_handler(CLOCK offset, void *data)
{
    CLOCK cycles = (usid_alarm_clk + offset) - usid_main_clk;

    if (cycles < USBSID_DELAY_CYCLES) {
        usid_alarm_clk = usid_main_clk + USBSID_DELAY_CYCLES;
    } else {
        /* uint delay = (uint) cycles; */
        usid_main_clk   = maincpu_clk - offset;
        usid_alarm_clk  = usid_main_clk + USBSID_DELAY_CYCLES;
    }
    alarm_set(usid_alarm, usid_alarm_clk);
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
