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
#include <stdlib.h>
#include <string.h>
#include <libusb.h>

#include "alarm.h"
#include "us-unix.h"
#include "usbsid.h"
#include "log.h"
#include "maincpu.h"
#include "sid-resources.h"
#include "types.h"


#define MAXSID 4

/* Approx 3 PAL screen updates */
#define USBSID_DELAY_CYCLES 50000

sid_us_snapshot_state_t sid_state;
static CLOCK usid_main_clk;
static CLOCK usid_alarm_clk;
static alarm_t *usid_alarm = 0;

static int read_completed, write_completed;
static int rc = -1, sids_found = -1, usid_dev = -1;
static uint8_t sidbuf[0x20 * US_MAXSID];

#define VENDOR_ID      0xcafe
#define PRODUCT_ID     0x4011
#define ACM_CTRL_DTR   0x01
#define ACM_CTRL_RTS   0x02
static int ep_out_addr = 0x02;
static int ep_in_addr  = 0x82;
static struct libusb_device_handle *devh = NULL;
static struct libusb_transfer *transfer_out = NULL;
static struct libusb_transfer *transfer_in = NULL;
static libusb_context *ctx = NULL;

#define LEN_IN_BUFFER  1
#define LEN_OUT_BUFFER 3
static uint8_t * in_buffer;
static uint8_t * out_buffer;
static unsigned char result[LEN_IN_BUFFER];

volatile bool * isasync = false;

enum clock_speeds
{
    DEFAULT = 1000000,
    PAL     = 985248,
    NTSC    = 1022730,
    DREAN   = 1023440,

};
static const enum clock_speeds clockSpeed[] = { DEFAULT, PAL, NTSC, DREAN };

/* pre declarations */
static void LIBUSB_CALL sid_out(struct libusb_transfer *transfer);
static void LIBUSB_CALL sid_in(struct libusb_transfer *transfer);
static void usbsid_alarm_handler(CLOCK offset, void *data);


static int usbsid_init(void)
{
    rc = read_completed = write_completed = -1;
    /* Already open */
    if (usid_dev >= 0) {
        return -1;
    }
    /* Line encoding ~ baud rate is ignored by TinyUSB */
    unsigned char encoding[] = { 0x40, 0x54, 0x89, 0x00, 0x00, 0x00, 0x08 };

    /* Initialize libusb */
    rc = libusb_init(&ctx);
	if (rc != 0) {
        log_error(LOG_ERR, "[USBSID] Error initializing libusb: %d %s: %s", rc, libusb_error_name(rc), libusb_strerror(rc));
        goto out;
    }

	/* Set debugging output to min/max (4) level */
	libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, 0);

	/* Look for a specific device and open it */
	devh = libusb_open_device_with_vid_pid(ctx, VENDOR_ID, PRODUCT_ID);
    if (!devh) {
        log_error(LOG_ERR, "[USBSID] Error opening USB device with VID & PID: %d %s: %s", rc, libusb_error_name(rc), libusb_strerror(rc));
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
            log_error(LOG_ERR, "[USBSID] Error claiming interface: %d, %s: %s", rc, libusb_error_name(rc), libusb_strerror(rc));
            goto out;
        }
    }

    /* Start configuring the device:
     * - set line state */
    rc = libusb_control_transfer(devh, 0x21, 0x22, ACM_CTRL_DTR | ACM_CTRL_RTS, 0, NULL, 0, 0);
    if (rc != 0 && rc != 7) {
        log_error(LOG_ERR, "[USBSID] Error configuring line state during control transfer: %d, %s: %s", rc, libusb_error_name(rc), libusb_strerror(rc));
        rc = -1;
        goto out;
    }

    /* - set line encoding here */
    rc = libusb_control_transfer(devh, 0x21, 0x20, 0, 0, encoding, sizeof(encoding), 0);
    if (rc != 0 && rc != 7) {
        log_error(LOG_ERR, "[USBSID] Error configuring line encoding during control transfer: %d, %s: %s", rc, libusb_error_name(rc), libusb_strerror(rc));
        rc = -1;
        goto out;
    }

    out_buffer = libusb_dev_mem_alloc(devh, LEN_OUT_BUFFER);
    if (out_buffer == NULL) {
        log_message(LOG_DEFAULT, "libusb_dev_mem_alloc failed on out_buffer, allocating with malloc");
        out_buffer = (uint8_t*)malloc( (sizeof(uint8_t)) * 3 );
    }
    log_message(LOG_DEFAULT, "[USBSID] alloc out_buffer complete\r");
    transfer_out = libusb_alloc_transfer(0);
    log_message(LOG_DEFAULT, "[USBSID] alloc transfer_out complete\r");
    /* libusb_fill_bulk_transfer(transfer_out, devh, ep_out_addr, out_buffer, LEN_OUT_BUFFER, sid_out, &write_completed, 0); */
    libusb_fill_bulk_transfer(transfer_out, devh, ep_out_addr, out_buffer, LEN_OUT_BUFFER, sid_out, NULL, 0);
    // libusb_submit_transfer(transfer_out);  // TEST
    log_message(LOG_DEFAULT, "[USBSID] libusb_fill_bulk_transfer transfer_out complete\r");

    in_buffer = libusb_dev_mem_alloc(devh, LEN_IN_BUFFER);
    if (in_buffer == NULL) {
        log_message(LOG_DEFAULT, "libusb_dev_mem_alloc failed on in_buffer, allocating with malloc");
        in_buffer = (uint8_t*)malloc( (sizeof(uint8_t)) * 1 );
    }
    log_message(LOG_DEFAULT, "[USBSID] alloc in_buffer complete\r");
    transfer_in = libusb_alloc_transfer(0);
    log_message(LOG_DEFAULT, "[USBSID] alloc transfer_in complete\r");
    libusb_fill_bulk_transfer(transfer_in, devh, ep_in_addr, in_buffer, LEN_IN_BUFFER, sid_in, &read_completed, 0);
    /* libusb_submit_transfer(transfer_in); */
    if (isasync) libusb_submit_transfer(transfer_out);
    log_message(LOG_DEFAULT, "[USBSID] libusb_fill_bulk_transfer transfer_in complete\r");

    usid_dev = (rc == 0 || rc == 7) ? 0 : -1;
    sids_found = usid_dev == 0 ? 1 : -1;

    if (usid_dev < 0)
    {
        log_error(LOG_ERR, "[USBSID] Could not open SID device");
        goto out;
    }

	return usid_dev;
out:
    us_device_close();
    return rc;
}

static void LIBUSB_CALL sid_out(struct libusb_transfer *transfer)
{
    /* write_completed = (*(int *)transfer->user_data); */

    if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
        rc = transfer->status;
        if (rc != LIBUSB_TRANSFER_CANCELLED) {
		    log_error(LOG_ERR, "Warning: transfer out interrupted with status %d, %s: %s", rc, libusb_error_name(rc), libusb_strerror(rc));
        }
		libusb_free_transfer(transfer);
        return;
    }

    if (transfer->actual_length != LEN_OUT_BUFFER) {
        log_error(LOG_ERR, "Sent data length %d is different from the defined buffer length: %d or actual length %d", transfer->length, LEN_OUT_BUFFER, transfer->actual_length);
    }

    /* write_completed = 1; */
    if (isasync) libusb_submit_transfer(transfer_out);
    // libusb_submit_transfer(transfer_out);
}

static void LIBUSB_CALL sid_in(struct libusb_transfer *transfer)
{
    read_completed = (*(int *)transfer->user_data);

    if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
        rc = transfer_in->status;
		if (rc != LIBUSB_TRANSFER_CANCELLED) {
            log_error(LOG_ERR, "Warning: transfer in interrupted with status '%s'", libusb_error_name(rc));
        }
        libusb_free_transfer(transfer);
        return;
    }

    /* fprintf(stdout, "[RD] $%02x $%02x\r\n", in_buffer, in_buffer[0]); */

    memcpy(result, in_buffer, 1);
    read_completed = 1;
    /* libusb_submit_transfer(transfer_in); */
}

void us_device_reset(void)
{
    if (sids_found > 0) {
        unsigned char buff[3] = {0x3, 0x0, 0x0};
        memcpy(out_buffer, buff, 3);
        libusb_submit_transfer(transfer_out);
        /* libusb_handle_events_completed(ctx, &write_completed); */
        libusb_handle_events_completed(ctx, NULL);

        usid_main_clk  = maincpu_clk;
        usid_alarm_clk = USBSID_DELAY_CYCLES;
        alarm_set(usid_alarm, USBSID_DELAY_CYCLES);
        log_message(LOG_DEFAULT, "[USBSID] reset!");
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

    log_message(LOG_DEFAULT, "[USBSID] Detecting boards");

    if (usid_dev != 0) {
        rc = usbsid_init();
        if (rc != 0) {
            return -1;
        }
    }

    log_message(LOG_DEFAULT, "[USBSID] boards detected [rc]%d [usid_dev]%d [sids_found]%d", rc, usid_dev, sids_found);

    usid_alarm = alarm_new(maincpu_alarm_context, "usbsid", usbsid_alarm_handler, 0);
    sids_found = 1;
    usbsid_reset(); /* eventually calls us_device_reset  */

    log_message(LOG_DEFAULT, "[USBSID] opened");

    memset(sidbuf, 0, sizeof(sidbuf));
    return 0;
}

int us_device_close(void)
{
    log_message(LOG_DEFAULT, "[USBSID]: start closing");
    unsigned char buff[3] = {0x0, 0x0, 0x0};
    memcpy(out_buffer, buff, 3);

    libusb_submit_transfer(transfer_out);
    libusb_handle_events_completed(ctx, NULL);
    /* libusb_handle_events_completed(ctx, &write_completed); */

    rc = libusb_cancel_transfer(transfer_out);
    if (rc < 0 && rc != -5)
        log_error(LOG_ERR, "[USBSID] Failed to cancel transfer %d - %s: %s", rc, libusb_error_name(rc), libusb_strerror(rc));

    rc = libusb_cancel_transfer(transfer_in);
    if (rc < 0 && rc != -5)
        log_error(LOG_ERR, "[USBSID] Failed to cancel transfer %d - %s: %s", rc, libusb_error_name(rc), libusb_strerror(rc));

    rc = libusb_dev_mem_free(devh, in_buffer, LEN_OUT_BUFFER);
    if (rc < 0)
        log_error(LOG_ERR, "[USBSID] Failed to free in_buffer DMA memory: %d, %s: %s", rc, libusb_error_name(rc), libusb_strerror(rc));

    rc = libusb_dev_mem_free(devh, out_buffer, LEN_OUT_BUFFER);
    if (rc < 0)
        log_error(LOG_ERR, "[USBSID] Failed to free out_buffer DMA memory: %d, %s: %s", rc, libusb_error_name(rc), libusb_strerror(rc));

    for (int if_num = 0; if_num < 2; if_num++) {
        if (libusb_kernel_driver_active(devh, if_num)) {
            rc = libusb_detach_kernel_driver(devh, if_num);
            log_error(LOG_ERR, "[USBSID] libusb_detach_kernel_driver: %d, %s: %s", rc, libusb_error_name(rc), libusb_strerror(rc));
        }
        libusb_release_interface(devh, if_num);
    }
    if (devh)
        libusb_close(devh);
    libusb_exit(ctx);

    /* Clean up vars */
    alarm_destroy(usid_alarm);
    usid_alarm = 0;
    sids_found = -1;
    usid_dev = -1;
    rc = -1;
    devh = NULL;
    log_message(LOG_DEFAULT, "[USBSID] closed");
    return 0;
}

int us_device_read(uint16_t addr, int chipno)
{
    if (!isasync) {
        /* fixed: the blocking/timeout issue is caused by the vue check being set to 0 */
        if (chipno < MAXSID && usid_dev >= 0)
        {
            addr = ((addr & 0x1F) + (chipno * 0x20));
            uint8_t writebuff[3] = {0x1, addr, 0x0};

            read_completed = write_completed = 0;
            memcpy(out_buffer, writebuff, 3);
            libusb_submit_transfer(transfer_out);
            /* libusb_handle_events_completed(ctx, &write_completed); */
            libusb_handle_events_completed(ctx, NULL);
            /* read_completed = 0; */
            libusb_submit_transfer(transfer_in);
            libusb_handle_events_completed(ctx, &read_completed);
            sidbuf[addr] = result[0];
            /* printf("[R]$%02x\n", result[0]); */
            return result[0];
        }
    }
    return 0xFF;
}

void us_device_store(uint16_t addr, uint8_t val, int chipno) /* max chipno = 1 */
{
    if (chipno < MAXSID && usid_dev >= 0) {  /* remove 0x20 address limitation */

        addr = ((addr & 0x1F) + (chipno * 0x20));
        uint8_t writebuff[3] = { 0x0, addr, val };

        /* write_completed = 0; */
        memcpy(out_buffer, writebuff, 3);
        libusb_submit_transfer(transfer_out);
        libusb_handle_events_completed(ctx, NULL);
        /* libusb_handle_events_completed(ctx, &write_completed); */
        sidbuf[addr] = val;
    }
}

void us_set_machine_parameter(long cycles_per_sec)
{
    for (int i = 0; i < 4; i++) {
        if (clockSpeed[i] == cycles_per_sec) {
            printf("[USBSID] %s SET CLOCKSPEED TO: %ld [I]%ld\r\n", __func__, clockSpeed[i], cycles_per_sec);
            int actual_length;
            uint8_t writebuff[5] = {0x32, 0, i, 0, 0};
            if (libusb_bulk_transfer(devh, ep_out_addr, writebuff, 5, &actual_length, 0) < 0)
            {
                log_message(LOG_ERR, "[USBSID] Error while sending config\n");
            }
            return;
        }
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

void us_device_set_async(unsigned int val)
{
    printf("[USBSID] %s: %x\n", __func__, val);
    isasync = val == 0 ? false : true;
    if (isasync)
    {
        /* TODO: Start comms thread etc */
        libusb_submit_transfer(transfer_out);
    }
}

/* ---------------------------------------------------------------------*/

void us_device_state_read(int chipno, struct sid_us_snapshot_state_s *sid_state)
{
    printf("%s\r", __func__);
    sid_state->usid_main_clk = (uint32_t)usid_main_clk;
    sid_state->usid_alarm_clk = (uint32_t)usid_alarm_clk;
    sid_state->lastaccess_clk = 0;
    sid_state->lastaccess_ms = 0;
    sid_state->lastaccess_chipno = 0;
    sid_state->chipused = 0;
    sid_state->device_map[0] = 0;
    sid_state->device_map[1] = 0;
    sid_state->device_map[2] = 0;
    sid_state->device_map[3] = 0;
}

void us_device_state_write(int chipno, struct sid_us_snapshot_state_s *sid_state)
{
    printf("%s\r", __func__);
    usid_main_clk = (CLOCK)sid_state->usid_main_clk;
    usid_alarm_clk = (CLOCK)sid_state->usid_alarm_clk;
}
#endif /* HAVE_USBSID */
#endif /* UNIX_COMPILE */
