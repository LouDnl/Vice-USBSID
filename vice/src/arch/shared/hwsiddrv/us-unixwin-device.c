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

#if defined(UNIX_COMPILE) || defined(WINDOWS_COMPILE) || defined(WINDOWS_COMPILE)
#if defined(HAVE_USBSID)

/* #define USBSID_DEBUG */
#ifdef USBSID_DEBUG
#define DBG(...) printf(__VA_ARGS__)
#else
#define DBG(...) ((void)0)
#endif

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h> /* TIMESPEC */
#include <time.h> /* TIMESPEC */

#include "alarm.h"
#include "us-unixwin.h"
#include "usbsid.h"
#include "log.h"
#include "maincpu.h"
#include "machine.h"
#include "resources.h"
#include "sid-resources.h"
#include "types.h"

#include "USBSIDInterface.h"

#ifdef US_NOOPT
#pragma GCC push_options
#pragma GCC optimize ("O3")
#endif

static int rc = -1, sids_found = -1, no_sids = -1, soc_audio = -1, r_readmode = -1, readmode = -1;
static uint8_t sidbuf[0x20 * US_MAXSID];

static CLOCK usid_main_clk;
static CLOCK usid_alarm_clk;
static alarm_t *usid_alarm = NULL;
static long raster_rate;

static log_t usbsid_log = LOG_DEFAULT;

/* pre declarations */
static void usbsid_alarm_handler(CLOCK offset, void *data);

USBSIDitf usbsid;

void us_device_reset(bool us_reset)
{
    if (sids_found > 0) {
        raster_rate = getrasterrate_USBSID(usbsid);
        usid_main_clk = maincpu_clk;
        usid_alarm_clk = raster_rate;
        alarm_set(usid_alarm, (usid_main_clk + raster_rate));
        if (us_reset) {
            reset_USBSID(usbsid);
            log_message(usbsid_log, "Reset sent\r");
        }
        no_sids = -1;
        log_message(usbsid_log, "Clocks reset\r");
    }
    return;
}

int us_device_open(void)
{
    usbsid_log = log_open("USBSID");

    if (!sids_found) {
        return -1;
    }

    if (sids_found > 0) {
        return sids_found;
    }

    sids_found = 0;

    log_message(usbsid_log, "Detecting boards\r");

    if (usbsid == NULL) {
        usbsid = create_USBSID();
        if (usbsid) {
            resources_get_int("SidUSBSIDReadMode", &r_readmode);
            log_message(usbsid_log, "SidUSBSIDReadMode: %d, readmode: %d\r", r_readmode, readmode);
            readmode = r_readmode;
        }
          /* NOTICE: Digitunes only play with threaded cycles */
        if (readmode == 0) {
            log_message(usbsid_log, "Starting in normal mode\r");
            rc = init_USBSID(usbsid, true, true);  /* threading and cycles enabled */
        } else if (readmode == 1) {
            log_message(usbsid_log, "Starting in read mode\r");
            rc = init_USBSID(usbsid, false, false);  /* threading and cycles disabled */
        }

        if (rc < 0) {
            return -1;
        }
    }

    usid_alarm = alarm_new(maincpu_alarm_context, "usbsid", usbsid_alarm_handler, NULL);
    sids_found = getnumsids_USBSID(usbsid);
    no_sids = 0;
    log_message(usbsid_log, "Alarm set, reset sids\r");
    us_device_reset(false);  /* No reset on init! */
    log_message(usbsid_log, "Opened\r");

    return rc;
}

int us_device_close(void)
{
    log_message(usbsid_log, "Start device closing\r");
    if (usbsid) {
        mute_USBSID(usbsid);
        close_USBSID(usbsid);
    }

    /* Clean up vars */
    alarm_destroy(usid_alarm);
    usid_alarm = 0;
    sids_found = -1;
    no_sids = -1;
    rc = -1;
    usbsid = NULL;
    log_message(usbsid_log, "Closed\r");
    return 0;
}

int us_device_read(uint16_t addr, int chipno)
{   /* NOTICE: Disabled, unneeded */
    if (chipno < US_MAXSID) {
        if (readmode == 1) {
            uint8_t n_addr = ((addr & 0x1F) + (chipno * 0x20));
            sidbuf[addr] = read_USBSID(usbsid, n_addr);
            return sidbuf[addr];
        } else {
            return 0x0;
        }
    }
    return 0x0;
}

int_fast32_t us_delay(void)
{   /* ISSUE: This should return an unsigned 64 bit integer but that makes vice stall indefinately on negative integers */
    if (maincpu_clk < usid_main_clk) {  /* Sync reset */
        usid_main_clk = maincpu_clk;
        return 0;
    }
    /* Without substracting 1 cycle this would cause a clicking noise in cycle exact tunes */
    int_fast32_t cycles = maincpu_clk - usid_main_clk - 1;
    while (cycles > 0xffff)
    {
        cycles -= 0xffff;
    }
    usid_main_clk = maincpu_clk;
    return cycles;
}

void us_device_store(uint16_t addr, uint8_t val, int chipno) /* max chipno = 1 */
{
    if (chipno < US_MAXSID) {  /* remove 0x20 address limitation */
        addr = ((addr & 0x1F) + (chipno * 0x20));
        if (readmode == 0) {
            uint_fast32_t cycles = us_delay();
            writeringcycled_USBSID(usbsid, addr, val, cycles);
        } else if (readmode == 1) {
            write_USBSID(usbsid, addr, val);
        }
        sidbuf[addr] = val;
        return;
    }
    return;
}

void us_set_machine_parameter(long cycles_per_sec)
{
    setclockrate_USBSID(usbsid, cycles_per_sec, true); /* TESTING */
    raster_rate = getrasterrate_USBSID(usbsid);
    log_message(usbsid_log, "Clockspeed set to: %ld and rasterrate set to: %ld\r", cycles_per_sec, raster_rate);
    return;
}

unsigned int us_device_available(void)
{
    log_message(usbsid_log, "%d SIDs available\r", sids_found);
    return sids_found;
}

void us_set_audio(int val)
{   /* Gets set by x64sc from SID settings and by VSID at SID file change */
    resources_get_int("SoundOutput", &soc_audio);
    log_message(usbsid_log, "Global audio type is '%s'\r", (soc_audio == 2 ? "Stereo" : soc_audio == 1 ? "Mono" : "System"));


    int stereo = 0;
    no_sids = val+1; /* Number of SIDs requested +1 (val = 0 if 1 single SID is requested) */

    log_message(usbsid_log, "Set requested audio type for no. SIDs %d (No. SIDs available: %d)\r", no_sids, sids_found);

    stereo
        /* Number of requested SIDs equals no. available SIDs */
        = (no_sids == sids_found && soc_audio != 1)
        ? (no_sids == 2 || no_sids == 4)  /* 2 or 4 SIDs requested */
        ? 1  /* then Stereo */
        : 0  /* else Mono */
        /* else number of requested SIDs does not equal no. available SIDs */
        : 0; /* fallback to Mono */

    log_message(usbsid_log, "Audio type set to '%s' for %d requested SIDs\r", (stereo == 1 ? "Stereo" : "Mono"), no_sids);
    setstereo_USBSID(usbsid, stereo);
}

void us_set_readmode(int val)
{
    resources_get_int("SidUSBSIDReadMode", &r_readmode);
    if (readmode != val) {
        log_message(usbsid_log, "Set read mode from %d to %d (resource: %d)\r", readmode, val, r_readmode);
        readmode = val;
        if (val == 0) enablethread_USBSID(usbsid);
        if (val == 1) disablethread_USBSID(usbsid);
    }
    return;
}

static void usbsid_alarm_handler(CLOCK offset, void *data)
{
    CLOCK cycles = (usid_alarm_clk + offset) - usid_main_clk;

    if (cycles < raster_rate) {
        usid_alarm_clk = usid_main_clk + raster_rate;
    } else {
        setflush_USBSID(usbsid);
        usid_main_clk   = maincpu_clk - offset;
        usid_alarm_clk  = usid_main_clk + raster_rate;
    }
    alarm_set(usid_alarm, usid_alarm_clk);
    return;
}

/* ---------------------------------------------------------------------*/

void us_device_state_read(int chipno, struct sid_us_snapshot_state_s *sid_state)
{
    log_message(usbsid_log, "%s\r", __func__);
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
    log_message(usbsid_log, "%s\r", __func__);
    usid_main_clk = (CLOCK)sid_state->usid_main_clk;
    usid_alarm_clk = (CLOCK)sid_state->usid_alarm_clk;
}

#ifdef US_NOOPT
#pragma GCC pop_options
#endif
#endif /* HAVE_USBSID */
#endif /* UNIX_COMPILE || WINDOWS_COMPILE */
