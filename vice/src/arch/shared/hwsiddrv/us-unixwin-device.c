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

#if defined(UNIX_COMPILE) || defined(WINDOWS_COMPILE)
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
#include "sid-resources.h"
#include "types.h"

#include "usbsid-driver/USBSIDInterface.h"

#pragma GCC push_options
#pragma GCC optimize ("O3")

static int rc = -1, sids_found = -1;
static uint8_t sidbuf[0x20 * US_MAXSID];

static CLOCK usid_main_clk;
static CLOCK usid_alarm_clk;
static alarm_t *usid_alarm = NULL;

volatile int isasync = 0;

#define USBSID_DELAY_CYCLES 50000 /* 60000 */

/* pre declarations */
static void usbsid_alarm_handler(CLOCK offset, void *data);

USBSIDitf usbsid;

void us_device_reset(void)
{
    DBG("[%s]\n", __func__);
    if (sids_found > 0) {
        /* reset_USBSID(usbsid); */
        usid_main_clk  = maincpu_clk;
        usid_alarm_clk = USBSID_DELAY_CYCLES;
        alarm_set(usid_alarm, USBSID_DELAY_CYCLES);
        log_message(LOG_DEFAULT, "[USBSID] reset!\r");
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

    log_message(LOG_DEFAULT, "[USBSID] Detecting boards\r");

    if (usbsid == NULL) {
        usbsid = create_USBSID();
        if (isasync == 0) {
            rc = init_USBSID(usbsid, false, false);
        } else if (isasync == 1) {
            rc = init_USBSID(usbsid, true, true);  // ISSUE: Only with threaded cycles Vice plays digitunes okay...
        } /* else {
            rc = init_USBSID(usbsid, false, false);
        } */
        if (rc < 0) {
            return -1;
        }
    }

    usid_alarm = alarm_new(maincpu_alarm_context, "usbsid", usbsid_alarm_handler, NULL);
    usid_alarm = 0;
    sids_found = 1;
    log_message(LOG_DEFAULT, "[USBSID] alarm set, reset sids\r");
    /* us_device_reset(); */
    usbsid_reset();
    log_message(LOG_DEFAULT, "[USBSID] opened\r");

    return rc;
}

int us_device_close(void)
{
    log_message(LOG_DEFAULT, "[USBSID] Start device closing\r");
    if (usbsid) {
        close_USBSID(usbsid);
    }

    /* Clean up vars */
    alarm_destroy(usid_alarm);
    usid_alarm = 0;
    sids_found = -1;
    rc = -1;
    usbsid = NULL;
    log_message(LOG_DEFAULT, "[USBSID] closed\r");
    return 0;
}
int us_device_read(uint16_t addr, int chipno)
{  // BUG: Broken for some reason, use the new configtool for SKPico
    if (chipno < US_MAXSID) {
        addr = ((addr & 0x1F) + (chipno * 0x20));
        uint8_t writebuffer[3] = { 0x1, addr, 0x0 };
        uint8_t readresult[1] = {0};
        if (isasync == 0) {
            read_USBSID(usbsid, writebuffer, readresult);
            sidbuf[addr] = readresult[0];
            return readresult[0];
        }
        return 0x0;
    }
    return 0x0;
}

int_fast32_t us_delay(void)
{   // ISSUE: This should return an unsigned 64 bit integer but that makes vice stall indefinately on negative integers
    if (maincpu_clk < usid_main_clk) {  /* Sync reset */
        usid_main_clk = maincpu_clk;
        DBG("return_cycles: %ld ", 0);
        return 0;
    }
    int_fast32_t cycles = maincpu_clk - usid_main_clk - 1;
    DBG("[DELAY] maincpu_clk: %ld usid_main_clk: %ld ", maincpu_clk, usid_main_clk);
    DBG("cycles: %ld ", cycles);
    while (cycles > 0xffff)
    {
        // if (cycles > 0) {
            /* waitforcycle_USBSID(usbsid, 0xffff); */  // ISSUE: This stops the main thread from working!
        // }
        if (isasync == 1) {
            ringpushcycled_USBSID(usbsid, 0xFF, 0xFF, cycles);
        }
        cycles -= 0xffff;
    }
    // if (cycles >= 5) { cycles -= 1; };
    DBG("return_cycles: %ld ", cycles);
    usid_main_clk = maincpu_clk;
    return cycles;
}

void us_device_store(uint16_t addr, uint8_t val, int chipno) /* max chipno = 1 */
{
    if (chipno < US_MAXSID) {  /* remove 0x20 address limitation */

        DBG("WRITE: $%02X:%02X ", addr, val);
        const uint_fast32_t cycles = us_delay();
        DBG("delay_cycles: %ld\n", cycles);
        addr = ((addr & 0x1F) + (chipno * 0x20));
        if (isasync == 1) {
            ringpushcycled_USBSID(usbsid, addr, val, cycles);
        } else if (isasync == 0) {
            write_USBSID(usbsid, addr, val);
        }

        sidbuf[addr] = val;
    }
}

void us_set_machine_parameter(long cycles_per_sec)
{
    log_message(LOG_DEFAULT, "[USBSID] %s set clockspeed to: %ld\r", __func__, cycles_per_sec);
    setclockrate_USBSID(usbsid, cycles_per_sec);
}

unsigned int us_device_available(void)
{
    log_message(LOG_DEFAULT, "[USBSID] %s %d SIDs found\r", __func__, sids_found);
    return (sids_found == 1) ? 4 : 1;
}

static void usbsid_alarm_handler(CLOCK offset, void *data)
{
    DBG("CLOCK: ");
    CLOCK cycles = (usid_alarm_clk + offset) - usid_main_clk;
    DBG("[DELAY] maincpu_clk: %ld usid_main_clk: %ld ", maincpu_clk, usid_main_clk);
    DBG("cycles: %ld\n", cycles);
    /* DBG("[%s] %ld\n", __func__, cycles); */

    if (cycles < USBSID_DELAY_CYCLES) {
        usid_alarm_clk = usid_main_clk + USBSID_DELAY_CYCLES;
    } else {
        /* uint delay = (uint) cycles; */
        /* waitforcycle_USBSID(usbsid, cycles); */  // ISSUE: When enabled this breaks WARP speed etc.
        if (isasync == 1) ringpushcycled_USBSID(usbsid, 0xFF, 0xFF, cycles);
        usid_main_clk   = maincpu_clk - offset;
        usid_alarm_clk  = usid_main_clk + USBSID_DELAY_CYCLES;
    }
    alarm_set(usid_alarm, usid_alarm_clk);
}

void us_device_set_async(unsigned int val)
{
    log_message(LOG_DEFAULT, "[USBSID] %s: %x\r", __func__, val);
    isasync = (val == 0) ? 0 : 1;

    if (isasync == 1) restartthread_USBSID(usbsid, true);
}

/* ---------------------------------------------------------------------*/

void us_device_state_read(int chipno, struct sid_us_snapshot_state_s *sid_state)
{
    log_message(LOG_DEFAULT, "[USBSID] %s\r", __func__);
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
    log_message(LOG_DEFAULT, "[USBSID] %s\r", __func__);
    usid_main_clk = (CLOCK)sid_state->usid_main_clk;
    usid_alarm_clk = (CLOCK)sid_state->usid_alarm_clk;
}

#pragma GCC pop_options
#endif /* HAVE_USBSID */
#endif /* UNIX_COMPILE || WINDOWS_COMPILE */
