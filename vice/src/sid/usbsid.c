/*
 * usbsid.c - Generic usbsid abstraction layer.
 *
 * Written by
 *  Andreas Boose <viceteam@t-online.de>
 *  usbsid Support <support@usbsid.com>
 *  Marco van den Heuvel <blackystardust68@yahoo.com>
 *  LouDnl
 *
 * This file is part of VICE, the Versatile Commodore Emulator.
 * See README for copyright notice.
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
#ifdef HAVE_USBSID

#include <stdio.h>
#include <string.h>

#include "usbsid.h"
#include "log.h"
#include "sid-snapshot.h"
#include "types.h"

/* #define DEBUG_USBSID */

/* define to trace usbsid stuff without having a usbsid */
/* #define DEBUG_USBSID_DUMMY */

#if defined(DEBUG_USBSID_DUMMY)

#define usbsid_drv_available() 1
#define usbsid_drv_reset(bool us_reset) printf("[USBSID] usbsid_drv_reset\n")
#define usbsid_drv_open() (printf("[USBSID] usbsid_drv_open\n"), 0)
#define usbsid_drv_close() printf("[USBSID] usbsid_drv_close\n")
#define usbsid_drv_read(addr, chipno)  (printf("[USBSID] usbsid_drv_read addr:%02x chip:%d\n", addr, chipno), 1)
#define usbsid_drv_store(addr, val, chipno) printf("[USBSID] usbsid_drv_store addr:%02x val:%02x chip:%d\n", addr, val, chipno)
#define usbsid_drv_state_read(chipno, sid_state) printf("[USBSID] usbsid_drv_state_read chip:%d sid_state:%p\n", chipno, sid_state)
#define usbsid_drv_state_write(chipno, sid_state) printf("[USBSID] usbsid_drv_state_write chip:%d sid_state:%p\n", chipno, sid_state)
#define usbsid_drv_set_readmode(val) printf("[USBSID] usbsid_drv_set_readmode read_mode:%p\n", val)

#endif

#if defined(DEBUG_USBSID_DUMMY) || defined(DEBUG_USBSID)
#define DBG(x)  log_debug x
#else
#define DBG(x)
#endif

static int usbsid_is_open = -1;

/* Buffer containing current register state of SIDs */
static uint8_t sidbuf[0x20 * US_MAXSID];

int usbsid_open(void)
{
    DBG(("[USBSID] %s %d\r\n", __func__, usbsid_is_open));
    if (usbsid_is_open) {
        usbsid_is_open = usbsid_drv_open();
        memset(sidbuf, 0, sizeof(sidbuf));
    }
    if (usbsid_is_open == -1)
        log_error(LOG_DEFAULT, "[USBSID] Failed to open USBSID\r\n");
    DBG(("[USBSID] usbsid_open usbsid_is_open=%d\n", usbsid_is_open));
    return usbsid_is_open;
}

int usbsid_close(void)
{
    DBG(("[USBSID] %s %d\r\n", __func__, usbsid_is_open));
    if (!usbsid_is_open) {
        usbsid_drv_close();
        usbsid_is_open = -1;
    }
    DBG(("[USBSID] usbsid_close usbsid_is_open=%d\n", usbsid_is_open));
    return usbsid_is_open;
}

void usbsid_reset(bool us_reset)
{
    DBG(("[USBSID] %s %d\r\n", __func__, usbsid_is_open));
    if (!usbsid_is_open) {
        usbsid_drv_reset(us_reset);
    }
}

int usbsid_read(uint16_t addr, int chipno)
{
    if (!usbsid_is_open && chipno < US_MAXSID) {
        int val = usbsid_drv_read(addr, chipno);
        sidbuf[(chipno * 0x20) + addr] = (uint8_t)val;
        DBG(("[R]@0x%04x [S]%d [?]%d\r\n", addr, chipno, addr +(chipno * 0x20)));
        return val;
    }

    return usbsid_is_open;
}

void usbsid_store(uint16_t addr, uint8_t val, int chipno)
{
    if (!usbsid_is_open && chipno < US_MAXSID) {
        /* write to sidbuf[] for write-only registers */
        sidbuf[(chipno * 0x20) + addr] = val;
        DBG(("[W]@0x%04x [S]%d [?]%d\r\n", addr, chipno, addr + (chipno * 0x20)));
        usbsid_drv_store(addr, val, chipno);
    }
}

void usbsid_set_machine_parameter(long cycles_per_sec)
{
    usbsid_drv_set_machine_parameter(cycles_per_sec);
}

int usbsid_available(void)
{
    DBG(("[USBSID] %s %d\r\n", __func__, usbsid_is_open));
    if (usbsid_is_open < 0) {
        usbsid_open();
    }

    if (usbsid_is_open >= 0) {
        return usbsid_drv_available();
    }
    return usbsid_is_open;
}

void usbsid_set_audio(int val)
{
    if (!usbsid_is_open) {
        usbsid_drv_set_audio(val);
    }
}

void usbsid_set_readmode(int val)
{
    if (!usbsid_is_open) {
        usbsid_drv_set_readmode(val);
    }
}

/* ---------------------------------------------------------------------*/

void usbsid_state_read(int chipno, struct sid_us_snapshot_state_s *sid_state)
{
    DBG(("[USBSID] %s\r\n", __func__));
    int i;

    if (chipno < US_MAXSID) {
        for (i = 0; i < 32; ++i) {
            sid_state->regs[i] = sidbuf[i + (chipno * 0x20)];
        }
        usbsid_drv_state_read(chipno, sid_state);
    }
}

void usbsid_state_write(int chipno, struct sid_us_snapshot_state_s *sid_state)
{
    DBG(("[USBSID] %s\r\n", __func__));
    int i;

    if (chipno < US_MAXSID) {
        for (i = 0; i < 32; ++i) {
            sidbuf[i + (chipno * 0x20)] = sid_state->regs[i];
        }
        usbsid_drv_state_write(chipno, sid_state);
    }
}
#else
int usbsid_available(void)
{
    return 0;
}

#endif /* HAVE_USBSID */
#endif /* UNIX_COMPILE || WINDOWS_COMPILE */
