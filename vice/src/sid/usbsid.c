/*
 * usbsid.c - Generic usbsid abstraction layer.
 *
 * Written by
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
  // TODO: CHECK AND FINISH
/* #define DEBUG_usbsid */
/* define to trace usbsid stuff without having a usbsid */
/* #define DEBUG_usbsid_DUMMY */

#include <string.h>
#include "vice.h"


#ifdef HAVE_USBSID

#include <stdio.h>

#include "usbsid.h"
#include "sid-snapshot.h"
#include "types.h"
#include "log.h"

#if defined(DEBUG_USBSID_DUMMY)

#define usbsid_drv_available() 1
#define usbsid_drv_reset() printf("usbsid_drv_reset\n")
#define usbsid_drv_open() (printf("usbsid_drv_open\n"), 0)
#define usbsid_drv_close() printf("usbsid_drv_close\n")
#define usbsid_drv_read(addr, chipno)  (printf("usbsid_drv_read addr:%02x chip:%d\n", addr, chipno), 1)
#define usbsid_drv_store(addr, val, chipno) printf("usbsid_drv_store addr:%02x val:%02x chip:%d\n", addr, val, chipno)
#define usbsid_drv_state_read(chipno, sid_state) printf("usbsid_drv_state_read chip:%d sid_state:%p\n", chipno, sid_state)
#define usbsid_drv_state_write(chipno, sid_state) printf("usbsid_drv_state_write chip:%d sid_state:%p\n", chipno, sid_state)

#endif

#if defined(DEBUG_USBSID_DUMMY) || defined(DEBUG_USBSID)
#define DBG(x)  log_debug x
#else
#define DBG(x)
#endif

static int usbsid_is_open = -1;

/* buffer containing current register state of SIDs */
static uint8_t sidbuf[0x20 * US_MAXSID];

int usbsid_open(void)
{
    if (usbsid_is_open) {
        usbsid_is_open = usbsid_drv_open();
        memset(sidbuf, 0, sizeof(sidbuf));
    }
    DBG(("usbsid_open usbsid_is_open=%d\n", usbsid_is_open));
    return usbsid_is_open;
}

int usbsid_close(void)
{
    if (!usbsid_is_open) {
        usbsid_drv_close();
        usbsid_is_open = -1;
    }
    DBG(("usbsid_close usbsid_is_open=%d\n", usbsid_is_open));
    return 0;
}

void usbsid_reset(void)
{
    if (!usbsid_is_open) {
        usbsid_drv_reset();
    }
}

int usbsid_read(uint16_t addr, int chipno)
{
    if (!usbsid_is_open && chipno < US_MAXSID) {
        /* use sidbuf[] for write-only registers */
        if (addr <= 0x18) {
            return sidbuf[(chipno * 0x20) + addr];
        }
        return usbsid_drv_read(addr, chipno);
    }

    return 0;
}

void usbsid_store(uint16_t addr, uint8_t val, int chipno)
{
    if (!usbsid_is_open && chipno < US_MAXSID) {
        /* write to sidbuf[] for write-only registers */
        if (addr <= 0x18) {
            sidbuf[(chipno * 0x20) + addr] = val;
        }
        usbsid_drv_store(addr, val, chipno);
    }
}


int usbsid_available(void)
{
    if (usbsid_is_open) {
        usbsid_open();
    }

    if (!usbsid_is_open) {
        return usbsid_drv_available();
    }
    return 0;
}


/* ---------------------------------------------------------------------*/

// void usbsid_state_read(int chipno, struct sid_us_snapshot_state_s *sid_state)
// {
//     int i;

//     if (chipno < US_MAXSID) {
//         for (i = 0; i < 32; ++i) {
//             sid_state->regs[i] = sidbuf[i + (chipno * 0x20)];
//         }
//         usbsid_drv_state_read(chipno, sid_state);
//     }
// }

// void usbsid_state_write(int chipno, struct sid_us_snapshot_state_s *sid_state)
// {
//     int i;

//     if (chipno < US_MAXSID) {
//         for (i = 0; i < 32; ++i) {
//             sidbuf[i + (chipno * 0x20)] = sid_state->regs[i];
//         }
//         // usbsid_store(chipno, sid_state);
//         usbsid_drv_state_write(chipno, sid_state);
//     }
// }
#else
int usbsid_available(void)
{
    return 0;
}
#endif
