/*
 * soundasid.c - Implementation of the asid protocol midi sound device.
 *
 * Written by
 *  aTc <aTc@k-n-p.org>
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

/* #define MIDI_DEBUG */
#if defined(MIDI_DEBUG)
#define __RTMIDI_DEBUG__
#endif

#if defined(UNIX_COMPILE) && defined(USE_ALSA)
#define __LINUX_ALSA__
// #define __UNIX_JACK__
#endif
#if defined(WINDOWS_COMPILE)
#define __WINDOWS_MM__
#endif


// #if defined(USE_ALSA) || defined(WINDOWS_COMPILE)

#if defined(UNIX_COMPILE) && defined(__LINUX_ALSA__) || defined(WINDOWS_COMPILE) && defined(__WINDOWS_MM__)
extern "C" {

#include <stdio.h>

#include "debug.h"
#include "log.h"
#include "sound.h"
#include "types.h"

#include "../../sid/sid.h"
#include "../../sid/resid.h"
#include "maincpu.h"
#include "machine.h"

#include "RtMidi.cpp"

#ifdef HAVE_USBSID
#define ASID_MAXSID SID_ENGINE_USBSID_NUM_SIDS
#else
#define ASID_MAXSID 1
#endif

int regmap[] = {0,1,2,3,5,6,7,8,9,10,12,13,14,15,16,17,19,20,21,22,23,24,4,11,18,25,26,27};
RtMidiOut *midiout;
// static unsigned char sid_register[(ASID_MAXSID * 0x20)];  // static for auto zero
// static unsigned char sid_modified[(ASID_MAXSID * 0x20)];  // static for auto zero
unsigned char sid_register[28];
unsigned char sid_modified[28];
std::vector<unsigned char> message;


// static FILE *dump_fd = NULL;

static int asid_init(const char *param, int *speed,
             int *fragsize, int *fragnr, int *channels)
{
    log_message(LOG_DEFAULT,"[ASID DBG][param]%s[speed]%n[fragsize]%n[fragnr]%n[channels]%n",param,speed,fragsize,fragnr,channels);
    /* No stereo capability. */
    *channels = 1;
    int i, nports, asidport = 0;
    log_message(LOG_DEFAULT,"[ASID] open: %s",param);
    // dump_fd = fopen(param?param:"vicesnd.sid", "w");
    // return !dump_fd;
    //
    //

    asidport = atoi(param);
    midiout = new RtMidiOut();
    nports = midiout->getPortCount();
    log_message(LOG_DEFAULT,"[ASID] Available ports:");
    for(i = 0; i < nports; i++) {
        log_message(LOG_DEFAULT,"[ASID] Port %d : %s", i, midiout->getPortName(i).c_str());
    }

    if (asidport >= nports) {
        log_message(LOG_DEFAULT,"[ASID] Requested port: %d not available", asidport);
        asidport = 0;
    }

    log_message(LOG_DEFAULT,"[ASID] Using port: %d %s", asidport, midiout->getPortName(asidport).c_str());

    midiout->openPort(asidport);

    // disabled
    // for (i = 0; i < (ASID_MAXSID * 0x20); i++) {  /* Set 0 here!? */
    //     sid_register[i] = 0;
    //     sid_modified[i] = 0;
    // }

    message.clear();
    message.push_back(0xf0);
    message.push_back(0x2d);
    message.push_back(0x4c);
    message.push_back(0xf7);
    midiout->sendMessage(&message); //start sid play mode

    return 0;
}

// static int asid_write(SWORD *pbuf, size_t nr)
static int asid_write(signed short *pbuf, size_t nr)
{
    printf("%s\n", __func__);
    return 0;
}

// static int sidsintune[ASID_MAXSID];
extern long machine_get_cycles_per_second(void);
extern long machine_get_cycles_per_frame(void);
// static int asid_dump(WORD addr, BYTE byte, CLOCK clks)
static int asid_dump(unsigned short addr, unsigned char byte, CLOCK clks)
{
    // static CLOCK maincpu_clk_prev;
    int reg,data;

    reg=addr & 0x1f;
    data=byte;
    if(sid_modified[reg]==0)
    {
        sid_register[reg]=data & 0xff;
        sid_modified[reg]++;
    }
    else
    {
        switch(reg)
        {
            case 0x04:
                if(sid_modified[0x19]!=0) sid_register[0x04]=sid_register[0x19]; //if already written to secondary,move back to original one
                sid_register[0x19]=data & 0xff;
                sid_modified[0x19]++;
            break;
            case 0x0b:
                if(sid_modified[0x1a]!=0) sid_register[0x0b]=sid_register[0x1a];
                sid_register[0x1a]=data & 0xff;
                sid_modified[0x1a]++;
            break;
            case 0x12:
                if(sid_modified[0x1b]!=0) sid_register[0x12]=sid_register[0x1b];
                sid_register[0x1b]=data & 0xff;
                sid_modified[0x1b]++;
            break;

            default:
                sid_register[reg]=data & 0xff;
                sid_modified[reg]++;
        }
    }
    // printf("[DUMP]$%02x$%02x[CLKCYCLES]%08ld[SINCELAST]%06ld[CLKS]%04ld(%s)\n", addr, byte, maincpu_clk, (maincpu_clk - maincpu_clk_prev), clks, __func__);
    // maincpu_clk_prev = maincpu_clk;

    // int reg, data, mask;
    // if (ASID_MAXSID > 1) mask = 0x7F;
    // else mask = 0x1F;
    // reg = addr & mask;  // Add 4x SID /* 0x1f; */
    // data = byte;
    // if (addr <= 0x1F) sidsintune[0] = 1;
    // if (ASID_MAXSID > 1) {
    //     if (addr >= 0x20 || addr <= 0x3F) sidsintune[1] = 1;
    //     if (addr >= 0x40 || addr <= 0x5F) sidsintune[2] = 1;
    //     if (addr >= 0x60 || addr <= 0x7F) sidsintune[3] = 1;
    // }

    // if(sid_modified[reg] == 0)
    // {
    //     sid_register[reg] = (data & 0xff);
    //     sid_modified[reg]++;
    // }
    // else
    // {
    //     switch(reg)
    //     {
    //         /* Voice 1 */
    //         case 0x04:
    //             if(sid_modified[0x19] != 0) {
    //                 sid_register[0x04] = sid_register[0x19]; // if already written to secondary, move back to original one
    //             }
    //             sid_register[0x19] = (data & 0xff);
    //             sid_modified[0x19]++;
    //             break;
    //         case 0x24:
    //             if(sid_modified[0x39] != 0) {
    //                 sid_register[0x24] = sid_register[0x39]; // if already written to secondary, move back to original one
    //             }
    //             sid_register[0x39] = (data & 0xff);
    //             sid_modified[0x39]++;
    //             break;
    //         case 0x44:
    //             if(sid_modified[0x59] != 0) {
    //                 sid_register[0x44] = sid_register[0x59]; // if already written to secondary, move back to original one
    //             }
    //             sid_register[0x59] = (data & 0xff);
    //             sid_modified[0x59]++;
    //             break;
    //         case 0x64:
    //             if(sid_modified[0x79] != 0) {
    //                 sid_register[0x64] = sid_register[0x79]; // if already written to secondary, move back to original one
    //             }
    //             sid_register[0x79] = (data & 0xff);
    //             sid_modified[0x79]++;
    //             break;
    //         /* Voice 2 */
    //         case 0x0b:
    //             if(sid_modified[0x1a] != 0) {
    //                 sid_register[0x0b] = sid_register[0x1a];
    //             }
    //             sid_register[0x1a] = (data & 0xff);
    //             sid_modified[0x1a]++;
    //             break;
    //         case 0x2b:
    //             if(sid_modified[0x3a] != 0) {
    //                 sid_register[0x2b] = sid_register[0x3a];
    //             }
    //             sid_register[0x3a] = (data & 0xff);
    //             sid_modified[0x3a]++;
    //             break;
    //         case 0x4b:
    //             if(sid_modified[0x5a] != 0) {
    //                 sid_register[0x4b] = sid_register[0x5a];
    //             }
    //             sid_register[0x5a] = (data & 0xff);
    //             sid_modified[0x5a]++;
    //             break;
    //         case 0x6b:
    //             if(sid_modified[0x7a] != 0) {
    //                 sid_register[0x6b] = sid_register[0x7a];
    //             }
    //             sid_register[0x7a] = (data & 0xff);
    //             sid_modified[0x7a]++;
    //             break;
    //         /* Voice 3 */
    //         case 0x12:
    //             if(sid_modified[0x1b] != 0) {
    //                 sid_register[0x12] = sid_register[0x1b];
    //             }
    //             sid_register[0x1b] = (data & 0xff);
    //             sid_modified[0x1b]++;
    //             break;
    //         case 0x32:
    //             if(sid_modified[0x3b] != 0) {
    //                 sid_register[0x22] = sid_register[0x3b];
    //             }
    //             sid_register[0x3b] = (data & 0xff);
    //             sid_modified[0x3b]++;
    //             break;
    //         case 0x52:
    //             if(sid_modified[0x5b] != 0) {
    //                 sid_register[0x42] = sid_register[0x5b];
    //             }
    //             sid_register[0x5b] = (data & 0xff);
    //             sid_modified[0x5b]++;
    //             break;
    //         case 0x72:
    //             if(sid_modified[0x7b] != 0) {
    //                 sid_register[0x62] = sid_register[0x7b];
    //             }
    //             sid_register[0x7b] = (data & 0xff);
    //             sid_modified[0x7b]++;
    //             break;
    //         default:
    //             sid_register[reg] = (data & 0xff);
    //             sid_modified[reg]++;
    //     }
    // }


    //printf("%d %d %d |", (int)clks, addr, byte) < 0;
    return 0;
}

// {
//     for (int i = 0; i < (4 * 0x20); i++) {
//         if ((i == 0) || (i % 0x20 == 0)) printf("[S] ");
//         if (((i & 0x1f) - 0x1c) > 0) continue;
//         printf("%02x ", i);
//         if (i != 0 && ((i & 0x1c) == 0x1c)) printf("[E]\n");
//     }
// }

static int asid_flush(char *state)
{
    // static CLOCK maincpu_clk_prev;
    int i,j;
    // unsigned int r=0;
    unsigned int mask=0;
    unsigned int msb=0;

    message.clear();
    message.push_back(0xf0);
    message.push_back(0x2d);
    message.push_back(0x4e);
    // set bits in mask for each register that has been written to
    // write last bit of each register into msb
    for(i=0;i<28;i++)
    {
        j=regmap[i];
        if(sid_modified[j]!=0)
        {
            mask=mask | (1<<i);
        }
        if(sid_register[j]>0x7f)
        {
            msb=msb | (1<<i);
        }
    }
    message.push_back(mask & 0x7f);
    message.push_back((mask>>7)&0x7f);
    message.push_back((mask>>14)&0x7f);
    message.push_back((mask>>21)&0x7f);
    message.push_back(msb & 0x7f);
    message.push_back((msb>>7)&0x7f);
    message.push_back((msb>>14)&0x7f);
    message.push_back((msb>>21)&0x7f);
    for(i=0;i<28;i++)
    {
        j=regmap[i];
        if(sid_modified[j]!=0)
        {
            message.push_back(sid_register[j]&0x7f);
        }
    }
    message.push_back(0xf7);
    midiout->sendMessage(&message);
    for(i=0;i<28;i++)
    {
        sid_modified[i]=0;
    }


    //printf("\n");
    //if (printf("*%s\n", state) < 0)
    //    return 1;

    // NOTE: FRAME FLUSH HAPPENS EVERY 60~65 CYCLES
    // printf("[FRAME FLUSH]%02x[CLKCYCLES]%08ld[SINCELAST]%06ld(%s)\n", state, maincpu_clk, (maincpu_clk - maincpu_clk_prev), __func__);
    // maincpu_clk_prev = maincpu_clk;
    return 0;
    //return fflush(dump_fd);

    // int i, j;
    // // unsigned int r=0;
    // unsigned int mask = 0;
    // unsigned int msb = 0;

    // for (int s = 0; s < ASID_MAXSID; s++) {
    //     if (sidsintune[s] == 0) continue;
    //     int sidno = (s == 0) ? 0x4e : (0x4f + s);

    //     message.clear();
    //     message.push_back(0xf0);
    //     message.push_back(0x2d);
    //     message.push_back(sidno /* 0x4e */);
    //     // set bits in mask for each register that has been written to
    //     // write last bit of each register into msb
    //     // for(i = 0; i < 28; i++) {
    //     // for(i = 0; i < (ASID_MAXSID * 0x20); i++) {
    //     int start_addr = (s * 0x20);
    //     for(i = start_addr; i < (start_addr + 0x1F) ; i++) {
    //         if (((i & 0x1F) - 0x1C) > 0) continue;  /* Workaround to skip 0x1D~0x1F addresses */
    //         if (i >= 0x0 && i <= 0x1C) j = regmap[i];
    //         if (i >= 0x20 && i <= 0x3C) j = regmap[(i - 0x20)] + 0x20;
    //         if (i >= 0x40 && i <= 0x5C) j = regmap[(i - 0x40)] + 0x40;
    //         if (i >= 0x60 && i <= 0x7C) j = regmap[(i - 0x60)] + 0x60;
    //         if(sid_modified[j] != 0) {
    //             // mask = mask | (1 << i);
    //             mask |= (1 << i);
    //         }
    //         if(sid_register[j] > 0x7f)
    //         {
    //             // msb = msb | (1 << i);
    //             msb |= (1 << i);
    //         }
    //     }
    //     message.push_back(mask & 0x7f);
    //     message.push_back((mask >> 7) & 0x7f);
    //     message.push_back((mask >> 14) & 0x7f);
    //     message.push_back((mask >> 21) & 0x7f);
    //     message.push_back(msb & 0x7f);
    //     message.push_back((msb >> 7) & 0x7f);
    //     message.push_back((msb >> 14) & 0x7f);
    //     message.push_back((msb >> 21) & 0x7f);
    //     // for(i = 0; i < 28; i++) {
    //     for(i = start_addr; i < (start_addr + 0x1F) ; i++) {
    //         // j = regmap[i];
    //         if (((i & 0x1F) - 0x1C) > 0) continue;  /* Workaround to skip 0x1D~0x1F addresses */
    //         if (i >= 0x0 && i <= 0x1C) j = regmap[i];
    //         if (i >= 0x20 && i <= 0x3C) j = regmap[(i - 0x20)] + 0x20;
    //         if (i >= 0x40 && i <= 0x5C) j = regmap[(i - 0x40)] + 0x40;
    //         if (i >= 0x60 && i <= 0x7C) j = regmap[(i - 0x60)] + 0x60;
    //         if(sid_modified[j] != 0) {
    //             message.push_back(sid_register[j] & 0x7f);
    //         }
    //     }
    //     message.push_back(0xf7);
    //     midiout->sendMessage(&message);
    //     // for(i = 0; i < 28; i++) {
    //     // for(i = 0; i < (ASID_MAXSID * 0x20); i++) {
    //     for(i = start_addr; i < (start_addr + 0x1F) ; i++) {
    //         sid_modified[i] = 0;
    //     }
    //     if (start_addr >= 0x0 && start_addr <= 0x1C) sidsintune[0] = 0;
    //     if (start_addr >= 0x20 && start_addr <= 0x3C) sidsintune[1] = 0;
    //     if (start_addr >= 0x40 && start_addr <= 0x5C) sidsintune[2] = 0;
    //     if (start_addr >= 0x60 && start_addr <= 0x7C) sidsintune[3] = 0;
    // }
    // // for(i = 0; i < (ASID_MAXSID * 0x20); i++) {
    // //     sid_modified[i] = 0;
    // // }
    // // memset(sidsintune, 0, ASID_MAXSID); /* Clear n sids */


    // //printf("\n");
    // //if (printf("*%s\n", state) < 0)
    // //    return 1;

    // return 0;
    // //return fflush(dump_fd);
}

static void asid_close(void)
{
    //printf("asidclose!\n");

    message.clear();
    message.push_back(0xf0);
    message.push_back(0x2d);
    message.push_back(0x4d);
    message.push_back(0xf7);
    midiout->sendMessage(&message);
    delete midiout;

    //fclose(dump_fd);
    //dump_fd = NULL;
}

static sound_device_t asid_device =
{
    "asid",
    asid_init,
    asid_write,
    asid_dump,
    asid_flush,
    NULL,
    asid_close,
    NULL,
    NULL,
    0
};

int sound_init_asid_device(void)
{
    return sound_register_device(&asid_device);
}

} // extern "C"

#endif /* UNIX_COMPILE && __LINUX_ALSA__|| WINDOWS_COMPILE && __WINDOWS_MM__ */
