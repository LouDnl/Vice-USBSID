/*
 * USBSID-Pico is a RPi Pico (RP2040/RP2350) based board for interfacing one
 * or two MOS SID chips and/or hardware SID emulators over (WEB)USB with your
 * computer, phone or ASID supporting player.
 *
 * USBSIDInterface.cpp
 * This file is part of USBSID-Pico (https://github.com/LouDnl/USBSID-Pico-driver)
 * File author: LouD
 *
 * Copyright (c) 2024 LouD
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "USBSIDInterface.h"
#include "USBSID.h"

using namespace USBSID_NS;

extern "C"
{
  USBSIDitf create_USBSID(void){
    return (USBSID_Class*) new USBSID_Class();
  };
  int init_USBSID(USBSIDitf p, bool start_threaded, bool with_cycles){
    if( p == NULL ) return -1;
    return ((USBSID_Class*) p)->USBSID_Init(start_threaded, with_cycles);
  };
  void close_USBSID(USBSIDitf p){
    if( p == NULL ) return;
    delete (USBSID_Class*) p;
  };
  void pause_USBSID(USBSIDitf p){
    if( p == NULL ) return;
    return ((USBSID_Class*)p)->USBSID_Pause();
  };
  void reset_USBSID(USBSIDitf p){
    if( p == NULL ) return;
    return ((USBSID_Class*)p)->USBSID_Reset();
  };
  void resetallregisters_USBSID(USBSIDitf p){
    if( p == NULL ) return;
    return ((USBSID_Class*)p)->USBSID_ResetAllRegisters();
  };
  void clearbus_USBSID(USBSIDitf p){
    if( p == NULL ) return;
    return ((USBSID_Class*)p)->USBSID_ClearBus();
  };
  void mute_USBSID(USBSIDitf p){
    if( p == NULL ) return;
    return ((USBSID_Class*)p)->USBSID_Mute();
  };
  void unmute_USBSID(USBSIDitf p){
    if( p == NULL ) return;
    return ((USBSID_Class*)p)->USBSID_UnMute();
  };
  void setclockrate_USBSID(USBSIDitf p, long clockrate_cycles, bool suspend_sids){
    if( p == NULL ) return;
    return ((USBSID_Class*)p)->USBSID_SetClockRate(clockrate_cycles, suspend_sids);
  };
  long getclockrate_USBSID(USBSIDitf p){
    if( p == NULL ) return 0;
    return ((USBSID_Class*)p)->USBSID_GetClockRate();
  };
  long getrefreshrate_USBSID(USBSIDitf p){
    return ((USBSID_Class*)p)->USBSID_GetRefreshRate();
  };
  long getrasterrate_USBSID(USBSIDitf p){
    return ((USBSID_Class*)p)->USBSID_GetRasterRate();
  };
  void writesingle_USBSID(USBSIDitf p, unsigned char *buff, size_t len){
    if( p == NULL ) return;
    return ((USBSID_Class*)p)->USBSID_SingleWrite(buff, len);
  };
  unsigned char readsingle_USBSID(USBSIDitf p, uint8_t reg){
    if( p == NULL ) return 0;
    return ((USBSID_Class*)p)->USBSID_SingleRead(reg);
  };
  void writebuffer_USBSID(USBSIDitf p, unsigned char *buff, size_t len){
    if( p == NULL ) return;
    return ((USBSID_Class*)p)->USBSID_Write(buff, len);
  };
  void write_USBSID(USBSIDitf p, uint16_t reg, uint8_t val){
    if( p == NULL ) return;
    return ((USBSID_Class*)p)->USBSID_Write(reg, val);
  };
  void writecycled_USBSID(USBSIDitf p, uint16_t reg, uint8_t val, uint16_t cycles){
    if( p == NULL ) return;
    return ((USBSID_Class*)p)->USBSID_Write(reg, val, cycles);
  };
  unsigned char read_USBSID(USBSIDitf p, unsigned char *writebuff){
    if( p == NULL ) return 0;
    return ((USBSID_Class*)p)->USBSID_Read(writebuff);
  };
  void writering_USBSID(USBSIDitf p, uint16_t reg, uint8_t val){
    if( p == NULL ) return;
    return ((USBSID_Class*) p)->USBSID_WriteRing(reg, val);
  };
  void writeringcycled_USBSID(USBSIDitf p, uint16_t reg, uint8_t val, uint16_t cycles){
    if( p == NULL ) return;
    return ((USBSID_Class*) p)->USBSID_WriteRingCycled(reg, val, cycles);
  };
  void setflush_USBSID(USBSIDitf p){
    if( p == NULL ) return;
    return ((USBSID_Class*) p)->USBSID_SetFlush();
  };
  void flush_USBSID(USBSIDitf p){
    if( p == NULL ) return;
    return ((USBSID_Class*) p)->USBSID_Flush();
  };
  void restartthread_USBSID(USBSIDitf p, bool with_cycles){
    if( p == NULL ) return;
    return ((USBSID_Class*) p)->USBSID_RestartThread(with_cycles);
  }
  int_fast64_t waitforcycle_USBSID(USBSIDitf p, uint_fast64_t cycles){
    if( p == NULL ) return 0;
    return ((USBSID_Class*) p)->USBSID_WaitForCycle(cycles);
  };
}
