#ifndef VICE_USBSID_H
#define VICE_USBSID_H
#ifdef HAVE_USBSID
  // TODO: CHECK AND FINISH
#include "sid-snapshot.h"
#include "types.h"

#define US_MAXSID 2

int usbsid_open(void);
int usbsid_close(void);
void usbsid_reset(void);
int usbsid_read(uint16_t addr, int chipno);
void usbsid_store(uint16_t addr, uint8_t val, int chipno);
int usbsid_available(void);

int usbsid_drv_open(void);
int usbsid_drv_close(void);
void usbsid_drv_reset(void);
int usbsid_drv_read(uint16_t addr, int chipno);
void usbsid_drv_store(uint16_t addr, uint8_t val, int chipno);
int usbsid_drv_available(void);

// void usbsid_state_read(int chipno, struct sid_us_snapshot_state_s *sid_state);
// void usbsid_state_write(int chipno, struct sid_us_snapshot_state_s *sid_state);

// void usbsid_drv_state_read(int chipno, struct sid_us_snapshot_state_s *sid_state);
// void usbsid_drv_state_write(int chipno, struct sid_us_snapshot_state_s *sid_state);

#endif
#endif
