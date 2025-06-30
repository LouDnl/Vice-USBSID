# USBSID-Pico-driver
Driver for USBSID-Pico

This repo is a work-in-progress and still in early crude form!

Compilation test
```shell
rm *.o ; g++ $(pkg-config --cflags --libs libusb-1.0) -c *.cpp $(pkg-config --cflags --libs libusb-1.0) -Wall
```
