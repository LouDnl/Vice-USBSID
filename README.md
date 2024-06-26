# Vice [USBSID-Pico](https://github.com/LouDnl/USBSID-Pico) fork
For building you can mostly follow the instructions in the [Linux-GTK3-Howto](vice/doc/building/Linux-GTK3-Howto.txt) \
add `--enable-usbsid` to `./configure` for [USBSID-Pico](https://github.com/LouDnl/USBSID-Pico) support

## My go-to build sequence for Linux
```bash
    # generate configure and make files
    ./autogen.sh

    # configure make with what you need
    ./configure \
       --enable-arch=native \
       --enable-gtk3ui \
       --enable-ethernet \
       --enable-cpuhistory \
       --enable-debug \
       --enable-debug-threads \
       --enable-io-simulation \
       --enable-experimental-devices \
       --enable-x64-image \
       --disable-hardsid \
       --enable-usbsid \
       --with-pulse \
       --with-alsa \
       --with-resid

    # run make
    make -j$(nproc)

    # start after compile
    ## if already installed first
    ./src/x64sc
    ./src/vsid
    ## if not previously installed (will throw error otherwise)
    ./src/x64sc -directory ./data
    ./src/vsid -directory ./data
```

# VICE GitHub Mirror
This is the official git mirror of the [VICE subversion repo](https://sourceforge.net/p/vice-emu/code/HEAD/tree/).

For news, documentation, developer information, [visit the VICE website](https://vice-emu.sourceforge.io/).

## Download VICE
* [Official Releases](https://vice-emu.sourceforge.io/#download)
* [Snapshot builds of the latest code](https://github.com/VICE-Team/svn-mirror/releases)
