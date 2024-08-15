# Vice [USBSID-Pico](https://github.com/LouDnl/USBSID-Pico) fork
For building you can mostly follow the instructions in the [Linux-GTK3-Howto](vice/doc/building/Linux-GTK3-Howto.txt) \
add `--enable-usbsid` to `./configure` for [USBSID-Pico](https://github.com/LouDnl/USBSID-Pico) support

## Onliner for installing the dependencies
```bash
sudo apt install autoconf automake build-essential byacc flex xa65 gawk libgtk-3-dev texinfo texlive-fonts-recommended texlive-latex-extra dos2unix libpulse-dev libasound2-dev libglew-dev libcurl4-openssl-dev libevdev-dev libpng-dev libgif-dev libpcap-dev libusb-1.0-0 libusb-1.0-0-dev libusb-dev
```

## My go-to build sequence for Linux
```bash
    # clone the repository
    git clone https://github.com/LouDnl/Vice-USBSID.git
    cd Vice-USBSID/vice

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

    # Installation
    sudo make install
    # Now you can run vice directly
    x64sc
    vice
```

# VICE GitHub Mirror
This is the official git mirror of the [VICE subversion repo](https://sourceforge.net/p/vice-emu/code/HEAD/tree/).

For news, documentation, developer information, [visit the VICE website](https://vice-emu.sourceforge.io/).

## Download VICE
* [Official Releases](https://vice-emu.sourceforge.io/#download)
* [Snapshot builds of the latest code](https://github.com/VICE-Team/svn-mirror/releases)
