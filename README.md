# Vice [USBSID-Pico](https://github.com/LouDnl/USBSID-Pico) fork
This fork has built-in support for USBSID-Pico. \
USBSID-Pico is a RPi Pico based board for interfacing one or two MOS SID chips and/or hardware SID emulators with your PC over USB.

# Linux Building and installing
For building you can mostly follow the instructions in the [Linux-GTK3-Howto](vice/doc/building/Linux-GTK3-Howto.txt) \
add `--enable-usbsid` to `./configure` for [USBSID-Pico](https://github.com/LouDnl/USBSID-Pico) support

## Onliner for installing the dependencies
```bash
sudo apt install autoconf automake build-essential byacc flex xa65 gawk libgtk-3-dev texinfo texlive-fonts-recommended texlive-latex-extra dos2unix libpulse-dev libasound2-dev libglew-dev libcurl4-openssl-dev libevdev-dev libpng-dev libgif-dev libpcap-dev libusb-1.0-0 libusb-1.0-0-dev libusb-dev libmpg123-dev libmp3lame-dev
```

## My build sequence for Linux
```bash
    # clone the repository
    git clone https://github.com/LouDnl/Vice-USBSID.git
    cd Vice-USBSID

    # clone the driver
    git clone https://github.com/LouDnl/USBSID-Pico-driver.git

    # copy the driver files into the correct directory
    cd USBSID-Pico-driver
    cp USBSID* ../vice/src/arch/shared/hwsiddrv/
    cd ..

    # create a build directory
    mkdir -p usbsid/{build,release/usr}
    export OUTDIR=$(pwd)/usbsid/release

    # generate configure and make files
    cd vice
    ./src/buildtools/genvicedate_h.sh
    ./autogen.sh

    # change to the build directory
    cd ../usbsid/build

    # configure make with what you need
    ../../vice/configure \
       --enable-usbsid \
       --enable-option-checking=fatal \
       --prefix=/usr \
       --enable-gtk3ui \
       --disable-arch \
       --disable-html-docs \
       --enable-cpuhistory \
       --enable-io-simulation \
       --enable-experimental-devices \
       --enable-x64-image \
       --enable-ethernet \
       --enable-midi \
       --disable-catweasel \
       --disable-hardsid \
       --with-alsa \
       --with-pulse \
       --with-fastsid \
       --with-gif \
       --with-lame \
       --with-libcurl \
       --with-libieee1284 \
       --with-mpg123 \
       --with-png \
       --with-resid

    # optional recording types
    --with-flac 
    --with-vorbis

    # optional config options
    --enable-debug
    --enable-debug-threads
    --enable-pdf-docs

    # run make
    make -j$(nproc) -s --no-print-directory
    # (OPTIONAL) Create installation files
    make DESTDIR=$OUTDIR install

    # start Vice
    # after first compile
    ./src/x64sc -directory ./data
    ./src/vsid -directory ./data
    # after first installation to current system
    ./src/x64sc
    ./src/vsid

    # Installation to current system
    sudo make install
    # Now you can run vice directly
    x64sc
    vice
    # Optional commandline options
    -sidenginemodel usbsid  # enables USBSID (available in settings menu too)
```

# Windows building (on linux) and creating a zip file with binaries
For building you can use the instructions in the [Docker-Mingw32-build](vice/build/mingw/docker/README-docker-mingw32-build.md) as guideline.  
After installing docker follow the following steps to create win32 or win64 binaries.
``` shell
    # clone the repository
    git clone https://github.com/LouDnl/Vice-USBSID.git
    cd Vice-USBSID

    # generate configure and make files
    cd vice
    ./src/buildtools/genvicedate_h.sh
    ./autogen.sh
    cd ..

    # copy the docker files to current directory 
    cp vice/build/mingw/docker/* .

    # creating the base docker container
    docker build --tag vice-buildcontainer:base .

    # choose win32 or win64
    ## for win32 rename the correct build script and create acontainer
    cp build-vice-usbsid-win32.sh build-vice.sh
    docker build --tag vice-buildcontainer:0.3 -f Dockerfile.usbsid-win32 .
    ## for win64 rename the correct build script and create acontainer
    cp build-vice-usbsid-win64.sh build-vice.sh
    docker build --tag vice-buildcontainer:0.3 -f Dockerfile.usbsid-win64 .

    # depending on the container type you created this will create a zip file in the current directory
    ./dock-run.sh $(pwd)
```


# VICE GitHub Mirror
This is the official git mirror of the [VICE subversion repo](https://sourceforge.net/p/vice-emu/code/HEAD/tree/).

For news, documentation, developer information, [visit the VICE website](https://vice-emu.sourceforge.io/).

## Download VICE
* [Official Releases](https://vice-emu.sourceforge.io/#download)
* [Snapshot builds of the latest code](https://github.com/VICE-Team/svn-mirror/releases)
