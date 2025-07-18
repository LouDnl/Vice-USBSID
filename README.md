# Vice [USBSID-Pico](https://github.com/LouDnl/USBSID-Pico) fork
This is a fork of the official [Vice svn-mirror](https://github.com/VICE-Team/svn-mirror) and has built-in support for USBSID-Pico. \
USBSID-Pico is a RPi Pico based board for interfacing one or two MOS SID chips and/or hardware SID emulators with your PC over USB.

# Usage
Use `-usreadmode 1` with to enable readmode for FPGASID, default is disabled.  
Use `-usaudiomode 1` to enable Stereo mode, default is Mono mode. Works on PCB v1.3 only!
```shell
   # Examples
   x64sc -usreadmode 1 -usaudiomode 1
   vsid -usreadmode 1 -usaudiomode 1
```
# Optional tuning of write buffer and it's head->tail difference size
Buffer and difference size must always be a multiple of 8!  
Use `-usdiffsize 64` to set the head->tail difference size  
Use `-usbuffsize 8192` to set the buffer size  
```shell
   # Examples
   x64sc -usdiffsize 64 -usbuffsize 512
   vsid -usdiffsize 64 -usbuffsize 512
```

# Updates
I try to keep the repo in sync with the main Vice repo. See anything missing? Send me a message.

# Releases
Updated builds can be found in the [TAGS](https://github.com/VICE-Team/svn-mirror/tags) section and  \
Releases can be found in the [RELEASES](https://github.com/LouDnl/Vice-USBSID/releases) section.

# Linux Building and installing
For building you can mostly follow the instructions in the [Linux-GTK3-Howto](vice/doc/building/Linux-GTK3-Howto.txt) \
add `--enable-usbsid` to `./configure` for [USBSID-Pico](https://github.com/LouDnl/USBSID-Pico) support

## Onliner for installing the dependencies
```bash
sudo apt install autoconf automake build-essential byacc flex xa65 gawk libgtk-3-dev texinfo texlive-fonts-recommended texlive-latex-extra dos2unix libpulse-dev libasound2-dev libglew-dev libcurl4-openssl-dev libevdev-dev libpng-dev libgif-dev libpcap-dev libusb-1.0-0 libusb-1.0-0-dev libusb-dev libmpg123-dev libmp3lame-dev
```
For GTK3 Install these dependencies:
??
For SDL2 Install these dependencies:
libsdl2-2.0-0 libsdl2-image-2.0-0
For SDL1 Install these dependencies:
libsdl-image1.2

## My build sequence for Linux
```bash
    # clone the repository
    git clone https://github.com/LouDnl/Vice-USBSID.git
    cd Vice-USBSID

    # clone the driver
    git clone https://github.com/LouDnl/USBSID-Pico-driver.git

    # copy the driver files into the correct directory
    mkdir -p vice/src/lib/libusbsiddrv
    cd USBSID-Pico-driver
    cp README.md LICENSE USBSID* vice-makefile/Makefile.am ../vice/src/lib/libusbsiddrv/
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

    # depending on your preference add on of the
    # following sets to the configure command below
    # GTK3
       --enable-gtk3ui \
    # SDL2
       --enable-sdl2ui \
       --with-sdlsound \
    # SDL1
       --enable-sdl1ui \
       --with-sdlsound \

    # configure make with what you need
    ../../vice/configure \
       --prefix=/usr \
       --enable-option-checking=fatal \
       --enable-usbsid \
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
       --with-pulse \
       --with-alsa \
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
    vsid
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

    # clone the driver
    git clone https://github.com/LouDnl/USBSID-Pico-driver.git

    # copy the driver files into the correct directory
    mkdir -p vice/src/lib/libusbsiddrv
    cd USBSID-Pico-driver
    cp README.md LICENSE USBSID* vice-makefile/Makefile.am ../vice/src/lib/libusbsiddrv/
    cd ..

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
