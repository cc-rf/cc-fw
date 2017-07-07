# Cloud Chaser
Primary Contact: Phillip Sitbon <phillip.sitbon@intel.com>

### Purpose
Cloud Chaser is a sub-GHz radio platform designed to provide robust and reliable wireless
communications with mobile systems (e.g. wearable and IoT)
in a small form factor. Capabilities include high-bandwidth sensor streaming,
ultra-low latency response, long range (1km+) and high speed (up to 1Mbps) configurations,
easy programmability, and full regulatory compliance within the 915MHz/868MHz bands.

### Project Summary

This project supports the Cloud Chaser v1 and v2 boards, which have:

- An NXP (Formerly Freescale) Kinetis K66 180MHz CPU (MK66FX1M0VMD18)
- Two CC1200 radios (v2 has one)
- Two CC1190 range extenders (v2 has one)

The base system code here is largely an adaptation of the Kinetis SDK v2.0.
It has been converted to use CMake but otherwise efforts were
 taken to leave the SDK code as unchanged as possible.

### Project Structure

 - `fw`: The base firmware project directory.
   - `config.cmake`: Defines the project name, loaded from `fw/project/<name>`.
 - `fw/kinetis`: The primary SDK codebase, organized into `freertos`,
 `sdk`, and `middleware` categories.
 - `fw/board`: SDK-integrated board parameters / support.
 - `fw/project/cloudchaser`: The default main project that implements application functionality.

## Build & Flash

Currently only Linux is supported as a build/flash platform, but build on Mac have been reported as successful as well.

### Building

CMake does most of the work, you lucky dog.

#### Prerequisites

- Ubuntu/Debian packages:
  - build-essential
  - cmake (>=3.4)
  - binutils-arm-none-eabi
  - gcc-arm-none-eabi
  - gdb-arm-none-eabi
  - libnewlib-arm-none-eabi
- JLink Tools:
  - (64-bit deb) https://www.segger.com/jlink-software.html

#### Configuring

The default configuration builds the project in `fw/project/cloudchaser`, which is
set in `fw/config.cmake`. The board selection for the project is defined
in `fw/project/cloudchaser/config.cmake`, which is `cloudchaser` by default.

If any `config.cmake` parameters are changed, the `build` directory
below needs to be wiped and `cmake` re-run.

#### Build

From the top level folder:

    sky:cloudchaser$ mkdir build && cd $_
    sky:build$ cmake ..
    sky:build$ make

After this, the target `.elf` file is `fw.elf` in the `build` folder.

#### Flash

This process is currently a bit janky, and will eventually include a nice
make command to go with it. For now, you have two options: a one-off
flash or a flash & debug (with `gdb`).

##### One-off Flash

Connect your J-Link to Cloud Chaser and run the following:

    sky:build$ ../flash.sh

##### Flash & Debug

    sky:build$ ../jlink.sh

In a separate console:

    sky:build$ ../debug.sh

At this point, the board is running and you can use Ctrl+C to halt
and do fancy debug stuff.
If you desire SWO (debug) output, in yet another window do:

    sky:build$ telnet localhost 2332

You may need to install the telnet package.

#### Advanced Debugging

If you want to get serious about debugging and, like the maintainer
don't know gdb very deeply, you might like Segger's J-Link Debugger from their software page.
It has a graphical interface and is easy to use.

If you want to get going with it fast, point it to the settings file
located at `etc/fw.jdebug`.

## Usage

Currently the primary interface to Cloud Chaser is via Python (2.7+) scripts.
Within the `etc/scripts` folder, `cc.py` defines the interface class and provides
examples on how to send and receive packets.
