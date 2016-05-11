# Cloud Chaser

This project supports the Cloud Chaser board, which has:

- An NXP (Formerly Freescale) Kinetis K66 180MHz CPU (MK66FX1M0VMD18)
- Two CC1200 radios
- Two CC1190 range extenders

The code here is largely an adaptation of the Kinetis SDK v2.0.
It has been converted to use CMake but otherwise efforts were
 taken to leave the SDK code as unchanged as possible.

Things are still in the early stages. Please contact
Phillip Sitbon <phillip.sitbon@intel.com> with inquiries.

### Project Structure

 - `fw`: The base firmware project directory
 (this repository will eventually hold additional support projects
 and libraries).
 - `fw/core`: The primary SDK codebase, organized into `board`,
 `cpu`, `driver`, and other components.
 - `fw/project`: Project folders that define the operation of the
 device. Each project specifies which board it uses, which in turn
 specifies which CPU is used. One project is configured as the "main"
 project for the primary configure & build phases (more below).

## Build & Flash

Currently only Linux is supported as a build/flash platform, but there
is potential for that to change in the future.

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
  - (64-bit deb) https://www.segger.com/jlink-software.html?step=1&file=JLinkLinuxDEB64_5.12.5

#### Configuring

The default configuration builds the project in `fw/project/default`, which is
set in `fw/config.cmake`. The board selection for the project is defined
in `fw/project/default/config.cmake`, which is `cloudchaser` by default.

If any `config.cmake` parameters are changed, the `build` directory
below needs to be wiped and `cmake` re-run.

#### Build

From the top level folder:

    sky:cloudchaser$ mkdir build && cd $_
    sky:build$ cmake ..
    sky:build$ make fw

After this, the target `.elf` file is `fw/fw.elf`.

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
don't know gdb very deeply, you might like Segger's J-Link Debugger.
It has a graphical interface and is easy to use.

It can be found here: https://download.segger.com/J-Link/J-LinkDebugger/jlinkdebugger_2.14.8_x86_64.deb

If you want to get going with it fast, point it to the settings file
located at `etc/fw.jdebug`.
