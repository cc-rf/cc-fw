#!/bin/bash -e
d=$(dirname $(readlink -f "$0"))
"${d}/jlink.sh" -select usb=203200983 -port 2331 -swoport 2332 -telnetport 2333
