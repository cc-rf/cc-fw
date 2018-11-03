#!/bin/bash -e
d=$(dirname $(readlink -f "$0"))
"${d}/jlink.sh" -select usb=174402262 -port 2341 -swoport 2342 -telnetport 2343
