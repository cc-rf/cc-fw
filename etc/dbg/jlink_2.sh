#!/bin/bash -e
d=$(dirname $(readlink -f "$0"))
"${d}/jlink.sh" -select usb=203201171 -port 2351 -swoport 2352 -telnetport 2353
