#!/bin/bash -e
JLinkGDBServer -if swd -device MK66FN2M0VMD18 -endian little \
    -port 2331 -swoport 2332 -telnetport 2333 \
    "$@"
