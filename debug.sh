#!/bin/bash -e
d=$(dirname $(readlink -f "$0"))
cd $d/build
arm-none-eabi-gdb -x $d/target$1.gdb -x $d/debug.gdb
