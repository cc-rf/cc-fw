#!/bin/bash -e
d=$(dirname $(readlink -f "$0"))
cd $d/build
gdb-multiarch -x $d/target$1.gdb -x $d/debug.gdb
