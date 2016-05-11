#!/bin/bash -e
d=$(dirname $(readlink -f "$0"))
cd $d/build

JLinkGDBServer -if swd -device MK66FX1M0VMD18 -endian little -singlerun &
jlink_pid=$!

function wait_jlink() {
    wait $jlink_pid 2>/dev/null
}

function kill_jlink() {
    kill $jlink_pid 2>/dev/null
}

trap wait_jlink EXIT
trap kill_jlink SIGINT

sleep 5
arm-none-eabi-gdb -x $d/flash.gdb
