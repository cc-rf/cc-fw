target remote :2331
monitor speed 1000
#monitor reset
#monitor halt
#flushreg
monitor speed auto
monitor flash breakpoints 1

monitor semihosting enable
monitor semihosting breakOnError 1
monitor semihosting IOClient 3
# monitor semihosting setargs "<argv>" # (in case SYS_GET_CMDLINE command is used)
monitor SWO DisableTarget 0xFFFFFFFF
monitor SWO EnableTarget 0 0 0xFFFFFFFF 0
set mem inaccessible-by-default off

symbol-file fw/fw.elf
#monitor reset
monitor halt
monitor regs
#flushreg
#continue
