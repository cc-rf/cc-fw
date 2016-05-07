target remote :2331
monitor speed 1000
monitor clrbp
monitor reset
monitor halt
#monitor regs
flushreg
monitor speed auto
monitor flash breakpoints 1
symbol-file fw/fw.elf
load fw/fw.elf
monitor clrbp
monitor reset
monitor halt

monitor semihosting disable
## new: page 437 of J-Link doc UM08001.pdf
#monitor semihosting breakOnError 1
##monitor semihosting IOClient 1
## from doc: 3=both GDB and telnet
#monitor semihosting IOClient 3
## from doc: passes args to main in fw?
## monitor semihosting setargs "<argv>" # (in case SYS_GET_CMDLINE command is used)

monitor SWO DisableTarget 0xFFFFFFFF
#monitor SWO EnableTarget 0 0 0x1 0
monitor SWO EnableTarget 0 1200000 0xFFFFFFFF 0
#monitor SWO EnableTarget 0 0 0xFFFFFFFF 0
#set mem inaccessible-by-default off

#monitor regs
flushreg
#break 568
continue
