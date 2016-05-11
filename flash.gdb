target remote :2331
monitor speed 1000
monitor clrbp
monitor reset
monitor halt
flushreg
monitor speed auto
symbol-file fw/fw.elf
load fw/fw.elf
monitor clrbp
monitor reset
disconnect
quit
