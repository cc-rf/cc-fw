source /home/jq/evo/intel/pr/rf/cloudchaser/etc/gdb/freertos-helper.gdb

## https://mcuoneclipse.com/2015/07/05/debugging-arm-cortex-m-hard-faults-with-gdb-custom-command/
define armex
	if $lr & 0xd == 0xd
		printf "Uses PSP 0x%x return.\n", $psp
		set $armex_base = $psp
	else
		printf "Uses MSP 0x%x return.\n", $msp
		set $armex_base = $msp
	end

	printf "xPSR            0x%x\n", *($armex_base+28)
	printf "ReturnAddress   0x%x\n", *($armex_base+24)
	printf "LR (R14)        0x%x\n", *($armex_base+20)
	printf "R12             0x%x\n", *($armex_base+16)
	printf "R3              0x%x\n", *($armex_base+12)
	printf "R2              0x%x\n", *($armex_base+8)
	printf "R1              0x%x\n", *($armex_base+4)
	printf "R0              0x%x\n", *($armex_base)
	printf "Return instruction:\n"
	x/i *($armex_base+24)
end

document armex
ARMv7 Exception entry behavior.
xPSR, ReturnAddress, LR (R14), R12, R3, R2, R1, and R0
end
## --

monitor speed 1000
monitor clrbp
monitor reset
monitor halt
#monitor regs
flushreg
monitor speed auto
monitor flash breakpoints 1
symbol-file fw.elf
load fw.elf
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
