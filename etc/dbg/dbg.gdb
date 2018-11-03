
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

monitor speed auto

monitor halt

symbol-file fw.elf
load fw.elf

monitor SWO DisableTarget 0xFFFFFFFF
monitor SWO EnableTarget 0 1200000 0xFFFFFFFF 0

monitor clrbp
monitor reset
#monitor halt

flushreg
continue
