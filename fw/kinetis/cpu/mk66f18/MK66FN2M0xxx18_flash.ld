/*
** ###################################################################
**     Processors:          MK66FN2M0VLQ18
**                          MK66FN2M0VMD18
**
**     Compiler:            GNU C Compiler
**     Reference manual:    K66P144M180SF5RMV2, Rev. 1, Mar 2015
**     Version:             rev. 3.0, 2015-03-25
**     Build:               b170214
**
**     Abstract:
**         Linker file for the GNU C Compiler
**
**     Copyright 2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2017 NXP
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of the copyright holder nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
** ###################################################################
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Phillip: Increasing defaults (chip has 256K SRAM, 2M Flash). Old values are both 0x0400 (1024).
*/
HEAP_SIZE  = DEFINED(__heap_size__)  ? __heap_size__  : 0x00002000; /* 8 Kb */
STACK_SIZE = DEFINED(__stack_size__) ? __stack_size__ : 0x00000800; /* 2 Kb */

M_VECTOR_RAM_SIZE = 0x0400;

/* Specify the memory areas */
MEMORY
{
  m_interrupts          (RX)  : ORIGIN = 0x00000000, LENGTH = 0x00000400
  m_flash_config        (RX)  : ORIGIN = 0x00000400, LENGTH = 0x00000010
  m_fast_text           (RX)  : ORIGIN = 0x00000410, LENGTH = 0x0000BC00
  m_text                (RX)  : ORIGIN = 0x0000C010, LENGTH = 0x001F2FF0
  m_user                (RW)  : ORIGIN = 0x001FF000, LENGTH = 0x00001000
  m_interrupts_ram      (RW)  : ORIGIN = 0x1FFF0000, LENGTH = 0x00000400
  m_fast_data           (RW)  : ORIGIN = 0x1FFF0400, LENGTH = 0x00004000
  m_fast_code           (RX)  : ORIGIN = 0x1FFF4400, LENGTH = 0x0000BC00
  m_data                (RW)  : ORIGIN = 0x20000000, LENGTH = 0x00030000
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into internal flash */
  .interrupts :
  {
    __VECTOR_TABLE = .;
    . = ALIGN(4);
    KEEP(*(.isr_vector))     /* Startup code */
    . = ALIGN(4);
  } > m_interrupts

  .flash_config :
  {
    . = ALIGN(4);
    KEEP(*(.FlashConfig))    /* Flash Configuration Field (FCF) */
    . = ALIGN(4);
  } > m_flash_config

  /* The program code and other data goes into internal flash */
  .text :
  {
    . = ALIGN(4);
    *(.text)                 /* .text sections (code) */
    *(.text*)                /* .text* sections (code) */
    *(.rodata)               /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)              /* .rodata* sections (constants, strings, etc.) */
    *(.glue_7)               /* glue arm to thumb code */
    *(.glue_7t)              /* glue thumb to arm code */
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
  } > m_text

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > m_text

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > m_text

 .ctors :
  {
    __CTOR_LIST__ = .;
    /* gcc uses crtbegin.o to find the start of
       the constructors, so we make sure it is
       first.  Because this is a wildcard, it
       doesn't matter if the user does not
       actually link against crtbegin.o; the
       linker won't look for a file to match a
       wildcard.  The wildcard also means that it
       doesn't matter which directory crtbegin.o
       is in.  */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*crtbegin?.o(.ctors))
    /* We don't want to include the .ctor section from
       from the crtend.o file until after the sorted ctors.
       The .ctor section from the crtend file contains the
       end of ctors marker and it must be last */
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    __CTOR_END__ = .;
  } > m_text

  .dtors :
  {
    __DTOR_LIST__ = .;
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    __DTOR_END__ = .;
  } > m_text

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } > m_text

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } > m_text

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } > m_text

  __etext = .;    /* define a global symbol at end of code */
  __DATA_ROM = .; /* Symbol is used by startup for data initialization */

  .data : AT(__DATA_ROM)
  {
    . = ALIGN(4);
    __DATA_RAM = .;
    __data_start__ = .;      /* create a global symbol at data start */
    *(.data)                 /* .data sections */
    *(.data*)                /* .data* sections */
    KEEP(*(.jcr*))
    . = ALIGN(4);
    __data_end__ = .;        /* define a global symbol at data end */
  } > m_data

  __DATA_END = __DATA_ROM + (__data_end__ - __data_start__);
  text_end = ORIGIN(m_text) + LENGTH(m_text);
  ASSERT(__DATA_END <= text_end, "region m_text overflowed with text and data")

  __fast_text_begin = ORIGIN(m_fast_text);

  .fast_text (NOLOAD) : AT(__fast_text_begin)
  {
    . = ALIGN(4);
    __fast_code_begin = .;
    *(.fast_text)
    *(.fast_text*)
    /* this is big: *\kinetis_freertos.a:* */
    . = ALIGN(4);
    __fast_code_end = .;
  } > m_fast_code

  __fast_text_size = SIZEOF(.fast_text);
  __fast_text_end = __fast_text_begin + __fast_text_size;

  .user (NOLOAD) : AT(ORIGIN(m_user))
  {
    . = ALIGN(4);
    *(.user)
    . = ALIGN(4);
  } > m_user

  .interrupts_ram (NOLOAD) :
  {
    . = ALIGN(4);
    __VECTOR_RAM__ = .;
    __interrupts_ram_start__ = .; /* Create a global symbol at data start */
    . += M_VECTOR_RAM_SIZE;
    . = ALIGN(4);
    __interrupts_ram_end__ = .; /* Define a global symbol at data end */
  } > m_interrupts_ram

  __VECTOR_RAM = __VECTOR_RAM__;
  __RAM_VECTOR_TABLE_SIZE_BYTES = __interrupts_ram_end__ - __interrupts_ram_start__;

    .fast_data (NOLOAD) : AT(ORIGIN(m_fast_data))
    {
      . = ALIGN(4);
      *(.fast_data)
      . = ALIGN(4);
    } > m_fast_data

  /* Uninitialized data section */
  .bss (NOLOAD) :
  {
    /* This is used by the startup in order to initialize the .bss section */
    . = ALIGN(4);
    __START_BSS = .;
    __bss_start__ = .;
    *(.bss)
    *(.bss*)
    . = ALIGN(512);
    *(m_usb_bdt)
    *(m_usb_global)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
    __END_BSS = .;
  } > m_data

  .band (NOLOAD) :
  {
    . = ALIGN(8);
    *(.band)
  } > m_data

  .heap (NOLOAD) :
  {
    . = ALIGN(8);
    __end__ = .;
    PROVIDE(end = .);
    __HeapBase = .;
    . += HEAP_SIZE;
    __HeapLimit = .;
    __heap_limit = .; /* Add for _sbrk */
    *(.heap)
    __heap_limit_rtos = .;
  } > m_data

  .stack (NOLOAD) :
  {
    . = ALIGN(8);
    . += STACK_SIZE;
  } > m_data

  /* Initializes stack on the end of block */
  __StackTop   = ORIGIN(m_data) + LENGTH(m_data);
  __StackLimit = __StackTop - STACK_SIZE;
  PROVIDE(__stack = __StackTop);

  .ARM.attributes 0 : { *(.ARM.attributes) }

  ASSERT(__StackLimit >= __HeapLimit, "region m_data overflowed with stack and heap")
}
