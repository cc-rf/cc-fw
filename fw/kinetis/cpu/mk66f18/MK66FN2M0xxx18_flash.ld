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

HEAP_SIZE  = 0x00001000; /* 4KB   */
STACK_SIZE = 0x00000500; /* 1280B */

M_VECTOR_RAM_SIZE = 0x0400;

/* Specify the memory areas */
MEMORY
{
  m_interrupts          (RX)  : ORIGIN = 0x00000000, LENGTH = 0x00000400
  m_flash_config        (RX)  : ORIGIN = 0x00000400, LENGTH = 0x00000010
  m_user                (RW)  : ORIGIN = 0x0001F000, LENGTH = 0x00001000
  m_user_init           (RW)  : ORIGIN = 0x00020000, LENGTH = 0x00001000
  m_fast_text           (RX)  : ORIGIN = 0x00021000, LENGTH = 0x0000BC00
  m_text                (RX)  : ORIGIN = 0x0002CC00, LENGTH = 0x00040000
  m_interrupts_ram      (RW)  : ORIGIN = 0x1FFF0000, LENGTH = 0x00000400
  m_fast_data           (RW)  : ORIGIN = 0x1FFF0400, LENGTH = 0x00004000
  m_fast_code           (RX)  : ORIGIN = 0x1FFF4400, LENGTH = 0x0000BC00
  m_user_ram            (RW)  : ORIGIN = 0x20000000, LENGTH = 0x00001000
  m_data                (RW)  : ORIGIN = 0x20001000, LENGTH = 0x0002F000
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

  __flash_header_begin = ORIGIN(m_interrupts);
  __flash_header_end = __flash_header_begin + LENGTH(m_interrupts) + LENGTH(m_flash_config);

  __user_flash_init_base = ORIGIN(m_user_init);
  __user_flash_base = ORIGIN(m_user);

  .user_rom : AT(__user_flash_init_base)
  {
    __user_ram_base = .;
    *(.user.base)
    *(.user)
    *(.user*)
    . = ALIGN(16);
    __user_ram_end = .;
  } > m_user_ram

  __user_flash_size = __user_ram_end - __user_ram_base;
  __user_flash_init_end = __user_flash_init_base + __user_flash_size;
  __user_flash_end = __user_flash_base + __user_flash_size;

  __fast_text_begin = ORIGIN(m_fast_text);

  .fast_code : AT(__fast_text_begin)
  {
    __fast_code_begin = .;
    *(.fast_text)
    *(.fast_text*)
    *freertos.a:*(.text)
    *freertos.a:*(.text.*)
    *gcc.a:*(.text)
    *gcc.a:*(.text.*)
    /* If doing this one, need to copy to ram earlier.
    *nano.a:*(.text) /* adds 10K */
    *nano.a:*(.text.*)
    */
    . = ALIGN(16);
    __fast_code_end = .;
  } > m_fast_code

  __fast_text_end = __fast_text_begin + (__fast_code_end - __fast_code_begin);

  /* __fast_data_rom_begin = ORIGIN(m_fast_data_rom); */

  .fast_data (NOLOAD) : AT(ORIGIN(m_fast_data))
  {
    . = ALIGN(4);
    __fast_data_begin = .;
    *(.fast_data)
    *(.fast_data*)
    . = ALIGN(4);
    __fast_data_end = .;

  } > m_fast_data

  __fast_data_size = __fast_data_end - __fast_data_begin;
  /*__fast_data_rom_end = .;
  __fast_data_size = __fast_data_rom_end - __fast_data_rom_begin;
  */


  /* The program code and other data goes into internal flash */
  .text : AT(ORIGIN(m_text))
  {
    . = ALIGN(16);
    __text_begin = .;
    *(.text)                 /* .text sections (code) */
    *(.text*)                /* .text* sections (code) */
    *(.rodata)               /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)              /* .rodata* sections (constants, strings, etc.) */
    *(.glue_7)               /* glue arm to thumb code */
    *(.glue_7t)              /* glue thumb to arm code */
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(16);
    __text_end = .;
  } > m_text

    .ARM :
    {
      __exidx_start = .;
      *(.ARM.exidx*)
      __exidx_end = .;
    } > m_text


    . = ALIGN(16);
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
    . = ALIGN(16);
    __data_end__ = .;        /* define a global symbol at data end */
  } > m_data

  __DATA_END = __DATA_ROM + (__data_end__ - __data_start__);

  m_text_end = ORIGIN(m_text) + LENGTH(m_text);
  ASSERT(__DATA_END <= m_text_end, "region m_text overflowed with text and data")

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

