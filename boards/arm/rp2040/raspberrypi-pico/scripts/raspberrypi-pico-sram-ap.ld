/****************************************************************************
 * boards/arm/rp2040/raspberrypi-pico/scripts/raspberrypi-pico-sram-ap.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#include <nuttx/config.h>

/* AP linker script: SRAM base at 0x20007800 (bootloader occupies 0x20000000-0x200077FF)
 * Available: 0x20042000 - 0x20007800 = 0x3A800 = 234KB
 *
 * IMPORTANT: ORIGIN must match CONFIG_BOOT_RP2040BOOT_AP_BASE in Kconfig.
 * If either value changes, the other must be updated to match.
 *
 * This is a pure SRAM image loaded by the bootloader.  No flash MEMORY
 * region or .flash_section -- the AP does not own any flash.
 */

#ifdef CONFIG_RP2040_AP_PIE
/* PIE layout: VMA starts at 0 (position-independent), LMA starts at
 * sram_base.  Every output section uses explicit AT(ADDR(.section) + sram_base)
 * so the VMA-to-LMA delta is exactly sram_base for every section -- no
 * drift between two independent MEMORY-region location counters (which
 * is what a plain "AT> sram" second-region approach would be prone to).
 */
sram_base = 0x20007800;
MEMORY
{
  image (rwx) : ORIGIN = 0, LENGTH = 234K
}
#  define AP_REGION      > image
#  define AP_AT(name)    AT(ADDR(name) + sram_base)
#  define AP_TEXT        : text
#  define AP_DATA        : data
#  define AP_DYNAMIC     : data : dynamic
#else
MEMORY
{
  sram (rwx) : ORIGIN = 0x20007800, LENGTH = 234K
}
#  define AP_REGION      > sram
#  define AP_AT(name)
#  define AP_TEXT
#  define AP_DATA
#  define AP_DYNAMIC
#endif

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(__start)

#ifdef CONFIG_RP2040_AP_PIE
PHDRS
{
  text    PT_LOAD FLAGS(5);
  data    PT_LOAD FLAGS(6);
  dynamic PT_DYNAMIC FLAGS(6);
}
#endif

SECTIONS
{
#ifdef CONFIG_RP2040_AP_PIE
    .dynsym : AP_AT(.dynsym) {
        *(.dynsym)
    } AP_REGION AP_TEXT

    .dynstr : AP_AT(.dynstr) {
        *(.dynstr)
    } AP_REGION AP_TEXT

    .hash : AP_AT(.hash) {
        *(.hash)
    } AP_REGION AP_TEXT

    .rel.dyn : AP_AT(.rel.dyn) {
        *(.rel.*)
    } AP_REGION AP_TEXT
#endif

    .text : AP_AT(.text) {
        _stext = ABSOLUTE(.);
        *(.vectors)
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } AP_REGION AP_TEXT

    /* Vector table must be 256-byte aligned for Cortex-M VTOR.
     * This catches CONFIG_BOOT_RP2040BOOT_AP_BASE being set to
     * a non-aligned value.
     */

    ASSERT((_vectors & 0xff) == 0, "vector table must be 256-byte aligned (check AP_BASE)")

    .init_section : AP_AT(.init_section) {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } AP_REGION AP_TEXT

    .ARM.extab : AP_AT(.ARM.extab) {
        *(.ARM.extab*)
    } AP_REGION AP_TEXT

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx : AP_AT(.ARM.exidx) {
        *(.ARM.exidx*)
    } AP_REGION AP_TEXT
    __exidx_end = ABSOLUTE(.);

    . = ALIGN(0x100);

    .data : AP_AT(.data) {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        *(.ram_code.*)
        *(.got.plt)
        *(.ram_vectors)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } AP_REGION AP_DATA

#ifdef CONFIG_RP2040_AP_PIE
    .dynamic : AP_AT(.dynamic) {
        *(.dynamic)
    } AP_REGION AP_DYNAMIC
#endif

    .bss : AP_AT(.bss) {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } AP_REGION AP_DATA

    /* Stabs debugging sections. */
    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}
