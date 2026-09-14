/****************************************************************************
 * boards/arm/rp2040/waveshare-rp2040-lcd-1.28/scripts/waveshare-rp2040-lcd-1-28-ap-elf.ld
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

/* AP ELF image: a fully linked, fixed-address ET_EXEC that executes entirely
 * from SRAM.  It is stored (as an ELF file) in the "ap" partition and loaded
 * by the first-stage bootloader's elfboot chain-loader using libelf with
 * CONFIG_LIBC_ELF_LOADTO_LMA=y.
 *
 * Every section is linked into SRAM (VMA == LMA), so libelf reads each
 * PT_LOAD segment straight to its link address with no relocation.  The AP
 * starts above the bootloader's own SRAM footprint (.data/.bss/idle stack
 * plus heap): ORIGIN below must match the bootloader's CONFIG_RAM_END and the
 * elfboot CONFIG_ELFBOOT_LOAD_START / CONFIG_ELFBOOT_VECTORS_ADDR.
 *
 * The vector table is placed first so its address equals ORIGIN, giving the
 * chain-loader a fixed, link-time known jump target.
 *
 * CONFIG_RP2040_FLASH_BOOT must be n for this image: __start() must NOT copy
 * .data from flash (there is no valid LMA copy in flash); libelf already
 * placed .data at its SRAM link address.
 */

MEMORY
{
  flash (rx) : ORIGIN = 0x10000000, LENGTH = 2048K
  sram (rwx) : ORIGIN = 0x20008000, LENGTH = 232K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

SECTIONS
{
    .text : {
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
    } > sram

    .init_section : {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > sram

    .ARM.extab : {
        *(.ARM.extab*)
    } > sram

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx : {
        *(.ARM.exidx*)
    } > sram
    __exidx_end = ABSOLUTE(.);

    _eronly = ABSOLUTE(.);

    .ram_vectors (COPY) : {
        *(.ram_vectors)
    } > sram

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        *(.ram_code.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram

    .bss : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > sram

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

    /* The boot2 head object is still compiled by the common build, but an AP
     * ELF image is loaded and started by the first-stage bootloader (not the
     * ROM), so its boot2 stage is unused and intentionally dropped here.
     */

    /DISCARD/ : { *(.boot2) }
}
