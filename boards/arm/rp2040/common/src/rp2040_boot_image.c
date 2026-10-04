/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_boot_image.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <nuttx/debug.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <inttypes.h>

#include <sys/boardctl.h>
#include <nuttx/irq.h>
#include <arch/barriers.h>

#include "nvic.h"
#include "arm_internal.h"

#ifdef CONFIG_RP2040_BOOT_ELF
#include <nuttx/lib/elf.h>
#include "hardware/rp2040_resets.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RP2040 SRAM end address */

#define RP2040_SRAM_END  0x20042000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the first two entries on NVIC vector table */

struct arm_vector_table
{
  uint32_t spr;   /* Stack pointer on reset */
  uint32_t reset; /* Pointer to reset exception handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void cleanup_arm_nvic(void);
static void systick_disable(void);
static void rp2040_boot_jump(uint32_t msp, uint32_t reset) noreturn_function;

#ifdef CONFIG_RP2040_BOOT_ELF
static bool rp2040_boot_is_elf(const char *path, uint32_t hdr_size);
static int  verify_phdr(FAR Elf_Phdr *phdr);
static inline uintptr_t
rp2040_boot_elf_addr(FAR struct mod_loadinfo_s *loadinfo, uintptr_t vaddr);
static int rp2040_boot_elf_image(const char *path, uint32_t hdr_size);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  cleanup_arm_nvic
 *
 * Description:
 *   Acknowledge and disable all interrupts in NVIC
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

static void cleanup_arm_nvic(void)
{
  /* Allow any pending interrupts to be recognized */

  UP_ISB();
  up_irq_disable();

  /* Disable all interrupts */

  putreg32(0xffffffff, ARMV6M_NVIC_ICER);

  /* Clear all pending interrupts */

  putreg32(0xffffffff, ARMV6M_NVIC_ICPR);
}

/****************************************************************************
 * Name:  systick_disable
 *
 * Description:
 *   Disable the SysTick system timer
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

static void systick_disable(void)
{
  putreg32(0, ARMV6M_SYSTICK_CSR);
  putreg32(SYSTICK_RVR_MASK, ARMV6M_SYSTICK_RVR);
  putreg32(0, ARMV6M_SYSTICK_CVR);
}

/****************************************************************************
 * Name: rp2040_boot_jump
 *
 * Description:
 *   Disable interrupts/systick, reset the USB controller so the AP image
 *   starts from a clean USB peripheral state, set MSP, then branch to the
 *   reset handler.  Does NOT set VTOR -- the AP's __start() ->
 *   rp2040_irq_initialize() does it.  Does NOT add the Thumb+1 bit --
 *   vector table entries already have bit0 set, and bx honors it.
 *
 * Input Parameters:
 *   msp   - Main stack pointer value (from vector table entry 0)
 *   reset - Reset handler address (from vector table entry 1)
 *
 ****************************************************************************/

static void rp2040_boot_jump(uint32_t msp, uint32_t reset)
{
  systick_disable();
  cleanup_arm_nvic();

#ifdef CONFIG_RP2040_BOOT_ELF
  /* Reset USB controller so AP starts with clean USB peripheral state.
   * Assert reset, deassert, then wait for RESET_DONE.
   */

  modifyreg32(RP2040_RESETS_RESET, 0, RP2040_RESETS_RESET_USBCTRL);
  modifyreg32(RP2040_RESETS_RESET, RP2040_RESETS_RESET_USBCTRL, 0);
  while (!(getreg32(RP2040_RESETS_RESET_DONE) &
           RP2040_RESETS_RESET_DONE_USBCTRL))
    {
    }

  UP_ISB();
#endif

  /* Set main and process stack pointers, then branch to reset handler.
   * bx (rather than mov pc,) honors the Thumb bit already set in the
   * vector table entry.
   */

  __asm__ __volatile__(
                       "\tmsr msp, %0\n"
                       "\tmsr control, %1\n"
                       "\tisb\n"
                       "\tbx %2\n"
                       :
                       : "r" (msp), "r" (0), "r" (reset));

  /* Should not reach here */

  for (; ; )
    {
    }
}

#ifdef CONFIG_RP2040_BOOT_ELF

/****************************************************************************
 * Name: rp2040_boot_is_elf
 *
 * Description:
 *   Peek at the ELF magic (e_ident[EI_MAG0..3]) at path+hdr_size to decide
 *   whether the image at that offset is an ELF file.  When it is not, the
 *   caller falls back to the raw vector-table jump so boards that still
 *   flash a plain binary continue to work unchanged.
 *
 ****************************************************************************/

static bool rp2040_boot_is_elf(const char *path, uint32_t hdr_size)
{
  struct file file;
  unsigned char mag[4];
  ssize_t bytes;
  int ret;

  ret = file_open(&file, path, O_RDONLY | O_CLOEXEC);
  if (ret < 0)
    {
      return false;
    }

  bytes = file_pread(&file, mag, sizeof(mag), hdr_size);
  file_close(&file);

  return bytes == sizeof(mag) &&
         mag[0] == 0x7f && mag[1] == 'E' && mag[2] == 'L' && mag[3] == 'F';
}

/****************************************************************************
 * Name: verify_phdr
 *
 * Description:
 *   Validate a single PT_LOAD program header before calling libelf_load().
 *   libelf internally does not perform these range checks.
 *
 * NOTE: This validation covers PT_LOAD segments only.  Sections with
 * SHF_ALLOC that fall outside any phdr range bypass these checks.
 * This is a sanity net, not a security boundary.  Only trusted ELF
 * images should be loaded.
 *
 ****************************************************************************/

static int verify_phdr(FAR Elf_Phdr *phdr)
{
  uintptr_t load_end;

  if (phdr->p_type != PT_LOAD)
    {
      return OK;
    }

  /* p_filesz must be <= p_memsz (check before memsz==0 skip) */

  if (phdr->p_filesz > phdr->p_memsz)
    {
      syslog(LOG_ERR, "p_filesz(0x%" PRIx32 ") > p_memsz(0x%" PRIx32 ")\n",
             (uint32_t)phdr->p_filesz, (uint32_t)phdr->p_memsz);
      return -EINVAL;
    }

  /* Skip segments with zero memory size (e.g. .flash_section placeholder) */

  if (phdr->p_memsz == 0)
    {
      return OK;
    }

  /* Check integer overflow on offset+filesz, vaddr+memsz, paddr+memsz */

  if (phdr->p_offset + phdr->p_filesz < phdr->p_offset)
    {
      syslog(LOG_ERR, "Integer overflow: p_offset=0x%" PRIx32
             " p_filesz=0x%" PRIx32 "\n",
             (uint32_t)phdr->p_offset, (uint32_t)phdr->p_filesz);
      return -EINVAL;
    }

  if (phdr->p_vaddr + phdr->p_memsz < phdr->p_vaddr)
    {
      syslog(LOG_ERR, "Integer overflow: p_vaddr=0x%" PRIx32
             " p_memsz=0x%" PRIx32 "\n",
             (uint32_t)phdr->p_vaddr, (uint32_t)phdr->p_memsz);
      return -EINVAL;
    }

  if (phdr->p_paddr + phdr->p_memsz < phdr->p_paddr)
    {
      syslog(LOG_ERR, "Integer overflow: p_paddr=0x%" PRIx32
             " p_memsz=0x%" PRIx32 "\n",
             (uint32_t)phdr->p_paddr, (uint32_t)phdr->p_memsz);
      return -EINVAL;
    }

  /* Check that the LMA (p_paddr) is within AP-owned SRAM.
   * AP memory starts at CONFIG_RP2040_AP_BASE; everything below that
   * belongs to the bootloader and must not be overwritten.
   */

  load_end = phdr->p_paddr + phdr->p_memsz;
  if (phdr->p_paddr < CONFIG_RP2040_AP_BASE || load_end > RP2040_SRAM_END)
    {
      syslog(LOG_ERR, "LMA range 0x%" PRIx32 "-0x%" PRIx32
             " outside AP SRAM [0x%" PRIx32 "-0x%" PRIx32 "]\n",
             (uint32_t)phdr->p_paddr, (uint32_t)load_end,
             (uint32_t)CONFIG_RP2040_AP_BASE, (uint32_t)RP2040_SRAM_END);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: rp2040_boot_elf_addr
 *
 * Description:
 *   Translate a link-time address in a loaded ELF module to the address it
 *   occupies now.  This mirrors the internal libelf_addr() helper (in
 *   libs/libc/elf/elf.h), which is not exported through the public
 *   <nuttx/lib/elf.h> header.  An address below the data segment's
 *   link-time base belongs to text, anything at or above it to data.
 *
 ****************************************************************************/

static inline uintptr_t
rp2040_boot_elf_addr(FAR struct mod_loadinfo_s *loadinfo, uintptr_t vaddr)
{
  if (loadinfo->datasec != 0 && vaddr >= loadinfo->datasec)
    {
      return loadinfo->datastart + (vaddr - loadinfo->datasec);
    }

  return loadinfo->textalloc + vaddr;
}

/****************************************************************************
 * Name: rp2040_boot_elf_image
 *
 * Description:
 *   Load, relocate and validate an ELF (ET_EXEC/ET_DYN) AP image found at
 *   path+hdr_size, then jump to it.  Returns a negated errno if the image
 *   could not be loaded or validated; does not return on success.
 *
 ****************************************************************************/

static int rp2040_boot_elf_image(const char *path, uint32_t hdr_size)
{
  struct mod_loadinfo_s loadinfo;
  struct module_s mod;
  FAR Elf_Phdr *phdr = NULL;
  size_t phdrsize;
  uintptr_t vt_addr;
  uint32_t msp;
  uint32_t reset;
  int ret;
  int i;

  ret = libelf_initialize(path, &loadinfo);
  if (ret < 0)
    {
      syslog(LOG_ERR, "libelf_initialize failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "ELF opened: filelen=%zu e_phnum=%u\n",
         (size_t)loadinfo.filelen, (unsigned)loadinfo.ehdr.e_phnum);

  if (loadinfo.ehdr.e_type != ET_EXEC && loadinfo.ehdr.e_type != ET_DYN)
    {
      syslog(LOG_ERR, "Not ET_EXEC/ET_DYN (type=%d)\n",
             loadinfo.ehdr.e_type);
      ret = -ENOEXEC;
      goto errout;
    }

  /* Read program headers and pre-validate PT_LOAD segments.  We use
   * libelf_read() to read phdr directly from the file rather than calling
   * libelf_loadhdrs(), which is an internal libelf API.  libelf_load()
   * will call libelf_loadhdrs() internally later.
   */

  if (loadinfo.ehdr.e_phnum == 0)
    {
      syslog(LOG_ERR, "No program headers\n");
      ret = -ENOEXEC;
      goto errout;
    }

  if (loadinfo.ehdr.e_phentsize != sizeof(Elf_Phdr))
    {
      syslog(LOG_ERR, "e_phentsize %u != sizeof(Elf_Phdr) %zu\n",
             (unsigned)loadinfo.ehdr.e_phentsize, sizeof(Elf_Phdr));
      ret = -ENOEXEC;
      goto errout;
    }

  phdrsize = (size_t)loadinfo.ehdr.e_phentsize *
             (size_t)loadinfo.ehdr.e_phnum;
  if (loadinfo.ehdr.e_phoff + phdrsize < loadinfo.ehdr.e_phoff ||
      loadinfo.ehdr.e_phoff + phdrsize > loadinfo.filelen)
    {
      syslog(LOG_ERR, "Program header table extends past EOF\n");
      ret = -ENOEXEC;
      goto errout;
    }

  phdr = malloc(phdrsize);
  if (phdr == NULL)
    {
      syslog(LOG_ERR, "Failed to allocate phdr (%zu bytes)\n", phdrsize);
      ret = -ENOMEM;
      goto errout;
    }

  ret = libelf_read(&loadinfo, (FAR uint8_t *)phdr, phdrsize,
                     loadinfo.ehdr.e_phoff);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to read program headers: %d\n", ret);
      goto errout;
    }

  for (i = 0; i < loadinfo.ehdr.e_phnum; i++)
    {
      ret = verify_phdr(&phdr[i]);
      if (ret < 0)
        {
          syslog(LOG_ERR, "phdr %d validation failed\n", i);
          goto errout;
        }
    }

  free(phdr);
  phdr = NULL;

  /* Load and relocate the ELF before reading its vector table. */

  ret = libelf_load(&loadinfo);
  if (ret < 0)
    {
      syslog(LOG_ERR, "libelf_load failed: %d\n", ret);
      goto errout;
    }

  syslog(LOG_INFO, "libelf_load OK\n");

  memset(&mod, 0, sizeof(mod));
  ret = libelf_bind(&mod, &loadinfo, NULL, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "libelf_bind failed: %d\n", ret);
      goto errout;
    }

  /* Read vector table from loaded image.  With PIE, the linker script
   * places dynamic metadata (.dynsym, .dynstr, .hash, .rel.dyn) before
   * .text in the RX PT_LOAD, so textalloc points to those sections, not
   * _vectors.  Look up the _vectors symbol and convert its link-time VMA
   * to the runtime LMA.
   */

  {
    Elf_Sym vsym;

    ret = libelf_findsymbol(&loadinfo, "_vectors", &vsym);
    if (ret < 0)
      {
        syslog(LOG_ERR, "Failed to find _vectors symbol: %d\n", ret);
        goto errout;
      }

    vt_addr = rp2040_boot_elf_addr(&loadinfo, vsym.st_value);
  }

  msp   = *(FAR uint32_t *)vt_addr;
  reset = *(FAR uint32_t *)(vt_addr + 4);

  syslog(LOG_INFO, "MSP=0x%" PRIx32 " Reset=0x%" PRIx32 "\n", msp, reset);

  /* Validate MSP: within AP SRAM and 4-byte aligned.  An unaligned MSP
   * would fault (or silently misalign the stack) the instant the AP's
   * reset handler pushes its first word.
   */

  if (msp < CONFIG_RP2040_AP_BASE || msp > RP2040_SRAM_END)
    {
      syslog(LOG_ERR, "MSP 0x%" PRIx32 " outside AP SRAM\n", msp);
      ret = -EINVAL;
      goto errout;
    }

  if ((msp & 0x3) != 0)
    {
      syslog(LOG_ERR, "MSP 0x%" PRIx32 " is not 4-byte aligned\n", msp);
      ret = -EINVAL;
      goto errout;
    }

  if ((reset & ~1u) < CONFIG_RP2040_AP_BASE ||
      (reset & ~1u) >= RP2040_SRAM_END ||
      (reset & 1u) == 0)
    {
      syslog(LOG_ERR, "Reset 0x%" PRIx32 " invalid\n", reset);
      ret = -EINVAL;
      goto errout;
    }

  libelf_uninitialize(&loadinfo);

  rp2040_boot_jump(msp, reset);

  /* Not reached */

  return 0;

errout:
  free(phdr);
  libelf_uninitialize(&loadinfo);
  return ret;
}

#endif /* CONFIG_RP2040_BOOT_ELF */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_boot_image
 *
 * Description:
 *   This entry point is called by bootloader to jump to application image.
 *
 *   When CONFIG_RP2040_BOOT_ELF is enabled and the image found at
 *   path+hdr_size is an ELF file, it is loaded via libelf (relocated,
 *   validated and bound) before jumping to its _vectors entry.  Otherwise
 *   hdr_size bytes are skipped and a raw two-word ARM vector table
 *   (initial MSP + reset handler) is read directly and jumped to, which
 *   keeps boards that still flash a plain binary working unchanged.
 *
 ****************************************************************************/

int board_boot_image(const char *path, uint32_t hdr_size)
{
  static struct arm_vector_table vt;
  struct file file;
  ssize_t bytes;
  int ret;

#ifdef CONFIG_RP2040_BOOT_ELF
  if (rp2040_boot_is_elf(path, hdr_size))
    {
      return rp2040_boot_elf_image(path, hdr_size);
    }
#endif

  ret = file_open(&file, path, O_RDONLY | O_CLOEXEC);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to open %s with: %d", path, ret);
      return ret;
    }

  bytes = file_pread(&file, &vt, sizeof(vt), hdr_size);
  file_close(&file);
  if (bytes != sizeof(vt))
    {
      syslog(LOG_ERR, "Failed to read ARM vector table: %d", bytes);
      return bytes < 0 ? bytes : -1;
    }

  /* Validate the non-ELF vector table before jumping:
   * - MSP must be inside SRAM (0x20000000..0x20042000)
   * - Reset must be odd (Thumb), non-erased, and inside flash or SRAM.
   */

  if (vt.spr < 0x20000000u || vt.spr > 0x20042000u)
    {
      syslog(LOG_ERR, "Non-ELF MSP 0x%08" PRIx32 " outside SRAM\n",
             vt.spr);
      return -ENOEXEC;
    }

  if ((vt.reset & 1u) == 0 ||
      vt.reset == 0xffffffffu ||
      !((vt.reset >= 0x10000001u && vt.reset <= 0x10200000u) ||
        (vt.reset >= 0x20000001u && vt.reset <= 0x20042000u)))
    {
      syslog(LOG_ERR, "Non-ELF Reset 0x%08" PRIx32 " invalid\n",
             vt.reset);
      return -ENOEXEC;
    }

  rp2040_boot_jump(vt.spr, vt.reset);

  /* Not reached */

  return 0;
}
