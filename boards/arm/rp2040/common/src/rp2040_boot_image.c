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

#include <nuttx/debug.h>
#include <stdio.h>
#include <fcntl.h>

#include <sys/boardctl.h>
#include <nuttx/irq.h>
#include <arch/barriers.h>

#ifdef CONFIG_LIBC_ELF_LOADTO_LMA
#  include <errno.h>
#  include <elf.h>
#  include <nuttx/lib/elf.h>
#endif

#include "nvic.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIBC_ELF_LOADTO_LMA
/* SRAM window an AP ELF may occupy.  The lower bound is the bootloader's own
 * CONFIG_RAM_END (its .data/.bss/stack/heap stop there), the upper bound is
 * the top of the RP2040's 264 KiB SRAM.
 */

#  define RP2040_SRAM_TOP  (0x20000000 + (264 * 1024))
#endif

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

#ifdef CONFIG_LIBC_ELF_LOADTO_LMA
/****************************************************************************
 * Name:  boot_elf_validate
 *
 * Description:
 *   Pre-scan the program headers and confirm every PT_LOAD segment lands in
 *   the SRAM window reserved for the image, above the bootloader's own
 *   CONFIG_RAM_END.  LOADTO_LMA makes libelf write to p_paddr without any
 *   range check of its own, so this guard must run before libelf_load().
 *   The lowest PT_LOAD p_paddr (the AP vector table address) is returned in
 *   vectors.
 *
 ****************************************************************************/

static int boot_elf_validate(FAR struct mod_loadinfo_s *loadinfo,
                             FAR uintptr_t *vectors)
{
  uintptr_t lowest = (uintptr_t)-1;
  Elf_Phdr phdr;
  int ret;
  int i;

  for (i = 0; i < loadinfo->ehdr.e_phnum; i++)
    {
      ret = libelf_read(loadinfo, (FAR uint8_t *)&phdr, sizeof(phdr),
                        loadinfo->ehdr.e_phoff + i * sizeof(phdr));
      if (ret < 0)
        {
          return ret;
        }

      if (phdr.p_type != PT_LOAD)
        {
          continue;
        }

      if (phdr.p_paddr + phdr.p_memsz < phdr.p_paddr ||
          phdr.p_filesz > phdr.p_memsz)
        {
          return -EINVAL;
        }

      if (phdr.p_paddr < CONFIG_RAM_END ||
          phdr.p_paddr + phdr.p_memsz > RP2040_SRAM_TOP)
        {
          return -EINVAL;
        }

      if (phdr.p_paddr < lowest)
        {
          lowest = phdr.p_paddr;
        }
    }

  if (lowest == (uintptr_t)-1)
    {
      return -EINVAL;
    }

  *vectors = lowest;
  return 0;
}

/****************************************************************************
 * Name:  boot_elf_image
 *
 * Description:
 *   Load a fully linked, fixed-address ET_EXEC into SRAM with libelf
 *   (CONFIG_LIBC_ELF_LOADTO_LMA reads each segment straight to its link
 *   address, so no relocation is needed) and jump to it.  The vector table
 *   sits at the lowest loaded address; vector 0 is the initial MSP and
 *   vector 1 is the reset handler.  Interrupts are masked before the jump so
 *   a pending IRQ cannot fire against the outgoing vector table.  Returns a
 *   negative errno if the image is missing or invalid; does not return on
 *   success.
 *
 ****************************************************************************/

static int boot_elf_image(FAR const char *path)
{
  struct mod_loadinfo_s loadinfo;
  uintptr_t vectors = 0;
  uint32_t msp;
  uint32_t reset;
  int ret;

  ret = libelf_initialize(path, &loadinfo);
  if (ret < 0)
    {
      syslog(LOG_ERR, "boot: elf initialize %s failed: %d\n", path, ret);
      return ret;
    }

  if (loadinfo.ehdr.e_type != ET_EXEC)
    {
      libelf_uninitialize(&loadinfo);
      return -ENOEXEC;
    }

  ret = boot_elf_validate(&loadinfo, &vectors);
  if (ret < 0)
    {
      syslog(LOG_ERR, "boot: elf phdr check failed: %d\n", ret);
      libelf_uninitialize(&loadinfo);
      return ret;
    }

  ret = libelf_load(&loadinfo);
  if (ret < 0)
    {
      syslog(LOG_ERR, "boot: elf load %s failed: %d\n", path, ret);
      libelf_uninitialize(&loadinfo);
      return ret;
    }

  libelf_uninitialize(&loadinfo);

  msp   = ((FAR const uint32_t *)vectors)[0];
  reset = ((FAR const uint32_t *)vectors)[1];

  systick_disable();
  cleanup_arm_nvic();

  __asm__ __volatile__(
                       "\tmsr msp, %0\n"
                       "\tmsr control, %1\n"
                       "\tisb\n"
                       "\tmov pc, %2\n"
                       :
                       : "r" (msp), "r" (0), "r" (reset));

  return 0;
}
#endif /* CONFIG_LIBC_ELF_LOADTO_LMA */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_boot_image
 *
 * Description:
 *   This entry point is called by bootloader to jump to application image.
 *
 ****************************************************************************/

int board_boot_image(const char *path, uint32_t hdr_size)
{
  static struct arm_vector_table vt;
  struct file file;
  ssize_t bytes;
  int ret;

#ifdef CONFIG_LIBC_ELF_LOADTO_LMA
  /* Try to boot the image as a fully linked ET_EXEC loaded into SRAM.  If it
   * is not a valid ELF (or is missing), fall back to the raw in-place jump
   * below so callers that pass a plain binary still work.
   */

  ret = boot_elf_image(path);
  if (ret >= 0)
    {
      return ret;
    }
#endif

  ret = file_open(&file, path, O_RDONLY | O_CLOEXEC);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to open %s with: %d", path, ret);
      return ret;
    }

  bytes = file_pread(&file, &vt, sizeof(vt), hdr_size);
  if (bytes != sizeof(vt))
    {
      syslog(LOG_ERR, "Failed to read ARM vector table: %d", bytes);
      return bytes < 0 ? bytes : -1;
    }

  systick_disable();

  cleanup_arm_nvic();

  /* Set main and process stack pointers */

  __asm__ __volatile__(
                       "\tmsr msp, %0\n"
                       "\tmsr control, %1\n"
                       "\tisb\n"
                       "\tmov pc, %2\n"
                       :
                       : "r" (vt.spr), "r" (0), "r" (vt.reset));

  return 0;
}
