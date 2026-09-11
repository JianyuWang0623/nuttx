/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_boot_elf.c
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

#include <nuttx/irq.h>
#include <arch/barriers.h>

#include "nvic.h"
#include "arm_internal.h"
#include "hardware/rp2040_resets.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void rp2040_boot_cleanup(void)
{
  up_irq_disable();
  putreg32(0xffffffff, ARMV6M_NVIC_ICER);
  putreg32(0xffffffff, ARMV6M_NVIC_ICPR);
  putreg32(0, ARMV6M_SYSTICK_CSR);
  putreg32(SYSTICK_RVR_MASK, ARMV6M_SYSTICK_RVR);
  putreg32(0, ARMV6M_SYSTICK_CVR);

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
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_boot_jump
 *
 * Description:
 *   Disable interrupts and systick, then jump to the given vector table.
 *   Does NOT set VTOR -- the AP's __start() -> rp2040_irq_initialize()
 *   does it.  Does NOT add Thumb+1 -- vector table addresses already have
 *   bit0 set.
 *
 * Input Parameters:
 *   msp   - Main stack pointer value (from vector table entry 0)
 *   reset - Reset handler address (from vector table entry 1)
 *
 ****************************************************************************/

void rp2040_boot_jump(uint32_t msp, uint32_t reset)
{
  rp2040_boot_cleanup();

  __asm__ __volatile__(
                       "\tmsr msp, %0\n"
                       "\tmsr control, %1\n"
                       "\tisb\n"
                       "\tbx %2\n"
                       :
                       : "r" (msp), "r" (0), "r" (reset));
}
