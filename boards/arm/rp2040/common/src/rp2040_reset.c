/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_reset.c
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
#include <nuttx/board.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <sys/boardctl.h>

#include "arm_internal.h"
#include "rp2040_rom.h"
#include "hardware/rp2040_watchdog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Persist the "enter bootloader" request across a soft reset in a watchdog
 * SCRATCH register.  Verified on-target (PER-167 W1): the watchdog SCRATCH
 * registers survive an armv6-m AIRCR SYSRESETREQ (up_systemreset), and
 * SCRATCH2/3 do not collide with the bootrom watchdog-reboot region
 * (SCRATCH4..7).  The reset cause is cleared on read so the bootloader only
 * stays resident for the boot that was explicitly requested.
 */

#define RP2040_BOOTMAGIC_REG    RP2040_WATCHDOG_SCRATCH2
#define RP2040_BOOTMAGIC_VALUE  0xb00710adu   /* "boot load" */

#if defined(CONFIG_BOARDCTL_RESET) || defined(CONFIG_BOARDCTL_RESET_CAUSE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Reset board.  Support for this function is required by board-level
 *   logic if CONFIG_BOARDCTL_RESET is selected.
 *
 * Input Parameters:
 *   status - Status information provided with the reset event.  This
 *            meaning of this status information is board-specific.  If not
 *            used by a board, the value zero may be provided in calls to
 *            board_reset().
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_RESET
int board_reset(int status)
{
  if (status == BOARDIOC_SOFTRESETCAUSE_ENTER_BOOTLOADER)
    {
      /* Ask the first-stage bootloader (firmware 1) to stay resident in
       * fastboot after the reset.  Persist the request in a watchdog SCRATCH
       * register and do a normal soft reset; the bootloader reads and clears
       * it via board_reset_cause().  This intentionally does NOT drop into
       * the ROM BOOTSEL: BOOTSEL is only the recovery path below.
       */

      putreg32(RP2040_BOOTMAGIC_VALUE, RP2040_BOOTMAGIC_REG);
      up_systemreset();
    }
  else if (status == BOARDIOC_SOFTRESETCAUSE_ENTER_RECOVERY)
    {
      /* Recovery == last-resort brick-recovery path: drop into the RP2040
       * ROM USB bootloader (BOOTSEL, VID:PID 2e8a:0003) so a uf2 can always
       * be flashed even if both flash images are broken.  Reachable from the
       * host via "fastboot oem ..." mapped to ENTER_RECOVERY, or nsh
       * "reboot recovery".
       */

      rom_reset_usb_boot_fn reset_usb_boot;

      reset_usb_boot = (rom_reset_usb_boot_fn)ROM_LOOKUP(ROM_RESET_USB_BOOT);
      reset_usb_boot(0, 0);
    }
  else
    {
      up_systemreset();
    }

  return 0;
}
#endif /* CONFIG_BOARDCTL_RESET */

#ifdef CONFIG_BOARDCTL_RESET_CAUSE

/****************************************************************************
 * Name: board_reset_cause
 *
 * Description:
 *   Return the cause of the last reset.  On RP2040 the only persisted
 *   soft-reset subreason we track is ENTER_BOOTLOADER, stored as a magic in
 *   a watchdog SCRATCH register by board_reset().  The magic is cleared on
 *   read so the request is one-shot: the bootloader stays resident only for
 *   the boot that was explicitly requested, never permanently.
 *
 ****************************************************************************/

int board_reset_cause(FAR struct boardioc_reset_cause_s *cause)
{
  uint32_t magic = getreg32(RP2040_BOOTMAGIC_REG);

  if (magic == RP2040_BOOTMAGIC_VALUE)
    {
      /* Clear on read so we do not stay in the bootloader forever. */

      putreg32(0, RP2040_BOOTMAGIC_REG);

      cause->cause = BOARDIOC_RESETCAUSE_CORE_SOFT;
      cause->flag  = BOARDIOC_SOFTRESETCAUSE_ENTER_BOOTLOADER;
    }
  else
    {
      cause->cause = BOARDIOC_RESETCAUSE_CORE_SOFT;
      cause->flag  = BOARDIOC_SOFTRESETCAUSE_USER_REBOOT;
    }

  return 0;
}

#endif /* CONFIG_BOARDCTL_RESET_CAUSE */

#endif /* CONFIG_BOARDCTL_RESET || CONFIG_BOARDCTL_RESET_CAUSE */
