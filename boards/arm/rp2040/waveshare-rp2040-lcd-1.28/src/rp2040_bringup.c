/****************************************************************************
 * boards/arm/rp2040/waveshare-rp2040-lcd-1.28/src/rp2040_bringup.c
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
#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/wqueue.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <sys/boardctl.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/partition.h>
#include <nuttx/mtd/mtd.h>

#include <arch/board/board.h>

#include "rp2040_pico.h"
#include "rp2040_flash_mtd.h"

#ifdef CONFIG_ARCH_BOARD_COMMON
#include "rp2040_common_bringup.h"
#endif /* CONFIG_ARCH_BOARD_COMMON */

#ifdef CONFIG_TXTABLE_PARTITION

#ifdef CONFIG_RP2040_BOOTLOADER_DEBUG_NODES
/* Placeholder partition table for a 2 MB flash (4 KB aligned).  Sizes
 * are placeholders to validate the txtable path; the final layout is fixed
 * later once the real firmware sizes (including the staging budget) are
 * measured.  Format per fs_txtable.c: first line magic "TXTABLE0", then
 * "<name> <size_hex_bytes> <offset_hex_bytes>" per line.
 */

static const char g_rp2040_default_txtable[] =
  "TXTABLE0\n"
  "bootloader 80000 0\n"
  "ap 80000 80000\n"
  "boot_staging 80000 100000\n";
#endif

/****************************************************************************
 * Name: rp2040_txtable_handler
 *
 * Description:
 *   Called by parse_mtd_partition() for each parsed partition.  Registers a
 *   sub-MTD as /dev/<name> so that fastboot ("flash <name>") and boot code
 *   can address it by name.
 *
 ****************************************************************************/

static void rp2040_txtable_handler(FAR struct partition_s *part,
                                   FAR void *arg)
{
  FAR struct mtd_dev_s *parent = (FAR struct mtd_dev_s *)arg;
  FAR struct mtd_dev_s *sub;
  char devname[NAME_MAX + 6];
  int ret;

  /* Do not expose the "bootloader" partition as a writable device.  This is
   * the first-stage bootloader's own flash region; it is updated only
   * through the ROM BOOTSEL path with a host tool (picotool), so there is no
   * legitimate run-time writer.  Filesystem permission bits are not enforced
   * in this configuration, so a "read-only" node would still be writable and
   * a stray "fastboot flash bootloader" would erase the running image.
   */

  if (strcmp(part->name, "bootloader") == 0)
    {
      return;
    }

  snprintf(devname, sizeof(devname), "/dev/%s", part->name);

  sub = mtd_partition(parent, part->firstblock, part->nblocks);
  if (sub == NULL)
    {
      return;
    }

  ret = register_mtddriver(devname, sub, 0644, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "rp2040: register %s failed: %d\n", devname, ret);
    }
}

/****************************************************************************
 * Name: rp2040_txtable_setup
 *
 * Description:
 *   Ensure a txtable exists in the last erase block of the full-chip MTD
 *   (write the built-in default on a blank flash), then parse it and expose
 *   the partitions.  On a real product the default comes from a ROMFS
 *   /etc/txtable.txt; the built-in default keeps the first-boot path
 *   self-contained for bring-up.
 *
 ****************************************************************************/

static void rp2040_txtable_setup(FAR struct mtd_dev_s *full)
{
  int ret;

  /* Let parse_mtd_partition() run the txtable parser, which tries the
   * on-flash primary table (last erase block) first and, when that is
   * missing or corrupt, falls back to the backup file
   * CONFIG_TXTABLE_DEFAULT_PARTITION_PATH (/etc/txtable.txt).  Do NOT
   * pre-check the flash block here and return early: that would bypass the
   * parser's built-in /etc fallback.
   */

  ret = parse_mtd_partition(full, rp2040_txtable_handler, full);
  if (ret < 0)
    {
      syslog(LOG_ERR, "rp2040: txtable parse failed: %d\n", ret);
    }
}

#ifdef CONFIG_RP2040_BOOTLOADER_DEBUG_NODES
/****************************************************************************
 * Name: rp2040_txtable_mktable
 *
 * Description:
 *   Write the built-in default txtable into the last erase block of the
 *   full-chip MTD.  This performs a flash erase+program which stops XIP and
 *   masks interrupts; on RP2040 that disturbs a live USB CDC link (the host
 *   re-enumerates), so it must be triggered explicitly and the console
 *   re-attached afterwards.  Kept separate from the read-only probe so the
 *   probe stays USB-safe.
 *
 ****************************************************************************/

static void rp2040_txtable_mktable(FAR struct mtd_dev_s *full)
{
  struct mtd_geometry_s geo;
  FAR uint8_t *page;
  off_t lasterase;
  off_t lastblk;
  int ret;

  ret = MTD_IOCTL(full, MTDIOC_GEOMETRY, (unsigned long)(uintptr_t)&geo);
  if (ret < 0)
    {
      syslog(LOG_ERR, "rp2040: geometry failed %d\n", ret);
      return;
    }

  lasterase = geo.neraseblocks - 1;
  lastblk   = lasterase * (geo.erasesize / geo.blocksize);

  page = kmm_zalloc(geo.blocksize);
  if (page == NULL)
    {
      syslog(LOG_ERR, "rp2040: alloc page failed\n");
      return;
    }

  memcpy(page, g_rp2040_default_txtable,
         sizeof(g_rp2040_default_txtable) - 1);

  syslog(LOG_INFO, "rp2040: writing default txtable to last erase block "
         "(USB will re-enumerate)\n");

  MTD_ERASE(full, lasterase, 1);
  MTD_BWRITE(full, lastblk, 1, page);
  kmm_free(page);

  syslog(LOG_INFO, "rp2040: default txtable written\n");
}

/****************************************************************************
 * Name: rp2040_txtable_erase
 *
 * Description:
 *   Erase the last erase block so the on-flash primary txtable becomes
 *   invalid.  Used to exercise the /etc/txtable.txt backup fallback: after
 *   this, a probe must fall back to the ROMFS backup table.
 *
 ****************************************************************************/

static void rp2040_txtable_erase(FAR struct mtd_dev_s *full)
{
  struct mtd_geometry_s geo;
  off_t lasterase;
  int ret;

  ret = MTD_IOCTL(full, MTDIOC_GEOMETRY, (unsigned long)(uintptr_t)&geo);
  if (ret < 0)
    {
      syslog(LOG_ERR, "rp2040: geometry failed %d\n", ret);
      return;
    }

  lasterase = geo.neraseblocks - 1;

  syslog(LOG_INFO, "rp2040: erasing last erase block (invalidate primary "
         "txtable; USB will re-enumerate)\n");

  MTD_ERASE(full, lasterase, 1);

  syslog(LOG_INFO, "rp2040: primary txtable erased\n");
}
#endif /* CONFIG_RP2040_BOOTLOADER_DEBUG_NODES */
#endif /* CONFIG_TXTABLE_PARTITION */

/* Debug scaffolding: the full-chip probe (register /dev/rpflashall, read
 * offset 0/0x100, txtable parse) runs inline on the bring-up thread (see
 * rp2040_bringup).  Only writing the default txtable is deferred to a
 * manual /dev/txtable_mk because the flash erase+program stops XIP and
 * re-enumerates USB.
 */

#if defined(CONFIG_MTD) && defined(CONFIG_RP2040_BOOTLOADER_DEBUG_NODES)
static FAR struct mtd_dev_s *g_full_mtd;
static struct work_s g_txtable_mk_work;

static void rp2040_txtable_mk_worker(FAR void *arg)
{
#ifdef CONFIG_TXTABLE_PARTITION
  rp2040_txtable_mktable(g_full_mtd);
#else
  syslog(LOG_INFO, "rp2040: CONFIG_TXTABLE_PARTITION disabled\n");
#endif
}

static ssize_t rp2040_txtable_mk_read(FAR struct file *filep,
                                     FAR char *buffer, size_t buflen)
{
  if (filep->f_pos != 0 || g_full_mtd == NULL)
    {
      return 0;
    }

  work_queue(LPWORK, &g_txtable_mk_work, rp2040_txtable_mk_worker, NULL, 0);

  filep->f_pos = buflen;
  return snprintf(buffer, buflen, "txtable mk scheduled, see dmesg\n");
}

static const struct file_operations g_txtable_mk_ops =
{
  .read = rp2040_txtable_mk_read,
};

static struct work_s g_txtable_erase_work;

static void rp2040_txtable_erase_worker(FAR void *arg)
{
#ifdef CONFIG_TXTABLE_PARTITION
  rp2040_txtable_erase(g_full_mtd);
#else
  syslog(LOG_INFO, "rp2040: CONFIG_TXTABLE_PARTITION disabled\n");
#endif
}

static ssize_t rp2040_txtable_erase_read(FAR struct file *filep,
                                   FAR char *buffer, size_t buflen)
{
  if (filep->f_pos != 0 || g_full_mtd == NULL)
    {
      return 0;
    }

  work_queue(LPWORK, &g_txtable_erase_work,
             rp2040_txtable_erase_worker, NULL, 0);

  filep->f_pos = buflen;
  return snprintf(buffer, buflen, "txtable erase scheduled, see dmesg\n");
}

static const struct file_operations g_txtable_erase_ops =
{
  .read = rp2040_txtable_erase_read,
};

/* MTD-path probe as a normal kernel thread (not bring-up inline, not a
 * chardev read callback, not the low-priority work queue).  Started from
 * bring-up so it runs after nsh/USB are up: each step logs to the (now live)
 * console, and a hang here leaves nsh usable so the last log line pinpoints
 * the fault.
 */

static int rp2040_flash_probe_task(int argc, FAR char *argv[])
{
  uint8_t buf[256];  /* One MTD block (FLASH_SECTOR_SIZE); MTD_BREAD copies a
                      * whole block, so an 8-byte buffer overflowed the stack
                      * and looked like a "core hang".
                      */
  ssize_t rd;
  int ret;

  syslog(LOG_INFO, "rp2040: probe task start\n");

  ret = register_mtddriver("/dev/rpflashall", g_full_mtd, 0644, NULL);
  syslog(LOG_INFO, "rp2040: /dev/rpflashall register ret=%d\n", ret);

  syslog(LOG_INFO, "rp2040: before MTD_BREAD off0\n");
  rd = MTD_BREAD(g_full_mtd, 0, 1, buf);
  syslog(LOG_INFO,
         "rp2040: off 0x000 rd=%d boot2[0..7]="
         "%02x %02x %02x %02x %02x %02x %02x %02x\n",
         (int)rd, buf[0], buf[1], buf[2], buf[3],
         buf[4], buf[5], buf[6], buf[7]);

  rd = MTD_BREAD(g_full_mtd, 1, 1, buf);
  syslog(LOG_INFO,
         "rp2040: off 0x100 rd=%d vect[0..7]="
         "%02x %02x %02x %02x %02x %02x %02x %02x\n",
         (int)rd, buf[0], buf[1], buf[2], buf[3],
         buf[4], buf[5], buf[6], buf[7]);

#ifdef CONFIG_TXTABLE_PARTITION
  syslog(LOG_INFO, "rp2040: before txtable\n");
  rp2040_txtable_setup(g_full_mtd);
#endif

  syslog(LOG_INFO, "rp2040: probe task done\n");
  return 0;
}

/* Manual trigger for the MTD-path probe.  Kept off the boot path so nsh and
 * the "reboot recovery" escape hatch always come up; "cat /dev/flash_probe"
 * spawns the probe on its own thread when explicitly requested.
 */

static ssize_t rp2040_flash_probe_read(FAR struct file *filep,
                                   FAR char *buffer, size_t buflen)
{
  int pid;

  if (filep->f_pos != 0 || g_full_mtd == NULL)
    {
      return 0;
    }

  pid = kthread_create("flashprobe", 100, 4096,
                       rp2040_flash_probe_task, NULL);

  filep->f_pos = buflen;
  return snprintf(buffer, buflen,
                  "flash probe thread spawned (pid=%d)\n", pid);
}

static const struct file_operations g_flash_probe_ops =
{
  .read = rp2040_flash_probe_read,
};
#endif /* CONFIG_MTD && CONFIG_RP2040_BOOTLOADER_DEBUG_NODES */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_bringup
 ****************************************************************************/

int rp2040_bringup(void)
{
#ifdef CONFIG_ARCH_BOARD_COMMON

  int ret = rp2040_common_bringup();
  if (ret < 0)
    {
      return ret;
    }

#endif /* CONFIG_ARCH_BOARD_COMMON */

  /* --- Place any board specific bringup code here --- */

#ifdef CONFIG_MTD
  /* Expose an MTD instance spanning the whole flash chip so the partition
   * layer (txtable) can carve /dev/ap and the staging area across the full
   * device.  The legacy smartfs MTD only covers the region past the NuttX
   * image and cannot address offset 0 (vector table / boot2 / the bootloader
   * partition itself), so a dedicated full-chip instance is required.
   */

  {
    FAR struct mtd_dev_s *flash_full = rp2040_flash_mtd_initialize_full();

    if (flash_full == NULL)
      {
        syslog(LOG_ERR, "rp2040: full-chip flash MTD init failed\n");
      }
    else
      {
#if defined(CONFIG_TXTABLE_PARTITION) && defined(CONFIG_RP2040_BOOT_ELF)
        /* In the first-stage bootloader, parse the txtable at boot so the
         * named partitions (/dev/ap, ...) exist for "fastboot flash <name>"
         * before anything jumps to the AP.  The "bootloader" partition is
         * intentionally not exposed (see rp2040_txtable_handler()).
         */

        rp2040_txtable_setup(flash_full);
#endif

#ifdef CONFIG_RP2040_BOOTLOADER_DEBUG_NODES
        {
          int ret2;

          const volatile uint8_t *p0 =
            (const volatile uint8_t *)0x10000000;
          const volatile uint8_t *p1 =
            (const volatile uint8_t *)0x10000100;

          g_full_mtd = flash_full;

          /* Whole-chip flash node plus a plain-pointer read of offset
           * 0 / 0x100 (never through MTD block_read, which hangs the core on
           * this target).  Writable access to the entire flash: debug only.
           */

          ret2 = register_mtddriver("/dev/rpflashall", flash_full, 0644,
                                    NULL);
          syslog(LOG_INFO,
                 "rp2040: /dev/rpflashall register ret=%d\n", ret2);

          syslog(LOG_INFO,
                 "rp2040: off 0x000 boot2[0..7]="
                 "%02x %02x %02x %02x %02x %02x %02x %02x\n",
                 p0[0], p0[1], p0[2], p0[3], p0[4], p0[5], p0[6], p0[7]);
          syslog(LOG_INFO,
                 "rp2040: off 0x100 vect[0..7]="
                 "%02x %02x %02x %02x %02x %02x %02x %02x\n",
                 p1[0], p1[1], p1[2], p1[3], p1[4], p1[5], p1[6], p1[7]);

          /* Manual, USB-re-enumerating path to write the default txtable
           * on a blank flash (erase+program stops XIP).
           */

          ret2 = register_driver("/dev/txtable_mk", &g_txtable_mk_ops, 0444,
                                 NULL);
          syslog(LOG_INFO,
                 "rp2040: /dev/txtable_mk register ret=%d "
                 "(cat to write default txtable)\n", ret2);

          ret2 = register_driver("/dev/flash_probe", &g_flash_probe_ops,
                                 0444, NULL);
          syslog(LOG_INFO,
                 "rp2040: /dev/flash_probe register ret=%d "
                 "(cat to spawn MTD-path probe thread)\n", ret2);

          ret2 = register_driver("/dev/txtable_erase", &g_txtable_erase_ops,
                                 0444, NULL);
          syslog(LOG_INFO,
                 "rp2040: /dev/txtable_erase register ret=%d "
                 "(cat to erase primary txtable, test /etc fallback)\n",
                 ret2);
        }
#endif
      }
  }
#endif

#ifdef CONFIG_BOARDCTL_RESET_CAUSE
  /* Read (and clear) the persisted reset cause so we can confirm on the
   * console that board_reset(ENTER_BOOTLOADER) -> soft reset ->
   * board_reset_cause() round-trips and clears on read.  The first-stage
   * boot flow branches on this to stay in fastboot vs. boot the AP.
   */

  {
    struct boardioc_reset_cause_s rc;

    rc.cause = 0;
    rc.flag  = 0;
    board_reset_cause(&rc);
    syslog(LOG_INFO,
           "RESET-CAUSE: cause=%d flag=%d (flag==3 is ENTER_BOOTLOADER)\n",
           (int)rc.cause, (int)rc.flag);
  }
#endif

  return OK;
}
