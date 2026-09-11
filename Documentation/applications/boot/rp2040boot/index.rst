================================
``rp2040boot`` RP2040 ELF Loader
================================

ELF bootloader for RP2040 that loads and jumps to an application image
from flash.  The bootloader reads an ELF executable from a raw flash
partition (MTD char device ``/dev/ap0``), validates program headers,
copies PT_LOAD segments into SRAM via ``libelf``, and jumps to the
application entry point.

Configuration options:

- ``CONFIG_BOOT_RP2040BOOT`` - Enable the bootloader.
- ``CONFIG_BOOT_RP2040BOOT_ELF_PATH`` - Path to the AP ELF image.
  Default: ``/dev/ap0``
- ``CONFIG_BOOT_RP2040BOOT_AP_BASE`` - Start address of AP SRAM.
  Default: ``0x20007800``
- ``CONFIG_BOOT_RP2040BOOT_JUMP_DELAY_MS`` - Maximum DTE-wait timeout
  in milliseconds before producing output.  The bootloader polls
  ``CAIOC_GETCTRLLINE`` on the CDC/ACM port and proceeds as soon as
  the host opens the connection.  Default: ``5000``
- ``CONFIG_BOOT_RP2040BOOT_RESET_USB_BOOT`` - After loading (or on
  failure), reboot into USB BOOTSEL mode via ``boardctl(BOARDIOC_RESET,
  BOARDIOC_SOFTRESETCAUSE_ENTER_BOOTLOADER)`` so the board re-appears
  on USB without a physical BOOTSEL button press.

Build prerequisites
-------------------

- ``PICO_SDK_PATH`` must point to the Raspberry Pi Pico SDK (required
  for boot stage 2 assembly).
- ``CONFIG_LIBC_ELF=y`` for libelf support.
- ``CONFIG_RP2040_FLASH_MTD=y`` and
  ``CONFIG_RP2040_AP_FLASH_PART=y`` for the raw flash partition.
- ``CONFIG_BCH=y`` for the MTD block-to-character device bridge.
- ``CONFIG_CDCACM=y`` and ``CONFIG_USBDEV=y`` for USB CDC console.

Flashing
--------

The AP ELF is written to flash as a raw binary at a known offset::

  picotool load -v bootloader.uf2
  picotool load -v ap.elf -t bin -o 0x10040000
  picotool reboot -a

The offset (``0x10040000``) must match the partition offset configured
via ``CONFIG_RP2040_AP_FLASH_PART_OFFSET``.
