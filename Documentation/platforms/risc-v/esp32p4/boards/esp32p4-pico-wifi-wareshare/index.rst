==============================
ESP32-P4-PICO-WIFI (Waveshare)
==============================

.. tags:: chip:esp32p4, arch:risc-v, vendor:waveshare

The `Waveshare ESP32-P4-PICO-WIFI <https://www.waveshare.com/wiki/ESP32-P4-PICO-WIFI>`_
is a compact development board based on the ESP32-P4 SoC. It features an on-board
100 Mbps Ethernet PHY (IP101GRI), Wi-Fi 6 via an ESP32-C6 companion module, USB 2.0
High-Speed, and a Pico-compatible form factor.

Key hardware features relevant to NuttX:

* ESP32-P4 dual-core RISC-V HP (400 MHz) + LP (40 MHz)
* 32 MB PSRAM (in-package)
* 16 MB Flash
* 100 Mbps Ethernet: IP101GRI PHY, RMII interface
* Wi-Fi 6 + BLE 5.0 via ESP32-C6 companion
* USB 2.0 High-Speed OTG
* Pico-compatible 40-pin header

Ethernet
========

The board integrates an IP101GRI Ethernet PHY connected to the ESP32-P4 EMAC
via RMII interface. The PHY uses an external 50 MHz crystal for the RMII
reference clock (CLK_INPUT mode).

Pin assignments (directly routed via IO_MUX, no IO expander needed):

========== ======= ===========
Signal     GPIO    Description
========== ======= ===========
TXD0       GPIO34  Transmit Data 0
TXD1       GPIO35  Transmit Data 1
TX_EN      GPIO49  Transmit Enable
RXD0       GPIO29  Receive Data 0
RXD1       GPIO30  Receive Data 1
CRS_DV     GPIO28  Carrier Sense / Data Valid
REF_CLK    GPIO50  50 MHz RMII Reference Clock (input)
MDC        GPIO23  Management Data Clock
MDIO       GPIO18  Management Data I/O
PHY_RST    GPIO5   PHY Hardware Reset
========== ======= ===========

PHY address: **1**

Configurations
==============

nsh
---

Basic NuttShell configuration with serial console.

To configure and build::

  $ ./tools/configure.sh esp32p4-pico-wifi-wareshare:nsh
  $ make

ethernet
--------

NuttShell configuration with Ethernet networking enabled. This configuration
activates the ESP32-P4 EMAC driver with the IP101 PHY using RMII interface.

Enabled networking features:

* IPv4 with ARP
* TCP and UDP sockets
* ICMP (ping)
* NSH ``ifconfig``, ``ping`` commands

To configure and build::

  $ ./tools/configure.sh esp32p4-pico-wifi-wareshare:ethernet
  $ make

After booting, bring up the interface and configure an IP address::

  nsh> ifup eth0
  nsh> ifconfig eth0 192.168.1.100
  nsh> ping 192.168.1.1

spiflash
--------

NuttShell configuration with SPI flash support.

To configure and build::

  $ ./tools/configure.sh esp32p4-pico-wifi-wareshare:spiflash
  $ make
