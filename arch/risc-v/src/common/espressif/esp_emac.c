/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_emac.c
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

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/queue.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#if defined(CONFIG_NET_PKT)
#  include <nuttx/net/pkt.h>
#endif

#include "riscv_internal.h"
#include "esp_irq.h"
#include "esp_gpio.h"

#include "hal/efuse_ll.h"
#include "hal/emac_hal.h"
#include "hal/emac_ll.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/interrupts.h"

/* The emac_ll.h macros reference __DECLARE_RCC_ATOMIC_ENV which is
 * normally provided by ESP-IDF's periph_ctrl.h critical section helpers.
 * In NuttX we handle bus clock enable without that framework, so define
 * it as a harmless constant so (void)__DECLARE_RCC_ATOMIC_ENV compiles.
 */

#define __DECLARE_RCC_ATOMIC_ENV 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Number of DMA descriptors */

#ifndef CONFIG_ESPRESSIF_ETH_NRXDESC
#  define CONFIG_ESPRESSIF_ETH_NRXDESC 9
#endif

#ifndef CONFIG_ESPRESSIF_ETH_NTXDESC
#  define CONFIG_ESPRESSIF_ETH_NTXDESC 8
#endif

#ifndef CONFIG_ESPRESSIF_ETH_MDCPIN
#  define CONFIG_ESPRESSIF_ETH_MDCPIN 23
#endif

#ifndef CONFIG_ESPRESSIF_ETH_MDIOPIN
#  define CONFIG_ESPRESSIF_ETH_MDIOPIN 18
#endif

#ifndef CONFIG_ESPRESSIF_ETH_PHY_ADDR
#  define CONFIG_ESPRESSIF_ETH_PHY_ADDR 1
#endif

/* Buffer and descriptor constants */

#define EMAC_RX_BUF_NUM    CONFIG_ESPRESSIF_ETH_NRXDESC
#define EMAC_TX_BUF_NUM    CONFIG_ESPRESSIF_ETH_NTXDESC
#define EMAC_BUF_LEN       1600
/* +2 accounts for d_buf held by the network stack during RX processing
 * and one spare to avoid pool exhaustion under back-pressure.
 */

#define EMAC_BUF_NUM       (EMAC_RX_BUF_NUM + EMAC_TX_BUF_NUM + 2)
#define ETH_CRC_LEN        4

/* PHY configuration */

#define EMAC_PHY_ADDR      CONFIG_ESPRESSIF_ETH_PHY_ADDR

/* Timeout values */

#define EMAC_READPHY_TO    1000
#define EMAC_WRITEPHY_TO   1000
#define EMAC_RSTPHY_TO     1000
#define EMAC_WAITLINK_TO   50000
#define EMAC_RESET_TO      1000
#define EMAC_TX_TO         (1 * CLK_TCK)

/* Work queue selection */

#define EMACWORK           LPWORK

/* TX busy check */

#define TX_IS_BUSY(p)      ((p)->txcur->TDES0.Own == EMAC_LL_DMADESC_OWNER_DMA)

/* Helper to get private data from net_driver_s */

#define NET2PRIV(d)        ((struct esp_emac_s *)((d)->d_private))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_emac_s
{
  struct net_driver_s     dev;

  /* HAL context */

  emac_hal_context_t      hal;

  /* DMA descriptors - must be 64-byte aligned for ESP32-P4 */

  eth_dma_rx_descriptor_t rxdesc[EMAC_RX_BUF_NUM]
                          __attribute__((aligned(64)));
  eth_dma_tx_descriptor_t txdesc[EMAC_TX_BUF_NUM]
                          __attribute__((aligned(64)));

  /* Current DMA descriptor pointers */

  eth_dma_rx_descriptor_t *rxcur;
  eth_dma_tx_descriptor_t *txcur;

  /* Buffer pool */

  uint8_t                 bufpool[EMAC_BUF_NUM][EMAC_BUF_LEN]
                          __attribute__((aligned(64)));
  sq_queue_t              freeb;

  /* Work queue items */

  struct work_s           rxwork;
  struct work_s           txwork;
  struct work_s           pollwork;
  struct work_s           timeoutwork;

  /* TX timeout watchdog */

  struct wdog_s           txtimeout;

  /* Interrupt number */

  int                     cpuint;

  /* Interface state */

  bool                    ifup;
  bool                    mbps100;
  bool                    fduplex;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp_emac_s g_emac;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Buffer management */

static void    emac_init_buffer(struct esp_emac_s *priv);
static void   *emac_alloc_buffer(struct esp_emac_s *priv);
static void    emac_free_buffer(struct esp_emac_s *priv, void *buf);

/* Low-level hardware operations */

static void    emac_init_gpio(void);
static int     emac_reset(struct esp_emac_s *priv);
static void    emac_init_dma(struct esp_emac_s *priv);
static void    emac_deinit_dma(struct esp_emac_s *priv);

/* PHY operations */

static int     emac_read_phy(struct esp_emac_s *priv,
                             uint16_t dev_addr,
                             uint16_t reg_addr,
                             uint16_t *pdata);
static int     emac_write_phy(struct esp_emac_s *priv,
                              uint16_t dev_addr,
                              uint16_t reg_addr,
                              uint16_t data);
static int     emac_init_phy(struct esp_emac_s *priv);

/* Packet operations */

static int     emac_transmit(struct esp_emac_s *priv);
static int     emac_recvframe(struct esp_emac_s *priv);
static void    emac_dopoll(struct esp_emac_s *priv);

/* Interrupt and work queue handlers */

static int     emac_interrupt(int irq, void *context, void *arg);
static void    emac_rx_interrupt_work(void *arg);
static void    emac_tx_interrupt_work(void *arg);
static void    emac_txavail_work(void *arg);
static void    emac_txtimeout_work(void *arg);
static void    emac_txtimeout_expiry(wdparm_t arg);

/* Callback functions for net_driver_s */

static int     emac_txpoll(struct net_driver_s *dev);
static int     emac_ifup(struct net_driver_s *dev);
static int     emac_ifdown(struct net_driver_s *dev);
static int     emac_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int     emac_addmac(struct net_driver_s *dev,
                           const uint8_t *mac);
static int     emac_rmmac(struct net_driver_s *dev,
                          const uint8_t *mac);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: emac_init_buffer
 ****************************************************************************/

static void emac_init_buffer(struct esp_emac_s *priv)
{
  int i;

  sq_init(&priv->freeb);

  for (i = 0; i < EMAC_BUF_NUM; i++)
    {
      sq_addlast((sq_entry_t *)priv->bufpool[i], &priv->freeb);
    }
}

/****************************************************************************
 * Name: emac_alloc_buffer
 ****************************************************************************/

static void *emac_alloc_buffer(struct esp_emac_s *priv)
{
  return sq_remfirst(&priv->freeb);
}

/****************************************************************************
 * Name: emac_free_buffer
 ****************************************************************************/

static void emac_free_buffer(struct esp_emac_s *priv, void *buf)
{
  sq_addlast((sq_entry_t *)buf, &priv->freeb);
}

/****************************************************************************
 * Name: emac_init_gpio
 ****************************************************************************/

static void emac_init_gpio(void)
{
  /* MDC - output via GPIO matrix */

  esp_configgpio(CONFIG_ESPRESSIF_ETH_MDCPIN, OUTPUT);
  esp_gpio_matrix_out(CONFIG_ESPRESSIF_ETH_MDCPIN,
                      MII_MDC_PAD_OUT_IDX, false, false);

  /* MDIO - bidirectional via GPIO matrix */

  esp_configgpio(CONFIG_ESPRESSIF_ETH_MDIOPIN, INPUT | OUTPUT | OPEN_DRAIN);
  esp_gpio_matrix_out(CONFIG_ESPRESSIF_ETH_MDIOPIN,
                      MII_MDO_PAD_OUT_IDX, false, false);
  esp_gpio_matrix_in(CONFIG_ESPRESSIF_ETH_MDIOPIN,
                     MII_MDI_PAD_IN_IDX, false);

#ifdef CONFIG_ESPRESSIF_ETH_ENABLE_PHY_RSTPIN
  /* PHY reset pin */

  esp_configgpio(CONFIG_ESPRESSIF_ETH_PHY_RSTPIN, OUTPUT);
  esp_gpiowrite(CONFIG_ESPRESSIF_ETH_PHY_RSTPIN, false);
  up_mdelay(10);
  esp_gpiowrite(CONFIG_ESPRESSIF_ETH_PHY_RSTPIN, true);
  up_mdelay(10);
#endif
}

/****************************************************************************
 * Name: emac_reset
 ****************************************************************************/

static int emac_reset(struct esp_emac_s *priv)
{
  int i;

  emac_ll_reset(priv->hal.dma_regs);

  for (i = 0; i < EMAC_RESET_TO; i++)
    {
      up_udelay(100);
      if (emac_ll_is_reset_done(priv->hal.dma_regs))
        {
          return 0;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: emac_init_dma
 ****************************************************************************/

static void emac_init_dma(struct esp_emac_s *priv)
{
  int i;
  eth_dma_rx_descriptor_t *rxdesc = priv->rxdesc;
  eth_dma_tx_descriptor_t *txdesc = priv->txdesc;

  emac_init_buffer(priv);

  /* Initialize RX descriptors */

  for (i = 0; i < EMAC_RX_BUF_NUM; i++)
    {
      memset(&rxdesc[i], 0, sizeof(eth_dma_rx_descriptor_t));
      rxdesc[i].RDES0.Own = EMAC_LL_DMADESC_OWNER_DMA;
      rxdesc[i].RDES1.ReceiveBuffer1Size = EMAC_BUF_LEN;
      rxdesc[i].RDES1.SecondAddressChained = 1;
      rxdesc[i].Buffer1Addr = (uint32_t)emac_alloc_buffer(priv);
      DEBUGASSERT(rxdesc[i].Buffer1Addr);
      rxdesc[i].Buffer2NextDescAddr = (uint32_t)&rxdesc[i + 1];
    }

  rxdesc[i - 1].Buffer2NextDescAddr = (uint32_t)&rxdesc[0];
  priv->rxcur = &rxdesc[0];

  /* Initialize TX descriptors */

  for (i = 0; i < EMAC_TX_BUF_NUM; i++)
    {
      memset(&txdesc[i], 0, sizeof(eth_dma_tx_descriptor_t));
      txdesc[i].TDES0.SecondAddressChained = 1;
      txdesc[i].Buffer1Addr = 0;
      txdesc[i].Buffer2NextDescAddr = (uint32_t)&txdesc[i + 1];
    }

  txdesc[i - 1].Buffer2NextDescAddr = (uint32_t)&txdesc[0];
  priv->txcur = &txdesc[0];

  /* Set descriptor addresses in DMA */

  emac_hal_set_rx_tx_desc_addr(&priv->hal, rxdesc, txdesc);
}

/****************************************************************************
 * Name: emac_deinit_dma
 ****************************************************************************/

static void emac_deinit_dma(struct esp_emac_s *priv)
{
  int i;
  eth_dma_rx_descriptor_t *rxdesc = priv->rxdesc;
  eth_dma_tx_descriptor_t *txdesc = priv->txdesc;

  /* Return all buffers referenced by descriptors back to the free list.
   * Do NOT call emac_init_buffer() here — it rebuilds the entire free
   * list from the pool, and the subsequent per-descriptor free would
   * double-add buffers.
   */

  for (i = 0; i < EMAC_RX_BUF_NUM; i++)
    {
      if (rxdesc[i].Buffer1Addr)
        {
          emac_free_buffer(priv, (void *)rxdesc[i].Buffer1Addr);
          rxdesc[i].Buffer1Addr = 0;
        }
    }

  for (i = 0; i < EMAC_TX_BUF_NUM; i++)
    {
      if (txdesc[i].Buffer1Addr)
        {
          emac_free_buffer(priv, (void *)txdesc[i].Buffer1Addr);
          txdesc[i].Buffer1Addr = 0;
        }
    }
}

/****************************************************************************
 * Name: emac_read_phy
 ****************************************************************************/

static int emac_read_phy(struct esp_emac_s *priv,
                         uint16_t dev_addr,
                         uint16_t reg_addr,
                         uint16_t *pdata)
{
  int i;

  if (emac_ll_is_mii_busy(priv->hal.mac_regs))
    {
      return -EBUSY;
    }

  emac_hal_set_phy_cmd(&priv->hal, dev_addr, reg_addr, false);

  for (i = 0; i < EMAC_READPHY_TO; i++)
    {
      up_udelay(100);
      if (!emac_ll_is_mii_busy(priv->hal.mac_regs))
        {
          break;
        }
    }

  if (i >= EMAC_READPHY_TO)
    {
      return -ETIMEDOUT;
    }

  *pdata = (uint16_t)emac_ll_get_phy_data(priv->hal.mac_regs);
  return 0;
}

/****************************************************************************
 * Name: emac_write_phy
 ****************************************************************************/

static int emac_write_phy(struct esp_emac_s *priv,
                          uint16_t dev_addr,
                          uint16_t reg_addr,
                          uint16_t data)
{
  int i;

  if (emac_ll_is_mii_busy(priv->hal.mac_regs))
    {
      return -EBUSY;
    }

  emac_ll_set_phy_data(priv->hal.mac_regs, data);
  emac_hal_set_phy_cmd(&priv->hal, dev_addr, reg_addr, true);

  for (i = 0; i < EMAC_WRITEPHY_TO; i++)
    {
      up_udelay(100);
      if (!emac_ll_is_mii_busy(priv->hal.mac_regs))
        {
          break;
        }
    }

  if (i >= EMAC_WRITEPHY_TO)
    {
      return -ETIMEDOUT;
    }

  return 0;
}

/****************************************************************************
 * Name: emac_init_phy
 ****************************************************************************/

static int emac_init_phy(struct esp_emac_s *priv)
{
  int ret;
  int i;
  uint16_t val;

  /* Power on PHY chip */

  ret = emac_read_phy(priv, EMAC_PHY_ADDR, MII_MCR, &val);
  if (ret != 0)
    {
      nerr("ERROR: Failed to read PHY MCR: %d\n", ret);
      return ret;
    }

  val &= ~MII_MCR_PDOWN;
  ret = emac_write_phy(priv, EMAC_PHY_ADDR, MII_MCR, val);
  if (ret != 0)
    {
      nerr("ERROR: Failed to write PHY MCR: %d\n", ret);
      return ret;
    }

  /* Reset PHY */

  val |= MII_MCR_RESET;
  ret = emac_write_phy(priv, EMAC_PHY_ADDR, MII_MCR, val);
  if (ret != 0)
    {
      nerr("ERROR: Failed to reset PHY: %d\n", ret);
      return ret;
    }

  for (i = 0; i < EMAC_RSTPHY_TO; i++)
    {
      up_udelay(100);
      ret = emac_read_phy(priv, EMAC_PHY_ADDR, MII_MCR, &val);
      if (ret != 0)
        {
          return ret;
        }

      if (!(val & MII_MCR_RESET))
        {
          break;
        }
    }

  if (i >= EMAC_RSTPHY_TO)
    {
      return -ETIMEDOUT;
    }

  /* Read PHY ID */

  ret = emac_read_phy(priv, EMAC_PHY_ADDR, MII_PHYID1, &val);
  if (ret != 0)
    {
      return ret;
    }

  ninfo("PHY ID1: 0x%04x\n", val);

  ret = emac_read_phy(priv, EMAC_PHY_ADDR, MII_PHYID2, &val);
  if (ret != 0)
    {
      return ret;
    }

  ninfo("PHY ID2: 0x%04x\n", val);

  /* Wait for link up */

  for (i = 0; i < EMAC_WAITLINK_TO; i++)
    {
      up_udelay(10);
      ret = emac_read_phy(priv, EMAC_PHY_ADDR, MII_MSR, &val);
      if (ret != 0)
        {
          priv->ifup = false;
          return ret;
        }

      if (val & MII_MSR_LINKSTATUS)
        {
          break;
        }
    }

  if (i >= EMAC_WAITLINK_TO)
    {
      nerr("ERROR: Timeout waiting for PHY link up\n");
      priv->ifup = false;
      return -ETIMEDOUT;
    }

  priv->ifup = true;

  /* Determine speed and duplex from auto-negotiation result.
   * Read our advertised capabilities (MII_ADVERTISE, reg 4) and the
   * link partner's abilities (MII_LPA, reg 5).  The negotiated mode
   * is the intersection of both, picking the highest common mode.
   */

  uint16_t advertise;
  uint16_t lpa;

  ret = emac_read_phy(priv, EMAC_PHY_ADDR, MII_ADVERTISE, &advertise);
  if (ret != 0)
    {
      nwarn("WARNING: Failed to read ADVERTISE, assuming 10M half\n");
      advertise = 0;
    }

  ret = emac_read_phy(priv, EMAC_PHY_ADDR, MII_LPA, &lpa);
  if (ret != 0)
    {
      nwarn("WARNING: Failed to read LPA, assuming 10M half\n");
      lpa = 0;
    }

  if ((advertise & MII_ADVERTISE_100BASETXFULL) &&
      (lpa & MII_LPA_100BASETXFULL))
    {
      priv->mbps100 = true;
      priv->fduplex = true;
    }
  else if ((advertise & MII_ADVERTISE_100BASETXHALF) &&
           (lpa & MII_LPA_100BASETXHALF))
    {
      priv->mbps100 = true;
      priv->fduplex = false;
    }
  else if ((advertise & MII_ADVERTISE_10BASETXFULL) &&
           (lpa & MII_LPA_10BASETXFULL))
    {
      priv->mbps100 = false;
      priv->fduplex = true;
    }
  else
    {
      priv->mbps100 = false;
      priv->fduplex = false;
    }

  /* Configure MAC speed and duplex */

  emac_ll_set_port_speed(priv->hal.mac_regs,
                         priv->mbps100 ? ETH_SPEED_100M : ETH_SPEED_10M);
  emac_ll_set_duplex(priv->hal.mac_regs,
                     priv->fduplex ? ETH_DUPLEX_FULL : ETH_DUPLEX_HALF);

  ninfo("Link up: %s %s\n",
        priv->mbps100 ? "100Mbps" : "10Mbps",
        priv->fduplex ? "Full-Duplex" : "Half-Duplex");

  return 0;
}

/****************************************************************************
 * Name: emac_transmit
 ****************************************************************************/

static int emac_transmit(struct esp_emac_s *priv)
{
  int ret;
  eth_dma_tx_descriptor_t *txcur = priv->txcur;

  if (txcur->TDES0.Own == EMAC_LL_DMADESC_OWNER_DMA)
    {
      return -EBUSY;
    }

  if (txcur->Buffer1Addr)
    {
      emac_free_buffer(priv, (void *)txcur->Buffer1Addr);
    }

  txcur->Buffer1Addr = (uint32_t)priv->dev.d_buf;
  txcur->TDES1.TransmitBuffer1Size = priv->dev.d_len;
  /* Clear all control bits then set the ones we need.  This is
   * intentional — Value=0 wipes stale flags from the previous
   * transfer before we configure the new one.
   */

  txcur->TDES0.Value = 0;
  txcur->TDES0.FirstSegment = 1;
  txcur->TDES0.LastSegment = 1;
  txcur->TDES0.SecondAddressChained = 1;
  txcur->TDES0.InterruptOnComplete = 1;
  txcur->TDES0.Own = EMAC_LL_DMADESC_OWNER_DMA;

  priv->txcur = (eth_dma_tx_descriptor_t *)txcur->Buffer2NextDescAddr;

  /* Trigger DMA transmit */

  emac_ll_transmit_poll_demand(priv->hal.dma_regs, 0);

  ninfo("d_buf=%p d_len=%d\n", priv->dev.d_buf, priv->dev.d_len);

  priv->dev.d_buf = NULL;
  priv->dev.d_len = 0;

  /* Setup the TX timeout watchdog */

  ret = wd_start(&priv->txtimeout, EMAC_TX_TO,
                 emac_txtimeout_expiry, (wdparm_t)priv);
  if (ret)
    {
      nerr("ERROR: Failed to start TX timeout timer\n");
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: emac_recvframe
 ****************************************************************************/

static int emac_recvframe(struct esp_emac_s *priv)
{
  uint32_t len;
  eth_dma_rx_descriptor_t *rxcur = priv->rxcur;

  if (rxcur->RDES0.Own == EMAC_LL_DMADESC_OWNER_DMA)
    {
      return -EBUSY;
    }

  if (!rxcur->Buffer1Addr)
    {
      return -EINVAL;
    }

  len = rxcur->RDES0.FrameLength;
  priv->dev.d_buf = (uint8_t *)rxcur->Buffer1Addr;
  priv->dev.d_len = len - ETH_CRC_LEN;

  /* Allocate a fresh buffer for this descriptor.  If the pool is
   * exhausted, keep the old buffer (discard the received frame) so
   * that DMA never writes to address 0.
   */

  void *newbuf = emac_alloc_buffer(priv);
  if (newbuf != NULL)
    {
      rxcur->Buffer1Addr = (uint32_t)newbuf;
    }
  else
    {
      nwarn("WARNING: RX buffer pool exhausted, recycling descriptor\n");

      /* d_buf still points to the old buffer — the caller will free
       * it after processing.  We must NOT hand this descriptor back
       * to DMA without a valid buffer, so just leave the old address
       * and let the frame be overwritten on the next DMA cycle.
       */
    }

  rxcur->RDES0.Value = 0;
  rxcur->RDES0.Own = EMAC_LL_DMADESC_OWNER_DMA;
  rxcur->RDES1.ReceiveBuffer1Size = EMAC_BUF_LEN;
  rxcur->RDES1.SecondAddressChained = 1;

  priv->rxcur = (eth_dma_rx_descriptor_t *)rxcur->Buffer2NextDescAddr;

  emac_ll_receive_poll_demand(priv->hal.dma_regs, 0);

  ninfo("RX bytes %d\n", priv->dev.d_len);

  return 0;
}

/****************************************************************************
 * Name: emac_txpoll
 ****************************************************************************/

static int emac_txpoll(struct net_driver_s *dev)
{
  struct esp_emac_s *priv = NET2PRIV(dev);

  DEBUGASSERT(priv->dev.d_buf != NULL);

  emac_transmit(priv);
  DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

  if (TX_IS_BUSY(priv))
    {
      return -EBUSY;
    }

  dev->d_buf = (uint8_t *)emac_alloc_buffer(priv);
  if (dev->d_buf == NULL)
    {
      return -ENOMEM;
    }

  dev->d_len = EMAC_BUF_LEN;
  return 0;
}

/****************************************************************************
 * Name: emac_dopoll
 ****************************************************************************/

static void emac_dopoll(struct esp_emac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  if (!TX_IS_BUSY(priv))
    {
      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

      dev->d_buf = (uint8_t *)emac_alloc_buffer(priv);
      if (!dev->d_buf)
        {
          return;
        }

      dev->d_len = EMAC_BUF_LEN;

      devif_poll(dev, emac_txpoll);

      if (dev->d_buf)
        {
          emac_free_buffer(priv, dev->d_buf);
          dev->d_buf = NULL;
          dev->d_len = 0;
        }
    }
}

/****************************************************************************
 * Name: emac_rx_interrupt_work
 ****************************************************************************/

static void emac_rx_interrupt_work(void *arg)
{
  struct esp_emac_s *priv = (struct esp_emac_s *)arg;
  struct net_driver_s *dev = &priv->dev;

  net_lock();

  while (emac_recvframe(priv) == 0)
    {
      struct eth_hdr_s *eth_hdr = (struct eth_hdr_s *)dev->d_buf;

#ifdef CONFIG_NET_PKT
      pkt_input(&priv->dev);
#endif

#ifdef CONFIG_NET_IPv4
      if (eth_hdr->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");
          ipv4_input(&priv->dev);
          if (priv->dev.d_len > 0)
            {
              emac_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (eth_hdr->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");
          ipv6_input(&priv->dev);
          if (priv->dev.d_len > 0)
            {
              emac_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (eth_hdr->type == HTONS(ETHTYPE_ARP))
        {
          ninfo("ARP frame\n");
          arp_input(&priv->dev);
          if (priv->dev.d_len > 0)
            {
              emac_transmit(priv);
            }
        }
      else
#endif
        {
          nerr("ERROR: Dropped, Unknown type: %04x\n", eth_hdr->type);
        }

      if (dev->d_buf)
        {
          emac_free_buffer(priv, dev->d_buf);
          dev->d_buf = NULL;
          dev->d_len = 0;
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name: emac_tx_interrupt_work
 ****************************************************************************/

static void emac_tx_interrupt_work(void *arg)
{
  struct esp_emac_s *priv = (struct esp_emac_s *)arg;

  net_lock();
  wd_cancel(&priv->txtimeout);
  emac_dopoll(priv);
  net_unlock();
}

/****************************************************************************
 * Name: emac_interrupt
 ****************************************************************************/

static int emac_interrupt(int irq, void *context, void *arg)
{
  struct esp_emac_s *priv = (struct esp_emac_s *)arg;
  uint32_t status;

  status = emac_ll_get_intr_status(priv->hal.dma_regs);
  emac_ll_clear_corresponding_intr(priv->hal.dma_regs, status);

  if (!priv->ifup)
    {
      return 0;
    }

  if (status & EMAC_LL_DMA_RECEIVE_FINISH_INTR)
    {
      work_queue(EMACWORK, &priv->rxwork, emac_rx_interrupt_work, priv, 0);
    }

  if (status & EMAC_LL_DMA_TRANSMIT_FINISH_INTR)
    {
      work_queue(EMACWORK, &priv->txwork, emac_tx_interrupt_work, priv, 0);
    }

  return 0;
}

/****************************************************************************
 * Name: emac_txtimeout_work
 ****************************************************************************/

static void emac_txtimeout_work(void *arg)
{
  struct esp_emac_s *priv = (struct esp_emac_s *)arg;

  net_lock();
  emac_ifdown(&priv->dev);
  emac_ifup(&priv->dev);
  emac_dopoll(priv);
  net_unlock();
}

/****************************************************************************
 * Name: emac_txtimeout_expiry
 ****************************************************************************/

static void emac_txtimeout_expiry(wdparm_t arg)
{
  struct esp_emac_s *priv = (struct esp_emac_s *)arg;

  nerr("ERROR: TX Timeout!\n");

  work_queue(EMACWORK, &priv->timeoutwork, emac_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Name: emac_txavail_work
 ****************************************************************************/

static void emac_txavail_work(void *arg)
{
  struct esp_emac_s *priv = (struct esp_emac_s *)arg;

  net_lock();
  if (priv->ifup)
    {
      emac_dopoll(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Name: emac_ifup
 ****************************************************************************/

static int emac_ifup(struct net_driver_s *dev)
{
  int ret;
  irqstate_t flags;
  struct esp_emac_s *priv = NET2PRIV(dev);
  emac_hal_dma_config_t dma_config;

  flags = enter_critical_section();

  /* Enable EMAC bus clock and reset */

  emac_ll_enable_bus_clock(0, true);
  emac_ll_reset_register(0);

  /* Enable RMII clock - input mode (clock from PHY) */

  emac_ll_clock_enable_rmii_input(NULL);

  /* Initialize GPIO pins */

  emac_init_gpio();

  /* Initialize HAL context */

  emac_hal_init(&priv->hal);

  /* Reset EMAC DMA */

  ret = emac_reset(priv);
  if (ret)
    {
      leave_critical_section(flags);
      nerr("ERROR: Failed to reset EMAC: %d\n", ret);
      return ret;
    }

  /* Set CSR clock range for MDIO.
   * TODO: obtain actual APB clock frequency from the clock driver
   * instead of assuming 160 MHz.  If the clock tree changes, MDIO
   * timing will be wrong.
   */

  emac_hal_set_csr_clock_range(&priv->hal, 160000000);

  /* Initialize MAC defaults */

  emac_hal_init_mac_default(&priv->hal);

  /* Initialize DMA defaults */

  dma_config.dma_burst_len = ETH_DMA_BURST_LEN_32;
  emac_hal_init_dma_default(&priv->hal, &dma_config);

  /* Set MAC address */

  emac_hal_set_address(&priv->hal, priv->dev.d_mac.ether.ether_addr_octet);

  /* Initialize DMA descriptors */

  emac_init_dma(priv);

  /* Start EMAC */

  emac_hal_start(&priv->hal);

  /* Enable EMAC interrupt */

  up_enable_irq(priv->cpuint);

  leave_critical_section(flags);

  /* Initialize PHY — this involves MDIO polling and link-wait delays
   * (up to ~500 ms) so it MUST run outside the critical section to
   * avoid blocking interrupts for an extended period.
   */

  ret = emac_init_phy(priv);
  if (ret)
    {
      nerr("ERROR: Failed to initialize PHY: %d\n", ret);
      emac_ifdown(dev);
      return ret;
    }

  netdev_carrier_on(dev);

  return 0;
}

/****************************************************************************
 * Name: emac_ifdown
 ****************************************************************************/

static int emac_ifdown(struct net_driver_s *dev)
{
  struct esp_emac_s *priv = NET2PRIV(dev);
  irqstate_t flags;

  ninfo("Taking the network down\n");

  flags = enter_critical_section();

  /* Disable EMAC interrupt */

  up_disable_irq(priv->cpuint);

  /* Stop EMAC */

  emac_hal_stop(&priv->hal);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->txtimeout);

  /* Disable bus clock */

  emac_ll_enable_bus_clock(0, false);

  /* Free DMA resources */

  emac_deinit_dma(priv);

  /* Mark the device "down" */

  priv->ifup = false;

  leave_critical_section(flags);

  netdev_carrier_off(dev);

  return 0;
}

/****************************************************************************
 * Name: emac_txavail
 ****************************************************************************/

static int emac_txavail(struct net_driver_s *dev)
{
  struct esp_emac_s *priv = NET2PRIV(dev);

  if (work_available(&priv->pollwork))
    {
      work_queue(EMACWORK, &priv->pollwork, emac_txavail_work, priv, 0);
    }

  return 0;
}

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)

/****************************************************************************
 * Name: emac_addmac
 ****************************************************************************/

static int emac_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct esp_emac_s *priv = NET2PRIV(dev);

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* TODO: implement proper hash-based multicast filtering.
   * For now, enable pass-all-multicast so that IPv6 NDP,
   * mDNS and other multicast protocols work.
   */

  emac_ll_enable_recv_all_multicast(priv->hal.mac_regs, true);
  return 0;
}

/****************************************************************************
 * Name: emac_rmmac
 ****************************************************************************/

static int emac_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* TODO: implement proper hash-based multicast filtering.
   * We leave pass-all-multicast enabled since we cannot track
   * the remaining group memberships without a reference count.
   */

  return 0;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_emac_init
 ****************************************************************************/

int esp_emac_init(void)
{
  struct esp_emac_s *priv = &g_emac;
  int ret;

  memset(priv, 0, sizeof(struct esp_emac_s));

  /* Setup interrupt */

  priv->cpuint = esp_setup_irq(ETS_ETH_MAC_INTR_SOURCE,
                               ESP_IRQ_PRIORITY_DEFAULT,
                               ESP_IRQ_TRIGGER_LEVEL,
                               emac_interrupt,
                               priv);
  if (priv->cpuint < 0)
    {
      nerr("ERROR: Failed to allocate EMAC interrupt\n");
      return priv->cpuint;
    }

  /* Initialize the driver structure */

  priv->dev.d_buf     = NULL;
  priv->dev.d_ifup    = emac_ifup;
  priv->dev.d_ifdown  = emac_ifdown;
  priv->dev.d_txavail = emac_txavail;
#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
  priv->dev.d_addmac  = emac_addmac;
  priv->dev.d_rmmac   = emac_rmmac;
#endif
  priv->dev.d_private = priv;

  /* Read base MAC address from eFuse and derive the Ethernet MAC.
   * ESP32-P4 stores a 48-bit factory MAC in two eFuse registers.
   * Convention: Ethernet MAC = base MAC with last octet + 3.
   */

  uint32_t mac0 = efuse_ll_get_mac0();
  uint32_t mac1 = efuse_ll_get_mac1();
  uint8_t *mac = priv->dev.d_mac.ether.ether_addr_octet;

  mac[0] = (mac0 >> 0) & 0xff;
  mac[1] = (mac0 >> 8) & 0xff;
  mac[2] = (mac0 >> 16) & 0xff;
  mac[3] = (mac0 >> 24) & 0xff;
  mac[4] = (mac1 >> 0) & 0xff;
  mac[5] = ((mac1 >> 8) & 0xff) + 3;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Register the device with the OS */

  ret = netdev_register(&priv->dev, NET_LL_ETHERNET);
  if (ret != 0)
    {
      nerr("ERROR: netdev_register failed: %d\n", ret);
      esp_teardown_irq(ETS_ETH_MAC_INTR_SOURCE, priv->cpuint);
      return ret;
    }

  ninfo("ESP EMAC driver initialized\n");
  return 0;
}
