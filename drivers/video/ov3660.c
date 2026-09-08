/****************************************************************************
 * drivers/video/ov3660.c
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

#include <sys/param.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/arch.h>
#include <nuttx/video/video.h>
#include <nuttx/video/ov3660.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OV3660_I2C_ADDR         0x3c    /* SCCB address (0x78 >> 1) */
#define OV3660_I2C_FREQ         100000

/* Chip identification (16-bit registers) */

#define OV3660_REG_CHIP_ID_H    0x300a
#define OV3660_REG_CHIP_ID_L    0x300b
#define OV3660_CHIP_ID_H_VAL    0x36
#define OV3660_CHIP_ID_L_VAL    0x60

/* System control */

#define OV3660_REG_SYS_CTRL0    0x3008  /* Bit7: reset, Bit6: power down */
#define OV3660_SYS_CTRL0_RESET  0x82
#define OV3660_SYS_CTRL0_SLEEP  0x42
#define OV3660_SYS_CTRL0_WAKE   0x02

#define OV3660_REG_DRIVE_CAP    0x302c
#define OV3660_REG_CLOCK_POL    0x4740

/* PLL control (SC_PLLS_*) */

#define OV3660_REG_PLLS_CTRL0   0x303a
#define OV3660_REG_PLLS_CTRL1   0x303b
#define OV3660_REG_PLLS_CTRL2   0x303c
#define OV3660_REG_PLLS_CTRL3   0x303d
#define OV3660_REG_PCLK_RATIO   0x3824
#define OV3660_REG_VFIFO_CTRL0C 0x460c

/* Mirror / flip (TIMING_TC_REGxx) */

#define OV3660_REG_TC_REG20     0x3820  /* Vflip + vertical binning */
#define OV3660_REG_TC_REG21     0x3821  /* Hmirror + horizontal binning */
#define OV3660_REG_BLC_CTRL     0x4514
#define OV3660_REG_VFIFO_CTRL   0x4520
#define OV3660_REG_X_INCREMENT  0x3814
#define OV3660_REG_Y_INCREMENT  0x3815

/* Windowing / output size (16-bit pairs, high reg first) */

#define OV3660_REG_X_ADDR_ST_H    0x3800
#define OV3660_REG_X_ADDR_END_H   0x3804
#define OV3660_REG_X_OUTPUT_SZ_H  0x3808
#define OV3660_REG_X_TOTAL_SZ_H   0x380c
#define OV3660_REG_X_OFFSET_H     0x3810

/* ISP top control */

#define OV3660_REG_ISP_CTRL01   0x5001  /* Bit5: scale enable */
#define OV3660_ISP_CTRL01_SCALE 0x20

/* Output format control */

#define OV3660_REG_FORMAT_CTRL  0x501f
#define OV3660_REG_FORMAT_CTRL00 0x4300

/* Sensor native (full) resolution and 4:3 crop window, taken from the
 * OV3660 "4x3" ratio table (espressif/esp32-camera sensors/ov3660.c).
 * Only the sensor's binned output modes (<= 1024x768) are supported by
 * this driver; full-resolution readout is not implemented.
 */

#define OV3660_NATIVE_WIDTH     2048
#define OV3660_NATIVE_HEIGHT    1536
#define OV3660_CROP_END_X       2079
#define OV3660_CROP_END_Y       1547
#define OV3660_TOTAL_X          2300
#define OV3660_TOTAL_Y          1564
#define OV3660_BINNED_OFFSET_X  8
#define OV3660_BINNED_OFFSET_Y  2

#define OV3660_MAX_WIDTH        (OV3660_NATIVE_WIDTH / 2)   /* 1024 */
#define OV3660_MAX_HEIGHT       (OV3660_NATIVE_HEIGHT / 2)  /* 768 */

#define OV3660_DEFAULT_WIDTH    320
#define OV3660_DEFAULT_HEIGHT   240

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ov3660_reg_s
{
  uint16_t addr;
  uint8_t val;
};

struct ov3660_dev_s
{
  struct imgsensor_s sensor;
  struct i2c_master_s *i2c;
  uint16_t width;
  uint16_t height;
  uint32_t pixelformat;
  struct v4l2_frmsizeenum frmsizes;
  bool hmirror;
  bool vflip;
  bool streaming;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool ov3660_is_available(struct imgsensor_s *sensor);
static int ov3660_init(struct imgsensor_s *sensor);
static int ov3660_uninit(struct imgsensor_s *sensor);
static const char *ov3660_get_driver_name(struct imgsensor_s *sensor);
static int ov3660_validate_frame_setting(struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         imgsensor_format_t *datafmts,
                                         imgsensor_interval_t *interval);
static int ov3660_start_capture(struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                imgsensor_format_t *datafmts,
                                imgsensor_interval_t *interval);
static int ov3660_stop_capture(struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type);
static int ov3660_get_supported_value(struct imgsensor_s *sensor,
                                      uint32_t id,
                                      imgsensor_supported_value_t *value);
static int ov3660_get_value(struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t *value);
static int ov3660_set_value(struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* OV3660 default initialization register table, taken from the
 * "sensor_default_regs" table in espressif/esp32-camera
 * (sensors/private_include/ov3660_settings.h).  This is the proven
 * register set used by every OV3660-based ESP32 camera module; it
 * configures AWB/AEC/lens-shading/gamma and leaves the sensor in its
 * native 2048x1536 4:3 readout mode with YUV422 output.  The actual
 * output size and pixel format are programmed afterwards by
 * ov3660_set_framesize() / ov3660_set_pixformat().
 */

static const struct ov3660_reg_s g_ov3660_init_regs[] =
{
  { 0x3103, 0x13 },
  { OV3660_REG_SYS_CTRL0, 0x42 },
  { 0x3017, 0xff },
  { 0x3018, 0xff },
  { OV3660_REG_DRIVE_CAP, 0xc3 },
  { OV3660_REG_CLOCK_POL, 0x21 },
  { 0x3611, 0x01 },
  { 0x3612, 0x2d },
  { 0x3032, 0x00 },
  { 0x3614, 0x80 },
  { 0x3618, 0x00 },
  { 0x3619, 0x75 },
  { 0x3622, 0x80 },
  { 0x3623, 0x00 },
  { 0x3624, 0x03 },
  { 0x3630, 0x52 },
  { 0x3632, 0x07 },
  { 0x3633, 0xd2 },
  { 0x3704, 0x80 },
  { 0x3708, 0x66 },
  { 0x3709, 0x12 },
  { 0x370b, 0x12 },
  { 0x3717, 0x00 },
  { 0x371b, 0x60 },
  { 0x371c, 0x00 },
  { 0x3901, 0x13 },
  { 0x3600, 0x08 },
  { 0x3620, 0x43 },
  { 0x3702, 0x20 },
  { 0x3739, 0x48 },
  { 0x3730, 0x20 },
  { 0x370c, 0x0c },
  { 0x3a18, 0x00 },
  { 0x3a19, 0xf8 },
  { 0x3000, 0x10 },
  { 0x3004, 0xef },
  { 0x6700, 0x05 },
  { 0x6701, 0x19 },
  { 0x6702, 0xfd },
  { 0x6703, 0xd1 },
  { 0x6704, 0xff },
  { 0x6705, 0xff },
  { 0x3c01, 0x80 },
  { 0x3c00, 0x04 },
  { 0x3a08, 0x00 },  /* 50Hz banding step (10bit) */
  { 0x3a09, 0x62 },
  { 0x3a0e, 0x08 },  /* 50Hz max bands per frame (6bit) */
  { 0x3a0a, 0x00 },  /* 60Hz banding step (10bit) */
  { 0x3a0b, 0x52 },
  { 0x3a0d, 0x09 },  /* 60Hz max bands per frame (6bit) */
  { 0x3a00, 0x3a },  /* Night mode off */
  { 0x3a14, 0x09 },
  { 0x3a15, 0x30 },
  { 0x3a02, 0x09 },
  { 0x3a03, 0x30 },
  { 0x440e, 0x08 },  /* COMPRESSION_CTRL0E */
  { OV3660_REG_VFIFO_CTRL, 0x0b },
  { 0x460b, 0x37 },
  { 0x4713, 0x02 },
  { 0x471c, 0xd0 },
  { 0x5086, 0x00 },
  { 0x5002, 0x00 },
  { OV3660_REG_FORMAT_CTRL, 0x00 },
  { OV3660_REG_SYS_CTRL0, 0x02 },
  { 0x5180, 0xff },
  { 0x5181, 0xf2 },
  { 0x5182, 0x00 },
  { 0x5183, 0x14 },
  { 0x5184, 0x25 },
  { 0x5185, 0x24 },
  { 0x5186, 0x16 },
  { 0x5187, 0x16 },
  { 0x5188, 0x16 },
  { 0x5189, 0x68 },
  { 0x518a, 0x60 },
  { 0x518b, 0xe0 },
  { 0x518c, 0xb2 },
  { 0x518d, 0x42 },
  { 0x518e, 0x35 },
  { 0x518f, 0x56 },
  { 0x5190, 0x56 },
  { 0x5191, 0xf8 },
  { 0x5192, 0x04 },
  { 0x5193, 0x70 },
  { 0x5194, 0xf0 },
  { 0x5195, 0xf0 },
  { 0x5196, 0x03 },
  { 0x5197, 0x01 },
  { 0x5198, 0x04 },
  { 0x5199, 0x12 },
  { 0x519a, 0x04 },
  { 0x519b, 0x00 },
  { 0x519c, 0x06 },
  { 0x519d, 0x82 },
  { 0x519e, 0x38 },
  { 0x5381, 0x1d },  /* Color matrix */
  { 0x5382, 0x60 },
  { 0x5383, 0x03 },
  { 0x5384, 0x0c },
  { 0x5385, 0x78 },
  { 0x5386, 0x84 },
  { 0x5387, 0x7d },
  { 0x5388, 0x6b },
  { 0x5389, 0x12 },
  { 0x538a, 0x01 },
  { 0x538b, 0x98 },
  { 0x5480, 0x01 },  /* Gamma */
  { 0x5000, 0xa7 },
  { 0x5800, 0x0c },  /* Lens shading correction table */
  { 0x5801, 0x09 },
  { 0x5802, 0x0c },
  { 0x5803, 0x0c },
  { 0x5804, 0x0d },
  { 0x5805, 0x17 },
  { 0x5806, 0x06 },
  { 0x5807, 0x05 },
  { 0x5808, 0x04 },
  { 0x5809, 0x06 },
  { 0x580a, 0x09 },
  { 0x580b, 0x0e },
  { 0x580c, 0x05 },
  { 0x580d, 0x01 },
  { 0x580e, 0x01 },
  { 0x580f, 0x01 },
  { 0x5810, 0x05 },
  { 0x5811, 0x0d },
  { 0x5812, 0x05 },
  { 0x5813, 0x01 },
  { 0x5814, 0x01 },
  { 0x5815, 0x01 },
  { 0x5816, 0x05 },
  { 0x5817, 0x0d },
  { 0x5818, 0x08 },
  { 0x5819, 0x06 },
  { 0x581a, 0x05 },
  { 0x581b, 0x07 },
  { 0x581c, 0x0b },
  { 0x581d, 0x0d },
  { 0x581e, 0x12 },
  { 0x581f, 0x0d },
  { 0x5820, 0x0e },
  { 0x5821, 0x10 },
  { 0x5822, 0x10 },
  { 0x5823, 0x1e },
  { 0x5824, 0x53 },
  { 0x5825, 0x15 },
  { 0x5826, 0x05 },
  { 0x5827, 0x14 },
  { 0x5828, 0x54 },
  { 0x5829, 0x25 },
  { 0x582a, 0x33 },
  { 0x582b, 0x33 },
  { 0x582c, 0x34 },
  { 0x582d, 0x16 },
  { 0x582e, 0x24 },
  { 0x582f, 0x41 },
  { 0x5830, 0x50 },
  { 0x5831, 0x42 },
  { 0x5832, 0x15 },
  { 0x5833, 0x25 },
  { 0x5834, 0x34 },
  { 0x5835, 0x33 },
  { 0x5836, 0x24 },
  { 0x5837, 0x26 },
  { 0x5838, 0x54 },
  { 0x5839, 0x25 },
  { 0x583a, 0x15 },
  { 0x583b, 0x25 },
  { 0x583c, 0x53 },
  { 0x583d, 0xcf },
  { 0x3a0f, 0x30 },  /* AEC */
  { 0x3a10, 0x28 },
  { 0x3a1b, 0x30 },
  { 0x3a1e, 0x28 },
  { 0x3a11, 0x60 },
  { 0x3a1f, 0x14 },
  { 0x5302, 0x28 },  /* Sharpness */
  { 0x5303, 0x20 },
  { 0x5306, 0x1c },  /* De-noise offset 1 */
  { 0x5307, 0x28 },  /* De-noise offset 2 */
  { 0x4002, 0xc5 },
  { 0x4003, 0x81 },
  { 0x4005, 0x12 },
  { 0x5688, 0x11 },
  { 0x5689, 0x11 },
  { 0x568a, 0x11 },
  { 0x568b, 0x11 },
  { 0x568c, 0x11 },
  { 0x568d, 0x11 },
  { 0x568e, 0x11 },
  { 0x568f, 0x11 },
  { 0x5580, 0x06 },
  { 0x5588, 0x00 },
  { 0x5583, 0x40 },
  { 0x5584, 0x2c },
  { OV3660_REG_ISP_CTRL01, 0x83 },  /* Enable color matrix, AWB and SDE */
};

static const struct imgsensor_ops_s g_ov3660_ops =
{
  .is_available           = ov3660_is_available,
  .init                   = ov3660_init,
  .uninit                 = ov3660_uninit,
  .get_driver_name        = ov3660_get_driver_name,
  .validate_frame_setting = ov3660_validate_frame_setting,
  .start_capture          = ov3660_start_capture,
  .stop_capture           = ov3660_stop_capture,
  .get_supported_value    = ov3660_get_supported_value,
  .get_value              = ov3660_get_value,
  .set_value              = ov3660_set_value,
};

static const struct v4l2_fmtdesc g_ov3660_fmtdescs[] =
{
  {
    .pixelformat = V4L2_PIX_FMT_YUYV,
    .description = "YUV 4:2:2 (YUYV)",
  },
  {
    .pixelformat = V4L2_PIX_FMT_RGB565X,
    .description = "RGB565X (BE)",
  },
};

static const struct v4l2_frmivalenum g_ov3660_frmintervals[] =
{
  {
    .type = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete =
      {
        .numerator   = 1,
        .denominator = 10,
      },
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov3660_putreg
 *
 * Description:
 *   Write an 8-bit value to a 16-bit OV3660 (SCCB) register.
 *
 ****************************************************************************/

static int ov3660_putreg(struct i2c_master_s *i2c,
                         uint16_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msg;
  uint8_t buf[3];
  int ret;

  buf[0] = (regaddr >> 8) & 0xff;
  buf[1] = regaddr & 0xff;
  buf[2] = regval;

  msg.frequency = OV3660_I2C_FREQ;
  msg.addr      = OV3660_I2C_ADDR;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 3;

  ret = I2C_TRANSFER(i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("ERROR: I2C write to 0x%04x failed: %d\n", regaddr, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ov3660_getreg
 *
 * Description:
 *   Read an 8-bit value from a 16-bit OV3660 (SCCB) register.
 *
 ****************************************************************************/

static int ov3660_getreg(struct i2c_master_s *i2c,
                         uint16_t regaddr, uint8_t *regval)
{
  struct i2c_msg_s msg[2];
  uint8_t addrbuf[2];
  int ret;

  addrbuf[0] = (regaddr >> 8) & 0xff;
  addrbuf[1] = regaddr & 0xff;

  msg[0].frequency = OV3660_I2C_FREQ;
  msg[0].addr      = OV3660_I2C_ADDR;
  msg[0].flags     = 0;
  msg[0].buffer    = addrbuf;
  msg[0].length    = 2;

  msg[1].frequency = OV3660_I2C_FREQ;
  msg[1].addr      = OV3660_I2C_ADDR;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C read from 0x%04x failed: %d\n", regaddr, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ov3660_modreg
 *
 * Description:
 *   Read-modify-write an 8-bit register.
 *
 ****************************************************************************/

static int ov3660_modreg(struct i2c_master_s *i2c, uint16_t regaddr,
                         uint8_t clearbits, uint8_t setbits)
{
  uint8_t regval;
  int ret;

  ret = ov3660_getreg(i2c, regaddr, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval = (regval & ~clearbits) | setbits;

  return ov3660_putreg(i2c, regaddr, regval);
}

/****************************************************************************
 * Name: ov3660_write16
 *
 * Description:
 *   Write a 16-bit value across a register and its immediate successor
 *   (high byte first), as used by the X/Y address, size and offset
 *   register pairs.
 *
 ****************************************************************************/

static int ov3660_write16(struct i2c_master_s *i2c,
                          uint16_t regaddr, uint16_t val)
{
  int ret;

  ret = ov3660_putreg(i2c, regaddr, (val >> 8) & 0xff);
  if (ret < 0)
    {
      return ret;
    }

  return ov3660_putreg(i2c, regaddr + 1, val & 0xff);
}

/****************************************************************************
 * Name: ov3660_write_addr_reg
 *
 * Description:
 *   Write an X/Y register quad: regaddr/regaddr+1 = xval (16-bit),
 *   regaddr+2/regaddr+3 = yval (16-bit).  Matches the layout of the
 *   OV3660 X_ADDR_ST/X_ADDR_END/X_OUTPUT_SIZE/X_TOTAL_SIZE/X_OFFSET
 *   register groups.
 *
 ****************************************************************************/

static int ov3660_write_addr_reg(struct i2c_master_s *i2c,
                                 uint16_t regaddr,
                                 uint16_t xval, uint16_t yval)
{
  int ret;

  ret = ov3660_write16(i2c, regaddr, xval);
  if (ret < 0)
    {
      return ret;
    }

  return ov3660_write16(i2c, regaddr + 2, yval);
}

/****************************************************************************
 * Name: ov3660_putreglist
 ****************************************************************************/

static int ov3660_putreglist(struct i2c_master_s *i2c,
                             const struct ov3660_reg_s *reglist,
                             size_t nentries)
{
  size_t i;
  int ret;

  for (i = 0; i < nentries; i++)
    {
      ret = ov3660_putreg(i2c, reglist[i].addr, reglist[i].val);
      if (ret < 0)
        {
          snerr("OV3660 write[%d] 0x%04x=0x%02x FAILED: %d\n",
                (int)i, reglist[i].addr, reglist[i].val, ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ov3660_set_pixformat
 ****************************************************************************/

static int ov3660_set_pixformat(struct ov3660_dev_s *priv, uint32_t pixfmt)
{
  int ret;

  if (pixfmt == IMGSENSOR_PIX_FMT_RGB565X)
    {
      /* RGB565.  The sensor clocks out the high byte first on the 8-bit
       * DVP bus, so the resulting in-memory layout is big-endian
       * (RGB565X), same reasoning as the GC0308 driver.
       */

      ret = ov3660_putreg(priv->i2c, OV3660_REG_FORMAT_CTRL, 0x01);
      ret |= ov3660_putreg(priv->i2c, OV3660_REG_FORMAT_CTRL00, 0x61);
    }
  else
    {
      /* Default: YUV422 (YUYV) */

      ret = ov3660_putreg(priv->i2c, OV3660_REG_FORMAT_CTRL, 0x00);
      ret |= ov3660_putreg(priv->i2c, OV3660_REG_FORMAT_CTRL00, 0x30);
    }

  if (ret == OK)
    {
      priv->pixelformat = pixfmt;
    }

  return ret;
}

/****************************************************************************
 * Name: ov3660_set_image_options
 *
 * Description:
 *   Program the mirror/flip and (always-on, see ov3660_set_framesize())
 *   binning related registers.  Mirrors set_image_options() in
 *   espressif/esp32-camera sensors/ov3660.c, simplified for the
 *   always-binned case used by this driver.
 *
 ****************************************************************************/

static int ov3660_set_image_options(struct ov3660_dev_s *priv)
{
  uint8_t reg20 = 0x01;  /* Vertical binning always enabled */
  uint8_t reg21 = 0x01;  /* Horizontal binning always enabled */
  uint8_t reg4514;
  uint8_t testval = 4;   /* Binning bit, per vendor truth table */
  int ret;

  if (priv->vflip)
    {
      reg20 |= 0x06;
      testval |= 1;
    }

  if (priv->hmirror)
    {
      reg21 |= 0x06;
      testval |= 2;
    }

  switch (testval)
    {
      case 5:
      case 6:
        reg4514 = 0xbb;
        break;

      case 4:
      case 7:
      default:
        reg4514 = 0xaa;
        break;
    }

  ret = ov3660_putreg(priv->i2c, OV3660_REG_TC_REG20, reg20);
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_putreg(priv->i2c, OV3660_REG_TC_REG21, reg21);
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_putreg(priv->i2c, OV3660_REG_BLC_CTRL, reg4514);
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_putreg(priv->i2c, OV3660_REG_VFIFO_CTRL, 0x0b);
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_putreg(priv->i2c, OV3660_REG_X_INCREMENT, 0x31);
  if (ret < 0)
    {
      return ret;
    }

  return ov3660_putreg(priv->i2c, OV3660_REG_Y_INCREMENT, 0x31);
}

/****************************************************************************
 * Name: ov3660_set_pll
 *
 * Description:
 *   Program the OV3660 PLLS chain and manual PCLK divider.  Bypass and
 *   PLL root-doubler are not used by this driver (always false).
 *
 ****************************************************************************/

static int ov3660_set_pll(struct i2c_master_s *i2c, uint8_t multiplier,
                          uint8_t sys_div, uint8_t pre_div, uint8_t seld5,
                          uint8_t pclk_div)
{
  int ret;

  ret = ov3660_putreg(i2c, OV3660_REG_PLLS_CTRL0, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_putreg(i2c, OV3660_REG_PLLS_CTRL1, multiplier & 0x1f);
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_putreg(i2c, OV3660_REG_PLLS_CTRL2,
                      0x10 | (sys_div & 0x0f));
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_putreg(i2c, OV3660_REG_PLLS_CTRL3,
                      ((pre_div & 0x3) << 4) | (seld5 & 0x3));
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_putreg(i2c, OV3660_REG_PCLK_RATIO, pclk_div & 0x1f);
  if (ret < 0)
    {
      return ret;
    }

  /* Manual PCLK divider enable */

  return ov3660_putreg(i2c, OV3660_REG_VFIFO_CTRL0C, 0x22);
}

/****************************************************************************
 * Name: ov3660_set_framesize
 *
 * Description:
 *   Program the sensor crop window, DVP output size and PLL/PCLK for the
 *   requested resolution.  Only the sensor's binned 4:3 output modes
 *   (width <= 1024, height <= 768) are supported.
 *
 ****************************************************************************/

static int ov3660_set_framesize(struct ov3660_dev_s *priv,
                                uint16_t width, uint16_t height)
{
  bool scale;
  int ret;

  if (width == 0 || height == 0)
    {
      width  = OV3660_DEFAULT_WIDTH;
      height = OV3660_DEFAULT_HEIGHT;
    }

  if (width > OV3660_MAX_WIDTH || height > OV3660_MAX_HEIGHT)
    {
      snerr("ERROR: OV3660 only supports binned output <= %ux%u\n",
            OV3660_MAX_WIDTH, OV3660_MAX_HEIGHT);
      return -EINVAL;
    }

  /* DCW scaling is needed unless the output size exactly matches the
   * binned sensor array size (1024x768).
   */

  scale = !(width == OV3660_MAX_WIDTH && height == OV3660_MAX_HEIGHT);

  ret = ov3660_write_addr_reg(priv->i2c, OV3660_REG_X_ADDR_ST_H, 0, 0);
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_write_addr_reg(priv->i2c, OV3660_REG_X_ADDR_END_H,
                              OV3660_CROP_END_X, OV3660_CROP_END_Y);
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_write_addr_reg(priv->i2c, OV3660_REG_X_OUTPUT_SZ_H,
                              width, height);
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_write_addr_reg(priv->i2c, OV3660_REG_X_TOTAL_SZ_H,
                              OV3660_TOTAL_X, (OV3660_TOTAL_Y / 2) + 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_write_addr_reg(priv->i2c, OV3660_REG_X_OFFSET_H,
                              OV3660_BINNED_OFFSET_X,
                              OV3660_BINNED_OFFSET_Y);
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_modreg(priv->i2c, OV3660_REG_ISP_CTRL01,
                      OV3660_ISP_CTRL01_SCALE,
                      scale ? OV3660_ISP_CTRL01_SCALE : 0x00);
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_set_image_options(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* PCLK/SYSCLK tuning, grouped by output size (mirrors the thresholds
   * used by set_framesize() in espressif/esp32-camera sensors/ov3660.c).
   */

  if (width > 480)
    {
      /* >= HVGA: 8MHz SYSCLK / 8MHz PCLK */

      ret = ov3660_set_pll(priv->i2c, 4, 1, 0, 2, 2);
    }
  else if (width >= 320)
    {
      /* QVGA class: 16MHz SYSCLK / 8MHz PCLK */

      ret = ov3660_set_pll(priv->i2c, 8, 1, 0, 2, 4);
    }
  else
    {
      /* Smaller than QVGA: 32MHz SYSCLK / 8MHz PCLK */

      ret = ov3660_set_pll(priv->i2c, 8, 1, 0, 0, 8);
    }

  if (ret == OK)
    {
      priv->width  = width;
      priv->height = height;
    }

  return ret;
}

/****************************************************************************
 * Name: ov3660_is_available
 ****************************************************************************/

static bool ov3660_is_available(struct imgsensor_s *sensor)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;
  uint8_t idh = 0;
  uint8_t idl = 0;
  int ret;

  ret = ov3660_getreg(priv->i2c, OV3660_REG_CHIP_ID_H, &idh);
  ret |= ov3660_getreg(priv->i2c, OV3660_REG_CHIP_ID_L, &idl);
  if (ret < 0)
    {
      return false;
    }

  sninfo("OV3660 chip ID: 0x%02x%02x (expected 0x%02x%02x)\n",
         idh, idl, OV3660_CHIP_ID_H_VAL, OV3660_CHIP_ID_L_VAL);

  return (idh == OV3660_CHIP_ID_H_VAL && idl == OV3660_CHIP_ID_L_VAL);
}

/****************************************************************************
 * Name: ov3660_init
 ****************************************************************************/

static int ov3660_init(struct imgsensor_s *sensor)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;
  int ret;

  /* Software reset, then wait for it to complete */

  ret = ov3660_putreg(priv->i2c, OV3660_REG_SYS_CTRL0,
                      OV3660_SYS_CTRL0_RESET);
  if (ret < 0)
    {
      snerr("OV3660 soft reset failed: %d\n", ret);
      return ret;
    }

  up_mdelay(10);

  /* Write the vendor default initialization register table */

  ret = ov3660_putreglist(priv->i2c, g_ov3660_init_regs,
                          nitems(g_ov3660_init_regs));
  if (ret < 0)
    {
      snerr("OV3660 init regs failed: %d\n", ret);
      return ret;
    }

  up_mdelay(10);

  ret = ov3660_set_pixformat(priv, priv->pixelformat);
  if (ret < 0)
    {
      snerr("OV3660 set pixformat failed: %d\n", ret);
      return ret;
    }

  ret = ov3660_set_framesize(priv, priv->width, priv->height);
  if (ret < 0)
    {
      snerr("OV3660 set framesize failed: %d\n", ret);
      return ret;
    }

  sninfo("OV3660 init done: %ux%u pixfmt=%u\n",
         priv->width, priv->height, (unsigned int)priv->pixelformat);

  return OK;
}

/****************************************************************************
 * Name: ov3660_uninit
 ****************************************************************************/

static int ov3660_uninit(struct imgsensor_s *sensor)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;

  /* Soft reset back to the sensor's power-on default state */

  ov3660_putreg(priv->i2c, OV3660_REG_SYS_CTRL0, OV3660_SYS_CTRL0_RESET);
  priv->streaming = false;

  return OK;
}

/****************************************************************************
 * Name: ov3660_get_driver_name
 ****************************************************************************/

static const char *ov3660_get_driver_name(struct imgsensor_s *sensor)
{
  return "OV3660";
}

/****************************************************************************
 * Name: ov3660_validate_frame_setting
 ****************************************************************************/

static int ov3660_validate_frame_setting(struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         imgsensor_format_t *datafmts,
                                         imgsensor_interval_t *interval)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;

  if (nr_datafmts < 1 || !datafmts)
    {
      return -EINVAL;
    }

  if (datafmts[IMGSENSOR_FMT_MAIN].pixelformat !=
      IMGSENSOR_PIX_FMT_YUYV &&
      datafmts[IMGSENSOR_FMT_MAIN].pixelformat !=
      IMGSENSOR_PIX_FMT_RGB565X)
    {
      return -EINVAL;
    }

  if (datafmts[IMGSENSOR_FMT_MAIN].width != priv->width ||
      datafmts[IMGSENSOR_FMT_MAIN].height != priv->height)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: ov3660_start_capture
 ****************************************************************************/

static int ov3660_start_capture(struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                imgsensor_format_t *datafmts,
                                imgsensor_interval_t *interval)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;
  int ret;

  if (priv->streaming)
    {
      return -EBUSY;
    }

  ret = ov3660_set_pixformat(priv, datafmts[IMGSENSOR_FMT_MAIN].pixelformat);
  if (ret < 0)
    {
      return ret;
    }

  priv->streaming = true;
  return OK;
}

/****************************************************************************
 * Name: ov3660_stop_capture
 ****************************************************************************/

static int ov3660_stop_capture(struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;

  priv->streaming = false;
  return OK;
}

/****************************************************************************
 * Name: ov3660_get_supported_value
 ****************************************************************************/

static int ov3660_get_supported_value(struct imgsensor_s *sensor,
                                      uint32_t id,
                                      imgsensor_supported_value_t *value)
{
  switch (id)
    {
      case IMGSENSOR_ID_HFLIP_VIDEO:
      case IMGSENSOR_ID_HFLIP_STILL:
      case IMGSENSOR_ID_VFLIP_VIDEO:
      case IMGSENSOR_ID_VFLIP_STILL:
        value->type = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        value->u.range.minimum       = 0;
        value->u.range.maximum       = 1;
        value->u.range.step          = 1;
        value->u.range.default_value = 0;
        return OK;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: ov3660_get_value
 ****************************************************************************/

static int ov3660_get_value(struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t *value)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;

  switch (id)
    {
      case IMGSENSOR_ID_HFLIP_VIDEO:
      case IMGSENSOR_ID_HFLIP_STILL:
        value->value32 = priv->hmirror ? 1 : 0;
        return OK;

      case IMGSENSOR_ID_VFLIP_VIDEO:
      case IMGSENSOR_ID_VFLIP_STILL:
        value->value32 = priv->vflip ? 1 : 0;
        return OK;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: ov3660_set_value
 ****************************************************************************/

static int ov3660_set_value(struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t value)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;

  switch (id)
    {
      case IMGSENSOR_ID_HFLIP_VIDEO:
      case IMGSENSOR_ID_HFLIP_STILL:
        priv->hmirror = value.value32 ? true : false;
        return ov3660_set_image_options(priv);

      case IMGSENSOR_ID_VFLIP_VIDEO:
      case IMGSENSOR_ID_VFLIP_STILL:
        priv->vflip = value.value32 ? true : false;
        return ov3660_set_image_options(priv);

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov3660_initialize
 *
 * Description:
 *   Initialize the OV3660 camera sensor driver.
 *
 * Input Parameters:
 *   i2c    - I2C (SCCB) bus device
 *   width  - Desired frame width  (0 = QVGA 320)
 *   height - Desired frame height (0 = QVGA 240)
 *
 * Returned Value:
 *   Pointer to imgsensor_s on success; NULL on failure.
 *
 ****************************************************************************/

struct imgsensor_s *ov3660_initialize(struct i2c_master_s *i2c,
                                      uint16_t width,
                                      uint16_t height)
{
  struct ov3660_dev_s *priv;

  if (!i2c)
    {
      return NULL;
    }

  priv = kmm_zalloc(sizeof(struct ov3660_dev_s));
  if (!priv)
    {
      return NULL;
    }

  priv->i2c         = i2c;
  priv->streaming   = false;
  priv->hmirror     = false;
  priv->vflip       = false;
  priv->pixelformat = IMGSENSOR_PIX_FMT_YUYV;
  priv->width       = width  ? width  : OV3660_DEFAULT_WIDTH;
  priv->height      = height ? height : OV3660_DEFAULT_HEIGHT;

  priv->frmsizes.type             = V4L2_FRMSIZE_TYPE_DISCRETE;
  priv->frmsizes.discrete.width   = priv->width;
  priv->frmsizes.discrete.height  = priv->height;

  priv->sensor.ops              = &g_ov3660_ops;
  priv->sensor.fmtdescs         = g_ov3660_fmtdescs;
  priv->sensor.fmtdescs_num     = nitems(g_ov3660_fmtdescs);
  priv->sensor.frmsizes         = &priv->frmsizes;
  priv->sensor.frmsizes_num     = 1;
  priv->sensor.frmintervals     = g_ov3660_frmintervals;
  priv->sensor.frmintervals_num = nitems(g_ov3660_frmintervals);

  return &priv->sensor;
}
