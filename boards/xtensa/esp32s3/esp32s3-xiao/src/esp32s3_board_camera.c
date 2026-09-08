/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-xiao/src/esp32s3_board_camera.c
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

#include <nuttx/arch.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/v4l2_cap.h>
#include <nuttx/video/ov3660.h>

#include "esp32s3_i2c.h"
#include "esp32s3_cam.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_camera_initialize
 *
 * Description:
 *   Initialize the camera subsystem for the Seeed XIAO ESP32-S3 Sense
 *   expansion board (camera-equipped units only, see board README):
 *
 *   1. Register the ESP32-S3 CAM (DVP) imgdata driver.  This also starts
 *      the XCLK output that the OV3660 needs before it will respond on
 *      its SCCB bus.
 *   2. Bring up the dedicated SCCB I2C bus (ESP32S3_I2C1, wired to
 *      GPIO39/40 on this board's camera slot) and initialize the
 *      OV3660 sensor on it.  This is a different I2C peripheral than
 *      ESP32S3_I2C0, which board_i2c_init()/esp32s3_bringup() use for
 *      the XIAO main I2C header pins (D4/D5) -- the two buses must
 *      not be merged onto the same pins.
 *   3. Register imgdata + imgsensor globally, then call
 *      capture_initialize() to create /dev/video0 so that applications
 *      (e.g. nxcamera) can use it directly without having to call
 *      capture_initialize() themselves.
 *
 *   The Sense expansion board wires the OV3660 PWDN/RESET pins to fixed
 *   levels (no GPIO control), so this function does not drive any
 *   power-down/reset lines.
 *
 ****************************************************************************/

int esp32s3_camera_initialize(void)
{
  struct imgdata_s *imgdata;
  struct imgsensor_s *imgsensor;
  struct i2c_master_s *i2c;
  int ret;

  /* Step 1: Register ESP32-S3 CAM imgdata driver (also starts XCLK). */

  imgdata = esp32s3_cam_initialize();
  if (!imgdata)
    {
      snerr("ERROR: Failed to register CAM imgdata\n");
      return -ENODEV;
    }

  up_mdelay(10);  /* Wait for XCLK to stabilize before SCCB access */

  /* Step 2: Initialize OV3660 sensor via the dedicated SCCB I2C bus. */

  i2c = esp32s3_i2cbus_initialize(1);
  if (!i2c)
    {
      snerr("ERROR: Failed to initialize SCCB I2C bus\n");
      return -ENODEV;
    }

  imgsensor = ov3660_initialize(i2c, 0, 0);
  if (!imgsensor)
    {
      snerr("ERROR: Failed to initialize OV3660\n");
      return -ENODEV;
    }

  /* Step 3: Register imgdata and imgsensor, then create /dev/video0. */

  imgdata_register(imgdata);
  ret = imgsensor_register(imgsensor);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register imgsensor: %d\n", ret);
      return ret;
    }

  ret = capture_initialize("/dev/video0");
  if (ret < 0)
    {
      snerr("ERROR: Failed to create /dev/video0: %d\n", ret);
      return ret;
    }

  sninfo("Camera drivers registered\n");
  return OK;
}
