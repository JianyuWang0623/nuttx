/****************************************************************************
 * include/nuttx/usb/cdcncm.h
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

#ifndef __INCLUDE_NUTTX_USB_CDCNCM_H
#define __INCLUDE_NUTTX_USB_CDCNCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_CDCNCM_COMPOSITE
#  include <nuttx/usb/composite.h>
#endif

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

#define CDCNCM_EP_INTIN_IDX      (0)
#define CDCNCM_EP_BULKIN_IDX     (1)
#define CDCNCM_EP_BULKOUT_IDX    (2)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: cdcncm_initialize / cdcmbim_initialize
 *
 * Description:
 *   Register CDC_NCM/MBIM USB device interface. Register the corresponding
 *   network driver to NuttX and bring up the network.
 *
 * Input Parameters:
 *   minor - Device minor number.
 *   handle - An optional opaque reference to the CDC_NCM/MBIM class object
 *   that may subsequently be used with cdcncm_uninitialize().
 *
 * Returned Value:
 *   Zero (OK) means that the driver was successfully registered.  On any
 *   failure, a negated errno value is returned.
 *
 ****************************************************************************/

#ifndef CONFIG_CDCNCM_COMPOSITE
int cdcncm_initialize(int minor, FAR void **handle);
#  ifdef CONFIG_NET_CDCMBIM
int cdcmbim_initialize(int minor, FAR void **handle);
#  endif
#endif

/****************************************************************************
 * Name: cdcncm_get_composite_devdesc / cdcmbim_get_composite_devdesc
 *
 * Description:
 *   Helper function to fill in some constants into the composite
 *   configuration struct.
 *
 * Input Parameters:
 *     dev - Pointer to the configuration struct we should fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CDCNCM_COMPOSITE
void cdcncm_get_composite_devdesc(FAR struct composite_devdesc_s *dev);
#  ifdef CONFIG_NET_CDCMBIM
void cdcmbim_get_composite_devdesc(FAR struct composite_devdesc_s *dev);
#  endif
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_CDCNCM_H */
