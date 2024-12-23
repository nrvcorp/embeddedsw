/******************************************************************************
* Copyright (C) 2017 - 2020 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
 ******************************************************************************/


#ifndef __PLATFORM_H_
#define __PLATFORM_H_

#include "platform_config.h"

/* Platform timer is calibrated for 250 ms, so kept interval value 4 to call
 * eth_link_detect() at every one second
 */
#define ETH_LINK_DETECT_INTERVAL 4

void init_platform(void);
void cleanup_platform(void);

#endif
