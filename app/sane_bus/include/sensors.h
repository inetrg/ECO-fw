/*
 * Copyright (C) 2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sensors sane bus app sensors module
 * @ingroup     app_sane
 * @brief       Utility module to integrate various sensors to the sane app
 * @{
 *
 * @file
 * @brief       Interface to control power supply and sensing of various sensors
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */

#ifndef APP_SANE_BUS_SENSORS_H
#define APP_SANE_BUS_SENSORS_H

#include "kernel_types.h"

#ifdef __cplusplus
extern "C" {
#endif

extern kernel_pid_t sensing_thread_pid;

//#define SENSORS_DSDS011_SUPPLY_RISE_TIME_US (1000 * US_PER_MS)
#define SENSORS_DSDS011_WARMUP_TIME_MS   (20000)
#define SENSORS_SDS011_AVG_CNT               (5)

kernel_pid_t sensors_init_thread(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SANE_BUS_SENSORS_H */
/** @} */
