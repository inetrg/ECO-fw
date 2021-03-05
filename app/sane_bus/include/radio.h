/*
 * Copyright (C) 2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    radio module to take low level control over the radio(s)
 * @ingroup     app_sane
 * @brief       Utility module to control power and send data
 * @{
 *
 * @file
 * @brief       Interface to control radio(s)
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */

#ifndef RADIO_H
#define RADIO_H

#include "kernel_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LORA_STATE_FLAG_JOINED (0x01)
#define RADIO_LORA_DATARATE_DEFAULT (5)
#define RADIO_LORA_DATARATE_MIN     (2)
#define RADIO_LORA_DATARATE_MAX     (5)


extern kernel_pid_t radio_thread_pid;

kernel_pid_t radio_init_thread(void);
void radio_quick_power_down(void);

#ifdef __cplusplus
}
#endif

#endif /* RADIO_H */
/** @} */
