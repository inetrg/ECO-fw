/*
 * Copyright (C) 2018 Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
* @defgroup    gpio_wakeup
* @ingroup     app
* @brief       A helper module to ease usage of low-power gpio wakeup pins
* @{
*
* @file
* @brief       Definitions for the gpio_wakeup helper module
*
* @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
*/
#ifndef GPIO_WAKEUP_H
#define GPIO_WAKEUP_H

#include "periph/gpio.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* following values are only tested for the stm32-l476rg but probably
   apply to others */
#if defined(CPU_FAM_STM32L4)
#define WKUP_PIN_NUM (5U)

#define WKUP_IDX_1 (1)
#define WKUP_IDX_2 (2)
#define WKUP_IDX_3 (3)
#define WKUP_IDX_4 (4)
#define WKUP_IDX_5 (5)

static inline uint8_t _get_wkup_idx(gpio_t pin){
    switch(pin){
        case GPIO_PIN(0,0):  return WKUP_IDX_1;
        case GPIO_PIN(2,13): return WKUP_IDX_2;
        case GPIO_PIN(4,6):  return WKUP_IDX_3;
        case GPIO_PIN(0,2):  return WKUP_IDX_4;
        case GPIO_PIN(2,5):  return WKUP_IDX_5;
    }
    return 0;
}
#endif

/**
 * @brief   configures push/pull-resistors via the power control registers
 *          NOTE: this is required in addition to the periph_gpio configuration
 *          of pull-up/down resistors
 *
 *
 * @param[in] pin                   GPIO pin to to configure power port pull for
 *                                  Pin name is WKUP1 matches int value of 1
 * @param[in] config                GPIO_IN_PD = pull-down, GPIO_IN_PU = pull-up,
 *                                  GPIO_IN = disabled
 *
 * @return  < 0 on error
 */
int gpio_wakeup_pull_config(gpio_t pin, gpio_mode_t mode);
int gpio_wakeup_enable(gpio_t pin, gpio_flank_t flank);
int gpio_wakeup_disable(gpio_t pin);
int gpio_wakeup_clear(gpio_t pin);

#ifdef __cplusplus
} /* end extern "C" */
#endif

#endif /* GPIO_WAKEUP_H */
/** @} */
