/*
 * Copyright (C) 2018 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_common_stm32
 * @{
 *
 * @file
 * @brief       Common configuration for STM32 I2C
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

#ifndef CFG_I2C3_PC0_PC1_H
#define CFG_I2C3_PC0_PC1_H

#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name I2C configuration
 * @{
 */
static const i2c_conf_t i2c_config[] = {
    {
        //.dev            = I2C1,
        .dev            = I2C3,
        .speed          = I2C_SPEED_FAST,
        //.scl_pin        = GPIO_PIN(PORT_B, 8),
        //.sda_pin        = GPIO_PIN(PORT_B, 9),
        .scl_pin        = GPIO_PIN(PORT_C, 0),
        .sda_pin        = GPIO_PIN(PORT_C, 1),
#if CPU_FAM_STM32F0
        .scl_af         = GPIO_AF1,
        .sda_af         = GPIO_AF1,
#else
        .scl_af         = GPIO_AF4,
        .sda_af         = GPIO_AF4,
#endif
        .bus            = APB1,
#if CPU_FAM_STM32F4 || CPU_FAM_STM32F2
        .rcc_mask       = RCC_APB1ENR_I2C1EN,
        .clk            = CLOCK_APB1,
        .irqn           = I2C1_EV_IRQn,
#elif CPU_FAM_STM32L4
        .rcc_mask       = RCC_APB1ENR1_I2C3EN,
        .irqn           = I2C3_ER_IRQn,
#elif CPU_FAM_STM32F7
        .rcc_mask       = RCC_APB1ENR_I2C1EN,
        .irqn           = I2C1_ER_IRQn,
#elif CPU_FAM_STM32F0
        .rcc_mask       = RCC_APB1ENR_I2C1EN,
        .rcc_sw_mask    = RCC_CFGR3_I2C1SW,
        .irqn           = I2C1_IRQn,
#endif
    }
};

#if CPU_FAM_STM32F4 || CPU_FAM_STM32F2
#define I2C_0_ISR           isr_i2c1_ev
#elif CPU_FAM_STM32L4 || CPU_FAM_STM32F7
#define I2C_0_ISR           isr_i2c3_er
#elif CPU_FAM_STM32F0
#define I2C_0_ISR           isr_i2c1
#endif

#define I2C_NUMOF           (sizeof(i2c_config) / sizeof(i2c_config[0]))
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CFG_I2C3_PC0_PC1_H */
/** @} */
