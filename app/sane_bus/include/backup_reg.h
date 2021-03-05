/*
 * Copyright (C) 2018 Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
* @defgroup    backup_reg
* @ingroup     app
* @brief       A helper module to ease usage of backup registers
* @{
*
* @file
* @brief       Definitions for the backup_reg helper module
*
* @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
*/
#ifndef BACKUP_REG_H
#define BACKUP_REG_H

/* following values are only tested for the stm32-l476rg but probably
   apply to others */
#if defined(CPU_FAM_STM32L4)
#define RTC_WRITE_PROTECTION_KEY1        (0xCA)
#define RTC_WRITE_PROTECTION_KEY2        (0x53)

#define RTC_WRITE_PROTECTION_WRONG_KEY   (0xFF)

#define BACKUP_REG_COUNT                 (32)
#endif

#define SYS_STATE_BOOTSTRAPPED                 (0x01)
#define SYS_STATE_WAITING_TIME_DRIFT           (0x02)
#define SYS_STATE_WAITING_FOR_ECO_INTERRUPT    (0x04)
#define SYS_STATE_RTC_RESYNCED                 (0x08)
#define SYS_STATE_LORA_JOINED                  (0x10)
#define SYS_STATE_RTC_SYNCED                   (0x20)
#define SYS_STATE_HAD_GPS_FIX                  (0x40)
#define SYS_STATE_GRACEFULL_SHUTDOWN_REQUESTED (0x80)


int backup_reg_read(uint32_t idx, uint32_t *data);
int backup_reg_write(uint32_t idx, uint32_t val);
int backup_reg_write_bytes(uint32_t byte_idx, uint8_t *data, size_t cnt);
int backup_reg_read_bytes(uint32_t byte_idx, uint8_t *data, size_t cnt);
int backup_reg_write_u64(uint32_t idx, uint64_t val);
int backup_reg_read_u64(uint32_t idx, uint64_t *val);

#ifdef __cplusplus
 extern "C" {
#endif


#ifdef __cplusplus
} /* end extern "C" */
#endif

#endif /* BACKUP_REG_H */
/** @} */
