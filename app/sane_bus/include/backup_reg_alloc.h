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
* @brief       Global allocation of available Backup registers
* @{
*
* @file
* @brief       Static allocation of backup registers
*
* @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
*/
#ifndef BACKUP_REG_ALLOC_H
#define BACKUP_REG_ALLOC_H

//TODO registers could be shared for multiple values
#define BACKUP_REG_SYS_STATE                   (0)
//#define BACKUP_REG_CYCLE_CNT                   (1)

#define BAKCUP_REG_LAST_SYNC                   (2)
#define BAKCUP_REG_POWER_DOWN_UTC_MS_H         (3)
#define BAKCUP_REG_POWER_DOWN_UTC_MS_L         (4)

#define BACKUP_REG_LAST_MEASUREMENT_UTC_MS_H   (5)
#define BACKUP_REG_LAST_MEASUREMENT_UTC_MS_L   (6)


#define BACKUP_REG_NEXT_TX_ALLOWED_1           (7)
#define BACKUP_REG_NEXT_TX_ALLOWED_2           (8)

#define BACKUP_REG_LORA_PREV_DR                (13)



/* LORA state ---------------------------vvvvvvvvvv */
#define BACKUP_REG_LORA_0                      (14)
#define BACKUP_REG_LORA_MAX                    (31)
/* LORA state ---------------------------^^^^^^^^^^ */

#ifdef __cplusplus
 extern "C" {
#endif


#ifdef __cplusplus
} /* end extern "C" */
#endif

#endif /* BACKUP_REG_ALLOC_H */
/** @} */
