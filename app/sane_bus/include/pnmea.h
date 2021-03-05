/*
 * Copyright (C) 2018 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     app
 * @{
 *
 * @file
 * @brief       comfort functions for proprietary nmea commands
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */

 #ifndef PNMEA_H
 #define PNMEA_H

 #ifdef __cplusplus
 extern "C" {
 #endif

#define PNMEA_MAX_CMD_LEN (80)

typedef struct {
    char *cmdname;
    int  *params;
    int  param_num;
} pnmea_cmd_t;

int pnmea_format_cmd(char *str, size_t out_sz, char *mid, int *params, size_t nparams);

#ifdef __cplusplus
}
#endif

#endif /* PNMEA_H */
/** @} */
