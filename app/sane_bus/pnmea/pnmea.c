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
#include <string.h>
#include <stdio.h>

int pnmea_format_cmd(char *str, size_t out_sz, char *mid, int *params, size_t nparams) {
    size_t cur_len = 0;
    cur_len += snprintf(str, out_sz, "$%s", mid);
    int left;
    for (unsigned i = 0; i < nparams; i++) {
        left = out_sz - cur_len;
        int n = snprintf(&str[cur_len], left, ",%d", params[i]);
        if (n >= left) {
            return -1;
        }
        else {
            cur_len += n;
        }
    }

    uint8_t chksum = 0;
    for (unsigned i = 1; i < cur_len; i++) {
        chksum ^= str[i];
    }
    cur_len += snprintf(&str[cur_len], out_sz - cur_len, "*%X\r\n", chksum);

    if (cur_len >= out_sz){
        return -1;
    }
    return cur_len;
}
