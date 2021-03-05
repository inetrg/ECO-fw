/*
 * Copyright (C) 2019 Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
* @defgroup    coap
* @ingroup     app_sane
* @brief       A helper module to ease usage of coap
* @{
*
* @file
* @brief       Definitions for coap helper module
*
* @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
*/
#ifndef COAP_H
#define COAP_H

#include "net/gcoap.h"

int coap_request(unsigned code, char *addr_str, char *port_str, char *path, uint8_t *data, size_t len, bool confirm);
int coap_request_blocking(unsigned code, char *addr_str, char *port_str, char *path, uint8_t *data, size_t len, int timeout);
int coap_post_blocking(char *path, char *addr_str, char *json, size_t json_len);

#ifdef __cplusplus
 extern "C" {
#endif


#ifdef __cplusplus
} /* end extern "C" */
#endif

#endif /* COAP_H */
/** @} */
