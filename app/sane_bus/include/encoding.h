/*
 * Copyright (C) 2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    Defines for encoding settings
 * @ingroup     app_sane
 * @brief       Encoding format settings
 * @{
 *
 * @file
 * @brief       Literals for encoding settings (e.g. JSON/CBOR)
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */

#ifndef ENCODING_H
#define ENCODING_H

typedef enum {
    ENCODING_JSON,
    ENCODING_CBOR,
} encoding_format_t;

#ifdef __cplusplus
}
#endif

#endif /* ENCODING_H */
/** @} */
