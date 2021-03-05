/*
 * Copyright (C) 2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    control_messages literal values for controlling module functions
 * @ingroup     app_sane
 * @brief       Helpers for sending messages to other modules threads
 * @{
 *
 * @file
 * @brief       Literal values used within control messages to other threads
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */

#ifndef CONTROL_MESSAGES_H
#define CONTROL_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#define CBOR_MAX_BUFFER_LEN (192)

#define CTLMSG_TYPE1_MASK                (0x0F)
#define CTLMSG_TYPE1(X)                  (X & CTLMSG_TYPE1_MASK)
#define CTLMSG_TYPE1_DEBUG               (0x01)
#define CTLMSG_TYPE1_DUST_SENSOR         (0x02)
#define CTLMSG_TYPE1_POWER_MEASURE       (0x03)
#define CTLMSG_TYPE1_DATA_BUFF           (0x04)
#define CTLMSG_TYPE1_RADIO               (0x05)
#define CTLMSG_TYPE1_GPS_SENSOR          (0x06)
#define CTLMSG_TYPE1_ENV_SENSOR          (0x07)


#define CTLMSG_TYPE2_MASK                (0xF0)
#define CTLMSG_TYPE2(X)                  (X & CTLMSG_TYPE2_MASK)
#define CTLMSG_TYPE2_INIT                (0x10)
#define CTLMSG_TYPE2_MEASURE             (0x20)
#define CTLMSG_TYPE2_MEASURE_SINGLE_LONG (0x30)
#define CTLMSG_TYPE2_MEASURE_CONTINUOUS  (0x40)
#define CTLMSG_TYPE2_POWER_ON            (0x50)
#define CTLMSG_TYPE2_POWER_OFF           (0x60)
#define CTLMSG_TYPE2_DATA_RAW            (0x70)
#define CTLMSG_TYPE2_DATA_STRING         (0x80)
#define CTLMSG_TYPE2_DATA_PHYDAT         (0x90)
#define CTLMSG_TYPE2_DATA_CBOR           (0xA0)
#define CTLMSG_TYPE2_REQUEST_AVG         (0xB0)
#define CTLMSG_TYPE2_REQUEST_READ        (0xC0)
#define CTLMSG_TYPE2_NOTIFY_POWER_DOWN   (0xD0)
#define CTLMSG_TYPE2_NOTIFY_TASK_START   (0xE0)
#define CTLMSG_TYPE2_NOTIFY_TASK_STOP    (0xF0)

#define CTLMSG_ARG_FORCE_COLD_START      (0x01)

typedef struct {
    uint8_t* buf;
    size_t   len;
} buffer_descriptor_t;

typedef struct {
    uint8_t  buf[CBOR_MAX_BUFFER_LEN];
    size_t   len;
} cbor_buffer_t;

typedef struct {
    int16_t  tmp;
    uint32_t press;
    uint16_t hum;
} bmx_measure_t;

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_MESSAGES_H */
/** @} */
