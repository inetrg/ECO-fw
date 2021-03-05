/*
 * Copyright (C) 2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    coap_util module to help encoding various data to cbor
 * @ingroup     app_sane
 * @brief       Utility module to ecode data to cbor
 * @{
 *
 * @file
 * @brief       Interface to cbor encoding helpers
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */

#ifndef CBOR_UTIL_H
#define CBOR_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "eco.h"
#include "control_messages.h"
#include "sds011.h"
#include "minmea_helper.h"

enum value_type {
    TYPE_VOLTAGE,
    TYPE_CURRENT,
    TYPE_POWER,
    TYPE_ENERGY,
    TYPE_CURRENT_CHARGED,
    TYPE_VOLTAGE_LP,
    TYPE_CURRENT_LP,
    TYPE_PM10,
    TYPE_PM2_5,
    TYPE_PRESSURE,
    TYPE_TEMPERATURE,
    TYPE_HUMIDITY,
};

const char *cbor_util_type_to_str(uint8_t unit);

int encode_phydat_to_cbor(phydat_t *data, uint8_t dim, uint8_t type,
                          const char *unit, uint8_t *cbor_buf, size_t cbor_len);

int cbor_util_eco_encode(eco_t *eco, uint8_t *cbor_buf, size_t cbor_len);
int cbor_util_eco_encode_compact(eco_t *eco, uint8_t *cbor_buf, size_t cbor_len);

int cbor_util_sds011_encode(sds011_data_t *sds_data, uint8_t *cbor_buf, size_t cbor_len);

int cbor_util_bmx_encode(bmx_measure_t *m, uint8_t *cbor_buf, size_t cbor_len);

int cbor_util_energy_attr_encode(eco_task_consumption_spec_t *tx_task_spec,
                                 eco_task_consumption_spec_t *gps_task_spec,
                                 eco_task_consumption_spec_t *dust_task_spec,
                                 eco_task_consumption_spec_t *env_task_spec,
                                 eco_measurement_set_t *avg,
                                 uint8_t *cbor_buf, size_t cbor_len);

int cbor_util_sense_encode(sds011_data_t *sds, bmx_measure_t *bmx,
                            gps_data_t *gps, uint32_t cycle_cnt,
                            uint8_t *cbor_buf, size_t cbor_len);

#ifdef __cplusplus
}
#endif

#endif /* CBOR_UTIL_H */
/** @} */
