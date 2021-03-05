/*
 * Copyright (C) 2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cbor_util
 * @{
 *
 * @file
 * @brief       Cbor encoding helpers
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */
 #include <stdio.h>
 #include <math.h>

 #include "xtimer.h"
 #include "phydat.h"
 #include "saul_reg.h"
 //#include "net/gcoap.h"
 #include "cbor_util.h"
 #include "cbor.h"
 #define LOG_LEVEL LOG_NONE
 #include "log.h"

#include "net/gnrc/ipv6/nib.h"

#define SENSOR_SAMPLE_INTERVAL_US    (1LU * US_PER_SEC)

#define CBOR_KEY_STRING              "type"
#define CBOR_VAL_STRING              "value"
#define CBOR_UNIT_STRING             "unit"

#define CBOR_U_KEY                   "U"
#define CBOR_I_CHG_KEY               "Ic"
#define CBOR_P_CHG_AVG_KEY           "Pca"
#define CBOR_E_CYC_LATEST_KEY        "Ecl"
#define CBOR_E_CYC_AVG_KEY           "Eca"
#define CBOR_E_SLOT_SUM_KEY          "Es"
#define CBOR_T_PROC_KEY              "te"
#define CBOR_T_SENSE_KEY             "ts"
#define CBOR_TEMP_KEY                "T"
#define CBOR_PRESS_KEY               "p"
#define CBOR_HUM_KEY                 "rH"
#define CBOR_PM2_5_KEY               "PM2.5"
#define CBOR_PM10_KEY                "PM10"

#define CBOR_E_ATTR_GPS_KEY          "Eg"
#define CBOR_E_ATTR_ENV_KEY          "Ee"
#define CBOR_E_ATTR_TX_KEY           "Et"
#define CBOR_E_ATTR_DUST_KEY         "Ed"

#define CBOR_GPS_LAT_KEY             "la"
#define CBOR_GPS_LON_KEY             "lo"
#define CBOR_GPS_HEIGHT_KEY          "h"
#define CBOR_GPS_DEG_KEY             "d"
#define CBOR_GPS_SPEED_KEY           "v"
#define CBOR_GPS_SATS_KEY            "s"
#define CBOR_GPS_SATS_TRACKED_KEY    "st"
#define CBOR_GPS_TTF_KEY             "ttf"

#define CBOR_N_CYCLE_KEY             "nc"
#define CBOR_N_READ_ERR_KEY          "nre"
#define CBOR_N_ALERT_ERR_KEY         "nae"
#define CBOR_N_INIT_ERR_KEY          "nie"

#define CBOR_COMP_MSG_TYPE_ECO       0
#define CBOR_COMP_MSG_TYPE_SENSE     1

 #define REMOTE_SENSORDATA_RESSOURCE  "/v1/sensordata"

const char *cbor_util_type_to_str(uint8_t type)
 {
     switch (type) {
         case TYPE_VOLTAGE:          return "U";
         case TYPE_CURRENT:          return "I";
         case TYPE_POWER:            return "P";
         case TYPE_ENERGY:           return "E";
         case TYPE_CURRENT_CHARGED:  return "Ic";
         case TYPE_VOLTAGE_LP:       return "Uo";
         case TYPE_CURRENT_LP:       return "Io";
         case TYPE_PM10:             return "PM10";
         case TYPE_PM2_5:            return "PM2.5";
         case TYPE_PRESSURE:         return "p";
         case TYPE_TEMPERATURE:      return "T";
         case TYPE_HUMIDITY :        return "H";
         default:                    return "";
     }
 }

static void _map_append_float(CborEncoder *e, const char *key, float val)
{
    cbor_encode_text_stringz(e, key);
    cbor_encode_float(e, val);
    LOG_INFO("\"%s\": \"%f\", ", key, val);
}

static void _map_append_uint(CborEncoder *e, const char *key, uint32_t val)
{
    cbor_encode_text_stringz(e, key);
    cbor_encode_uint(e, val);
    LOG_INFO("\"%s\": \"%"PRIu32"\", ", key, val);
}

static void _map_append_int(CborEncoder *e, const char *key, int32_t val)
{
    cbor_encode_text_stringz(e, key);
    cbor_encode_int(e, val);
    LOG_INFO("\"%s\": \"%"PRId32"\", ", key, val);
}

static void _map_append_uint_as_float(CborEncoder *e, const char *key, uint32_t val,
                                  float scale)
{
    float fval = (float)val * scale ;
    _map_append_float(e, key, fval);
}

static void _map_append_int_as_float(CborEncoder *e, const char *key, int32_t val,
                                 float scale)
{
    float fval = (float)val * scale ;
    _map_append_float(e, key, fval);
}

size_t cbor_util_gps_encode(gps_data_t *data, uint8_t *cbor_buf, size_t cbor_len)
{
    CborEncoder encoder, mapEncoder;
    cbor_encoder_init(&encoder, cbor_buf, cbor_len, 0);
    if( cbor_encoder_create_map(&encoder, &mapEncoder, 8) != CborNoError ||
        cbor_encode_text_stringz(&mapEncoder, "la") != CborNoError ||
        cbor_encode_float(&mapEncoder, minmea_tocoord(&data->latitude)) != CborNoError ||
        cbor_encode_text_stringz(&mapEncoder, "lo") != CborNoError ||
        cbor_encode_float(&mapEncoder, minmea_tocoord(&data->longitude)) != CborNoError ||
        cbor_encode_text_stringz(&mapEncoder, "h") != CborNoError ||
        cbor_encode_float(&mapEncoder, minmea_tofloat(&data->height)) != CborNoError ||
        cbor_encode_text_stringz(&mapEncoder, "d") != CborNoError ||
        cbor_encode_float(&mapEncoder, minmea_tofloat(&data->true_track_degrees)) != CborNoError ||
        cbor_encode_text_stringz(&mapEncoder, "s") != CborNoError ||
        cbor_encode_float(&mapEncoder, minmea_tofloat(&data->speed_kph)) != CborNoError ||
        cbor_encode_text_stringz(&mapEncoder, "vs") != CborNoError ||
        cbor_encode_int(&mapEncoder, data->visible_sats) != CborNoError ||
        cbor_encode_text_stringz(&mapEncoder, "ts") != CborNoError ||
        cbor_encode_int(&mapEncoder, data->tracked_sats) != CborNoError ||
        cbor_encode_text_stringz(&mapEncoder, "ttf") != CborNoError ||
        cbor_encode_int(&mapEncoder, data->ttf) != CborNoError ||
        cbor_encoder_close_container(&encoder, &mapEncoder) != CborNoError) {
        return 0;
    }
    return cbor_encoder_get_buffer_size(&encoder, cbor_buf);
}

int cbor_util_eco_encode(eco_t *eco, uint8_t *cbor_buf, size_t cbor_len)
{
    CborEncoder encoder, mapEncoder;
    LOG_DEBUG("cbor_util_eco_to_cbor\n");

    LOG_INFO("{ ");
    cbor_encoder_init(&encoder, cbor_buf, cbor_len, 0);

    cbor_encoder_create_map(&encoder, &mapEncoder, CborIndefiniteLength);

    /* append this information only if we already collected enough data */
    if (eco->history.e_lp_measure_slot_cnt > 0) {
        _map_append_uint_as_float(&mapEncoder, CBOR_U_KEY, eco->history.u_latest_mv, 0.001);
        _map_append_int_as_float(&mapEncoder, CBOR_I_CHG_KEY, eco->history.i_chg_latest_uA, 0.000001);
    }

    if (eco->history.e_use_slot_cnt > 0) {
        _map_append_int_as_float(&mapEncoder, CBOR_E_CYC_LATEST_KEY, eco->history.e_use_latest_uws, 0.000001);
    }

    _map_append_int_as_float(&mapEncoder, CBOR_P_CHG_AVG_KEY, eco->history.p_chg_mavg_uw, 0.001);

    _map_append_int_as_float(&mapEncoder, CBOR_E_CYC_AVG_KEY, eco->history.e_use_mavg_uws, 0.000001);

    /* TODO: replace thins mapping with something more dynamic to let the
             application spcify names etc. for the task */
    eco_task_consumption_spec_t *gps_task_spec = &eco->prev_task_specs[0];
    eco_task_consumption_spec_t *dust_task_spec = &eco->prev_task_specs[1];
    eco_task_consumption_spec_t *env_task_spec = &eco->prev_task_specs[2];
    eco_task_consumption_spec_t *tx_task_spec = &eco->prev_task_specs[3];

    if (gps_task_spec->used) {
        _map_append_int(&mapEncoder, CBOR_E_ATTR_GPS_KEY,
                        (int32_t)(gps_task_spec->e_pws / 1000000));
    }

    if (dust_task_spec->used) {
        _map_append_int(&mapEncoder, CBOR_E_ATTR_DUST_KEY,
                        (int32_t)(dust_task_spec->e_pws / 1000000));
    }

    if (env_task_spec->used) {
        _map_append_int(&mapEncoder, CBOR_E_ATTR_ENV_KEY,
                        (int32_t)(env_task_spec->e_pws / 1000000));
    }

    if (tx_task_spec->used) {
        _map_append_int(&mapEncoder, CBOR_E_ATTR_TX_KEY,
                        (int32_t)(tx_task_spec->e_pws / 1000000));
    }

    /*TODO: change below values to integer encoded type */
    _map_append_uint_as_float(&mapEncoder, CBOR_E_SLOT_SUM_KEY,
                              (uint32_t)eco->history.e_chg_slot_sum_uws / 1000,
                              0.001);

    _map_append_uint_as_float(&mapEncoder, CBOR_T_SENSE_KEY,
                              (uint32_t)eco->sensing_period_s,
                              1.0);

    _map_append_uint_as_float(&mapEncoder, CBOR_T_PROC_KEY,
                              (uint32_t)eco->eco_processing_period_s,
                              1.0);

    _map_append_uint(&mapEncoder, CBOR_N_CYCLE_KEY, eco->cycle_cnt);
    //_map_append_uint(&mapEncoder, CBOR_N_READ_ERR_KEY, eco->reg_read_err_cnt);
    //_map_append_uint(&mapEncoder, CBOR_N_ALERT_ERR_KEY, eco->no_alert_err_cnt);
    //_map_append_uint(&mapEncoder, CBOR_N_INIT_ERR_KEY, eco->init_err_cnt);

    cbor_encoder_close_container(&encoder, &mapEncoder);
    LOG_INFO(" }\n");
    return cbor_encoder_get_buffer_size(&encoder, cbor_buf);
}

static void _append_nulls(CborEncoder *e, unsigned cnt) {
    for (unsigned i = 0; i < cnt; i++) {
        cbor_encode_null(e);
    }
}


int cbor_util_eco_encode_compact(eco_t *eco, uint8_t *cbor_buf, size_t cbor_len)
{
    CborEncoder encoder, arrayEncoder;
    LOG_DEBUG("cbor_util_eco_encode_compact\n");

    LOG_INFO("{ ");
    cbor_encoder_init(&encoder, cbor_buf, cbor_len, 0);

    cbor_encoder_create_array(&encoder, &arrayEncoder, 14);

    cbor_encode_int(&arrayEncoder, CBOR_COMP_MSG_TYPE_ECO);

    /* append this information only if we already collected enough data */
    if (eco->history.e_lp_measure_slot_cnt > 0) {
        cbor_encode_int(&arrayEncoder, eco->history.u_latest_mv); // CBOR_U_KEY
        cbor_encode_int(&arrayEncoder, eco->history.i_chg_latest_uA); // CBOR_I_CHG_KEY
    }
    else {
        _append_nulls(&arrayEncoder, 2);
    }

    if (eco->history.e_use_slot_cnt > 0) {
        //CBOR_E_CYC_LATEST_KEY
        cbor_encode_int(&arrayEncoder, eco->history.e_use_latest_uws);
    }
    else {
        _append_nulls(&arrayEncoder, 1);
    }

    // CBOR_P_CHG_AVG_KEY
    cbor_encode_int(&arrayEncoder, eco->history.p_chg_mavg_uw);

    // CBOR_E_CYC_AVG_KEY
    cbor_encode_int(&arrayEncoder, eco->history.e_use_mavg_uws);

    /* TODO: replace thins mapping with something more dynamic to let the
             application spcify names etc. for the task */
    eco_task_consumption_spec_t *gps_task_spec = &eco->prev_task_specs[0];
    eco_task_consumption_spec_t *dust_task_spec = &eco->prev_task_specs[1];
    eco_task_consumption_spec_t *env_task_spec = &eco->prev_task_specs[2];
    eco_task_consumption_spec_t *tx_task_spec = &eco->prev_task_specs[3];

    if (gps_task_spec->used) {
        /* CBOR_E_ATTR_GPS_KEY */
        cbor_encode_int(&arrayEncoder,
                        (int32_t)(gps_task_spec->e_pws / 1000000));
    }
    else {
        _append_nulls(&arrayEncoder, 1);
    }

    if (dust_task_spec->used) {
        /* CBOR_E_ATTR_DUST_KEY */
        cbor_encode_int(&arrayEncoder,
                        (int32_t)(dust_task_spec->e_pws / 1000000));
    }
    else {
        _append_nulls(&arrayEncoder, 1);
    }

    if (env_task_spec->used) {
        /* CBOR_E_ATTR_ENV_KEY */
        cbor_encode_int(&arrayEncoder,
                        (int32_t)(env_task_spec->e_pws / 1000000));
    }
    else {
        _append_nulls(&arrayEncoder, 1);
    }

    if (tx_task_spec->used) {
        /* CBOR_E_ATTR_TX_KEY */
        cbor_encode_int(&arrayEncoder,
                        (int32_t)(tx_task_spec->e_pws / 1000000));
    }
    else {
        _append_nulls(&arrayEncoder, 1);
    }

    /* CBOR_E_SLOT_SUM_KEY */
    cbor_encode_float(&arrayEncoder,
                     (eco->history.e_chg_slot_sum_uws / 1000) * 0.001);

    cbor_encode_uint(&arrayEncoder, eco->sensing_period_s); //CBOR_T_SENSE_KEY

    cbor_encode_uint(&arrayEncoder, eco->eco_processing_period_s); //CBOR_T_PROC_KEY

    /* CBOR_N_CYCLE_KEY */
    cbor_encode_uint(&arrayEncoder, eco->cycle_cnt);
    //_map_append_uint(&mapEncoder, CBOR_N_READ_ERR_KEY, eco->reg_read_err_cnt);
    //_map_append_uint(&mapEncoder, CBOR_N_ALERT_ERR_KEY, eco->no_alert_err_cnt);
    //_map_append_uint(&mapEncoder, CBOR_N_INIT_ERR_KEY, eco->init_err_cnt);

    cbor_encoder_close_container(&encoder, &arrayEncoder);
    LOG_INFO(" }\n");
    return cbor_encoder_get_buffer_size(&encoder, cbor_buf);
}

/* reduced size by not sending in self descibing format
message_type [1]
Cycle count  [unsigned count]
PM2.5        [1/10 µg/m^3]
PM10         [1/10 µg/m^3]
Temperatur   [1/100 °C]
Luftdruck    [Pascal]
Humidity     [1/100 %relative humidity]
Latitude     [float °]
Longitude    [float °]
TrackDegree  [float °]
Velocity     [float km/h]
TimeToFix    [1/10 s]
VisibleSats  [unsigned count]
TrackedSats  [unsigned count]
*/
int cbor_util_sense_encode(sds011_data_t *sds, bmx_measure_t *bmx,
                           gps_data_t *gps, uint32_t cycle_cnt,
                           uint8_t *cbor_buf, size_t cbor_len) {
    CborEncoder encoder, arrayEncoder;
    LOG_DEBUG("cbor_util_sense_encode\n");

    LOG_INFO("{ ");
    cbor_encoder_init(&encoder, cbor_buf, cbor_len, 0);

    cbor_encoder_create_array(&encoder, &arrayEncoder, 14);

    cbor_encode_int(&arrayEncoder, CBOR_COMP_MSG_TYPE_SENSE);

    cbor_encode_uint(&arrayEncoder, cycle_cnt);

    if (sds != NULL) {
        cbor_encode_uint(&arrayEncoder, sds->pm_2_5);
        cbor_encode_uint(&arrayEncoder, sds->pm_10);
    }
    else {
        _append_nulls(&arrayEncoder, 2);
    }

    if (bmx != NULL) {
        cbor_encode_int(&arrayEncoder, bmx->tmp);
        cbor_encode_uint(&arrayEncoder, bmx->press);
        cbor_encode_uint(&arrayEncoder, bmx->hum);
    }
    else {
        _append_nulls(&arrayEncoder, 3);
    }

    if (gps != NULL) {
        cbor_encode_float(&arrayEncoder, minmea_tocoord(&gps->latitude));
        cbor_encode_float(&arrayEncoder, minmea_tocoord(&gps->longitude));
        //cbor_encode_float(&arrayEncoder, minmea_tofloat(&gps->height));
        cbor_encode_float(&arrayEncoder, minmea_tofloat(&gps->true_track_degrees));
        cbor_encode_float(&arrayEncoder, minmea_tofloat(&gps->speed_kph));
        cbor_encode_int(&arrayEncoder, gps->ttf / 100000);
        cbor_encode_int(&arrayEncoder, gps->visible_sats);
        cbor_encode_int(&arrayEncoder, gps->tracked_sats);
    }
    else {
        _append_nulls(&arrayEncoder, 7);
    }

    if (cbor_encoder_close_container(&encoder, &arrayEncoder) == CborNoError) {
        return cbor_encoder_get_buffer_size(&encoder, cbor_buf);
    }

    return 0;
}

int cbor_util_sds011_encode(sds011_data_t *sds_data, uint8_t *cbor_buf, size_t cbor_len)
{
    CborEncoder encoder, mapEncoder;
    LOG_DEBUG("cbor_util_sds011_encode\n");

    LOG_INFO("{ ");
    cbor_encoder_init(&encoder, cbor_buf, cbor_len, 0);

    cbor_encoder_create_map(&encoder, &mapEncoder, 2);

    _map_append_int_as_float(&mapEncoder, CBOR_PM2_5_KEY, sds_data->pm_2_5, 0.1);
    _map_append_int_as_float(&mapEncoder, CBOR_PM10_KEY, sds_data->pm_10, 0.1);

    cbor_encoder_close_container(&encoder, &mapEncoder);
    LOG_INFO(" }\n");
    return cbor_encoder_get_buffer_size(&encoder, cbor_buf);
}

int cbor_util_bmx_encode(bmx_measure_t *m, uint8_t *cbor_buf, size_t cbor_len)
{
    CborEncoder encoder, mapEncoder;
    LOG_DEBUG("cbor_util_bmx_encode\n");

    LOG_INFO("{ ");
    cbor_encoder_init(&encoder, cbor_buf, cbor_len, 0);

    cbor_encoder_create_map(&encoder, &mapEncoder, 3);

    _map_append_int_as_float(&mapEncoder, CBOR_TEMP_KEY, m->tmp, 0.01);

    _map_append_uint_as_float(&mapEncoder, CBOR_PRESS_KEY, m->press, 1.0);

    _map_append_uint_as_float(&mapEncoder, CBOR_HUM_KEY, m->hum, 0.01);

    cbor_encoder_close_container(&encoder, &mapEncoder);
    LOG_INFO(" }\n");
    return cbor_encoder_get_buffer_size(&encoder, cbor_buf);
}

int cbor_util_energy_attr_encode(eco_task_consumption_spec_t *tx_task_spec,
                                 eco_task_consumption_spec_t *gps_task_spec,
                                 eco_task_consumption_spec_t *dust_task_spec,
                                 eco_task_consumption_spec_t *env_task_spec,
                                 eco_measurement_set_t *avg,
                                 uint8_t *cbor_buf, size_t cbor_len)
{
    CborEncoder encoder, mapEncoder;
    LOG_DEBUG("cbor_util_bmx_encode\n");

    LOG_INFO("{ ");
    cbor_encoder_init(&encoder, cbor_buf, cbor_len, 0);

    cbor_encoder_create_map(&encoder, &mapEncoder, 5);

    _map_append_int(&mapEncoder, CBOR_E_ATTR_TX_KEY,
                    (int32_t)(tx_task_spec->e_pws / 1000000));
    _map_append_int(&mapEncoder, CBOR_E_ATTR_GPS_KEY,
                    (int32_t)(gps_task_spec->e_pws / 1000000));
    _map_append_int(&mapEncoder, CBOR_E_ATTR_DUST_KEY,
                    (int32_t)(dust_task_spec->e_pws / 1000000));
    _map_append_int(&mapEncoder, CBOR_E_ATTR_ENV_KEY,
                    (int32_t)(env_task_spec->e_pws / 1000000));

    _map_append_int(&mapEncoder, CBOR_E_ATTR_ENV_KEY,
                    (int32_t)((int64_t)avg->power_uw * avg->t_us / 1000000000));

    cbor_encoder_close_container(&encoder, &mapEncoder);
    LOG_INFO(" }\n");
    return cbor_encoder_get_buffer_size(&encoder, cbor_buf);
}

/* used in the SANE context
static const char *_get_phydat_type_str(uint8_t type)
{
 switch (type) {
     case SAUL_SENSE_TEMP: return "temperature";
     case SAUL_SENSE_PM: return "PM";
     case SAUL_SENSE_VOLTAGE: return "U";
     case SAUL_SENSE_CURRENT: return "I";
     case SAUL_SENSE_POWER: return "P";
     default: return saul_class_to_str(type);
 }
}
*/

int encode_phydat_to_cbor(phydat_t *data, uint8_t dim, uint8_t type,
                          const char *unit, uint8_t *cbor_buf, size_t cbor_len)
{
    CborEncoder encoder, mapEncoder;

    LOG_INFO("{ ");
    cbor_encoder_init(&encoder, cbor_buf, cbor_len, 0);

    cbor_encoder_create_map(&encoder, &mapEncoder, 3);
    cbor_encode_text_stringz(&mapEncoder, CBOR_KEY_STRING);
    cbor_encode_text_stringz(&mapEncoder, cbor_util_type_to_str(type));
    LOG_INFO("\"%s\": \"%s\", ", CBOR_KEY_STRING, cbor_util_type_to_str(type));

    cbor_encode_text_stringz(&mapEncoder, CBOR_VAL_STRING);
    LOG_INFO("\"%s\": ", CBOR_VAL_STRING);

    if (dim > 1) {
        CborEncoder arrayEncoder;
        LOG_INFO("[ ");
        cbor_encoder_create_array (&mapEncoder, &arrayEncoder, dim);
        for(unsigned i = 0; i < dim; i++) {
            float val = data->val[i] * pow(10, data->scale);
            if ( i > 0) {
                LOG_INFO(", ");
            }
            LOG_INFO("%f", val);
            cbor_encode_float(&arrayEncoder, val);
        }
        LOG_INFO(" ], ");
        cbor_encoder_close_container(&mapEncoder, &arrayEncoder);
    }
    else {
        float val = data->val[0] * pow(10, data->scale);
        cbor_encode_float(&mapEncoder, val);
        LOG_INFO("%f, ", val);
    }

    cbor_encode_text_stringz(&mapEncoder, CBOR_UNIT_STRING);
    cbor_encode_text_stringz(&mapEncoder, unit);
    LOG_INFO("\"%s\": \"%s\"", CBOR_UNIT_STRING, unit);

    cbor_encoder_close_container(&encoder, &mapEncoder);
    LOG_INFO(" }\n");
    return cbor_encoder_get_buffer_size(&encoder, cbor_buf);
}
