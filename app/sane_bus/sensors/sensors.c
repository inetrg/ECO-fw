/*
 * Copyright (C) 2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sensors
 * @{
 *
 * @file
 * @brief       Sensor control interface
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */
#include "thread.h"
#include "kernel_types.h"
#include "sds011.h"
#include "sds011_params.h"
#include "sensors.h"
#include "msg.h"
#define LOG_LEVEL LOG_INFO
#include "log.h"
#include "control_messages.h"
#include "xtimer.h"
#include "gpio_wakeup.h"
#include "bmx280.h"
#include "bmx280_params.h"

#include <string.h>

kernel_pid_t sensing_thread_pid;
char sensing_thread_stack[THREAD_STACKSIZE_SENSORS];
static msg_t msg_queue[THREAD_MSG_QUEUE_SIZE_SENSORS];

#ifndef GPS_MINIMUM_CONTINUOUS_FIX_CNT
#define GPS_MINIMUM_CONTINUOUS_FIX_CNT (5)
#endif

/**
 * @brief   Allocate memory for the device descriptor
 */
static sds011_t sds011_dev;

/**
 * @brief   Allocate memory for the bme280 descriptor
 */
static bmx280_t bme280_dev;

bmx_measure_t bmx_measure;

typedef struct {
    uint32_t      avg_cnt;
    uint32_t      pm_2_5_avg;
    uint32_t      pm_10_avg;
    sds011_data_t sds011_data;
    kernel_pid_t  reply_pid;
    kernel_pid_t  sensor_pid;
} _sds011_measure_ctx_t;

_sds011_measure_ctx_t sds011_ctx;

phydat_t phydat_reading;
char sds011_data_str[32];
buffer_descriptor_t bufdesc;

/* using this for now to implement aginst phydat interface without using saul
   at the moment because of low level control */
static inline void _sds011_to_phydat(sds011_data_t *in, phydat_t *out)
{
    out->val[0] = in->pm_2_5;
    out->val[1] = in->pm_10;
    out->unit = UNIT_GPM3;
    out->scale = -7;
}

static inline int _sds011_to_str(uint8_t *buf, size_t buf_len, sds011_data_t *sds011_data){
    //TODO replace with cbor
    return snprintf((char*)buf, buf_len, "{ PM2.5: %d, PM10: %d }",
                    sds011_data->pm_2_5, sds011_data->pm_10);
}

static void _sds011_cb(sds011_data_t *data, void *ctx)
{
    _sds011_measure_ctx_t *sds_ctx = (_sds011_measure_ctx_t*)ctx;
    sds_ctx->pm_2_5_avg += data->pm_2_5;
    sds_ctx->pm_10_avg += data->pm_10;
    sds_ctx->avg_cnt++;

    LOG_DEBUG("_sds011_cb\n");

    if (sds_ctx->avg_cnt == SENSORS_SDS011_AVG_CNT) {
        sds_ctx->sds011_data.pm_2_5 = (sds_ctx->pm_2_5_avg +
                                       SENSORS_SDS011_AVG_CNT / 2)
                                      / SENSORS_SDS011_AVG_CNT;
        sds_ctx->sds011_data.pm_10  = (sds_ctx->pm_10_avg +
                                       SENSORS_SDS011_AVG_CNT / 2)
                                      / SENSORS_SDS011_AVG_CNT;

        msg_t avg_msg = { .type = CTLMSG_TYPE1_DUST_SENSOR | CTLMSG_TYPE2_DATA_RAW,
                          .content.ptr = sds_ctx };

        msg_send(&avg_msg, sds_ctx->sensor_pid);
    }
}

void _ctl_dust_sensor(msg_t *m)
{
    int res = 0;
    switch (CTLMSG_TYPE2(m->type)) {
        case CTLMSG_TYPE2_INIT:
            LOG_INFO("CTLMSG_TYPE2_INIT\n");
            if (sds011_init(&sds011_dev, &sds011_params[0]) == SDS011_OK) {
                LOG_INFO("sds011_init : SDS011_OK\n");

                /* set the sensor to working mode - this also ensures
                  that the sensor was powered up completely */
                do {
                   res = sds011_set_working_mode(&sds011_dev, SDS011_WMODE_WORK);
                   LOG_INFO("sds011_set_reporting_mode : %d\n", res);
                }
                while (res != 0);

                /* set the sensor to active reporting mode - this also ensures
                   that the sensor was powered up completely */
                do {
                    res = sds011_set_reporting_mode(&sds011_dev, SDS011_RMODE_ACTIVE);
                    LOG_INFO("sds011_set_reporting_mode : %d\n", res);
                }
                while (res != 0);

                xtimer_usleep(SENSORS_DSDS011_WARMUP_TIME_MS * US_PER_MS);
            }
            else {
                LOG_INFO("[_ctl_dust_sensor] error initializing sds011\n");
            }
            sds011_ctx.reply_pid = KERNEL_PID_UNDEF;
            break;

        case CTLMSG_TYPE2_MEASURE:
            LOG_INFO("CTLMSG_TYPE2_MEASURE\n");
            sds011_ctx.pm_2_5_avg = 0;
            sds011_ctx.pm_10_avg  = 0;
            sds011_ctx.avg_cnt    = 0;
            sds011_ctx.reply_pid  = m->sender_pid;
            sds011_ctx.sensor_pid = thread_getpid();

            res = sds011_register_callback(&sds011_dev, _sds011_cb, &sds011_ctx);

            LOG_INFO("sds011_register_callback : %d\n", res);
            LOG_INFO("set to active mode and registered callback done!\n");
            break;

        case CTLMSG_TYPE2_DATA_RAW:
            LOG_INFO("CTLMSG_TYPE2_DATA\n");
            /* disable callback */
            sds011_register_callback(&sds011_dev, NULL, NULL);

            _sds011_measure_ctx_t *sds_ctx = m->content.ptr;

            msg_t raw_data_msg = { .type = CTLMSG_TYPE2_DATA_RAW,
                                   .content.ptr =  &sds_ctx->sds011_data };

            msg_send(&raw_data_msg, sds_ctx->reply_pid);
            sds011_ctx.reply_pid = KERNEL_PID_UNDEF;
            break;

        case CTLMSG_TYPE2_POWER_ON:
            LOG_INFO("CTLMSG_TYPE2_POWER_ON\n");
            //TODO add synchronous option
            sds011_power_on(&sds011_dev);
            xtimer_usleep(SENSORS_DSDS011_WARMUP_TIME_MS);
            break;

        case CTLMSG_TYPE2_POWER_OFF:
            LOG_INFO("CTLMSG_TYPE2_POWER_OFF\n");
            xtimer_usleep(100 * 1000);
            sds011_set_working_mode(&sds011_dev, SDS011_WMODE_SLEEP);
            /* disable callback */
            xtimer_usleep(100 * 1000);
            sds011_register_callback(&sds011_dev, NULL, NULL);

            xtimer_usleep(100 * 1000);
            sds011_power_off(&sds011_dev);
            xtimer_usleep(100 * 1000);

            break;

        default: LOG_ERROR("_ctl_dust_sensor: unknown message value\n");
    }
}

void _ctl_env_sensor(msg_t *m)
{
    switch (CTLMSG_TYPE2(m->type)) {
        case CTLMSG_TYPE2_INIT:
            LOG_INFO("CTLMSG_TYPE2_INIT\n");

            gpio_init(BMX280_PWR_PIN, GPIO_OUT);
            gpio_write(BMX280_PWR_PIN, BMX280_PWR_PIN_AH);

            xtimer_usleep(1000 * 10);

            int result;

            result = bmx280_init(&bme280_dev, &bmx280_params[0]);
            if (result == -1) {
                LOG_ERROR("[Error] The given i2c is not enabled");
                break;
            }

            if (result == -2) {
                LOG_ERROR("[Error] The sensor did not answer correctly at address 0x%02X\n", bmx280_params[0].i2c_addr);
                break;
            }

            LOG_DEBUG("[sensors] bme280 init successful\n\n");

            xtimer_usleep(1000 * 100);

            break;

        case CTLMSG_TYPE2_MEASURE:
            LOG_INFO("CTLMSG_TYPE2_MEASURE\n");

            /* Get temperature in centi degrees Celsius */
            bmx_measure.tmp = bmx280_read_temperature(&bme280_dev);

            /* Get pressure in Pa */
            bmx_measure.press = bmx280_read_pressure(&bme280_dev);

            /* Get pressure in %rH */
            bmx_measure.hum = bme280_read_humidity(&bme280_dev);

            msg_t reply = { .content.ptr = &bmx_measure };

            msg_send(&reply, m->sender_pid);

            break;

        case CTLMSG_TYPE2_DATA_RAW:
            LOG_INFO("CTLMSG_TYPE2_DATA\n");
            break;

        case CTLMSG_TYPE2_POWER_ON:
            LOG_INFO("CTLMSG_TYPE2_POWER_ON\n");
            gpio_init(BMX280_PWR_PIN, GPIO_OUT);
            gpio_write(BMX280_PWR_PIN, BMX280_PWR_PIN_AH);
            break;

        case CTLMSG_TYPE2_POWER_OFF:
            gpio_init(BMX280_PWR_PIN, GPIO_OUT);
            gpio_write(BMX280_PWR_PIN, !BMX280_PWR_PIN_AH);
            break;

        default: LOG_ERROR("_ctl_dust_sensor: unknown message value\n");
    }
}

#include "minmea.h"
#include "minmea_helper.h"
#include "pnmea.h"

//#define UART_BUFSIZE            (128)
#define MINMEA_SENTENCE_BUF_NUM (20)

// TODO: refactor to gps_state struct
// TODO: split out individual sensors to separate files
typedef struct {
    uint8_t nmea_sentences[MINMEA_SENTENCE_BUF_NUM][MINMEA_MAX_LENGTH];
    int sentence_idx;
    int char_idx;
    uint8_t json_payload[128];
    cbor_buffer_t cbor_buf;
    gps_data_t gps_data;
    uint32_t t_gps_init;
    kernel_pid_t listener;
    bool req_only_time;
} gps_sensor_state_t;

static void _gps_rx_cb(void *arg, uint8_t data)
{
    gps_sensor_state_t *state = (gps_sensor_state_t*)arg;
    if (state->char_idx < (MINMEA_MAX_LENGTH -1)) {
        state->nmea_sentences[state->sentence_idx][state->char_idx++] = data;
    }

    if (data == '\n') {
        msg_t msg;
        state->nmea_sentences[state->sentence_idx][state->char_idx] = 0;
        msg.type = CTLMSG_TYPE1_GPS_SENSOR | CTLMSG_TYPE2_DATA_RAW;
        msg.content.ptr = state->nmea_sentences[state->sentence_idx];

        state->sentence_idx = (state->sentence_idx + 1) % MINMEA_SENTENCE_BUF_NUM;
        state->char_idx = 0;
        msg_send(&msg, sensing_thread_pid);

    }
}

static int _init_gps(gps_sensor_state_t *state, bool hard_reset){
    uint32_t baud = 9600;

    LOG_DEBUG("Init GPS-UART...\n");
    /* initialize UART */
    int res = uart_init(GPS_UART_DEV, baud, _gps_rx_cb, state);
    LOG_DEBUG("init done...\n");
    if (res == UART_NOBAUD) {
        LOG_ERROR("Error: Given baudrate (%u) not possible\n", (unsigned int)baud);
        return 1;
    }
    else if (res != UART_OK) {
        LOG_ERROR("Error: Unable to initialize UART device\n");
        return 1;
    }
    LOG_DEBUG("Success: Successfully initialized UART_DEV\n");

    char pnmea_cmd[PNMEA_MAX_CMD_LEN];

    pnmea_cmd_t cmds[] = {

        // query EPO data
        {.cmdname ="PMTK607",
         .params = (int[]){0},
         .param_num = 0,},

        {
        .cmdname = hard_reset ? "PMTK104" : "PMTK101",
        .params = (int[]){0},
        .param_num = 0,},

        // hot start (use all available data)
        // {.cmdname = "PMTK101",
        // .params = (int[]){0},
        // .param_num = 0,},

        // warm start (don't use ephemeris)
        // {.cmdname = "PMTK102",
        // .params = (int[]){0},
        // .param_num = 0,},

        // cold start (don't use time,pos,alamanacs and ephemeris)
        // {.cmdname = "PMTK103",
        // .params = (int[]){0},
        // .param_num = 0,},

        // cold start & full reset of data system/user config
        // {.cmdname = "PMTK104",
        // .params = (int[]){0},
        // .param_num = 0,},

        {.cmdname = "PMTK314", // set nmea sentences
          // GPGLL, GPRMC, GPVTG, GPGGA, GPGSA, GPGSV
         .params = (int[]){0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
         .param_num = 19,},
        {.cmdname = "PMTK301", // set SBAS mode to WAAS (EGNOS)
         .params = (int[]){2},
         .param_num = 1,},
        {.cmdname = "PMTK313", // enable SBAS
         .params = (int[]){1},
         .param_num = 1,},
        //{.cmdname = "PMTK225", // use normal mode
        // .params = (int[]){0},
        // .param_num = 1,},
        // {.cmdname = "PMTK223", // set extension time for getting a fix
        //  .params = (int[]){3, 25, 40000, 60000},
        //  .param_num = 4,},
        // {.cmdname = "PMTK225", // eanble periodic mode
        //  .params = (int[]){1, 1000, 20000, 0, 0},
        //  .param_num = 5,},
        //{.cmdname = "PMTK161", // force enter standby mode
        // .params = (int[]){0},
        // .param_num = 1,},
    };

    for (unsigned i = 0; i < sizeof(cmds) / sizeof(pnmea_cmd_t); i++) {
        pnmea_format_cmd(pnmea_cmd, PNMEA_MAX_CMD_LEN, cmds[i].cmdname, cmds[i].params, cmds[i].param_num);
        LOG_DEBUG("[_init_gps] sending command: %s\n", pnmea_cmd);
        uart_write(GPS_UART_DEV, (uint8_t*)pnmea_cmd, strlen(pnmea_cmd));

        xtimer_usleep(1000 * 500);
    }

    return 0;
}

void _ctl_gps_sensor(gps_sensor_state_t *state, msg_t *m)
{
    switch (CTLMSG_TYPE2(m->type)) {
        case CTLMSG_TYPE2_INIT:
            LOG_DEBUG("CTLMSG_TYPE2_INIT\n");
            memset(state, 0, sizeof(gps_sensor_state_t));
            state->listener = KERNEL_PID_UNDEF;
            bool cold_start = m->content.value & CTLMSG_ARG_FORCE_COLD_START;
            LOG_DEBUG("_ctl_gps_sensor performing %s start\n",
                      cold_start ? "cold" : "hot");
            _init_gps(state, cold_start);
            break;

        case CTLMSG_TYPE2_MEASURE:
            LOG_DEBUG("CTLMSG_TYPE2_MEASURE\n");
            state->listener = m->sender_pid;
            state->gps_data.cfc = 0;
            state->req_only_time = m->content.value;
            state->t_gps_init = xtimer_now_usec();
            break;

        case CTLMSG_TYPE2_DATA_RAW:
            {
                char *sentence = (char*)m->content.ptr;
                LOG_DEBUG("CTLMSG_TYPE2_DATA\n");
                LOG_DEBUG("%s", sentence);
                int res = populate_gps_data(&state->gps_data, sentence);
                //printf("rs: %d\n", res);
                if (res == MINMEA_SENTENCE_RMC) {
                    //printf("received RMC\n");
                    //printf("date: %02d-%02d-%02d\n", data->date.day, data->date.month,
                    //       data->date.year);

                    /* TODO: remove the year check after debugging */
                    // || state->gps_data.date.year != 80
                    if (state->req_only_time || state->gps_data.valid) {
                        state->gps_data.cfc++;
                        state->gps_data.ttf = xtimer_now_usec() - state->t_gps_init;
                        if (state->gps_data.cfc >=
                            GPS_MINIMUM_CONTINUOUS_FIX_CNT) {
                            minmea_helper_encode(&state->gps_data,
                                                 state->json_payload,
                                                 sizeof(state->json_payload),
                                                 ENCODING_JSON);
                            LOG_DEBUG("%s (fix %s, sats visible: %d)\n",
                                      state->json_payload,
                                      state->gps_data.valid ? "valid" : "invalid",
                                      state->gps_data.visible_sats);

                            if (state->listener != KERNEL_PID_UNDEF) {
                                msg_t gps_msg;
                                gps_msg.type = CTLMSG_TYPE1_GPS_SENSOR | CTLMSG_TYPE2_DATA_RAW;
                                gps_msg.content.ptr = &state->gps_data;
                                msg_try_send(&gps_msg, state->listener);
                                state->listener = KERNEL_PID_UNDEF;
                            }
                        }
                    }
                    else {
                        state->gps_data.cfc--;
                        if (state->gps_data.cfc < 0) {
                            state->gps_data.cfc = 0;
                        }
                    }
                }
            }
            break;

        case CTLMSG_TYPE2_POWER_ON:
            LOG_DEBUG("CTLMSG_TYPE2_POWER_ON\n");
            LOG_INFO("---------> gps power up\n");
            gpio_write(GPS_ENABLE_PIN, GPS_ENABLE_PIN_AH);
            break;

        case CTLMSG_TYPE2_POWER_OFF:
            LOG_DEBUG("CTLMSG_TYPE2_POWER_OFF\n");
            LOG_INFO("---------> gps power down\n");
            uart_poweroff(GPS_UART_DEV);
            gpio_write(GPS_ENABLE_PIN, !GPS_ENABLE_PIN_AH);
            //gpio_wakeup_pull_config(GPS_ENABLE_PIN, GPIO_IN_PU);
            break;

        default: LOG_ERROR("_ctl_dust_sensor: unknown message value\n");
    }
}

void *sensing_thread(void *arg)
{
    (void) arg;
    msg_t m;
    gps_sensor_state_t gps_state;

    gpio_init(GPS_ENABLE_PIN, GPIO_OUT);
    gpio_write(GPS_ENABLE_PIN, GPS_ENABLE_PIN_AH);

    msg_init_queue(msg_queue,THREAD_MSG_QUEUE_SIZE_SENSORS);

    sensing_thread_pid = thread_getpid();

    LOG_DEBUG("sensor thread started, pid: %" PRIkernel_pid "\n",
              sensing_thread_pid);

    while (1) {
        msg_receive(&m);
        LOG_DEBUG("[sensing_thread] got msg from %" PRIkernel_pid "\n",
                 m.sender_pid);

        switch (CTLMSG_TYPE1(m.type)) {
            case CTLMSG_TYPE1_DUST_SENSOR: _ctl_dust_sensor(&m); break;
            case CTLMSG_TYPE1_GPS_SENSOR:  _ctl_gps_sensor(&gps_state, &m); break;
            case CTLMSG_TYPE1_ENV_SENSOR: _ctl_env_sensor(&m); break;

            default: LOG_ERROR("sensors: unknown message type\n");
        }
    }

    return NULL;
}

kernel_pid_t sensors_init_thread(void)
{
    return thread_create(sensing_thread_stack, sizeof(sensing_thread_stack),
                         THREAD_PRIORITY_SENSORS, THREAD_CREATE_STACKTEST,
                         sensing_thread, NULL, "sensing");
}
