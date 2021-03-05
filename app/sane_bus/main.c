/*
 * Copyright (C) 2019 HAW Hamburg
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
 * @brief       SANE Energy Harvesting Sensor Application
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "sensors.h"
#include "msg.h"
#include "xtimer.h"
#include "control_messages.h"
#include "eco.h"
#include "radio.h"
//#include "coap.h"
#include "netif_types.h"
#include "saul.h"
#include "phydat.h"
#include "cbor_util.h"
#include "pm_layered.h"
#include "node_time.h"
#include "backup_reg.h"
#include "backup_reg_alloc.h"
#include "periph/rtc.h"
#include "periph/hwrng.h"
#include "random.h"
#include "gpio_wakeup.h"
#include "minmea_helper.h"
#include "fmt.h"
#include "net/gnrc/netif.h"
#include "stmclk.h"

#define LOG_LEVEL LOG_INFO
#include "log.h"

//#define VERBOSE_PRINTS

/* eco will perform one lp-measure every second cycle */
//#define SLEEP_POWER_MEASURE_CYCLE_INTERVAL     (2)
//#define RTC_SLEEP_DURATION_SECONDS_DEFAULT     (120)
//#define RTC_SLEEP_DURATION_SECONDS_MIN         (120)
/* low value to perform fast bootstrap */
#define RTC_SLEEP_DURATION_SECONDS_DEFAULT     (5)
#define RTC_SLEEP_DURATION_SECONDS_MIN         (5)
//#define RTC_SLEEP_DURATION_SECONDS_MAX         (60 * 60)

#define DUTY_CYCLE_VALID_CNT                   (5)

/* time to wait before abort trying to get a satellite fix for the first time */
#define GPS_COLD_START_MAX_TTF_WAIT_US         (120 * US_PER_SEC)
/* time to wait before abort trying to get a satellite re-fix (hot start) */
#define GPS_HOT_START_MAX_TTF_WAIT_US          (60 * US_PER_SEC)

// #define GPS_COLD_START_MAX_TTF_WAIT_US         (10 * US_PER_SEC)
// #define GPS_HOT_START_MAX_TTF_WAIT_US          (5 * US_PER_SEC)

//#define GPS_COLD_START_MAX_TTF_WAIT_US         (10 * US_PER_SEC)
//#define GPS_HOT_START_MAX_TTF_WAIT_US          (10 * US_PER_SEC)

/* backup wakeup duration to set on init as a watchdog replacement
   this value should be higher than any active cycle may take to allow
   overwriting it with a proper value  */
#define RTC_BACKUP_WATCHDOG_PERIOD_S (200)

#define POWER_TRACE_AFTER_SENSING_N_CYCLES (5)

//msg_t m;
/* TODO: refactor to either not hand it to the ina thread or allocate it to retained
   RAM area from here */
extern eco_t *eco;

#define BUFF_IDX_ECO            (0)
#define BUFF_IDX_SENSE          (1)

#define CBOR_OBJ_CNT            (BUFF_IDX_SENSE + 1)

cbor_buffer_t cbor_buffs[CBOR_OBJ_CNT];

eco_task_consumption_spec_t *tx_task_spec;
eco_task_consumption_spec_t *gps_task_spec;
eco_task_consumption_spec_t *dust_task_spec;
eco_task_consumption_spec_t *env_task_spec;

gps_data_t gps_data;

static void clear_buffs(cbor_buffer_t *buf, size_t cnt) {
    for (size_t i = 0; i < cnt ; i++) {
        buf->len = 0;
    }
}

int _send_msg(kernel_pid_t pid, uint16_t type, uint32_t val)
{
    msg_t m;
    m.type = type;
    m.content.value = val;
    return msg_send(&m, pid);
}

int _msg_send_receive(kernel_pid_t pid, uint16_t type, uint32_t val, msg_t *reply)
{
    msg_t m;
    m.type = type;
    m.content.value = val;
    return msg_send_receive(&m, reply, pid);
}

static void _node_allow_power_down(kernel_pid_t eco_pid){
    //TODO: check why pm 0 isn't working..
    //gpio_init(GPS_ENABLE_PIN, GPIO_OUT);
    //gpio_write(GPS_ENABLE_PIN, !GPS_ENABLE_PIN_AH);
    //gpio_init(GPS_ENABLE_PIN, GPIO_IN_PU);

    gpio_init(BMX280_PWR_PIN, GPIO_OUT);
    gpio_write(BMX280_PWR_PIN, !BMX280_PWR_PIN_AH);

    gpio_wakeup_pull_config(GP_PWR_GND_SW_PIN,
                            GP_PWR_GND_SW_PIN_AH ? GPIO_IN_PD : GPIO_IN_PU);

    uint32_t sleep_seconds = RTC_SLEEP_DURATION_SECONDS_DEFAULT;

    /* set a backup value if eco fails for whatever reason ... */
    node_time_set_rtc_alarm_in_seconds(sleep_seconds);

    if (eco_pid != KERNEL_PID_UNDEF) {
        LOG_DEBUG("[main] telling eco that we want to power down now...\n");
        msg_t reply;
        _msg_send_receive(eco_pid,
                   CTLMSG_TYPE1_POWER_MEASURE | CTLMSG_TYPE2_NOTIFY_POWER_DOWN,
                   0, &reply);

        eco_t *eco = (eco_t*)reply.content.ptr;
        LOG_INFO("[main] got reply from eco thread\n"
                 "        -> recommend %"PRIu32" s sleep cycle\n"
                 "                 and %"PRIu32" s sensing cycle\n",
                 eco->eco_processing_period_s, eco->sensing_period_s);

        /* TODO: wakeup for whatever is closer? */
        sleep_seconds = eco->eco_processing_period_s <= eco->sensing_period_s ?
                        eco->eco_processing_period_s : eco->sensing_period_s;
    }

    if (sleep_seconds < RTC_SLEEP_DURATION_SECONDS_MIN) {
        sleep_seconds = RTC_SLEEP_DURATION_SECONDS_MIN;
    }

    LOG_INFO("[main] will set RTC to %"PRIu32" s\n", sleep_seconds);
    //else if (sleep_seconds > RTC_SLEEP_DURATION_SECONDS_MAX) {
    //    sleep_seconds = RTC_SLEEP_DURATION_SECONDS_MAX;
    //}

    node_time_set_rtc_alarm_in_seconds(sleep_seconds);
    //ode_time_set_rtc_alarm_in_seconds(20);

    node_time_save_power_down_time_ms();

    LOG_DEBUG("[main] entering low power mode *NOW*\n");

    pm_set(STM32_PM_STANDBY);
    //pm_set(STM32_PM_SHUTDOWN);
}

extern void _radio_power_off(void);
extern void _radio_init(void);

static uint32_t _send_available_data(kernel_pid_t radio_pid)
{
    uint32_t toa_sum_us = 0;

    for (size_t i = 0; i < CBOR_OBJ_CNT; i++) {
        if( cbor_buffs[i].len > 0 ) {
#ifdef UPLINK_LOWPAN_ENABLED
            /* TODO: move this to the radio utility and and only give it the
               raw data */
            int send_res = coap_post_blocking("/logger/sensor", UPLINK_LLADDR,
                                          (char*)cbor_buffs[i].buf,
                                          cbor_buffs[i].len);
            LOG_DEBUG("[main] send_res: %d\n", send_res);
#endif
#ifdef UPLINK_LORAWAN_ENABLED
            LOG_DEBUG("[main] sending..\n");
            msg_t reply;
            _msg_send_receive(radio_pid, CTLMSG_TYPE1_RADIO |
                              CTLMSG_TYPE2_DATA_CBOR,
                              (uint32_t)&cbor_buffs[i], &reply);

            uint32_t toa = reply.content.value;
            toa_sum_us += toa;
            LOG_DEBUG("[main] used %lu us time on air\n", toa);

            xtimer_usleep(1000 * 300);
#endif
        }
    }

    return toa_sum_us;
}

static void _log_print_hex(uint8_t *buf, size_t len){
    for (size_t i = 0; i < len; i++) {
        LOG_DEBUG("%02X", buf[i]);
    }
    LOG_DEBUG("\n");
}

void init_random(void){
    int rnd = 0;
    hwrng_read(&rnd, sizeof(rnd));
    random_init(rnd);
}

void _backup_last_sensing_time(uint64_t utc_ms) {
   backup_reg_write(BACKUP_REG_LAST_MEASUREMENT_UTC_MS_H, (uint32_t)(utc_ms >> 32));
   backup_reg_write(BACKUP_REG_LAST_MEASUREMENT_UTC_MS_L, (uint32_t)(utc_ms & 0xFFFFFFFFU));
}

uint64_t _get_last_sensing_time(void) {
   uint32_t utc_ms_h;
   uint32_t utc_ms_l;
   backup_reg_read(BACKUP_REG_LAST_MEASUREMENT_UTC_MS_H, &utc_ms_h);
   backup_reg_read(BACKUP_REG_LAST_MEASUREMENT_UTC_MS_L, &utc_ms_l);
   return ((uint64_t)utc_ms_h << 32) | utc_ms_l;
}

/* This method performs all steps that are mandatory on every startup, be it the
   first time the power cable was plugged in or waking up from an interrupt. */
void _obligatory_init(void)
{
    xtimer_init();
    node_time_warm_init();
    uint32_t prev_sleep_dur_ms = node_time_get_prev_sleep_dur_ms();
    timestamp_t time;
    node_time_get(&time);
    node_time_print(&time);
    gpio_init(INA22X_PARAM_ALERT_PIN, GPIO_IN_PU);
    gpio_wakeup_pull_config(INA22X_PARAM_ALERT_PIN, GPIO_IN_PU);
    gpio_wakeup_disable(INA22X_PARAM_ALERT_PIN);
    gpio_wakeup_clear(INA22X_PARAM_ALERT_PIN);

    /* disable all available devices by default */
    gpio_init(SDS011_PARAM_PWR_PIN, GPIO_OUT);
    gpio_write(SDS011_PARAM_PWR_PIN, !SDS011_PARAM_PWR_PIN_AH);

    gpio_init(GPS_ENABLE_PIN, GPIO_OUT);
    gpio_write(GPS_ENABLE_PIN, !GPS_ENABLE_PIN_AH);

    gpio_init(LTC3105_PARAM_SHDN_PIN, GPIO_OUT);
    gpio_write(LTC3105_PARAM_SHDN_PIN, 1);


    gpio_init(GP_PWR_GND_SW_PIN, GPIO_OUT);
    gpio_write(GP_PWR_GND_SW_PIN, GP_PWR_GND_SW_PIN_AH);
    //xtimer_sleep(2);
    init_random();
    LOG_DEBUG("[main] slept for %"PRIu32" ms\n", prev_sleep_dur_ms);

    uint32_t sys_state = 0;
    backup_reg_read(BACKUP_REG_SYS_STATE, &sys_state);
    LOG_DEBUG("[main] sysstate: 0x%lx\n", sys_state);

    bool need_to_bootstrap = !(sys_state & SYS_STATE_BOOTSTRAPPED);

    periph_clk_en(APB1, RCC_APB1ENR1_PWREN);

    LOG_DEBUG("[main] PWR->CR3: 0x%08lX\n", PWR->CR3);
    if (PWR->CR3 & PWR_CR3_RRS) {
        LOG_DEBUG("[main] RAM retained!\n");
    }
    else {
        LOG_DEBUG("[main] RAM not retained\n");
        need_to_bootstrap = true;
    }

    /* The first thing the EH-node must do after it was powered up the first
       time is bootstrapping itself.
       This essentially means that the node must sync to an absolute time
       reference that is used to annotate the sensed data.
       Also authentication to LoRa could be done here */
    if (!need_to_bootstrap) {
        LOG_DEBUG("[main] no need to bootstrap\n");
    }
    else {
        /* TODO shut down everything properly after bootstrapping */
        LOG_INFO("[main] bootstrapping now...\n");

        LOG_DEBUG("[main] clearing all backup registers...\n");

        for (int i = 0; i < BACKUP_REG_COUNT; i++) {
            backup_reg_write(i, 0);
        }

        backup_reg_write(BACKUP_REG_LORA_PREV_DR, RADIO_LORA_DATARATE_DEFAULT);

        /* this cold init code only gets executed when the node is powered up
           for the first time.
           Steps that need to be done in this step:
                1) init RTC to be used for alarmed wakeups
                2) set system state backup registers to initial state
                    -> cycle count, default sleep duration, bootstrapped flag */
        node_time_init();

        backup_reg_write(BACKUP_REG_SYS_STATE, SYS_STATE_BOOTSTRAPPED);

        LOG_DEBUG("[main] clearing low-power retained ram section...\n");
        uint8_t *retram = (uint8_t*)STM32_L476RG_SRAM2_START;
        memset(retram, 0, STM32_L476RG_SRAM2_LEN);
    }

    node_time_set_rtc_alarm_in_seconds(RTC_BACKUP_WATCHDOG_PERIOD_S);
}

static void _sync_with_gps_time(gps_data_t *gps_data)
{
    timestamp_t gps_time = { .tm = { .tm_sec  = gps_data->time.seconds,
                                     .tm_min  = gps_data->time.minutes,
                                     .tm_hour = gps_data->time.hours,
                                     .tm_mday = gps_data->date.day,
                                     .tm_mon  = gps_data->date.month - 1,
                                     .tm_year = gps_data->date.year
                                                + GPS_YEAR_OFFSET
                                                - TM_YEAR_OFFSET},
                             .ms = gps_data->time.microseconds / 1000};

    uint64_t gps_utc_ms = node_time_to_utc_ms(&gps_time);

    uint64_t rtc_utc_ms = node_time_get_utc_ms();

    int64_t t_diff_ms = gps_utc_ms - rtc_utc_ms;

    char s64_str[32] = {0};
    fmt_s64_dec(s64_str, t_diff_ms);
    printf("t_diff_ms:  %s\n", s64_str);

    node_time_set_utc_ms(gps_utc_ms);

    uint32_t sys_state = 0;
    backup_reg_read(BACKUP_REG_SYS_STATE, &sys_state);
    backup_reg_write(BACKUP_REG_SYS_STATE, sys_state | SYS_STATE_RTC_SYNCED);
}

msg_t gps_msg;
msg_t missed_gps_message;
static bool _try_gps_fix(kernel_pid_t sensors_pid, uint32_t timeout_us,
                         gps_data_t *gps_data, bool only_time) {
    LOG_INFO("[main] initing gps... \n");
    _send_msg(sensors_pid, CTLMSG_TYPE1_GPS_SENSOR | CTLMSG_TYPE2_INIT, 0);
              //CTLMSG_ARG_FORCE_COLD_START);

    _send_msg(sensors_pid, CTLMSG_TYPE1_GPS_SENSOR | CTLMSG_TYPE2_MEASURE, only_time);

    /* after telling the sensors thread that we want a GPS measurement wait
       till we get a valid fix but timeout if it takes to long */
    if (xtimer_msg_receive_timeout(&gps_msg, timeout_us) >= 0) {
        printf("[main] got RAW GPS data\n");
        /* TODO: replace with handing over the memory to the gps thread */
        memcpy(gps_data, gps_msg.content.ptr, sizeof(gps_data_t));
        _send_msg(sensors_pid, CTLMSG_TYPE1_GPS_SENSOR | CTLMSG_TYPE2_POWER_OFF, 0);
#ifdef VERBOSE_PRINTS
        print_gps_data(gps_data);
#endif
        return true;
    }
    else {
        printf("main GPS fix timeout!\n");
    }

    _send_msg(sensors_pid, CTLMSG_TYPE1_GPS_SENSOR | CTLMSG_TYPE2_POWER_OFF, 0);

    /*TODO: may need to clear unused incoming message from the GPS? */
    msg_try_receive(&missed_gps_message);
    return false;
}

static eco_task_consumption_spec_t* _tell_eco_about_task_start(kernel_pid_t eco_pid) {
    msg_t task_start_reply_msg;
    _msg_send_receive(eco_pid,
               CTLMSG_TYPE1_POWER_MEASURE | CTLMSG_TYPE2_NOTIFY_TASK_START,
               0, &task_start_reply_msg);
    return task_start_reply_msg.content.ptr;
}

static void _tell_eco_about_task_stop(kernel_pid_t eco_pid,
                                      eco_task_consumption_spec_t *task) {
    _send_msg(eco_pid, CTLMSG_TYPE1_POWER_MEASURE | CTLMSG_TYPE2_NOTIFY_TASK_STOP, (uint32_t)task);
}

static bool _gps_stuff(kernel_pid_t sensors_pid, gps_data_t *gps_val) {
    uint32_t gps_timeout_s = GPS_COLD_START_MAX_TTF_WAIT_US;
    uint32_t sys_state = 0;
    backup_reg_read(BACKUP_REG_SYS_STATE, &sys_state);

    if (sys_state & SYS_STATE_HAD_GPS_FIX) {
        gps_timeout_s = GPS_HOT_START_MAX_TTF_WAIT_US;
    }

    /* after telling the sensors thread that we want a GPS measurement wait
       till we get a valid fix but timeout if it takes to long */
    if (_try_gps_fix(sensors_pid, gps_timeout_s, gps_val, false)) {
        LOG_INFO("[main] got GPS fix\n");

        LOG_DEBUG("[main] resyncing RTC with GPS time reference\n");
        _sync_with_gps_time(gps_val);
        uint32_t sys_state = 0;
        backup_reg_read(BACKUP_REG_SYS_STATE, &sys_state);
        backup_reg_write(BACKUP_REG_SYS_STATE, sys_state | SYS_STATE_HAD_GPS_FIX);

        return true;
    }

    return false;
}

/* TODO: move this to radio module */
static void _radio_update_dr(void) {
    gnrc_netif_t* netif = NULL;
    while((netif = gnrc_netif_iter(netif)) != NULL){
        uint32_t lora_prev_dr;
        backup_reg_read(BACKUP_REG_LORA_PREV_DR, &lora_prev_dr);

        LOG_INFO("previous DR was: %lu\n", lora_prev_dr);

        netopt_t dr_opt = NETOPT_DATARATE;
        uint16_t ctx = 0;
        uint8_t new_dr = lora_prev_dr;

        if (new_dr <= RADIO_LORA_DATARATE_MIN ||
            (new_dr > RADIO_LORA_DATARATE_MAX)) {
            new_dr = RADIO_LORA_DATARATE_MAX;
        }
        else {
            new_dr--;
        }

        printf("setting DR to %u\n", new_dr);
        if (gnrc_netapi_set(netif->pid, dr_opt, ctx, &new_dr, sizeof(new_dr)) < 0) {
            LOG_ERROR("_netapi_up error: unable to set DR\n");
        }
        else {
            printf("updated DR...\n");
            backup_reg_write(BACKUP_REG_LORA_PREV_DR, new_dr);
        }
    }
}

static void _init_radio_and_send(kernel_pid_t radio_pid) {
    msg_t reply;
    _msg_send_receive(radio_pid, CTLMSG_TYPE1_RADIO | CTLMSG_TYPE2_INIT, 0, &reply);
    LOG_INFO("[main] sending data ...\n");

    uint64_t next_tx_allowed;
    uint64_t now = node_time_get_utc_ms();

    backup_reg_read_u64(BACKUP_REG_NEXT_TX_ALLOWED_1, &next_tx_allowed);

    if (now > next_tx_allowed) {
        _radio_update_dr();

        uint32_t toa_sum_us = _send_available_data(radio_pid);

        uint32_t seconds_before_next_tx = toa_sum_us / 1000 / 10;
        LOG_DEBUG("[main] next transmission can only happen after %lu seconds\n",
                seconds_before_next_tx);

        next_tx_allowed = node_time_get_utc_ms() + toa_sum_us / 10;

        backup_reg_write_u64(BACKUP_REG_NEXT_TX_ALLOWED_1, next_tx_allowed);

        xtimer_usleep(3000 * 1000);
    }
    else {
        LOG_INFO("[main] TOA exhausted ... wait another %lu s\n", (uint32_t)(next_tx_allowed - now) / 1000);
    }

    LOG_INFO("[main] shutting down radio ...\n");
    _send_msg(radio_pid, CTLMSG_TYPE1_RADIO | CTLMSG_TYPE2_POWER_OFF, 0);
    xtimer_usleep(1000 * 200);
}

static void sense(kernel_pid_t eco_pid, kernel_pid_t sensors_pid,
                  kernel_pid_t radio_pid, eco_t *eco) {
    msg_t m;

    tx_task_spec = NULL;
    gps_task_spec = NULL;
    dust_task_spec = NULL;
    env_task_spec = NULL;

    sds011_data_t *sds_data = NULL;

    LOG_INFO("[main] starting continuous power measurements...\n");

    clear_buffs(cbor_buffs, CBOR_OBJ_CNT);

    bool measure_cycle_consumption =
        eco->history.e_use_slot_cnt < 1 ||
        ((eco->history.e_use_slot_cnt % POWER_TRACE_AFTER_SENSING_N_CYCLES) == 0);
    if (measure_cycle_consumption) {
        /* enable continuous measurement while performing sensing */
        _send_msg(eco_pid, CTLMSG_TYPE1_POWER_MEASURE | CTLMSG_TYPE2_MEASURE_CONTINUOUS, ECO_SAMPLE_BUFFER_CNT);
    }


    gps_task_spec = _tell_eco_about_task_start(eco_pid);
    bool got_gps_fix = _gps_stuff(sensors_pid, &gps_data);
    _tell_eco_about_task_stop(eco_pid, gps_task_spec);

    dust_task_spec = _tell_eco_about_task_start(eco_pid);

    /* reading all sensors is done depending on the available energy.
       There are different minimum SOC-threshholds for each sensing operation */
    _send_msg(sensors_pid, CTLMSG_TYPE1_DUST_SENSOR | CTLMSG_TYPE2_INIT, 0);
    _send_msg(sensors_pid, CTLMSG_TYPE1_DUST_SENSOR | CTLMSG_TYPE2_MEASURE, 0);
    //* wait for a message that contains the formatted data */
    /* after telling the sensors thread that we want a GPS measurement wait
       till we get a valid fix but timeout if it takes to long ( 10% margin) */
    if (xtimer_msg_receive_timeout(&m, (SENSORS_SDS011_AVG_CNT * 1000
                   + SENSORS_DSDS011_WARMUP_TIME_MS) * 2 * 1000) >= 0) {
        LOG_DEBUG("[main] got msg (type 0x%02X) from %" PRIkernel_pid " (dust values)\n",
                  m.type, m.sender_pid);

        if (m.type == CTLMSG_TYPE2_DATA_RAW) {
            LOG_DEBUG("got RAW reading\n");
            sds_data = m.content.ptr;
        }
    }
    else {
        LOG_INFO("[main] dust sensor timeout!\n");
    }

    LOG_DEBUG("[main] turning off dust sensor...\n");
    _send_msg(sensors_pid, CTLMSG_TYPE1_DUST_SENSOR | CTLMSG_TYPE2_POWER_OFF, 0);

    _tell_eco_about_task_stop(eco_pid, dust_task_spec);

    env_task_spec = _tell_eco_about_task_start(eco_pid);

    _send_msg(sensors_pid, CTLMSG_TYPE1_ENV_SENSOR | CTLMSG_TYPE2_INIT, 0);

    _send_msg(sensors_pid, CTLMSG_TYPE1_ENV_SENSOR | CTLMSG_TYPE2_MEASURE, 0);

    msg_t reply;

    bmx_measure_t *bmx_meas = NULL;

    if (xtimer_msg_receive_timeout(&reply, 10 * 1000 * 1000) >= 0) {
        LOG_DEBUG("[main] got msg (type 0x%02X) from %" PRIkernel_pid " (BME280 values)\n",
                  m.type, m.sender_pid);
            bmx_meas = (bmx_measure_t*)reply.content.ptr;
    }
    else {
        LOG_ERROR("[main] bmx sensor timeout!\n");
    }

    _tell_eco_about_task_stop(eco_pid, env_task_spec);

    cbor_buffs[BUFF_IDX_ECO].len = cbor_util_eco_encode_compact(eco,
                                                cbor_buffs[BUFF_IDX_ECO].buf,
                                                CBOR_MAX_BUFFER_LEN);

    LOG_DEBUG("CBOR encoded eco stats have %u bytes\n", cbor_buffs[BUFF_IDX_ECO].len);

    if (cbor_buffs[BUFF_IDX_ECO].len > 0) {
        _log_print_hex(cbor_buffs[BUFF_IDX_ECO].buf, cbor_buffs[BUFF_IDX_ECO].len);
    }

    printf("encoding sensed data...\n");
    cbor_buffs[BUFF_IDX_SENSE].len = cbor_util_sense_encode(
                                             sds_data, bmx_meas,
                                             got_gps_fix ? &gps_data : NULL,
                                             eco->cycle_cnt,
                                             cbor_buffs[BUFF_IDX_SENSE].buf,
                                             CBOR_MAX_BUFFER_LEN);

    printf("sensed data has %d bytes:\n", cbor_buffs[BUFF_IDX_SENSE].len);

    if (cbor_buffs[BUFF_IDX_SENSE].len > 0) {
        _log_print_hex(cbor_buffs[BUFF_IDX_SENSE].buf, cbor_buffs[BUFF_IDX_SENSE].len);
    }

    bool data_to_send = false;
    for (size_t i = 0; i < CBOR_OBJ_CNT; i++) {
        if (cbor_buffs[i].len > 0) {
            data_to_send = true;
        }
    }

    if (data_to_send) {
        LOG_DEBUG("going to init radio...\n");
        tx_task_spec = _tell_eco_about_task_start(eco_pid);
        _init_radio_and_send(radio_pid);
        _tell_eco_about_task_stop(eco_pid, tx_task_spec);
    }
    else {
        LOG_ERROR("[main] cbor payload empty!\n");
    }

    if (measure_cycle_consumption) {
        eco_measurement_set_t avg;
        _send_msg(eco_pid, CTLMSG_TYPE1_POWER_MEASURE |
                           CTLMSG_TYPE2_REQUEST_AVG, (uint32_t)&avg);

        msg_receive(&m);
        LOG_DEBUG("[main] got msg from %" PRIkernel_pid "\n", m.sender_pid);

        if (m.type == CTLMSG_TYPE2_DATA_RAW) {
            eco_measurement_set_t *avg = (eco_measurement_set_t*)m.content.ptr;

            LOG_DEBUG("[main] stats of this cycle: %"PRIu32" mV | %"PRId32" uA | %"
                   PRId32" uW | %"PRIu32" us\n", avg->bus_voltage_mv,
                   avg->current_ua, avg->power_uw, avg->t_us);

            /* TODO: add interface for accessing this data */
            if (gps_task_spec != NULL) {
                LOG_DEBUG("[main] GPS consumed:          %12"PRId32" uWs\n",
                       (int32_t)(gps_task_spec->e_pws / 1000000));
            }

            if (dust_task_spec != NULL) {
                LOG_DEBUG("[main] SDS011 sensing consumed: %12"PRId32" uWs\n",
                       (int32_t)(dust_task_spec->e_pws / 1000000));
            }

            if (env_task_spec != NULL) {
                LOG_DEBUG("[main] BME consumed:          %12"PRId32" uWs\n",
                       (int32_t)(env_task_spec->e_pws / 1000000));
            }

            if (tx_task_spec != NULL) {
                LOG_DEBUG("[main] LoRa consumed:           %12"PRId32" uWs\n",
                       (int32_t)(tx_task_spec->e_pws / 1000000));
            }
        }

        LOG_INFO("[main] powering off power measurement\n");
        _send_msg(eco_pid, CTLMSG_TYPE1_POWER_MEASURE | CTLMSG_TYPE2_POWER_OFF, 0);
    }
}

int main(void)
{
    LOG_INFO("\n\n\nSANE bus sensor app running on RIOT");
    _obligatory_init();

    LOG_DEBUG("[main] init eco...\n");
    /* after the system is booted and all required peripherals are in a well
       defined state start thread for the following functionalities:
        1) self-measurement of power consumption
           -> this is done first because it provides information
              that is required to decide if we should actually continue with
              further operations or go back to sleep to collect more energy

        2) (evironmental) sensor handling
        3) uplink radio
    */
    kernel_pid_t eco_pid = eco_init_thread(eco);
    msg_t reply;
    _msg_send_receive(eco_pid, CTLMSG_TYPE1_POWER_MEASURE | CTLMSG_TYPE2_INIT,
                      0, &reply);

    /* TODO replace this by a more restricted return value once
       it's well defined what information needs to be passed */
    eco_t *eco = (eco_t*)reply.content.ptr;


    uint32_t sys_state = 0;
    backup_reg_read(BACKUP_REG_SYS_STATE, &sys_state);

    if (!(sys_state & SYS_STATE_RTC_SYNCED)) {
        gps_data_t gps_data;
        kernel_pid_t sensors_pid = sensors_init_thread();
        if (_try_gps_fix(sensors_pid, GPS_COLD_START_MAX_TTF_WAIT_US,
                         &gps_data, true)) {
            _sync_with_gps_time(&gps_data);

            timestamp_t now_ts;
            node_time_get(&now_ts);
            uint64_t now = node_time_to_utc_ms(&now_ts);
            /* by default set last sensing to now to avoid sensing at first */
            /* maybe just add another check that also looks at the energy availability before sensing? */
            _backup_last_sensing_time(now);
            backup_reg_write_u64(BACKUP_REG_NEXT_TX_ALLOWED_1, now);
        }
        else {
            LOG_INFO("[main] couldn't get GPS fix, but still need to sync RTC"
                     " -> back to sleep for now\n");
            _node_allow_power_down(eco_pid);
        }
        _node_allow_power_down(eco_pid);
    }

    uint64_t now_utc_ms = node_time_get_utc_ms();
    uint64_t last_sensing_time_utc_ms = _get_last_sensing_time();

    int64_t sensed_seconds_ago = (now_utc_ms - last_sensing_time_utc_ms) / 1000;

    LOG_INFO("[main] sensed %"PRIu32" s ago\n", (uint32_t)sensed_seconds_ago);

    if (sensed_seconds_ago >= eco->sensing_period_s) {
        LOG_INFO("[main] last sensing cycle was over %"PRIu32
               " seconds ago! -> sense now!\n", eco->sensing_period_s);
        kernel_pid_t sensors_pid = sensors_init_thread();
        kernel_pid_t radio_pid = radio_init_thread();
        LOG_INFO("[main] starting sensing... \n");
        sense(eco_pid, sensors_pid, radio_pid, eco);
        _backup_last_sensing_time(node_time_get_utc_ms());
    }
    else {
        LOG_INFO("[main] wait another %"PRIu32
               " s before sensing the next time...\n",
               (uint32_t)(eco->sensing_period_s - sensed_seconds_ago));
        LOG_INFO("[main] no environmental sensing... \n");
        radio_quick_power_down();
    }

    LOG_INFO("[main] allowing power down now...\n");
    _node_allow_power_down(eco_pid);

    return 0;
}
