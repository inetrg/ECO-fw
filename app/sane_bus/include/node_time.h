/*
 * Copyright (C) 2018 Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
* @defgroup    node_time
* @ingroup     app_sane
* @brief       A helper module to ease usage of RTC together with sntp
* @{
*
* @file
* @brief       Definitions for the node_time helper module
*
* @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
*/
#ifndef NODE_TIME_H
#define NODE_TIME_H

#include <stdint.h>
#include "time.h"
//#include "timex.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct {
    struct tm tm;
    uint16_t ms;
} timestamp_t;

/* maximum rtc calibration values for stm32-l476rg (and probably others) */
#define RTC_CAL_REG_MAX     ( 512)
#define RTC_CAL_REG_MIN     (-511)

/* The cal_cnt register is counted with RTCCLK. The registers RTC_CALM and
 * RTC_CALP can then be used to effectively lower or increase this count
 * in the range by -512 to 511 by masking/inserting single clock counts */
#define RTC_CAL_CUNT_REG_MAX (2L<<19)

/* Increasing RTC_CALM by 1 corresponds to slowing down the RTC by this many
 * parts per billion */
#define RTC_CAL_DIGIT_PPB (1000000000L / (RTC_CAL_CUNT_REG_MAX))

/* Nucleo64 boards (board rev. MB1136C-03) use a ABS25-32.768KHZ-6-T crystal for LSE
   (RTC). The datasheet states 20 PPM general frequency tolerance. Using the additional
   specifications for aging, temperature coefficient etc. this leads to the following
   worst-case value of over the full temperature range */
#define NODE_TIME_RTC_CLOCK_CRYSTAL_PPM             (30)

/* Defines the resolution the RTC can be calibrated with */
#define NODE_TIME_RTC_CLOCK_CALIBRATION_PPM         (1)

/* This specifies the maximum time offset your application can live with.
   It is used to calculate NODE_TIME_INIT_DRIFT_ACC_MAX_S as the maximum time
   between clock syncs, that are needed to fullfil this requirement. */
#define NODE_TIME_MAX_TOLERATED_OFFSET_MS   (50)

/* based on the worst case hardware inaccuracies we can wait this many seconds to
   still fulfil the NODE_TIME_MAX_TOLERATED_OFFSET_MS constraint.
   Keep in mind that when setting NODE_TIME_MAX_TOLERATED_OFFSET_MS to a relatively
   small value can lead to many updates in short time and errors of measuring and
   updating the time may become significant.
   To determine the clock drift as accurate as possible, it's best to measure
   over a long period */
#define NODE_TIME_MAX_SYNC_INTERVAL_FOR_DRIFT(PPM) ((NODE_TIME_MAX_TOLERATED_OFFSET_MS * MS_PER_SEC) /\
                                                   ((PPM * MS_PER_SEC * MS_PER_SEC) / 1000000))

/* define how many sync measurements are averaged */
#define NODE_TIME_SYNC_AVG (5)

#define NODE_TIME_SNTP_SYNC_TIMEOUT_US (500000)
#define NODE_TIME_FALLBACK_RTC_ALARM_S (30)

#define TM_YEAR_OFFSET      (1900)

int node_time_get(timestamp_t *time);
uint64_t node_time_get_utc_ms(void);
uint32_t node_time_to_seconds_of_day(timestamp_t *time);
uint32_t node_time_get_seconds_of_day(void);
void node_time_print(timestamp_t *time);
int node_time_get_time_string(timestamp_t *time, char *str, size_t len);
int node_time_get_date_string(timestamp_t *time, char *str, size_t len);
uint32_t node_time_get_last_sync_utc_s(void);
uint32_t node_time_get_seconds_since_last_sync(void);
uint64_t node_time_to_utc_ms(timestamp_t *time);
void node_time_save_power_down_time_ms(void);
uint64_t node_time_get_power_down_time_utc_ms(void);
uint64_t node_time_get_boot_time_utc_ms(void);
uint32_t node_time_get_prev_sleep_dur_ms(void);
int node_time_init(void);
int node_time_warm_init(void);
int node_time_set_rtc_alarm_in_seconds(uint32_t seconds);
int node_time_sync(void);
int node_time_get_ntp_rtc_diff_us(int64_t *diff, int avg_cnt);
int32_t node_time_get_drift_cal_ppb(void);
int node_time_adjust_rtc_us(int64_t offset_us, int32_t drift_ppb);
void tm_add_seconds(struct tm *time, int32_t seconds);
void node_time_set_utc_ms(uint64_t utc_ms);

#ifdef __cplusplus
} /* end extern "C" */
#endif

#endif /* NODE_TIME_H */
/** @} */
