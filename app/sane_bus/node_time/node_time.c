#include <stdio.h>
#include <string.h>

#include "timex.h"
#include "xtimer.h"
#include "mutex.h"
#include "board.h"
#include "stmclk.h"
#include "periph/rtc.h"
//#include "net/sntp.h"
#include "node_time.h"
#include "backup_reg.h"
#include "backup_reg_alloc.h"
#define LOG_LEVEL LOG_ALL
#include "log.h"
#include "periph/pm.h"


mutex_t mutex_rtc_sync = MUTEX_INIT;
int64_t alarm_as_utc_us;

static timestamp_t boot_time;
static uint64_t boot_time_utc_ms;
static uint64_t power_down_time_utc_ms;
static uint64_t prev_sleep_dur_ms;

static uint32_t _ssr_to_ms(uint32_t ssr){
    ssr = (ssr & RTC_SSR_SS_Msk) >> RTC_SSR_SS_Pos;
    uint32_t prediv_s = (RTC->PRER & RTC_PRER_PREDIV_S_Msk) >> RTC_PRER_PREDIV_S_Pos;
    return (MS_PER_SEC * (prediv_s - ssr)) / (prediv_s + 1);
}

static uint32_t _ms_to_ssr(uint32_t ms){
    uint32_t prediv_s = (RTC->PRER & RTC_PRER_PREDIV_S_Msk) >> RTC_PRER_PREDIV_S_Pos;
    return (prediv_s + 1) * ms / MS_PER_SEC;
}

static inline void _save_last_sync_time(const struct tm *time){
    struct tm t;
    memcpy(&t, time, sizeof(struct tm));
    time_t time_sec = mktime(&t);
    backup_reg_write(BAKCUP_REG_LAST_SYNC, (uint32_t)time_sec);
}

void tm_add_seconds(struct tm *time, int32_t seconds){
    time_t time_sec = mktime(time);
    time_sec += seconds;
    struct tm *new_time = gmtime(&time_sec);
    memcpy(time, new_time, sizeof(struct tm));
}

/**
 * This callback is used to get (sub-second) sync to the rtc
 * as this call gets issued by an interrupt the call should be issued pretty close to
 * the alarm target time that we set before. On platforms where you have access to a
 * sub second register it would save some time to use that instead.
 * The upside of this is that it can also be used with RTCs that don't have sub second
 * resolution counters available.
 *
 */
static void sync_cb(void *arg)
{
    (void)arg;
    /* give mutex back to node_time_sync_rtc */
    mutex_unlock(&mutex_rtc_sync);
}

/* poor mans watchdog */
static void alarm_cb(void *arg)
{
    (void)arg;
    printf("node_time alarm_cb\n\n\n");
    pm_reboot();
}

int node_time_get(timestamp_t *time){
    uint32_t ssr = RTC->SSR;
    time->ms = _ssr_to_ms(ssr);

    int res = rtc_get_time(&time->tm);
    return res;
}

uint64_t node_time_get_utc_ms(void){
    timestamp_t time;
    node_time_get(&time);
    return node_time_to_utc_ms(&time);
}

void node_time_set_utc_ms(uint64_t utc_ms)
{
    time_t utc_seconds = utc_ms / 1000;
    uint32_t ms = utc_ms % 1000;

    struct tm *new_time = gmtime(&utc_seconds);

    RTC->SSR = _ms_to_ssr(ms);
    rtc_set_time(new_time);
}

uint32_t node_time_to_seconds_of_day(timestamp_t *time){
    return (time->tm.tm_hour * 60 + time->tm.tm_min) * 60 +
           time->tm.tm_sec;
}

uint32_t node_time_get_seconds_of_day(void){
    timestamp_t time;
    node_time_get(&time);
    return node_time_to_seconds_of_day(&time);
};

void node_time_print(timestamp_t *time){
    printf("%04d-%02d-%02d %02d:%02d:%02d.%03d\n",time->tm.tm_year + TM_YEAR_OFFSET,
                                                  time->tm.tm_mon + 1,
                                                  time->tm.tm_mday,
                                                  time->tm.tm_hour,
                                                  time->tm.tm_min,
                                                  time->tm.tm_sec,
                                                  time->ms);
}

int node_time_get_time_string(timestamp_t *time, char *str, size_t len){
    return snprintf(str, len, "%04d-%02d-%02d %02d:%02d:%02d.%03d", time->tm.tm_year + TM_YEAR_OFFSET,
                                                          time->tm.tm_mon + 1,
                                                          time->tm.tm_mday,
                                                          time->tm.tm_hour,
                                                          time->tm.tm_min,
                                                          time->tm.tm_sec,
                                                          time->ms);
}

int node_time_get_date_string(timestamp_t *time, char *str, size_t len){
    return snprintf(str, len, "%04d-%02d-%02d", time->tm.tm_year + TM_YEAR_OFFSET,
                                                  time->tm.tm_mon + 1,
                                                  time->tm.tm_mday);
}

uint32_t node_time_get_last_sync_utc_s(void){
    time_t last_sync = 0;
    backup_reg_read(BAKCUP_REG_LAST_SYNC, (uint32_t*)&last_sync);
    return last_sync;
}


uint32_t node_time_get_seconds_since_last_sync(void){
    uint32_t last_sync = node_time_get_last_sync_utc_s();

    if (last_sync == 0){
        return UINT32_MAX;
    }

    struct tm rtc_now;
    rtc_get_time(&rtc_now);
    time_t time_now = mktime(&rtc_now);

    return time_now - last_sync;
}

uint64_t node_time_to_utc_ms(timestamp_t *time){
    return MS_PER_SEC * mktime(&time->tm) + time->ms;
}

void node_time_save_power_down_time_ms(void){
    uint64_t time_utc_ms = node_time_get_utc_ms();

    backup_reg_write(BAKCUP_REG_POWER_DOWN_UTC_MS_H, (uint32_t)(time_utc_ms >> 32));
    backup_reg_write(BAKCUP_REG_POWER_DOWN_UTC_MS_L, (uint32_t)(time_utc_ms & 0xFFFFFFFFU));
}

uint64_t node_time_get_power_down_time_utc_ms(void){
    return power_down_time_utc_ms;
}

uint64_t node_time_get_boot_time_utc_ms(void){
    return boot_time_utc_ms;
}

uint32_t node_time_get_prev_sleep_dur_ms(void){
    return prev_sleep_dur_ms;
}

int node_time_init(void){
    timestamp_t rtc_time;

    LOG_DEBUG("bootstraping rtc initialization...\n");

    rtc_poweron();
    rtc_init();

    /* default time that is set on reset (should be updated by sntp later) */
    rtc_time.ms = 0;
    rtc_time.tm.tm_sec  = 00;
    rtc_time.tm.tm_min  = 58;
    rtc_time.tm.tm_hour = 7;
    rtc_time.tm.tm_mday = 1;
    rtc_time.tm.tm_mon  = 3 - 1;
    rtc_time.tm.tm_year = 2018 - TM_YEAR_OFFSET;

    node_time_print(&rtc_time);
    rtc_set_time(&rtc_time.tm);
    return 0;
}

int node_time_warm_init(void){
    uint32_t time_utc_ms_h;
    uint32_t time_utc_ms_l;
    node_time_get(&boot_time);
    boot_time_utc_ms = node_time_to_utc_ms(&boot_time);
    backup_reg_read(BAKCUP_REG_POWER_DOWN_UTC_MS_H, &time_utc_ms_h);
    backup_reg_read(BAKCUP_REG_POWER_DOWN_UTC_MS_L, &time_utc_ms_l);
    power_down_time_utc_ms = ((uint64_t)time_utc_ms_h << 32) | time_utc_ms_l;
    prev_sleep_dur_ms = boot_time_utc_ms - power_down_time_utc_ms;
    LOG_DEBUG("\n\n\nslept for %lu ms\n", (uint32_t)prev_sleep_dur_ms);
    return 0;
}

int node_time_set_rtc_alarm_in_seconds(uint32_t seconds){
    struct tm alarm_time;
    rtc_get_time(&alarm_time);
    tm_add_seconds(&alarm_time, seconds);
    rtc_set_alarm(&alarm_time, alarm_cb, NULL);
    return 0;
}

int node_time_sync(void) {
    uint32_t last_sync = node_time_get_seconds_since_last_sync();
    int64_t offset_us;
    int64_t drift_ppb;
    bool synced_before = false;

    while(node_time_get_ntp_rtc_diff_us(&offset_us, NODE_TIME_SYNC_AVG) != 0) {
      LOG_DEBUG("sync with ntp server failed!\n");
      return -1;
    }

    if(last_sync != UINT32_MAX && last_sync > 0){
        drift_ppb = offset_us * 1000 / (int64_t)last_sync;
        LOG_DEBUG("time drifted by %d.%03d ms in %lu seconds (%ld.%03ld ppm)\n", (int)(offset_us / US_PER_MS),
               (int)(offset_us > 0 ? (offset_us % US_PER_MS) : ((offset_us*-1) % US_PER_MS)),
               last_sync,
               (int32_t)(drift_ppb / 1000),
               (int32_t)((drift_ppb > 0 ? drift_ppb: drift_ppb * -1) % 1000));
        synced_before = true;
    } else{
        LOG_DEBUG("syncing for the first time\n");
    }

    return node_time_adjust_rtc_us(offset_us, (synced_before) ? drift_ppb : 0);
}

#ifdef UPLINK_LOWPAN_ENABLED
int node_time_get_ntp_rtc_diff_us(int64_t *diff, int avg_cnt){
    int first_sleep = 1;
    int nsync = 0;
    int64_t diffs[avg_cnt];
    char addr_str[] = NTP_SERVER_ADDR;

    sock_udp_ep_t server = { .port = NTP_PORT, .family = AF_INET6 };
    ipv6_addr_t *addr = (ipv6_addr_t *)&server.addr;

    kernel_pid_t src_iface = ipv6_addr_split_iface(addr_str);
    if (src_iface == -1) {
        src_iface = KERNEL_PID_UNDEF;
    }

    if (ipv6_addr_from_str(addr, addr_str) == NULL) {
        LOG_ERROR("error: malformed address");
        return -1;
    }

    if (ipv6_addr_is_link_local(addr) || (src_iface != KERNEL_PID_UNDEF)) {
        size_t ifnum = gnrc_netif_numof();

        if (src_iface == KERNEL_PID_UNDEF) {
            if (ifnum == 1) {
                src_iface = gnrc_netif_iter(NULL)->pid;
            }
            else {
                LOG_ERROR("error: link local target needs interface parameter (use 'address%%ifnum')\n");
                return -2;
            }
        }
        else {
            if (gnrc_netif_get_by_pid(src_iface) == NULL) {
                LOG_ERROR("error: %"PRIkernel_pid" is not a valid interface.\n", src_iface);
                return -3;
            }
        }
        server.netif = src_iface;
    }

    /* try to sync <avg_cnt> times by best effort - if a sync fails dont retry */
    while (avg_cnt--) {
        if( sntp_sync(&server, NODE_TIME_SNTP_SYNC_TIMEOUT_US) == 0 ){
            struct tm time;
            rtc_get_time(&time);

            if (first_sleep){
                /* the first time we set an alarm we add two seconds
                   to be sure not to miss the alarm */
                tm_add_seconds(&time, 2);
                first_sleep = 0;
            }else{
                /* knowing the timeout for sntp_sync and assuming that no other
                   operations are done in parallel we can be sure that an alarm
                   scheduled for the next second will not be missed */
                tm_add_seconds(&time, 1);
            }

            /* this will be the the exact time of the rtc at the moment the
               interrupt callback is entered (the interrupt latency is ignored here,
               but in relation to the simple sntp logic used that's insignificant)*/
            alarm_as_utc_us = mktime(&time) * US_PER_SEC;

            mutex_lock(&mutex_rtc_sync);
            rtc_set_alarm(&time, sync_cb, NULL);

            /* sync with the rtc callback */
            mutex_lock(&mutex_rtc_sync);
            diffs[nsync++] = sntp_get_unix_usec() - alarm_as_utc_us;

            /* reset mutex */
            mutex_unlock(&mutex_rtc_sync);
        }
    }

    LOG_DEBUG("did %d successful syncs with the ntp server\n", nsync);

    /* if no a single sync was successful */
    if (nsync == 0) {
        return -4;
    }

    /* average all diffs we got. This could be improved by filtering values
       based on the standard deviation. This is ignored for now */
    for(int i = 0; i < nsync; i++){
        if( i == 0){
            *diff = diffs[i];
        }else{
            *diff = (*diff + diffs[i]) / 2;
        }
    }

    return 0;
}
#endif

int32_t node_time_get_drift_cal_ppb(void){
    uint32_t cal = RTC->CALR;
    uint32_t calm = ((cal & RTC_CALR_CALM_Msk) >> RTC_CALR_CALM_Pos);
    uint32_t calp = ((cal & RTC_CALR_CALP_Msk) >> RTC_CALR_CALP_Pos);
    return (512 * calp - calm) * RTC_CAL_DIGIT_PPB;
}

int node_time_adjust_rtc_us(int64_t offset_us, int32_t drift_ppb){
    struct tm rtc_now;
    uint32_t cal = RTC->CALR;
    uint32_t calm = ((cal & RTC_CALR_CALM_Msk) >> RTC_CALR_CALM_Pos);
    uint32_t calp = ((cal & RTC_CALR_CALP_Msk) >> RTC_CALR_CALP_Pos);
    int32_t cal_old = 512 * calp - calm;
    int32_t ppb_as_cal_steps = (drift_ppb + (drift_ppb < 0 ? (RTC_CAL_DIGIT_PPB/-2) : (RTC_CAL_DIGIT_PPB/2))) / RTC_CAL_DIGIT_PPB;
    int32_t cal_new = (cal_old + ppb_as_cal_steps);

    if (cal_new > RTC_CAL_REG_MAX) {
        cal_new = RTC_CAL_REG_MAX;
    } else if (cal_new < RTC_CAL_REG_MIN) {
        cal_new = RTC_CAL_REG_MIN;
    }

    /* if positive calibration (faster clock) is needed,
       always use CALP to add 512 clocks... */
    calp = (cal_new > 0);

    /* ...then subtract the difference to match the actual value cal_new */
    if (cal_new > 0) {
        calm = RTC_CAL_REG_MAX - cal_new;
    } else {
        calm = (cal_new * -1) % RTC_CAL_REG_MAX;
    }

    /* unlock RTC */
    stmclk_dbp_unlock();
    RTC->WPR = RTC_WRITE_PROTECTION_KEY1;
    RTC->WPR = RTC_WRITE_PROTECTION_KEY2;
    cal &= ~(RTC_CALR_CALM_Msk | RTC_CALR_CALP_Msk); /*clear old cal value */
    RTC->CALR = (cal | ((calm << RTC_CALR_CALM_Pos) | (calp << RTC_CALR_CALP_Pos)));
    while(RTC->ISR & RTC_ISR_RECALPF){} /* wait for the calibration to finish */
    RTC->WPR = RTC_WRITE_PROTECTION_WRONG_KEY; /* lock rtc registers */
    stmclk_dbp_lock();

    rtc_get_time(&rtc_now);
    timestamp_t new_rtc_time;
    memcpy(&new_rtc_time, &rtc_now, sizeof(struct tm));
    int32_t diff_sec = (int32_t)(offset_us / US_PER_SEC);
    int32_t diff_us = offset_us - (diff_sec * US_PER_SEC);

    /* the sleep will be needed to allign to the next full second.
       Depending on whether the correction is positive or negative we either
       wait a short amount of time so the rtc catches up to a full second or we
       let the rtc run as long as needed to add another second when syncing */
    uint32_t sync_sleep = (diff_us > 0) ? (US_PER_SEC - diff_us) : (uint32_t)(diff_us * -1);

    /* wait at least two seconds from now to sync with the rtc by an alarm interrupt */
    tm_add_seconds(&rtc_now, 2);

    /*add the measured difference plus two seconds needed to sync with the rtc
      and another second if the sync_sleep will result in waiting for another second */
    tm_add_seconds(&new_rtc_time.tm, diff_sec + 2 + (diff_us > 0));

    mutex_lock(&mutex_rtc_sync);
    rtc_set_alarm(&rtc_now, sync_cb, NULL);

    LOG_DEBUG("syncing rtc to: ");
    node_time_print(&new_rtc_time);

    /* sync with the rtc callback */
    mutex_lock(&mutex_rtc_sync);

    /* wait for the sub seconds to allign */
    xtimer_usleep(sync_sleep);

    /* set the new time instantly when synced to rtc (used to align to subseconds) */
    rtc_set_time(&new_rtc_time.tm);

    /* reset mutex */
    mutex_unlock(&mutex_rtc_sync);

    _save_last_sync_time(&new_rtc_time.tm);
    LOG_DEBUG("---> RTC speed changed by %ld ppb calibration value is now %ld\n", ppb_as_cal_steps * RTC_CAL_DIGIT_PPB, cal_new);

    return 0;
}
