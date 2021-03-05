/*
 * Copyright (C) 2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     power_measure
 * @{
 *
 * @file
 * @brief       Power self-measurement control interface
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */
#include <string.h>

#include "thread.h"
#include "kernel_types.h"
#include "msg.h"
#define LOG_LEVEL LOG_NONE
#include "log.h"
#include "control_messages.h"
#include "xtimer.h"
#include "eco.h"
#include "ina22x.h"
#include "ina22x_params.h"
#include "ina22x-regs.h"
#include "ina226-regs.h"
#include "node_time.h"
#include "gpio_wakeup.h"
#include "backup_reg.h"
#include "backup_reg_alloc.h"
#include "fmt.h"

#define INA22X_CONFIG_SINGLE_SHORT (INA22X_MODE_TRIGGER_SHUNT_AND_BUS | \
                                    INA226_VBUSCT_588_US | \
                                    INA226_VSHCT_588_US | \
                                    INA226_AVG_64)

#define INA22X_CONFIG_TRACE  (INA22X_MODE_CONTINUOUS_SHUNT_AND_BUS | \
                              INA226_VBUSCT_588_US | \
                              INA226_VSHCT_588_US | \
                              INA226_AVG_64)

#define INA22X_CONFIG_SINGLE_LONG (INA22X_MODE_TRIGGER_SHUNT_AND_BUS | \
                                   INA226_VBUSCT_2_116_MS | \
                                   INA226_VSHCT_2_116_MS | \
                                   INA226_AVG_1024)

#define INA22X_CONFIG_THREAD (INA22X_CONFIG_TRACE)

#define PERIODIC_RTC_WAKEUP_CHARGING_S     (20)
#define PERIODIC_RTC_WAKEUP_NOT_CHARGING_S (20)

/* TODO: these values can be used to statically setup prorites of measurements
   for every single timeslot. Higher values will allocate more energy.
   NOTE: for now this is only considered for discharging state */
uint32_t measurement_utility[ECO_PM_INFO_SLOTS_PER_DAY] =
                /* 02:00, 04:00, 06:00, 08:00, 10:00, 12:00, 14:00, 16:00, 18:00, 20:00, 22:00, 00:00*/
                /* 00:00, 02:00, 04:00, 06:00, 08:00, 10:00, 12:00, 14:00, 16:00, 18:00, 20:00, 22:00 */
                {  5,     5,     10,     10,    10,    10,    10,    10,    10,    10,    5,     5 };

/* TODO: use section attribute later to automatically put these variables
   to the retained ram section. For now care must be taken here! */
uint8_t *retained_ram = (uint8_t*)STM32_L476RG_SRAM2_START;
eco_t *eco = (eco_t*)STM32_L476RG_SRAM2_START;

/* the eco stack cannot live on the retained ram for now as this crashes the thread create */
char stack[THREAD_STACKSIZE_ECO];
msg_t msg_queue[THREAD_MSG_QUEUE_SIZE_ECO];

eco_measurement_set_t eco_samples[ECO_SAMPLE_BUFFER_CNT];

static void ina226_callback(void *arg)
{
    eco_t *eco = (eco_t*)arg;
    msg_t msg;
    msg.type = CTLMSG_TYPE1_POWER_MEASURE | CTLMSG_TYPE2_DATA_RAW;
    msg_send_int(&msg, eco->pid);
}

static eco_task_consumption_spec_t* _eco_notify_task_start(eco_t *eco) {
    eco_task_consumption_spec_t *tasks = eco->task_specs;

    //mutex_lock(&eco->mutex);

    for (int i = 0; i < ECO_ATTRIBUTION_TASKS_MAX_NUMOF; i++) {
        if (!tasks[i].used) {
            tasks[i].t_start_us = xtimer_now_usec();
            tasks[i].used = true;
            //mutex_unlock(&eco->mutex);
            return &tasks[i];
        }
    }

    //mutex_unlock(&eco->mutex);

    return NULL;
}

static void _eco_notify_task_stop(eco_task_consumption_spec_t *task_spec) {
    task_spec->t_end_us = xtimer_now_usec();
}

static void _eco_process_and_attribute(eco_t *eco,
                                       eco_measurement_set_t *measure,
                                       size_t cnt,
                                       eco_measurement_set_t *avg)
{
    int64_t voltage_sum = 0;
    int64_t current_sum = 0;
    int64_t power_sum = 0;
    int16_t bus_voltage_reg;
    int16_t shunt_voltage_reg;
    int16_t current_reg;
    int16_t power_reg;

    for (unsigned i = 0; i < cnt; i++) {
        bus_voltage_reg = measure[i].bus_voltage_reg;
        shunt_voltage_reg = measure[i].shunt_voltage_reg;
        current_reg = measure[i].current_reg;
        power_reg = measure[i].power_reg;
        measure[i].bus_voltage_mv = (bus_voltage_reg * INA226_BUS_VOLTAGE_LSB_UV) / MICROS_PER_MILLI;
        measure[i].shunt_voltage_uv = (shunt_voltage_reg * INA226_SHUNT_VOLTAGE_LSB_NV) / NANOS_PER_MICRO;
        measure[i].current_ua = (current_reg * INA22X_PARAM_LSB_NA) / NANOS_PER_MICRO;
        measure[i].power_uw = (current_reg < 0 ? -1 : 1) *
                              (INA226_POWER_LSB_CURRENT_LSB_RATIO *
                               INA22X_PARAM_LSB_NA * power_reg) / NANOS_PER_MICRO;
        if (avg != NULL) {
            voltage_sum += measure[i].bus_voltage_mv;
            current_sum += measure[i].current_ua;
            power_sum += measure[i].power_uw;
        }


        //mutex_lock(&eco->mutex);
        eco_task_consumption_spec_t *tasks = eco->task_specs;

        LOG_DEBUG("measure %d | t: %ld | p: %ld\n", i, measure[i].t_us, measure[i].power_uw);

        for (int t = 0; t < ECO_ATTRIBUTION_TASKS_MAX_NUMOF; t++) {
            if (tasks[t].used) {
                if ((measure[i].t_us >= tasks[t].t_start_us) &&
                    (measure[i].t_us <= tasks[t].t_end_us) &&
                    (i > 0) ) {
                    int64_t e_add = ((int64_t)measure[i].power_uw *
                                     (int64_t)(measure[i].t_us - measure[i-1].t_us));
                    tasks[t].e_pws += e_add;
#if LOG_LEVEL == LOG_DEBUG
                    char s64_str[32] = {0};
                    char s64_str2[32] = {0};
                    fmt_s64_dec(s64_str, e_add);
                    fmt_s64_dec(s64_str2, tasks[t].e_pws);
#endif
                    LOG_DEBUG("t%d | dt: %ld | p: %ld | adding %s pWs | sum: %s\n", t, measure[i].t_us - measure[i-1].t_us, measure[i].power_uw, s64_str, s64_str2);
                }
            }
            else {
                break;
            }
        }

        //mutex_unlock(&eco->mutex);
    }

    if (avg != NULL) {
        LOG_DEBUG("processed %d measurements\n", cnt);
        avg->bus_voltage_mv = voltage_sum / cnt;
        avg->current_ua = current_sum / cnt;
        avg->power_uw = power_sum / cnt;
        avg->t_us = measure[cnt - 1].t_us - measure[0].t_us;
    }
}

eco_t test_eco;
#define TEST_MEASURES (100)
eco_measurement_set_t measures[TEST_MEASURES];
void eco_test_attr(void) {
    eco_measurement_set_t avg;
    xtimer_init();
    //mutex_init(&test_eco.mutex);

    eco_task_consumption_spec_t *t1 = NULL;
    eco_task_consumption_spec_t *t2 = NULL;

    for (int i = 0; i < TEST_MEASURES; i++) {
        measures[i].bus_voltage_reg = 2600000 / INA226_BUS_VOLTAGE_LSB_UV;
        measures[i].shunt_voltage_reg = 20000 / INA226_SHUNT_VOLTAGE_LSB_NV;
        measures[i].current_reg = 60000000 / INA22X_PARAM_LSB_NA;
        measures[i].power_reg = 100000000 / (INA226_POWER_LSB_CURRENT_LSB_RATIO * INA22X_PARAM_LSB_NA);
        measures[i].t_us = xtimer_now_usec();

        if (i == 20) {
            t1 = _eco_notify_task_start(&test_eco);
        }

        if (i == 40) {
            _eco_notify_task_stop(t1);
        }

        if (i == 50) {
            t2 = _eco_notify_task_start(&test_eco);
        }

        if (i == 60) {
            _eco_notify_task_stop(t2);
        }

        xtimer_usleep(1000 * 20);


    }

    _eco_process_and_attribute(&test_eco,
                               &measures[0],
                               TEST_MEASURES,
                               &avg);

   for (int i = 0; i < ECO_ATTRIBUTION_TASKS_MAX_NUMOF; i++) {
       if (test_eco.task_specs[i].used) {
           char s64_str[32] = {0};
           fmt_s64_dec(s64_str, test_eco.task_specs[i].e_pws);
           printf("task %d:  %s pWs\n", i, s64_str);
       }
       else {
           printf("unused...\n");
           break;
       }
   }
}

static void _eco_process_measures(eco_measurement_set_t *measure, size_t cnt,
                                  eco_measurement_set_t *avg)
{
    int64_t voltage_sum = 0;
    int64_t current_sum = 0;
    int64_t power_sum = 0;
    int16_t bus_voltage_reg;
    int16_t shunt_voltage_reg;
    int16_t current_reg;
    int16_t power_reg;
    for (unsigned i = 0; i < cnt; i++) {
        bus_voltage_reg = measure[i].bus_voltage_reg;
        shunt_voltage_reg = measure[i].shunt_voltage_reg;
        current_reg = measure[i].current_reg;
        power_reg = measure[i].power_reg;
        measure[i].bus_voltage_mv = (bus_voltage_reg * INA226_BUS_VOLTAGE_LSB_UV) / MICROS_PER_MILLI;
        measure[i].shunt_voltage_uv = (shunt_voltage_reg * INA226_SHUNT_VOLTAGE_LSB_NV) / NANOS_PER_MICRO;
        measure[i].current_ua = (current_reg * INA22X_PARAM_LSB_NA) / NANOS_PER_MICRO;
        measure[i].power_uw = (current_reg < 0 ? -1 : 1) *
                              (INA226_POWER_LSB_CURRENT_LSB_RATIO *
                               INA22X_PARAM_LSB_NA * power_reg) / NANOS_PER_MICRO;
        if (avg != NULL) {
            voltage_sum += measure[i].bus_voltage_mv;
            current_sum += measure[i].current_ua;
            power_sum += measure[i].power_uw;
        }
    }

    if (avg != NULL) {
        LOG_DEBUG("processed %d measurements\n", cnt);
        avg->bus_voltage_mv = voltage_sum / cnt;
        avg->current_ua = current_sum / cnt;
        avg->power_uw = power_sum / cnt;
        avg->t_us = measure[cnt - 1].t_us - measure[0].t_us;
    }
}

int _ina22x_read_all_regs(ina22x_t *dev, eco_measurement_set_t *measure, bool clear_interrupt)
{
  bool all_read_success = false;

  /* TODO add timeout condition in case the measurement fails completely */
  while (!all_read_success) {

      all_read_success = true;
      if ( ina22x_read_current_reg(dev, &measure->current_reg) != INA22X_OK) {
          LOG_ERROR("ina22x_read_current_reg [FAILED]\n");
          all_read_success = false;
      }

      if ( ina22x_read_bus_reg(dev, &measure->bus_voltage_reg) != INA22X_OK) {
          LOG_ERROR("ina22x_read_bus_reg [FAILED]\n");
          all_read_success = false;
      }

      if ( ina22x_read_shunt_reg(dev, &measure->shunt_voltage_reg) != INA22X_OK) {
          LOG_ERROR("ina22x_read_shunt_reg [FAILED]\n");
          all_read_success = false;
      }

      if ( ina22x_read_power_reg(dev,&measure->power_reg) != INA22X_OK) {
          LOG_ERROR("ina22x_read_power_reg [FAILED]\n");
          all_read_success = false;
      }

      if (all_read_success) {
         LOG_DEBUG("eco_read_all_regs [OK]\n");
         //all_read_success = true;
      }else{
          LOG_ERROR("eco_read_all_regs [FAILED] 1\n");
      }
  }

  if (clear_interrupt == true) {
    //DEBUG("clearing interrupt from ina22x...\n");
    uint16_t dummy;

    if (ina226_read_mask_enable_reg(dev, &dummy) != 0) {
        LOG_ERROR("eco_read_all_regs [FAILED] 2\n");
        return all_read_success ? 0 : -1;
    }
    else {
        LOG_DEBUG("ina226_read_mask_enable_reg [OK] 2\n");
    }
  }

  return all_read_success ? 0 : -2;
}

int _ina22x_init(eco_t *e)
{
    uint16_t ina_226_die_id;
    uint16_t ina_226_manufac_id;

    ina22x_t *ina = &e->ina22x_dev;

    if ((ina22x_init(ina, e->ina22x_params) != 0) ||
        (ina226_read_die_id_reg(ina, &ina_226_die_id) != 0) || ((ina_226_die_id != INA226_EXPECTED_DIE_ID)) ||
        (ina226_read_manufacturer_id_reg(ina, &ina_226_manufac_id) != 0) || (ina_226_manufac_id != INA226_EXPECTED_MANUF_ID) ||
        (ina226_write_mask_enable_reg(ina, 0) != 0)){
        //(ina226_activate_int(ina22x_dev, INA226_MASK_ENABLE_CNVR, ina22x_dev->alert_pin, NULL ) != 0) ) {
        LOG_ERROR("[eco] error when configuring ina226 [FAILED]\n");
        return -2;
    }

    if (ina22x_write_config_reg(ina, ina->config) != 0) {
        LOG_ERROR("[eco] ina22x_write_config_reg [FAILED]\n");
        return -4;
    }

    LOG_DEBUG("[eco] _ina22x_init [OK]\n");
    return 0;
}

static void _ina22x_warm_init(ina22x_t *ina22x_dev,
                              const ina22x_params_t *ina22x_params){
    ina22x_dev->model = ina22x_params->model;
    ina22x_dev->i2c = ina22x_params->i2c;
    ina22x_dev->addr = ina22x_params->addr;
    ina22x_dev->config = ina22x_params->config;
    ina22x_dev->cal = ina22x_params->cal;
    ina22x_dev->current_lsb_na = ina22x_params->current_lsb_na;
    ina22x_dev->alert_pin = ina22x_params->alert_pin;

    /*
    uint32_t sys_state = 0;
    backup_reg_read(BACKUP_REG_SYS_STATE, &sys_state);
    if (gpio_init(ina22x_dev->alert_pin, GPIO_IN_PU) == 0) {
        if(eco_alert_active(e)) {
            DEBUG("rehmon_alert_active -> reading sleep measurement\n");
            gpio_wakeup_clear(INA22X_ALERT_WKUP_PIN_IDX);
            gpio_wakeup_disable(INA22X_ALERT_WKUP_PIN_IDX);
            rehmon_measurement_set_t measure;
            bool read_success = rehmon_read_all_regs(rehmon, &measure, true) == 0;
            rehmon_power_down(rehmon);
            backup_reg_write(BACKUP_REG_SYS_STATE, sys_state & (~SYS_STATE_WAITING_FOR_REHMON_INTERRUPT));
            if (read_success) {
                rehmon_process_measures(&measure, 1);
                int32_t ua = measure.current_ua;
                int32_t mv = measure.bus_voltage_mv;
                int32_t p_off_uw = (int32_t)((int64_t)mv * ua / 1000);
                DEBUG("------------> p_off_uw: %ld\n", p_off_uw);
                uint32_t p_sleep_measure_cnt;
                uint32_t t_sleep_measure_avg_ms;
                backup_reg_read(BACKUP_REG_T_SLEEP_MEASURE_AVG_MS, &t_sleep_measure_avg_ms);
                backup_reg_read(BACKUP_REG_P_OFF_AVG_CNT, &p_sleep_measure_cnt);
                t_sleep_measure_avg_ms = (((int64_t)t_sleep_measure_avg_ms * p_sleep_measure_cnt) +
                                         node_time_get_prev_sleep_dur_ms()) / (p_sleep_measure_cnt + 1);
                backup_reg_write(BACKUP_REG_T_SLEEP_MEASURE_AVG_MS, t_sleep_measure_avg_ms);

                _update_avg_reg(BACKUP_REG_P_OFF_AVG_UW, BACKUP_REG_P_OFF_AVG_CNT, p_off_uw);

                return 0;
            }
            DEBUG("_read_rehmon_sleep_measure(): [FAILED]\n");
            return -1;
        } else {
            if (sys_state & SYS_STATE_WAITING_FOR_REHMON_INTERRUPT) {
                DEBUG("##### was waiting for REHMON, but alarm was not active!!!\n");
                gpio_wakeup_pull_config(INA22X_ALERT_WKUP_PIN_IDX, GPIO_IN_PU);
                gpio_wakeup_clear(INA22X_ALERT_WKUP_PIN_IDX);
                gpio_wakeup_disable(INA22X_ALERT_WKUP_PIN_IDX);
            }
        }
    }
    */
}

void _pretty_print_measure(eco_measurement_set_t *m, bool raw)
{
    if (raw) {
        printf("bus_voltage_reg: %"PRId16"\n",m->bus_voltage_reg);
        printf("shunt_voltage_reg: %"PRId16"\n",m->shunt_voltage_reg);
        printf("current_reg: %"PRId16"\n",m->current_reg);
        printf("power_reg: %"PRId16"\n",m->power_reg);
    }
    else {
        printf("bus_voltage_mv: %"PRIu32"\n",m->bus_voltage_mv);
        printf("shunt_voltage_uv: %"PRId32"\n",m->shunt_voltage_uv);
        printf("current_ua: %"PRId32"\n",m->current_ua);
        printf("power_uw: %"PRId32"\n",m->power_uw);
        printf("t_us: %"PRIu32"\n",m->t_us);
    }
}

int _request_single_long_measure(eco_t *eco){
    ina22x_t *ina = &eco->ina22x_dev;
    LOG_DEBUG("[eco] config: 0x%x\n", ina->config);
    if (ina22x_write_config_reg(ina, INA22X_CONFIG_SINGLE_LONG) == 0) {
        LOG_DEBUG("[eco] ina22x_write_config_reg [OK]\n");
        uint16_t dummy;
        if (ina226_read_mask_enable_reg(ina, &dummy) != 0) {
            LOG_ERROR("[eco] ina226_read_mask_enable [FAILED]\n");
            return -1;
        }
        if(ina226_activate_int(ina, INA226_MASK_ENABLE_CNVR,
                               ina->alert_pin, ina226_callback,
                               eco ) == 0 ) {
            LOG_ERROR("[eco] ina226_activate_int [OK]\n");
            return 0;
        }
        //gpio_irq_disable(ina->alert_pin);
        LOG_ERROR("[eco] ina226_activate_int [FAILED]\n");
        return -2;
    }

    return -3;
}

uint32_t _get_available_mws(eco_t *eco)
{
    eco_energy_history_t *h = &eco->history;

    uint32_t u = h->u_latest_mv;

    uint32_t all_mws = (SUPERCAP_CAPACITIANCE_F / 2 * ((u * u) / 1000));

    if (all_mws <= SUPERCAP_UNUSABLE_E_MWS) {
        return 0;
    }

    return all_mws - SUPERCAP_UNUSABLE_E_MWS;
}

#define ENERGY_STAT_NUM_WIDTH    "12"
#define ENERGY_STAT_CUR_MARK_POS "11"
#define ENERGY_STAT_TIM_MARK_POS "7"
#define ENERGY_STAT_NAME_WIDTH   "16"

static inline uint32_t _get_seconds_in_slot(uint32_t seconds_of_day){
    return seconds_of_day % (ECO_PM_INFO_SLOT_LEN_MINUTES * 60);
}

static inline uint32_t _get_slot_cnt(uint32_t start, uint32_t end)
{
    if (start <= end) {
        return end - start;
    }

    return ECO_PM_INFO_SLOTS_PER_DAY - start + end;
}

static inline uint32_t _get_prev_slot_idx(uint32_t idx)
{
    if (idx == 0) {
        return ECO_PM_INFO_SLOTS_PER_DAY - 1;
    }

    return idx -1;
}

static inline uint32_t _get_next_slot_idx(uint32_t idx)
{
    if (idx == ECO_PM_INFO_SLOTS_PER_DAY - 1) {
        return 0;
    }

    return idx + 1;
}

static inline uint32_t _get_prev_day_idx(uint32_t day_idx)
{
    if (day_idx == 0) {
        return ECO_STATS_DAY_CNT - 1;
    }

    return day_idx -1;
}

static inline uint32_t _get_next_day_idx(uint32_t day_idx)
{
    if (day_idx == ECO_STATS_DAY_CNT - 1) {
        return 0;
    }

    return day_idx + 1;
}

static inline uint32_t _get_day_idx_of_prev_slot(uint32_t day_idx, uint32_t slot_idx)
{
    if (slot_idx == 0) {
        _get_prev_day_idx(day_idx);
    }

    return day_idx;
}

static void _print_energy_stat_history(eco_t *eco){
    eco_energy_history_t *h = &eco->history;

    eco_day_stat_t *td = &h->days[h->current_day_idx];
    eco_day_stat_t *yd = &h->days[_get_prev_day_idx(h->current_day_idx)];

    /* TODO, change to iteratively updating these values */
    td->e_chg_sum_mws     = 0;
    td->e_chg_sum_now_mws = 0;
    td->e_use_sum_mws     = 0;

    /* smoe of this can be optimized away if the rest is calculated properly
       incremental */
    yd->e_chg_sum_mws     = 0;
    yd->e_chg_sum_now_mws = 0;
    yd->e_use_sum_mws     = 0;

    printf("%-"ENERGY_STAT_NAME_WIDTH"s\n", "timescale:");
    printf("%"ENERGY_STAT_NAME_WIDTH"s", "");
    for (unsigned i = 0; i < ECO_PM_INFO_SLOTS_PER_DAY; i++) {
        printf("%"ENERGY_STAT_TIM_MARK_POS"s%02d:%02d", "",
               (i * ECO_PM_INFO_SLOT_LEN_MINUTES) / 60,
               (i * ECO_PM_INFO_SLOT_LEN_MINUTES) % 60);
    }
    printf("\n");

    printf("%"ENERGY_STAT_NAME_WIDTH"s", "");
    for (unsigned i = 0; i < h->current_slot_idx; i++) {
        printf("%"ENERGY_STAT_NUM_WIDTH"s", "");
    }
    printf("%"ENERGY_STAT_TIM_MARK_POS"s""=====\n", "");

    printf("%-"ENERGY_STAT_NAME_WIDTH"s\n", "charge stats:");
    printf("%-"ENERGY_STAT_NAME_WIDTH"s", "current day:");
    for (unsigned i = 0; i < ECO_PM_INFO_SLOTS_PER_DAY; i++) {
        if (i < h->current_slot_idx) {
            /* use the value from the history */
            printf("%"ENERGY_STAT_NUM_WIDTH PRId32, td->e_chg_mws[i]);
            td->e_chg_sum_mws += td->e_chg_mws[i];
        }
        else if (i == h->current_slot_idx) {
            /* use the currently still accumulating value */
            uint32_t e_chg_slot_mws = (h->e_chg_slot_sum_uws / 1000);
            printf("%"ENERGY_STAT_NUM_WIDTH PRIu32, e_chg_slot_mws);
            td->e_chg_sum_mws += e_chg_slot_mws;
        }
        else{
            printf("%"ENERGY_STAT_NUM_WIDTH "s", "---");
        }
    }
    printf("\n");

    printf("%-"ENERGY_STAT_NAME_WIDTH"s", "history:");
    for (unsigned i = 0; i < ECO_PM_INFO_SLOTS_PER_DAY; i++) {
        printf("%"ENERGY_STAT_NUM_WIDTH PRId32, yd->e_chg_mws[i]);

        yd->e_chg_sum_mws += yd->e_chg_mws[i];

        /* for the prediction calculate how much energy was charged yesterday
           over the same time period (scale current slot linearly)*/
        if (i < h->current_slot_idx) {
            yd->e_chg_sum_now_mws += yd->e_chg_mws[i];

        }
        else if (i == h->current_slot_idx){
            yd->e_chg_sum_now_mws += yd->e_chg_mws[i]
                                     / ECO_PM_INFO_SLOT_LEN_MINUTES
                                     * (h->secs_in_slot / 60);
        }
    }
    printf("\n");

    printf("\n%-"ENERGY_STAT_NAME_WIDTH"s\n", "usage stats:");
    printf("%-"ENERGY_STAT_NAME_WIDTH"s", "current day:");
    for (unsigned i = 0; i < ECO_PM_INFO_SLOTS_PER_DAY; i++) {
        if (i < h->current_slot_idx) {
            /* use the value from the history */
            printf("%"ENERGY_STAT_NUM_WIDTH PRId32, td->e_use_mws[i]);
            td->e_use_sum_mws += td->e_use_mws[i];
        }
        else if (i == h->current_slot_idx) {
            /* use the currently still accumulating value */
            uint32_t e_use_slot_mws = (h->e_use_slot_sum_uws / 1000);
            printf("%"ENERGY_STAT_NUM_WIDTH PRIu32, e_use_slot_mws);
            td->e_use_sum_mws += e_use_slot_mws;
        }
        else{
            printf("%"ENERGY_STAT_NUM_WIDTH "s", "---");
        }
        //td->e_use_sum_mws += td->e_use_mws[i];
    }
    printf("\n");

    printf("%-"ENERGY_STAT_NAME_WIDTH"s", "history:");
    for (int i = 0; i < ECO_PM_INFO_SLOTS_PER_DAY; i++) {
        printf("%"ENERGY_STAT_NUM_WIDTH PRId32, yd->e_use_mws[i]);
    }
    printf("\n\n");

    printf("\n%-"ENERGY_STAT_NAME_WIDTH"s\n", "plan:");
    printf("%-"ENERGY_STAT_NAME_WIDTH"s", "E_alloc:");
    for (int i = 0; i < ECO_PM_INFO_SLOTS_PER_DAY; i++) {
        printf("%"ENERGY_STAT_NUM_WIDTH PRIu32, eco->e_alloc_mws[i]);
    }
    printf("\n");

    printf("%-"ENERGY_STAT_NAME_WIDTH"s", "t_sample:");
    for (int i = 0; i < ECO_PM_INFO_SLOTS_PER_DAY; i++) {
        printf("%"ENERGY_STAT_NUM_WIDTH PRIu32, eco->t_sense_s[i]);
    }
    printf("\n");

    printf("%-"ENERGY_STAT_NAME_WIDTH"s", "sense cnt:");
    for (int i = 0; i < ECO_PM_INFO_SLOTS_PER_DAY; i++) {
        printf("%"ENERGY_STAT_NUM_WIDTH PRIu32, eco->alloc_sensing_cycles[i]);
    }
    printf("\n");

    printf("%-"ENERGY_STAT_NAME_WIDTH"s", "utility:");
    for (int i = 0; i < ECO_PM_INFO_SLOTS_PER_DAY; i++) {
        printf("%"ENERGY_STAT_NUM_WIDTH PRIu32, measurement_utility[i]);
    }
    printf("\n\n");

    printf("energy balance for today (charged/discharged): %"PRIu32" / %"PRIu32
           "\n", td->e_chg_sum_mws, td->e_use_sum_mws);
}

/* if no measurement was performed before
   we need some charging history to bootstrap
   the duty cycle determination.
   For that the available SOC is just split
   evenly over the halve h the slots of the previous day, and middle of the day */
static void _set_initial_charging_history(eco_t *eco)
{
    eco_energy_history_t *h = &eco->history;

    h->current_day_idx = 0;
    h->current_slot_idx = node_time_to_seconds_of_day(&h->time_ref) /
                          ECO_PM_INFO_SLOT_LEN_SECONDS;

    uint32_t prev_day = _get_prev_day_idx(h->current_day_idx);

    uint32_t chg_per_slot = SUPERCAP_MAX_AVAIL_E_MWS * 2 / ECO_PM_INFO_SLOTS_PER_DAY;

    /* TODO: make this more configurable */
    for (unsigned i = 0; i < ECO_PM_INFO_SLOTS_PER_DAY / 2; i++) {
        h->days[h->current_day_idx].e_chg_mws[ECO_PM_INFO_SLOTS_PER_DAY / 4 + i]
                                                                 = chg_per_slot;
        if ((i + 1) == h->current_slot_idx)  {
            break;
        }
    }

    /* TODO: make this more configurable */
    for (int i = 0; i < ECO_PM_INFO_SLOTS_PER_DAY / 2; i++) {
        h->days[prev_day].e_chg_mws[ECO_PM_INFO_SLOTS_PER_DAY / 4 + i]
                                                                 = chg_per_slot;
    }
}

uint32_t _get_chg_day_sum_mws(eco_day_stat_t *day) {
    uint32_t mws = 0;

    for (unsigned i = 0; i < ECO_PM_INFO_SLOTS_PER_DAY; i++) {
        mws += day->e_chg_mws[i];
    }

    return mws;
}

int _get_slot_of_n_percent_chg(eco_day_stat_t *day, int percent){
    uint32_t percentage_mws = _get_chg_day_sum_mws(day) * percent / 100;

    uint32_t mws = 0;
    unsigned i = 0;
    while ((i < ECO_PM_INFO_SLOTS_PER_DAY) && (mws < percentage_mws)) {
        mws += day->e_chg_mws[i];
        i++;
    }

    return i - 1;
}

int _seconds_till_end_of_slot(uint32_t from_s, int to_idx_end){
    uint32_t eos_sod = (to_idx_end + 1) * ECO_PM_INFO_SLOT_LEN_MINUTES * 60;

    /* if targetted slot is on the next day */
    if (eos_sod < from_s) {
        return 24 * 60 * 60 - from_s + eos_sod;
    }
    else{
        return eos_sod - from_s;
    }
}

int _slot_of_day_second(uint32_t seconds){
    return (seconds / ECO_PM_INFO_SLOT_LEN_SECONDS) % ECO_PM_INFO_SLOTS_PER_DAY;
}

uint32_t _get_average_e_mws_per_cycle(eco_t *eco, uint32_t avg_horizon_s){
    (void) avg_horizon_s;

    eco_energy_history_t *h = &eco->history;

    /* TODO: calculate this value for the specified horizon */
    if (h->e_use_slot_cnt > 0) {
        return h->e_use_slot_sum_uws / h->e_use_slot_cnt / 1000;
    }
    else {
        return ASSUMED_FIXED_CYCLE_COST_E_MWS;
    }
}

uint32_t _get_utility_sum(uint32_t first_idx, uint32_t last_idx) {
    uint32_t util_sum = 0;
    uint32_t i = first_idx;
    do {
        util_sum += measurement_utility[i];
        i = _get_next_slot_idx(i);
    } while (i != _get_next_slot_idx(last_idx));

    return util_sum;
}


uint32_t _get_e_mws_sum(eco_day_stat_t *day, uint32_t first_slot,
                        uint32_t last_slot){
    uint32_t sum = 0;

    for (unsigned i = first_slot; i <= last_slot; i++) {
        sum += day->e_chg_mws[i];
    }

    return sum;
}

uint32_t _get_expected_chg_mws(eco_t *eco, uint32_t seconds){
    eco_energy_history_t *h = &eco->history;
    //eco_day_stat_t *td = &h->days[h->current_day_idx];
    eco_day_stat_t *yd = &h->days[_get_prev_day_idx(h->current_day_idx)];

    uint32_t s_of_day = node_time_to_seconds_of_day(&h->time_ref);

    uint32_t last_slot = _slot_of_day_second(s_of_day + seconds);

    /* effective charging rate (corrected by very simplified leakage factor) */
    int32_t p_chg_eff_uw = (h->p_chg_mavg_uw +
                           ECO_I_LEAKAGE_CURRENT_UA * h->u_latest_mv / 1000) ;

    /* if positive this means there is no charging! -> fix to zero
       if negative change to absolute positive value for later calculations */
    uint32_t p_chg_pos_capped_uw = p_chg_eff_uw >= 0 ? 0 : -p_chg_eff_uw;

    /* max out with given seconds in case the end of the prediction falls
       in the current slot */
    uint32_t secs_in_cur_slot = (last_slot == h->current_slot_idx && seconds <= ECO_PM_INFO_SLOT_LEN_SECONDS) ? seconds :
                                (ECO_PM_INFO_SLOT_LEN_SECONDS - h->secs_in_slot);

    /* start with the expected amount of energy that will be charged within
       (the relevant seconds of) this slot */
    uint32_t E_exp_mws = (uint32_t)((uint64_t)p_chg_pos_capped_uw *
                                              secs_in_cur_slot / 1000);

    uint32_t E_exp_cs_mws = E_exp_mws + h->e_chg_slot_sum_uws / 1000;

    uint32_t perc_of_yd_slot;
    if ((h->e_chg_slot_sum_uws / 1000) != 0) {
        perc_of_yd_slot = E_exp_cs_mws / (yd->e_chg_mws[h->current_slot_idx] / 100);
    }
    else {
        perc_of_yd_slot = 200;
    }

    printf("based on charge stats and extrapolation we expect %"PRIu32
           " mws (%"PRIu32" %% compared to yd (%"PRIu32"))\n",
           E_exp_cs_mws, perc_of_yd_slot, yd->e_chg_mws[h->current_slot_idx]);

    /* TODO: fade out this correction with time from now */
    uint32_t pos_perc_diff = (perc_of_yd_slot - 100) / 2;

    if (pos_perc_diff > 100) {
        pos_perc_diff = 100;
    }

    /* add prediction for the coming slots */
    for (unsigned i = _get_next_slot_idx(h->current_slot_idx);
         i != _get_next_slot_idx(last_slot); i = _get_next_slot_idx(i)) {

        uint32_t pred_slot_mws = yd->e_chg_mws[i];

        if (perc_of_yd_slot <= 100) {
            uint32_t perc_corr = 100 - perc_of_yd_slot;

            /* same as perc_corr + perc_corr * perc_corr * 0.01 */
            perc_corr = perc_corr + perc_corr * perc_corr / 100;
            if (perc_corr >= 100) {
                pred_slot_mws = 0;
            }
            else {
                pred_slot_mws -= pred_slot_mws * perc_corr / 100;
            }

            printf("[eco] decreasing the prediction by %"PRIu32" %% to %"PRIu32" mWs\n",
                   perc_corr, pred_slot_mws);
        }
        else {
            pred_slot_mws += pos_perc_diff * pred_slot_mws / 100;
            printf("[eco] increasing the prediction by %"PRIu32" %% to %"PRIu32" mWs\n",
                   pos_perc_diff, pred_slot_mws);
            /* reduce the impact of increased charging for every step */
            pos_perc_diff = pos_perc_diff / 2;
        }

        if (i == last_slot) {
            pred_slot_mws = pred_slot_mws / ECO_PM_INFO_SLOT_LEN_SECONDS *
                            (seconds % ECO_PM_INFO_SLOT_LEN_SECONDS);
        }

        E_exp_mws += pred_slot_mws;

        printf("slot %u: %"PRIu32" mWs (sum: %"PRIu32" mWs)\n", i, pred_slot_mws,
               E_exp_mws);
    }

    return E_exp_mws;
}

/* calculate when the next sensing cylce can happen return seconds */
uint32_t _get_sensing_period(eco_t *eco)
{
    eco_energy_history_t *h = &eco->history;
    eco_day_stat_t *td = &h->days[h->current_day_idx];
    eco_day_stat_t *yd = &h->days[_get_prev_day_idx(h->current_day_idx)];

    /* if requirements for calculating duty_cycle are met */
    if (eco->cycle_cnt < ECO_MIN_CYCLES_BEFORE_UPDATING_SENSING_PERIOD) {
        LOG_DEBUG("[eco] collecting more data before calcing duty cycle...\n");
        return ENVIRO_SENSING_PERIOD_BOOTSTRAP_S;
    }

    /* effective charging rate (corrected by very simplified leakage factor) */
    int32_t p_chg_eff_uw = (h->p_chg_mavg_uw +
                           ECO_I_LEAKAGE_CURRENT_UA * h->u_latest_mv / 1000) ;

    /* if positive this means there is no charging! -> fix to zero
       if negative change to absolute positive value for later calculations */
    uint32_t p_chg_pos_capped_uw = p_chg_eff_uw >= 0 ? 0 : -p_chg_eff_uw;

    printf("vvv---------- current state ----------vvv\n\n");

    printf("td idx:             %12"PRIu32"\n", h->current_day_idx);
    printf("yd idx:             %12"PRIu32"\n", _get_prev_day_idx(h->current_day_idx));
    printf("slot active since:  %12"PRIu32" s\n", h->secs_in_slot);



    printf("U_sc:               %12"PRIu32" mV\n", h->u_latest_mv);

    uint32_t stored_mWs = _get_available_mws(eco);

    if (stored_mWs > SUPERCAP_MAX_AVAIL_E_MWS) {
        LOG_DEBUG("limiting SOC to %"PRIu32" mWs\n", SUPERCAP_MAX_AVAIL_E_MWS);
        stored_mWs = SUPERCAP_MAX_AVAIL_E_MWS;
    }

    printf("SOC:                %12"PRIu32" mWs\n", stored_mWs);

    uint32_t slots_left = ECO_PM_INFO_SLOTS_PER_DAY - h->current_slot_idx;

    printf("diff to SOC_max:    %12"PRIu32" mWs\n", SUPERCAP_MAX_AVAIL_E_MWS -
                                                   stored_mWs);

    printf("E_c_td:             %12"PRIu32" mWs\n", td->e_chg_sum_mws);
    printf("E_c_yd:             %12"PRIu32" mWs\n", yd->e_chg_sum_now_mws);
    printf("remaining slots:    %12"PRIu32"\n", slots_left);

    uint32_t soc_min_idx = _get_slot_of_n_percent_chg(yd,
                                   ECO_TARGET_MIN_SOC_AT_N_PERCENT_OF_PREV_DAY);

    /* TODO: for planning "until when to spend all energy use t_max_soc++
    for planning on when to reach SOC_full use t_max_soc-- */
    uint32_t soc_max_idx = _get_slot_of_n_percent_chg(yd,
                                   ECO_TARGET_MAX_SOC_AT_N_PERCENT_OF_PREV_DAY);

    printf("slot(E_c(yd, %3"PRIu32"%%)):%12"PRId32"\n",
           ECO_TARGET_MIN_SOC_AT_N_PERCENT_OF_PREV_DAY, soc_min_idx);

    uint32_t s_of_day = node_time_to_seconds_of_day(&h->time_ref);
    int32_t dt_soc_min = _seconds_till_end_of_slot(s_of_day, soc_min_idx);
    int32_t dt_soc_max = _seconds_till_end_of_slot(s_of_day, soc_max_idx);

    uint32_t e_cycle_avg_mws = _get_average_e_mws_per_cycle(eco, dt_soc_min);

    /* if charging more than a cycle costs at the moment ..*/
    uint32_t available_avg_cycles = stored_mWs / e_cycle_avg_mws;

    printf("P_chg_mavg:         %12"PRId32" uW\n", -1 * h->p_chg_mavg_uw);
    printf("P_chg_mavg_eff:     %12"PRId32" uW\n", p_chg_pos_capped_uw);
    printf("sense cycle cnt:    %12"PRIu32"\n", h->e_use_slot_cnt);
    printf("E_cycle_avg:        %12"PRId32" mWs\n", e_cycle_avg_mws);
    printf("dt(SOC_min):        %12"PRIu32" s\n", dt_soc_min);
    printf("dt(SOC_max):        %12"PRIu32" s\n", dt_soc_max);
    printf("cycles_av:          %12"PRIu32"\n", available_avg_cycles);
    printf("sense period:       %12"PRIu32" s\n", eco->sensing_period_s);
    printf("eco period:         %12"PRIu32" s\n", eco->eco_processing_period_s);
    printf("^^^---------- current state ----------^^^\n");

    /* TODO either only predict once we have enough data, or use a reasonable
       fallback for the prediction */
    uint32_t perc_of_yd = td->e_chg_sum_mws / (yd->e_chg_sum_now_mws / 100);

    /* assuming a fixed charging rate for the same slot yesterday we get this
       charging rate */
    int32_t p_avg_yd_cs_uW = ((int64_t)yd->e_chg_mws[h->current_slot_idx]
                                 * 1000) / ECO_PM_INFO_SLOT_LEN_SECONDS;

    /* gives a more timely (short term) estimate of how the current charging
       rate will influence the prediction */
    uint32_t p_rate_diff_perc = p_chg_pos_capped_uw / (p_avg_yd_cs_uW / 100);

    int32_t p_avg_td_cs_uW = h->e_chg_slot_sum_uws / h->secs_in_slot;

    int32_t E_exp_chg_cs_mWs = (h->e_chg_slot_sum_uws +
                               (int64_t)p_chg_pos_capped_uw *
                               (ECO_PM_INFO_SLOT_LEN_SECONDS - h->secs_in_slot))
                               / 1000;

    /* TODO separate sum of full day and "sum until same time" */

    /* assuming that today will be like yesterday we will get that much energy */
    uint32_t e_chg_rem_yd = 0;

    /* if there is at least one comlete slot ahead that we didn't enter yet,
       add this to the expected remaining energy for today */
    if (h->current_slot_idx < ECO_PM_INFO_SLOTS_PER_DAY -1) {
        e_chg_rem_yd += _get_e_mws_sum(yd,
                                       _get_next_slot_idx(h->current_slot_idx),
                                       ECO_PM_INFO_SLOTS_PER_DAY -1);
    }

    /* the amount of energy that is still expected to be charged today */
    uint32_t e_exp_chg_sum_mws = e_chg_rem_yd / 100 * p_rate_diff_perc;

    printf("vvv----------  prediction  -----------vvv\n\n");
    printf("P_(now):            %12"PRId32" uW\n", -h->p_chg_latest_uw);
    printf("P_avg(yd, cs):      %12"PRId32" uW\n", p_avg_yd_cs_uW);
    printf("P_avg(td, cs):      %12"PRId32" uW\n", p_avg_td_cs_uW);
    printf("E(yd, cs):          %12"PRIu32" mWs\n", yd->e_chg_mws[h->current_slot_idx]);
    printf("Exp_dE(chg_acc):    %12"PRIu32" %%\n", perc_of_yd);
    printf("dP:                 %12"PRId32" %%\n", p_rate_diff_perc);
    printf("E_exp(cs):          %12"PRIu32" mWs\n", E_exp_chg_cs_mWs);
    printf("E_exp(rest(td)):    %12"PRIu32" mWs\n", e_exp_chg_sum_mws);
    printf("E(td):              %12"PRIu32" mWs\n", td->e_chg_sum_mws);
    printf("E_exp(td):          %12"PRIu32" mWs\n", td->e_chg_sum_mws +
                                                    e_exp_chg_sum_mws);

    uint32_t now_available_mws = _get_available_mws(eco);

    uint32_t e_to_full_wms = 0;
    if (now_available_mws < SUPERCAP_FULL_AVAIL_E_MWS) {
        e_to_full_wms = SUPERCAP_FULL_AVAIL_E_MWS - now_available_mws;
    }
    else {
        /* maybe SUPERCAP_FULL_E_MWS is better here? */
        e_to_full_wms = 0;
    }

    printf("diff to SOC_full    %12"PRIu32" mWs\n", e_to_full_wms);

    /* TODO use this to derive the sampling period in a way we reach
       SOC_full at the configured time (relative to % of SOC of prev day? ) */
    int32_t e_exp_for_sampling_mws = e_exp_chg_sum_mws > e_to_full_wms ?
                                     e_exp_chg_sum_mws - e_to_full_wms : 0;

    printf("E_exp_smpl:         %12"PRId32" mWs\n", e_exp_for_sampling_mws);

    int32_t eta_full_s = 0;
    if (p_chg_pos_capped_uw > 0) {
        eta_full_s = (e_to_full_wms / ((-1 * h->p_chg_mavg_uw) / 1000));
        printf("ETA_ns(SOC_full):   %12"PRIu32" s\n", eta_full_s);
    }
    else {
        printf("ETA_ns(SOC_full):            ??? s\n");
    }

    /* to consider the power usage for sampling for the ETA calulation, we
       derive a rough equivalent permanent power draw for the active
       sampling rate an the average energy consumption of one sensig cycle */
    /* TODO factor in the real sampling time (from continuous measurements)*/
    int32_t p_sample_avg_uw = ((h->e_use_slot_sum_uws / h->e_use_slot_cnt) / (eco->sensing_period_s + 50));
    int32_t p_chg_ws_mavg_uw = (-1 * h->p_chg_mavg_uw) - p_sample_avg_uw;

    int32_t eta_full_ws_s = 0xFFFFFFFF;

    /* charging rate with considering sampling */
    printf("P_chg_ws:           %12"PRId32" uW\n", p_chg_ws_mavg_uw);
    printf("P_s_cycle:          %12"PRId32" uW\n", p_sample_avg_uw);

    if (p_chg_ws_mavg_uw > 0) {
        eta_full_ws_s = (e_to_full_wms / (p_chg_ws_mavg_uw / 1000));
        printf("ETA(SOC_full):      %12"PRId32" s\n", eta_full_ws_s);
    }
    else {
        /* based on the current stats it looks like SOC_full won't be reached
           today */
        printf("ETA(SOC_full):      %12"PRIu32" d+\n", 1LU);
    }
    printf("^^^----------  prediction  -----------^^^\n");

    _print_energy_stat_history(eco);

    /* TODO: in descending priority calculate duty-cycle based on:
             - available energy
             - current charging
             - prediction ( start prediction only after populating n slots)
             - diff from prediction*/

    /* if voltage is below critical level we shouldn't calculate anyting
       but go back to sleep ASAP */
    if (h->u_latest_mv < ECO_CRITICAL_VOLTAGE_MV) {
        eco->hit_critical_voltage = true;
        /* TODO replace rmove this flag again after reaching normal voltage */
        return ECO_CRITICAL_VOLTAGE_HIBERNATE_S;
    }
    /* if not enough data is available to do a proper decision */
    else if(eco->cycle_cnt == 0) {
        LOG_DEBUG("never did a sensing cycle -> returning default period to"
                  "collect data soon...\n");
        return ENVIRO_SENSING_PERIOD_BOOTSTRAP_S;
    }
    /* if energy availability is above the FULL threshold we allocate it solely
       based live charging statistics */
    else if (h->u_latest_mv >= SUPERCAP_VOLTAGE_FULL_MV) {
        printf("SOC over full threshold! -> should schedule sensing cycle soon\n");
        printf("this slot was active for %"PRIu32" seconds!\n", h->secs_in_slot);
        printf("charged %"PRIu32" mWs within this slot\n", (uint32_t)(h->e_chg_slot_sum_uws / 1000));
        printf("currently charging with %"PRId32" uW\n", p_chg_eff_uw);

        if (p_chg_pos_capped_uw == 0) {
            printf("not charging -> assume we need to wait a full slot before sensing again\n");
            return ECO_PM_INFO_SLOT_LEN_SECONDS;
        }

        int32_t cycle_time_s = (e_cycle_avg_mws * 1000) / p_chg_pos_capped_uw;

        printf("for collecting %"PRId32" mWs this takes %"PRIu32" s charging time\n", e_cycle_avg_mws, cycle_time_s);

        if (h->u_latest_mv >= SUPERCAP_VOLTAGE_FORCE_SENSE_MV) {
            printf("supercap is completely full -> sense ASAP\n");
            return 0;
        }
        return cycle_time_s;
    }
    else {
        uint32_t soc_min_idx = _get_slot_of_n_percent_chg(yd,
                                   ECO_TARGET_MIN_SOC_AT_N_PERCENT_OF_PREV_DAY);

        uint32_t dt_soc_min_s = _seconds_till_end_of_slot(s_of_day, soc_min_idx);

        uint32_t dt_soc_max_s = _seconds_till_end_of_slot(s_of_day, soc_max_idx);

        /* TODO: add minimum fallbacks for transition from SOC_min to SOC_max target */
        if (eco->currently_charging) {
            printf("assuming a charging rate of %"PRId32" uw\n", p_chg_pos_capped_uw);

            /* if we are still planning to achieve SOC_full */
            if (soc_max_idx >= h->current_slot_idx) {
                printf("before t(SOC_full) -> target is SOC_full in %"PRIu32" s\n",
                       dt_soc_max_s);

                /* TODO: don't extrapolate for more than the current slot weighted calculation* */
                uint32_t assumed_chg_mws = _get_expected_chg_mws(eco, dt_soc_max_s);

                int32_t e_to_allocate_mws = assumed_chg_mws - e_to_full_wms;
                printf("e_to_allocate_mws: %"PRId32" mWs\n", e_to_allocate_mws);

                if (e_to_allocate_mws > 0) {
                    /* apart from the available charging rate we also have to ensure
                       not to use more energy than our energy buffer holds */
                    if ((stored_mWs < e_cycle_avg_mws) ||
                        ((uint32_t)e_to_allocate_mws < e_cycle_avg_mws)) {
                        printf("less energy to allocate than a cycle would consume!\n");
                        /* return a very high value to ensure time for charging up
                           in case we collect some more energy this will be corrected
                           the next time the energy allocation is calculated */
                        return ECO_CRITICAL_VOLTAGE_HIBERNATE_S;
                    }

                    printf("splitting energy over the remaining time..\n");

                    return dt_soc_max_s / (e_to_allocate_mws / e_cycle_avg_mws);
                }

                printf("no energy to allocate!\n");
                return dt_soc_max_s;
            }
            /* if we already passed the target for reaching SOC_max
               use minimum period possible of either allocation till SOC_min,
               or spending the average charging rate */
            else {
                /* TODO: just using the charging rate was way to agressive! */
                uint32_t period_chg_s = e_cycle_avg_mws * 1000
                                        / (p_chg_pos_capped_uw * ECO_USE_PERCENTAGE_OF_CHARGING_RATE / 100);
                printf("the current charging rate alows one sample every %"
                       PRIu32" s\n", period_chg_s);
                uint32_t period_strg_s = 0;

                if (e_cycle_avg_mws <= 0) {
                    period_strg_s = 0;
                }
                else if (now_available_mws <= (uint32_t)e_cycle_avg_mws) {
                    period_strg_s = ECO_PM_INFO_SLOT_LEN_SECONDS * ECO_PM_INFO_SLOTS_PER_DAY;
                }
                else {
                    period_strg_s = dt_soc_min_s / (now_available_mws / e_cycle_avg_mws);
                }

                printf("allocating all available energy till t(SOC_min) allows %"
                       PRIu32" s\n", period_strg_s);

                return ((period_chg_s < period_strg_s) ? period_chg_s : period_strg_s);
            }
        }
        else {
            /* TODO: add exception for "no charging before reaching t(SOC_min)" */
            printf ("not charging anymore -> tagetting SOC_min in %"
                    PRIu32" s\n", dt_soc_min_s);
            /* if !charging always plan t(SOC_min) */
            if (now_available_mws <= e_cycle_avg_mws) {
                //return dt_soc_min_s;
                printf ("less energy available than a single cycle costs %"PRIu32" / %"PRIu32"\n", now_available_mws, e_cycle_avg_mws);
                return ECO_PM_INFO_SLOT_LEN_SECONDS * ECO_PM_INFO_SLOTS_PER_DAY;
            }

            uint32_t util_sum = _get_utility_sum(h->current_slot_idx,
                                                 soc_min_idx);

            printf ("overall utility from %"PRIu32" to %"PRIu32" is %"PRIu32"\n",
                    h->current_slot_idx, soc_min_idx, util_sum);

            uint32_t i =  h->current_slot_idx;
            do {
                /* energy to allocate for that slot... */
                eco->e_alloc_mws[i] = now_available_mws / util_sum
                                      * measurement_utility[i];

                eco->alloc_sensing_cycles[i] = eco->e_alloc_mws[i] / e_cycle_avg_mws;

                uint32_t slot_secs = ECO_PM_INFO_SLOT_LEN_SECONDS;
                if (i == h->current_slot_idx) {
                    slot_secs -= h->secs_in_slot;
                }

                if (eco->alloc_sensing_cycles[i] > 0) {
                    eco->t_sense_s[i] = slot_secs / eco->alloc_sensing_cycles[i];
                }
                else {
                    /* no sensing step in this slot anymore */
                    eco->t_sense_s[i] = ECO_PM_INFO_SLOT_LEN_SECONDS * ECO_PM_INFO_SLOTS_PER_DAY;
                }
                i = _get_next_slot_idx(i);
            } while (i != _get_next_slot_idx(soc_min_idx));


            printf("E_alloc(cs): %"PRIu32" mws | t_sense: %"PRIu32" | cycles: %"PRIu32"\n",
                   eco->e_alloc_mws[h->current_slot_idx],
                   eco->t_sense_s[h->current_slot_idx],
                   eco->alloc_sensing_cycles[h->current_slot_idx]);

            return eco->t_sense_s[h->current_slot_idx];
        }
    }
}

bool _eco_init(eco_t *eco){
    return _ina22x_init(eco) == 0;
}

void _update_time_ref_and_idxes(eco_t *eco) {
    eco_energy_history_t *h = &eco->history;

    node_time_get(&h->time_ref);

    uint32_t s_of_day = node_time_to_seconds_of_day(&h->time_ref);

    h->secs_in_slot = _get_seconds_in_slot(s_of_day);


    uint32_t old_idx = h->current_slot_idx;
    uint32_t new_idx = s_of_day / (ECO_PM_INFO_SLOT_LEN_MINUTES * 60);

    /* if entered a new slot */
    if (old_idx != new_idx) {
        /* the energy stats are always split betwen old_idx, old_idx + n,
           ... and new_idx - 1, the values for new_idx will be assigned later
           then */
        uint32_t affected_slots = _get_slot_cnt(old_idx, new_idx);

        /* TODO: maybe split more acurately based on seconds ?
           (the new idx maybe active for a while and should get its share) */
        LOG_DEBUG("[eco] splitting collected energy stats between %"PRIu32
                  " slot(s) (moving from %"PRIu32" to %"PRIu32")\n",
                  affected_slots, old_idx, new_idx);

        /* the share to assign to each slot */
        uint32_t e_chg_share_mws = h->e_chg_slot_sum_uws / (affected_slots * 1000);
        uint32_t e_use_share_mws = h->e_use_slot_sum_uws / (affected_slots * 1000);

        int32_t u_last_mv = h->days[h->current_day_idx].u_latest_mv[_get_prev_slot_idx(old_idx)];
        int32_t dU_per_slot_mv = (h->u_latest_mv - u_last_mv) / affected_slots;
        uint32_t iter_cnt = 1;

        /* split collected stats over all slots in case we skipped any */
        for (unsigned i = old_idx; i != new_idx; i = _get_next_slot_idx(i)) {
            LOG_DEBUG("[eco] assigning share to slot %d\n", i);
            /* if just entered a new day, i.e. going to move
               from last slot to first.. */
            if (i == (ECO_PM_INFO_SLOTS_PER_DAY - 1)) {
                LOG_DEBUG("[eco] just enertered a new day %lu -> %lu\n", h->current_day_idx,
                       _get_next_day_idx(h->current_day_idx));
                h->current_day_idx = _get_next_day_idx(h->current_day_idx);
                h->day_cycles_cnt = 1; /* first cycle this day */
            }

            h->days[h->current_day_idx].e_chg_mws[i] = e_chg_share_mws;
            h->days[h->current_day_idx].e_use_mws[i] = e_use_share_mws;

            /* TODO: scale between old and new value? */
            h->days[h->current_day_idx].u_latest_mv[i] = u_last_mv + (iter_cnt * dU_per_slot_mv);

            iter_cnt++;
        }

        h->current_slot_idx = new_idx;

        /* clear any previous values for the new slot */
        h->days[h->current_day_idx].e_chg_mws[new_idx]   = 0;
        h->days[h->current_day_idx].e_use_mws[new_idx]   = 0;
        h->days[h->current_day_idx].u_latest_mv[new_idx] = 0;

        /* clear values of the running slot counters */
        //h->p_chg_mavg_uw = 0;
        h->e_lp_measure_slot_cnt = 0;
        h->e_chg_slot_sum_uws = 0;
        h->e_chg_slot_cnt = 0;
        h->e_use_slot_sum_uws = 0;
        h->e_use_slot_cnt = 0;
        h->u_latest_mv = 0;
    }
}

static void _enable_charging(void) {
    gpio_init(LTC3105_PARAM_SHDN_PIN, GPIO_OUT);
    /* enable charging */
    gpio_set(LTC3105_PARAM_SHDN_PIN);
}

static void _disable_charging(void) {
    gpio_init(LTC3105_PARAM_SHDN_PIN, GPIO_OUT);
    /* enable charging */
    gpio_clear(LTC3105_PARAM_SHDN_PIN);
}


int32_t _mavg_append(int32_t current, int32_t new_val, int cnt)
{
    if (cnt == 1) {
        return current;
    }
    return (current * 60 + new_val * 40) / 100;
}

void _append_lp_measurement(eco_t *eco, eco_measurement_set_t *measure)
{
    uint64_t now_utc_ms = node_time_to_utc_ms(&eco->history.time_ref);
    eco_energy_history_t *h = &eco->history;

    /* if no charge measure was performed before
       derive charge time from sleep duration */
    if (h->last_chg_measure_utc_ms == 0) {
        h->last_chg_measure_utc_ms =
                      node_time_get_utc_ms() -
                      node_time_get_prev_sleep_dur_ms();
    }

    /* a maximum time up to ~50 days will always fit into
      a 32 bit var (far more than we need here) */
    uint32_t t_chg_ms =
           now_utc_ms - h->last_chg_measure_utc_ms;

    /* calculate how much energy was available for charging
       Note: this number includes energy that was
             actually not charged but instead used for
             the measurement action itself */
    int64_t measure_uws = ((int64_t)t_chg_ms * measure->power_uw) / 1000;

    /* calculate the amount of energy that was spent
       for the measurment instead of being stored to
       the energy storage */
    uint32_t measure_cost_uws =
                   (INA226_MEASUREMENT_CONSUMPTION_UW *
                   node_time_get_prev_sleep_dur_ms() )
                   / 1000;

    /* negative measured value means energy was charged
       -> so correct charged value by adding the cost
          of the measurement */
    measure_uws += measure_cost_uws;

    LOG_INFO("[eco] %scharged with %"PRId32" uA "
             "for (%"PRIu32" ms) ~(%"PRId32" uWs)"
             " measurement cost was %"PRIu32" uWs\n",
             measure->current_ua < 0 ? "" : "dis",
             measure->current_ua, t_chg_ms,
             (int32_t)measure_uws,
             measure_cost_uws);

    /* maybe add explicit testing to detect this (shut down charging and self measure) */
    if (measure->power_uw > ECO_LOW_POWER_CONSUMPTION_WARNING_LEVEL_UW) {
        printf("############## WARNING! -> too high consumption "
               "for low power mode! ( %"PRId32" uW)\n", measure->power_uw);
    }

    h->last_chg_measure_utc_ms = now_utc_ms;

    /* count that we read a lp measurement */
    h->e_lp_measure_slot_cnt++;

    /* depending on the value decide if we attribute it to charging or
       discharging stats */
    if (measure_uws < 0) {
        h->e_chg_slot_sum_uws += measure_uws * -1;
        h->e_chg_slot_cnt++;
    }
    else {
        /* add the consumption to the usage but don't
           increment the counter to spread it over all performed cycles */
        h->e_use_slot_sum_uws += measure_uws;
        //ds->e_use_slot_cnt++;
    }

    h->u_latest_mv = measure->bus_voltage_mv;

    printf("measure.power_uw: %"PRId32" uW\n", measure->power_uw);
    h->p_chg_mavg_uw = _mavg_append(h->p_chg_mavg_uw, measure->power_uw, h->e_lp_measure_slot_cnt);
    h->p_chg_latest_uw = measure->power_uw;
    h->i_chg_latest_uA = measure->current_ua;

    /* decide if the charging rate counts as charging rate */
    eco->currently_charging = (h->p_chg_mavg_uw <= ECO_P_CHARGING_THRESHOLD_UW);

    if (h->u_latest_mv >= SUPERCAP_VOLTAGE_FULL_MV) {
        eco->eco_processing_period_s = ECO_PROCESSING_PERIOD_CHARGING_MIN_S;
    }
    else if (eco->currently_charging) {
        //eco->eco_processing_period_s = ECO_PROCESSING_PERIOD_CHARGING_S;
        eco->eco_processing_period_s =
                            ECO_ENERGY_CONSUMTION_OF_PROCESSING_STEP_UWS
                            / (-h->p_chg_mavg_uw / 1000 *
                               ECO_ENERGY_PERMILL_FOR_PROCESSING);

        printf("[eco]: processing period %"PRIu32" s\n",
                 eco->eco_processing_period_s);

        if (eco->eco_processing_period_s < ECO_PROCESSING_PERIOD_CHARGING_MIN_S) {
            eco->eco_processing_period_s = ECO_PROCESSING_PERIOD_CHARGING_MIN_S;
        }
        else if (eco->eco_processing_period_s > ECO_PROCESSING_PERIOD_CHARGING_MAX_S) {
            eco->eco_processing_period_s = ECO_PROCESSING_PERIOD_CHARGING_MAX_S;
        }
    }
    else{
        eco->eco_processing_period_s = ECO_PROCESSING_PERIOD_DISCHARGING_S;
    }

    printf("current avg charging rate with ~ %"PRId32" uW\n", h->p_chg_mavg_uw);

    printf("currently %scharging\n", eco->currently_charging ? "" : "NOT ");

    int32_t mws_chg_sum = h->e_chg_slot_sum_uws / 1000;
    int chgslots = h->e_chg_slot_cnt;
    printf("[eco]: Sum of the last %d charge measures: %"PRIu32" mWs"
           " avg Pc: %"PRId32" uW\n",
           chgslots, mws_chg_sum, h->p_chg_mavg_uw);

    if (eco->state_retained != true) {
        _set_initial_charging_history(eco);
        /* next time a warm-init will be performed */
        eco->state_retained = true;
    }
}

void _ctl_power_measure(eco_t *eco, msg_t *m)
{
    ina22x_t *ina = &eco->ina22x_dev;

    switch (CTLMSG_TYPE2(m->type)) {
        case CTLMSG_TYPE2_INIT:
            LOG_DEBUG("[eco] CTLMSG_TYPE2_INIT\n");
            if (!eco->cold_init_done) {
                LOG_DEBUG("[eco] no retained state issuing cold init...\n");
                eco->cold_init_done = _eco_init(eco);
                eco->cycle_cnt = 0;
                eco->sensing_period_s = ENVIRO_SENSING_PERIOD_BOOTSTRAP_S;
                eco->eco_processing_period_s = ECO_PROCESSING_PERIOD_BOOTSTRAP_S;
                eco->samples = eco_samples;
            }

            printf("[eco] cycle_cnt: %"PRIu32"\n", eco->cycle_cnt);
            eco->cycle_cnt++;

            /* to be sure, check if the init actually worked */
            if (eco->cold_init_done) {

                _update_time_ref_and_idxes(eco);

                LOG_DEBUG("[eco] cold_init was already performed...\n");
                if (eco->waiting_for_alert) {
                    LOG_DEBUG("[eco] ...also waiting for ALERT\n");
                    /* init should only be called after a fresh wakeup
                       -> increment cycle count */
                    eco->history.day_cycles_cnt++;
                    _ina22x_warm_init(&eco->ina22x_dev, eco->ina22x_params);
                    if (eco_alert_active(eco)) {
                        LOG_DEBUG("[eco] alert active...\n");
                        uint32_t t_us = xtimer_now_usec();
                        eco_measurement_set_t measure = { .t_us = t_us };
                        int res = _ina22x_read_all_regs(ina, &measure, true);
                        _eco_process_measures(&measure, 1, NULL);
                        //_pretty_print_measure(&measure, false);

                        if (res == 0) {
                            _append_lp_measurement(eco, &measure);
                        }
                        else{
                            eco->reg_read_err_cnt++;
                            LOG_ERROR("[eco] reading all regs failed!\n");
                        }
                        eco->waiting_for_alert = false;
                    }
                    else{
                        eco->no_alert_err_cnt++;
                        LOG_ERROR("[eco] waiting for alert but not active!\n");
                    }
                }

            }
            else {
                eco->init_err_cnt++;
                LOG_ERROR("[eco] cold init failed!\n");
            }

            msg_t reply = { .content.ptr = eco };
            msg_reply(m, &reply);
            break;

        /* TODO: refactor measurement message to single CTLMSG_TYPE2_MEASURE
           but using argument for the configuration */
        case CTLMSG_TYPE2_MEASURE:
            LOG_DEBUG("[eco] CTLMSG_TYPE2_MEASURE\n");
            LOG_DEBUG("[eco] config: 0x%x\n", eco->ina22x_dev.config);

            if(ina226_activate_int(ina, INA226_MASK_ENABLE_CNVR,
                                   ina->alert_pin, ina226_callback,
                                   eco ) == 0 ) {
                if (ina22x_write_config_reg(ina, INA22X_CONFIG_SINGLE_SHORT) == 0) {
                    LOG_DEBUG("[eco] ina22x_measure_thread [OK]\n");
                    uint16_t dummy;
                    if (ina226_read_mask_enable_reg(ina, &dummy) != 0) {
                        LOG_ERROR("[eco] ina226_read_mask_enable [FAILED]\n");
                    }
                }
            }

            eco->requester_pid = m->sender_pid;
            break;

        case CTLMSG_TYPE2_MEASURE_SINGLE_LONG:
            LOG_DEBUG("[eco] CTLMSG_TYPE2_MEASURE_SINGLE_LONG\n");
            _request_single_long_measure(eco);
            break;

        case CTLMSG_TYPE2_MEASURE_CONTINUOUS:
            LOG_DEBUG("[eco] CTLMSG_TYPE2_MEASURE_CONTINUOUS\n");
            LOG_DEBUG("[eco] config: 0x%x\n", ina->config);
            _disable_charging();
            eco->sample_cnt = 0;
            if (m->content.value != 0) {
                if (m->content.value <= ECO_SAMPLE_BUFFER_CNT) {
                    eco->sample_cnt_target = m->content.value;
                }
                else{
                    eco->sample_cnt_target = ECO_SAMPLE_BUFFER_CNT;
                }
                LOG_DEBUG("[eco] will measure %d samples\n", eco->sample_cnt_target);
            }

            /* clear collected task usage stats */
            for (int i = 0; i < ECO_ATTRIBUTION_TASKS_MAX_NUMOF; i++) {
                if (eco->task_specs[i].used) {
                    eco->task_specs[i].e_pws = 0;
                    eco->task_specs[i].t_start_us = 0;
                    eco->task_specs[i].t_end_us = 0;
                    eco->task_specs[i].used = false;
                }
                else {
                    break;
                }
            }

            /* workaround to ensure proper interrupt triggering */
            //ina22x_write_config_reg(ina, INA22X_MODE_POWERDOWN);

            if(ina226_activate_int(ina, INA226_MASK_ENABLE_CNVR, ina->alert_pin, ina226_callback, eco) == 0 ) {
                if (ina22x_write_config_reg(ina, INA22X_CONFIG_TRACE) == 0) {
                    LOG_DEBUG("[eco] ina22x_measure_thread [OK]\n");
                    uint16_t dummy;
                    if (ina226_read_mask_enable_reg(ina, &dummy) != 0) {
                        LOG_ERROR("[eco] ina226_read_mask_enable [FAILED]\n");
                    }
                }
                else {
                    LOG_ERROR("[eco] ina22x_write_config_reg [ERROR]\n");
                }
            }
            else {
                LOG_ERROR("[eco] ina226_activate_int [ERROR]\n");
            }

            break;

        case CTLMSG_TYPE2_DATA_RAW:
            LOG_DEBUG("[eco] CTLMSG_TYPE2_DATA\n");
            uint32_t t_us = xtimer_now_usec();
            if (eco->sample_cnt < eco->sample_cnt_target) {
                eco->samples[eco->sample_cnt].t_us = t_us;
                _ina22x_read_all_regs(ina, &eco->samples[eco->sample_cnt], true);
                if (eco->requester_pid != KERNEL_PID_UNDEF) {
                    _eco_process_measures(&eco->samples[eco->sample_cnt],
                                          1, NULL);
                    eco_measurement_set_t *measure =
                                    (eco_measurement_set_t*)eco->requester_ctx;

                    memcpy(measure, &eco->samples[eco->sample_cnt],
                           sizeof(eco_measurement_set_t));

                    msg_t reply_msg = { .type = CTLMSG_TYPE2_DATA_RAW,
                                        .content.ptr = measure };

                    msg_send(&reply_msg, eco->requester_pid);

                    /* remove requester of single read */
                    eco->requester_pid = KERNEL_PID_UNDEF;
                }
                eco->sample_cnt++;
            }
            else {
                printf("sampled enough!\n");
            }
            //thread_yield();
            break;

        case CTLMSG_TYPE2_POWER_ON:
            _enable_charging();
            break;

        case CTLMSG_TYPE2_POWER_OFF:
            LOG_DEBUG("[eco] CTLMSG_TYPE2_POWER_OFF\n");
            ina22x_write_reg(ina, INA226_REG_MASK_ENABLE, 0);
            ina22x_write_config_reg(ina, INA22X_MODE_POWERDOWN);
            _enable_charging();
            break;

        case CTLMSG_TYPE2_NOTIFY_POWER_DOWN:
            LOG_DEBUG("[eco] CTLMSG_TYPE2_NOTIFY_POWER_DOWN\n");

            _enable_charging();

            bool want_powerdown_measure = (eco->cycle_cnt % 2 == 0);

            eco->sensing_period_s = _get_sensing_period(eco);

            if (want_powerdown_measure) {
                LOG_DEBUG("[eco] going to perform standby measure..\n");
                /* do cold init before to be sure that we are in well defined stated */
                bool init_done = _eco_init(eco);

                if (!init_done) {
                    LOG_ERROR("[eco] init for lp measutre failed!...\n");
                    eco->init_err_cnt++;
                }
                //_send_msg(eco_pid, CTLMSG_TYPE1_POWER_MEASURE | CTLMSG_TYPE2_INIT, 0);
                // TODO replace with eco->params
                gpio_wakeup_pull_config(INA22X_PARAM_ALERT_PIN, GPIO_IN_PU);
                // TODO check if still required how low this can be
                //xtimer_usleep(1000 * 100);
                // TODO replace with eco->params
                gpio_wakeup_enable(INA22X_PARAM_ALERT_PIN, GPIO_FALLING);

                eco->waiting_for_alert = true;
                if (_request_single_long_measure(eco) != 0){
                    LOG_ERROR("[eco] starting lp measure failed!\n");
                }
                /* clear any pending alert interrupts before just in case */
                gpio_wakeup_clear(INA22X_PARAM_ALERT_PIN);
            }
            else {
                LOG_DEBUG("[eco] NOT going to perform standby measure..\n");
            }

            /* backup current task consumption stats to send them next time */
            memcpy(eco->prev_task_specs, eco->task_specs,
                   ECO_ATTRIBUTION_TASKS_MAX_NUMOF * sizeof(eco_task_consumption_spec_t));

            LOG_DEBUG("[eco] will reply with new sensing cycle recommendation\n");
            msg_t pd_reply = { .content.ptr = eco };
            msg_reply(m, &pd_reply);
            break;

        case CTLMSG_TYPE2_REQUEST_AVG:
            {
            LOG_DEBUG("[eco] CTLMSG_TYPE2_REQUEST_AVG\n");
            eco_measurement_set_t *avg = (eco_measurement_set_t*)m->content.ptr;
            //_eco_process_measures(eco->samples, eco->sample_cnt, avg);
            _eco_process_and_attribute(eco, eco->samples, eco->sample_cnt, avg);

            /* add this consumption measurement to the energy usage stats */
            eco_energy_history_t *h = &eco->history;
            h->e_use_latest_uws = (int32_t)((int64_t)avg->power_uw * avg->t_us
                                            / 1000000);

            h->e_use_mavg_uws = _mavg_append(h->e_use_mavg_uws,
                                             h->e_use_latest_uws,
                                             h->e_use_slot_cnt + 1);

            printf("avg->t_us: %"PRIu32" us\n", avg->t_us);
            int64_t cycle_uws = (int64_t)avg->power_uw * avg->t_us / 1000000;

            h->e_use_slot_sum_uws += cycle_uws;
            h->e_use_slot_cnt++;

            printf("[eco] cycle energy usage was %"PRId32" mWs\n", (int32_t)(cycle_uws / 1000000000));


            uint32_t available_mWs = _get_available_mws(eco);
            uint32_t cycle_mWs = _get_average_e_mws_per_cycle(eco, ECO_PM_INFO_SLOT_LEN_MINUTES * 60);
            uint32_t possible_cycles = available_mWs / cycle_mWs;

            //sleep_interval_s = RUNTIME_GOAL_S / possible_cycles;

            LOG_INFO("[eco] SOC: %"PRIu32" mWs (%"PRIu32" mV)| "
                     "enough for %"PRIu32" cycles\n", available_mWs,
                     avg->bus_voltage_mv,
                     possible_cycles);

            msg_t avg_msg = { .type = CTLMSG_TYPE2_DATA_RAW,
                              .content.ptr = avg };

            msg_send(&avg_msg, m->sender_pid);
            }
            break;

        case CTLMSG_TYPE2_REQUEST_READ:
            {
            LOG_DEBUG("[eco] CTLMSG_TYPE2_REQUEST_READ\n");
            eco_measurement_set_t *measure = (eco_measurement_set_t*)m->content.ptr;
            _ina22x_read_all_regs(ina, measure, true);
            _eco_process_measures(measure, 1, measure);

            _pretty_print_measure(measure, false);

            msg_t avg_msg = { .type = CTLMSG_TYPE2_DATA_RAW,
                              .content.ptr = measure };

            msg_send(&avg_msg, m->sender_pid);
            }
            break;

        case CTLMSG_TYPE2_NOTIFY_TASK_START:
            {
            LOG_DEBUG("[eco] CTLMSG_TYPE2_NOTIFY_TASK_START\n");
            eco_task_consumption_spec_t *task_spec = _eco_notify_task_start(eco);
            msg_t reply = { .content.ptr = task_spec };
            msg_reply(m, &reply);
            break;
            }
        case CTLMSG_TYPE2_NOTIFY_TASK_STOP:
        LOG_DEBUG("[eco] CTLMSG_TYPE2_NOTIFY_TASK_STOP\n");
            _eco_notify_task_stop((eco_task_consumption_spec_t*)m->content.ptr);
            break;




        default: LOG_ERROR("[eco] _ctl_power_measure: unknown message value\n");
    }
}

void *eco_thread(void *arg)
{
    eco_t *eco = (eco_t*)arg;
    msg_t m;

    LOG_DEBUG("[eco] power measure thread started, pid: %" PRIkernel_pid "\n",
           thread_getpid());

   msg_init_queue(msg_queue,THREAD_MSG_QUEUE_SIZE_ECO);

   eco->requester_pid = KERNEL_PID_UNDEF;

    while (1) {
        msg_receive(&m);
        LOG_DEBUG("[eco] got msg from %" PRIkernel_pid "\n",
                 m.sender_pid);

        switch (CTLMSG_TYPE1(m.type)) {
            case CTLMSG_TYPE1_POWER_MEASURE: _ctl_power_measure(eco, &m); break;
            default: LOG_ERROR("sensors: unknown message type\n");
        }
    }

    return NULL;
}

bool eco_alert_active(eco_t *e) {
    gpio_init(e->ina22x_dev.alert_pin, GPIO_IN_PU);
    return gpio_read(e->ina22x_dev.alert_pin) == 0;
}

kernel_pid_t eco_init_thread(eco_t *eco)
{
    LOG_DEBUG("[eco]: eco_init_thread\n");
    eco->ina22x_params = ina22x_params;
    eco->pid = thread_create(stack,
                            sizeof(stack),
                            THREAD_PRIORITY_ECO, THREAD_CREATE_STACKTEST,
                            eco_thread, eco, "power measure");
    LOG_DEBUG("[eco]: thread created\n");
    return  eco->pid;
}
