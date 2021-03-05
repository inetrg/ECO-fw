/*
 * Copyright (C) 2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    eco module to self measure power consumption
 * @ingroup     app_sane
 * @brief       Utility module to measure consumption as traces or by threads
 * @{
 *
 * @file
 * @brief       Interface to control self measurement
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */

#ifndef ECO_H
#define ECO_H

#include "kernel_types.h"
#include "node_time.h"
/* The calibration values need to be included before ina22x.h to be used in default dev descriptor */
//--------------------------------- INA226 CONFIGURATION PARAMETERS -----vvvvv
/* The maximum current that is expected to flow through the shunt resistor in microampere. See section 7.5 in INA226 data sheet */
#define INA22X_MAX_EXPECTED_CURRENT_UA ((INA226_MAX_SHUNT_VOLTAGE_UV * MICROS_PER_MILLI) / INA22X_SHUNT_RESISTOR_MILLI_OHM)

/* The value of one LSB of the current register in nanoampere. See section 7.5 in INA226 data sheet */
#define INA22X_PARAM_LSB_NA ((INA22X_MAX_EXPECTED_CURRENT_UA * NANOS_PER_MICRO) / INA22X_MAX_CURRENT_TO_LSB_RATIO)

/* Calculated value for the calibration register for use with a 0.75 Ohm shunt resistor. See section 7.5 in INA226 data sheet */
#define INA22X_CALIBRATION_VALUE ((uint16_t)(INA226_CAL_SCALE_FACTOR_INT / (INA22X_PARAM_LSB_NA * INA22X_SHUNT_RESISTOR_MILLI_OHM)))

#define INA226_EXPECTED_MANUF_ID (0x5449)
#define INA226_EXPECTED_DIE_ID   (0x2260)
//--------------------------------- GENERAL CONFIGURATION PARAMETERS -----^^^^^

//--------------------- REFERENCE MEASUREMENT CALIBRATION PARAMETERS -----vvvvv
/* This value is derived by the equation shown in INA226 datasheet section 7.5.2
* Write this value to the calibration register when the above reference measurement values ('TEST_INA226_CAL_AMM_MEAS_CURRENT' and
* 'TEST_INA226_CAL_REPORTED_CURRENT') are set to values that were measured in your specific setup. */
#define INA22X_CORRECTED_FULL_SCALE_CAL ((uint16_t)((INA22X_CALIBRATION_VALUE * INA22X_CAL_AMM_MEAS_CURRENT) / INA22X_CAL_REPORTED_CURRENT))

/* Uncomment this line if after performing reference measurements with your hardware setup
* and updating 'TEST_INA226_CAL_AMM_MEAS_CURRENT' 'TEST_INA226_CAL_REPORTED_CURRENT' accordingly. */
#define INA22X_REF_MEASUREMENTS_DONE

#ifndef INA22X_REF_MEASUREMENTS_DONE
#define INA22X_PARAM_CAL INA22X_CALIBRATION_VALUE
#else
#define INA22X_PARAM_CAL INA22X_CORRECTED_FULL_SCALE_CAL
#endif
//--------------------- REFERENCE MEASUREMENT CALIBRATION PARAMETERS -----^^^^^
#include "ina22x.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INA226_EXPECTED_MANUF_ID           (0x5449)
#define INA226_EXPECTED_DIE_ID             (0x2260)

/* power consumption of the self measurement (based on fixed current of 330 µA,
   @the regulated 3.3V assuming (pessimistic) efficiency of around 70% */
#define INA226_MEASUREMENT_CONSUMPTION_UW  (1560)

#define MEASURE_TRACE_MAX_SAMPLES (3500)
#define MEASURE_THREAD_MSG_QUEUE_SIZE (1)
#define ECO_MSG_VAL_WARM_INIT         (1)

#define STM32_L476RG_SRAM2_START (0x10000000)
#define STM32_L476RG_SRAM2_LEN   (32 * 1024)

#define ECO_STATS_DAY_CNT (2)
#define ECO_PM_INFO_SLOT_LEN_MINUTES  (120)
#define ECO_PM_INFO_SLOT_LEN_SECONDS  (ECO_PM_INFO_SLOT_LEN_MINUTES * 60)
#define ECO_PM_INFO_SLOTS_PER_DAY (24 * 60 / ECO_PM_INFO_SLOT_LEN_MINUTES)

#define SUPERCAP_CAPACITIANCE_F                (350UL)

/* this is the maximum voltage which is also sent on the charging module
   the charging already starts to get ineffective slightly (few 10s of mV) before
   reaching this voltage */
#define SUPERCAP_VOLTAGE_MAX_MV                (2650UL)

/* If a voltage greater than this is available on the supercap always start sening to avoid
   available energy not to be used */
#define SUPERCAP_VOLTAGE_FORCE_SENSE_MV        (2650UL)

/* when over this threshold calculate the duty cycle only based on charging power */
#define SUPERCAP_VOLTAGE_FULL_MV                      (2630UL)
#define MIN_VOLTAGE_MV                                (1900UL)

/* if this voltage was reached for some reason switch to a hibernation mode
   the hibernation essentially means that the system will only wakeup
   at an extremely reduced period to check if the energy availability recovered. */
#define ECO_CRITICAL_VOLTAGE_MV                       (1800UL)


/* if critical voltage condition was detected the system will wait till this
   voltage is reached before going back to normal operation.
   Beacuse this state is considered to be a faulty state the energy history
   will be deleted (fresh bootstrap) after this */
//#define ECO_CRITICAL_VOLTAGE_RECOVER_MV               (2300UL)


#define ASSUMED_FIXED_CYCLE_COST_E_MWS                (6000UL)
#define ECO_MIN_CYCLES_BEFORE_UPDATING_SENSING_PERIOD (5)
#define ECO_LOW_POWER_CONSUMPTION_WARNING_LEVEL_UW    (INA226_MEASUREMENT_CONSUMPTION_UW * 2)

#define ECO_USE_PERCENTAGE_OF_CHARGING_RATE           (30)

#define SUPERCAP_E_MWS(U) ((SUPERCAP_CAPACITIANCE_F / 2 * U * U) / 1000)


/* charging less than this value (higher value) will be considered as *not* charging condition
   this value should be somewhere in the region of the supercap leakage and similar losses
   that are not acutally available after charging. */
#define ECO_P_CHARGING_THRESHOLD_UW                   (-900)

/* Leacage current of the storage element that will be considered for duty
   cycle planning*/
#define ECO_I_LEAKAGE_CURRENT_UA                      (500)

#define SUPERCAP_UNUSABLE_E_MWS                SUPERCAP_E_MWS(MIN_VOLTAGE_MV)
#define SUPERCAP_MIN_E_MWS                     SUPERCAP_E_MWS(MIN_VOLTAGE_MV)
#define SUPERCAP_MAX_AVAIL_E_MWS               (SUPERCAP_E_MWS(SUPERCAP_VOLTAGE_MAX_MV)  - SUPERCAP_UNUSABLE_E_MWS)
#define SUPERCAP_FULL_AVAIL_E_MWS              (SUPERCAP_E_MWS(SUPERCAP_VOLTAGE_FULL_MV) - SUPERCAP_UNUSABLE_E_MWS)

/* The energy allocation plans for using up all available energy until the end of the
   slot where - on the previous day - n percent of the overall charged energy was reached.
   This acts as a safety margin in case charging starts later/slower as expected */
#define ECO_TARGET_MIN_SOC_AT_N_PERCENT_OF_PREV_DAY (30UL)

/* The energy allocation plans for reaching SOC_max until the end of the
   slot where - on the previous day - n percent of the overall charged energy was reached.
   This acts as a safety margin in case charging stops earlier as expected */
#define ECO_TARGET_MAX_SOC_AT_N_PERCENT_OF_PREV_DAY (80UL)


/* minimum processing period when any charging */
#define ECO_PROCESSING_PERIOD_CHARGING_MIN_S           (5)

/* default sensing cycle that is used before we collected some info about energy availability
   should be overwritten to the respective backup reg as soon as possible */
#define ENVIRO_SENSING_PERIOD_BOOTSTRAP_S              (ECO_PROCESSING_PERIOD_CHARGING_MIN_S)

/* maximum processing period when any charging */
#define ECO_PROCESSING_PERIOD_CHARGING_MAX_S           (300)

#define ECO_ENERGY_PERMILL_FOR_PROCESSING              (15)

/* rough estimate for now - perfect accuracy not so important yet */
#define ECO_ENERGY_CONSUMTION_OF_PROCESSING_STEP_UWS   (30000)

#define ECO_PROCESSING_PERIOD_DISCHARGING_S            (900)
#define ECO_PROCESSING_PERIOD_BOOTSTRAP_S              (ENVIRO_SENSING_PERIOD_BOOTSTRAP_S)

#define ECO_CRITICAL_VOLTAGE_HIBERNATE_S               (60 * 60 * 5)

#define ECO_ATTRIBUTION_TASKS_MAX_NUMOF                (10)

typedef struct {
    union{
        struct{
            uint32_t bus_voltage_mv;  //bus voltage [mV]
            int32_t shunt_voltage_uv; //shunt voltage [uV]
            int32_t current_ua;       //current [uA]
            int32_t power_uw;         //power [uW]
            uint32_t t_us;            //relative timestamp of sample [us]
        };
        struct{
            //--- raw values of INA226 registers ---vvv
            int16_t bus_voltage_reg;
            int16_t shunt_voltage_reg;
            int16_t current_reg;
            int16_t power_reg;
            //--- raw values of INA226 registers ---^^^
        };
    };
} eco_measurement_set_t;

/* datatype to describe an operation which consumed energy
   e.g. the sampling of a sensor can be such an operation or the transmission
   of data over the radio link */
typedef struct {
    uint32_t t_start_us; /**< relative timestamp of operation start [us] */
    uint32_t t_end_us;   /**< relative timestamp of operation end [us] */
    int64_t  e_pws;      /**< energy consumed while the task was active [pWs] */
    bool     used;       /**< marker to tell if the spec is used */
} eco_task_consumption_spec_t;

typedef struct {
    uint32_t e_chg_mws[ECO_PM_INFO_SLOTS_PER_DAY];
    uint32_t e_use_mws[ECO_PM_INFO_SLOTS_PER_DAY];
    uint32_t u_latest_mv[ECO_PM_INFO_SLOTS_PER_DAY];
    uint32_t e_chg_sum_mws;
    uint32_t e_chg_sum_now_mws;
    uint32_t e_use_sum_mws;
} eco_day_stat_t;

typedef struct {
    eco_day_stat_t days[ECO_STATS_DAY_CNT];
    uint32_t       current_day_idx;
    uint32_t       current_slot_idx;
    timestamp_t    time_ref; /* this time ref is always used for time related calculations
                                and is only updated in specific moments when adding new
                                measurements or reevaluating duty_cycle to ensure
                                consistency with slot/day idxes etc. */

    /* absolute time reference to utc - ensure that this is not changed
       before/after big adjustments (sync) to the systems time reference
       happened. */
    uint64_t last_chg_measure_utc_ms;

    /* overall count of cycles on this day */
    uint32_t day_cycles_cnt;

    /* a slightly filtered (WMA) representation of the current charging
       rate. This is used to decide if the system is currently in
       charging or discharging state, while avoiding fast oscillations
       due to e.g. temporary low-charging conditions */
    int32_t p_chg_mavg_uw;

    int32_t p_chg_latest_uw;
    int32_t i_chg_latest_uA;

    /* sum of charged energy within the current slot always positive */
    uint64_t e_chg_slot_sum_uws; /**< number of values used for averaging
                                      related values in the current slot */
    uint32_t e_chg_slot_cnt; /**< number of values used for averaging
                                 related values in the current slot */
    uint32_t e_lp_measure_slot_cnt; /**< number of LP-measurements
                                    performed in this slot */

    /* sum of consumed energy within the current slot always positive */
    uint64_t e_use_slot_sum_uws;
    uint32_t e_use_slot_cnt;
    int32_t  e_use_latest_uws;
    int32_t e_use_mavg_uws;

    /* latest available voltage reading for current slot */
    uint32_t u_latest_mv;

    /* seconds the current slot has been active */
    uint32_t secs_in_slot;
} eco_energy_history_t;

typedef struct{
    uint32_t Ea_mWs;      /*< energy available [mWs] */
    uint32_t Ec_mWs;      /*< energy capacity [mWs] */
    uint32_t Es_mWs;      /*< charged power in the current slot [mWs] */
    uint32_t Et_mWs;      /*< charged power today [mWs] */
    uint32_t Em_mWs;      /*< Weighted moving average of charged power over 24h [mWs] */
    uint32_t Pon_avg_uW;  /*< Average power consumtion in ON-State [uW] */
    uint32_t ton_avg_ms;  /*< Average time in ON-State [ms] */
    int32_t  Poff_avg_uW; /*< Average power consumtion in OFF-State [uW] */
    uint32_t toff_avg_ms; /*< Average time in OFF-State [ms] */
    uint32_t t_s;         /*< starting time for avg calculations [s of day] */
    uint32_t nc_n;        /*< number of cycles since bootstrap */
    uint16_t SOC_pm;      /*< State-of-Charge [per mil] */
    uint16_t Usc_mv;      /*< Supercap Voltage [mv] */
    uint16_t Upv_mv;      /*< PV-Panel open circuit Voltage [mv] */
    uint16_t Pd_uW;       /*< Supercap self-discharge rate [uW] */
    uint16_t dc_pm;       /*< Duty-Cycle [per mil] */
    uint16_t nc_i;        /*< number of cycles used for avg measures */
} eco_energy_info_t;

/* TODO: the sample buffer doesn't need to be in retained ram! */
/* add dynamic sampling rate and RRD data processing */
typedef struct{
    eco_measurement_set_t *samples;
    eco_energy_history_t history;
    uint32_t             e_alloc_mws[ECO_PM_INFO_SLOTS_PER_DAY];
    uint32_t             t_sense_s[ECO_PM_INFO_SLOTS_PER_DAY];
    uint32_t             alloc_sensing_cycles[ECO_PM_INFO_SLOTS_PER_DAY];
    eco_task_consumption_spec_t task_specs[ECO_ATTRIBUTION_TASKS_MAX_NUMOF];
    eco_task_consumption_spec_t prev_task_specs[ECO_ATTRIBUTION_TASKS_MAX_NUMOF];
    eco_energy_info_t energy_info;
    size_t sample_cnt_target;
    size_t sample_cnt;
    size_t reg_read_err_cnt;
    size_t no_alert_err_cnt;
    size_t init_err_cnt;
    uint32_t sensing_period_s;
    uint32_t eco_processing_period_s;
    uint32_t cycle_cnt;
    kernel_pid_t pid;
    kernel_pid_t requester_pid;
    void *requester_ctx;
    gpio_t charger_disable_pin;
    adc_t pv_adc_line;
    mutex_t mutex;
    ina22x_t ina22x_dev;
    const ina22x_params_t *ina22x_params;
    bool cold_init_done;
    bool state_retained;
    bool waiting_for_alert;
    bool currently_charging;
    bool hit_critical_voltage;
} eco_t;

extern kernel_pid_t power_measure_thread_pid;

kernel_pid_t eco_init_thread(eco_t *eco);

bool eco_alert_active(eco_t *e);

#ifdef __cplusplus
}
#endif

#endif /* ECO_H */
/** @} */
