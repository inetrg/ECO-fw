/*
 * Copyright (C) 2018 HAW Hamburg
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
 * @brief       comfort functions for using minmea
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */

#ifndef MINMEA_HELPER_H
#define MINMEA_HELPER_H

#include "minmea.h"
#include "encoding.h"

 #ifdef __cplusplus
 extern "C" {
 #endif

#define GPS_YEAR_OFFSET (2000)

 typedef struct {
     struct minmea_date date;
     struct minmea_time time;
     struct minmea_float latitude;
     struct minmea_float longitude;
     struct minmea_float speed_kph;
     struct minmea_float true_track_degrees;
     struct minmea_float altitude;
     struct minmea_float height;
     struct minmea_float hdop;
     struct minmea_float pdop;
     struct minmea_float vdop;
     struct minmea_float dgps_age;
     struct minmea_sat_info sats[32];
     int visible_sats;
     int tracked_sats;
     int fix_quality;
     int ttf;        // time to first fix
     int sleep_dur; // sleep duration after max fixes
     int cfc;    // minimum continuous fix cnt
     enum minmea_faa_mode faa_mode;
     char height_units;
     char altitude_units;
     bool valid;
 } gps_data_t;

void print_minmea_float(const char* name, struct minmea_float* mf);
int snprint_minmea_float(char* str, int n, struct minmea_float* mf);
void print_rmc(struct minmea_sentence_rmc* rmc);
void print_gga(struct minmea_sentence_gga* gga);
void print_vtg(struct minmea_sentence_vtg* vtg);
void print_gsa(struct minmea_sentence_gsa* gsa);
void print_gsv(struct minmea_sentence_gsv* gsv);
void print_all_nmea_info(const char* sentence);
void print_gps_data(gps_data_t* data);
int populate_gps_data(gps_data_t* data, const char* sentence);
size_t minmea_helper_encode(gps_data_t *data, uint8_t *buff, size_t len,
                         uint8_t format);


#ifdef __cplusplus
}
#endif

#endif /* MINMEA_HELPER_H */
/** @} */
