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
#include <inttypes.h>

#include "minmea_helper.h"
#include "cbor.h"

static int get_number_of_zeros(int num){
    int cnt = 0;
    while(num > 1) {
        num /= (10);
        cnt++;
    }
    return cnt;
}

void print_minmea_float(const char* name, struct minmea_float* mf){
    int32_t integer_part = mf->value / mf->scale;
    int fr_lead_zeros = get_number_of_zeros(mf->scale);
    printf("%s: %"PRId32".%0*"PRId32"\n", name, integer_part, fr_lead_zeros,
           mf->value - integer_part * mf->scale);
}

int snprint_minmea_float(char* str, int n, struct minmea_float* mf){
    int32_t integer_part = mf->value / mf->scale;
    int fr_lead_zeros = get_number_of_zeros(mf->scale);
    return snprintf(str, n, "%"PRId32".%0*"PRId32"", integer_part, fr_lead_zeros, mf->value - integer_part * mf->scale);
}

void print_rmc(struct minmea_sentence_rmc* rmc) {
    printf("date: %02d-%02d-%02d\n", rmc->date.day, rmc->date.month,
           rmc->date.year);

    printf("time: %02d:%02d:%02d.%03d\n", rmc->time.hours,
           rmc->time.minutes, rmc->time.seconds,
           rmc->time.microseconds);

    printf("valid: %d\n", rmc->valid);
    printf("latitude: %f\n", minmea_tocoord(&rmc->latitude));
    printf("longitude: %f\n", minmea_tocoord(&rmc->longitude));
    print_minmea_float("speed",     &rmc->speed);
    print_minmea_float("course",    &rmc->course);
    print_minmea_float("variation", &rmc->variation);
}

void print_gga(struct minmea_sentence_gga* gga) {
    printf("time: %02d:%02d:%02d.%03d\n", gga->time.hours,
           gga->time.minutes, gga->time.seconds,
           gga->time.microseconds);

    printf("latitude: %f\n", minmea_tocoord(&gga->latitude));
    printf("longitude: %f\n", minmea_tocoord(&gga->longitude));
    printf("fix_quality: %d\n", gga->fix_quality);
    printf("satellites_tracked: %d\n", gga->satellites_tracked);

    print_minmea_float("hdop",     &gga->hdop);
    print_minmea_float("altitude", &gga->altitude);
    print_minmea_float("height",   &gga->height);

    printf("altitude_units: %c\n", gga->altitude_units);
    printf("height_units: %c\n", gga->height_units);
    printf("dgps_age: %f\n", minmea_tofloat(&gga->dgps_age));
}

void print_vtg(struct minmea_sentence_vtg* vtg) {
    print_minmea_float("true_track_degrees",     &vtg->true_track_degrees);
    print_minmea_float("magnetic_track_degrees", &vtg->magnetic_track_degrees);
    print_minmea_float("speed_knots",            &vtg->speed_knots);
    print_minmea_float("speed_kph",              &vtg->speed_kph);

    printf("minmea_faa_mode: %c\n", vtg->faa_mode);
}

void print_gsa(struct minmea_sentence_gsa* gsa) {
    printf("mode: %c\n", gsa->mode);
    printf("fix_type: %d\n", gsa->fix_type);
    for (unsigned i = 0; i < (sizeof(gsa->sats)/sizeof(gsa->sats[0])); i++) {
        printf("%d ", gsa->sats[i]);
    }
    printf("\n");

    print_minmea_float("pdop", &gsa->pdop);
    print_minmea_float("hdop", &gsa->hdop);
    print_minmea_float("vdop", &gsa->vdop);
}

void print_gsv(struct minmea_sentence_gsv* gsv) {
    printf("total_msgs: %d\n", gsv->total_msgs);
    printf("msg_nr: %d\n", gsv->msg_nr);
    printf("total_sats: %d\n", gsv->total_sats);

    unsigned sats_per_sentence = sizeof(gsv->sats)/sizeof(gsv->sats[0]);
    unsigned sats_to_print = gsv->total_sats - (gsv->msg_nr - 1) * sats_per_sentence;
    unsigned print_now = sats_to_print;
    if (sats_to_print < sats_per_sentence) {
        print_now = sats_to_print;
    }
    else {
        print_now = sats_per_sentence;
    }
    printf("sats_to_print: %d (%d now)\n", sats_to_print, print_now);

    for(unsigned i = 0; i < print_now; i++) {
        printf("-----------------------\n");
        printf("nr: %d\n",        gsv->sats[i].nr);
        printf("elevation: %d\n", gsv->sats[i].elevation);
        printf("azimuth: %d\n",   gsv->sats[i].azimuth);
        printf("snr: %d\n",       gsv->sats[i].snr);
        printf("-----------------------\n");
    }
}

int populate_gps_data(gps_data_t* data, const char* sentence) {
    enum minmea_sentence_id sentence_id;
    sentence_id = minmea_sentence_id(sentence, 1);

    struct minmea_sentence_rmc rmc;
    struct minmea_sentence_gga gga;
    struct minmea_sentence_gsa gsa;
    struct minmea_sentence_gsv gsv;
    struct minmea_sentence_vtg vtg;

    switch (sentence_id) {
        case MINMEA_SENTENCE_RMC:
            if (minmea_parse_rmc(&rmc, sentence)) {
                data->date = rmc.date;
                data->time = rmc.time;
                data->latitude = rmc.latitude;
                data->longitude = rmc.longitude;
                data->valid = rmc.valid;
            }
            break;
        case MINMEA_SENTENCE_GGA:
            if (minmea_parse_gga(&gga, sentence)) {
                data->fix_quality = gga.fix_quality;
                data->tracked_sats = gga.satellites_tracked;
                data->altitude = gga.altitude;
                data->height = gga.height;
                data->altitude_units = gga.altitude_units;
                data->height_units = gga.height_units;
                data->dgps_age = gga.dgps_age;
            }
            break;
        case MINMEA_SENTENCE_GSA:
            if (minmea_parse_gsa(&gsa, sentence)) {
                data->hdop = gsa.hdop;
                data->pdop = gsa.pdop;
                data->vdop = gsa.vdop;
            }
            break;
        case MINMEA_SENTENCE_GLL:
            break;
        case MINMEA_SENTENCE_GST:
            break;
        case MINMEA_SENTENCE_GSV:
            if (minmea_parse_gsv(&gsv, sentence)) {
                unsigned sats_per_sentence = sizeof(gsv.sats)/sizeof(gsv.sats[0]);
                unsigned sats_left = gsv.total_sats - (gsv.msg_nr - 1) * sats_per_sentence;
                unsigned sats_now = sats_left > sats_per_sentence ? sats_per_sentence : sats_left;

                data->visible_sats = gsv.total_sats;

                for(unsigned i = 0; i < sats_now; i++) {
                    data->sats[(gsv.msg_nr - 1) * sats_per_sentence + i] = gsv.sats[i];
                }
            }
            break;
        case MINMEA_SENTENCE_VTG:
            if (minmea_parse_vtg(&vtg, sentence)) {
                data->true_track_degrees = vtg.true_track_degrees;
                data->speed_kph = vtg.speed_kph;
                data->faa_mode = vtg.faa_mode;
            }
            break;
        case MINMEA_SENTENCE_ZDA:
            break;
        case MINMEA_INVALID:
            break;
        case MINMEA_UNKNOWN:
            break;
    }

    return sentence_id;
}

void print_gps_data(gps_data_t* data) {
    printf("\n\n####################################\n");
    printf("date: %02d-%02d-%02d\n", data->date.day, data->date.month,
           data->date.year);

    printf("time: %02d:%02d:%02d.%03d\n", data->time.hours,
           data->time.minutes, data->time.seconds,
           data->time.microseconds);

    printf("valid: %d\n", data->valid);
    printf("latitude: %f\n", minmea_tocoord(&data->latitude));
    printf("longitude: %f\n", minmea_tocoord(&data->longitude));
    printf("speed_kph: %f\n", minmea_tofloat(&data->speed_kph));
    printf("true_track_degrees: %f\n", minmea_tofloat(&data->true_track_degrees));
    printf("altitude: %f\n", minmea_tofloat(&data->altitude));
    printf("height: %f\n", minmea_tofloat(&data->height));
    printf("hdop: %f\n", minmea_tofloat(&data->hdop));
    printf("pdop: %f\n", minmea_tofloat(&data->pdop));
    printf("vdop: %f\n", minmea_tofloat(&data->vdop));
    printf("visible_sats: %d\n", data->visible_sats);
    printf("tracked_sats: %d\n", data->tracked_sats);

    printf("+-----------sats-----------+\n");
    printf("| nr | elev. | azim. | snr |\n");
    printf("+--------------------------+\n");
    for (int i = 0; i < data->visible_sats; i++) {
        printf("| %2d |  %3d  |  %3d  |  %2d |\n", data->sats[i].nr,
                                                   data->sats[i].elevation,
                                                   data->sats[i].azimuth,
                                                   data->sats[i].snr);
    }
    printf("+--------------------------+\n");
    printf("continuus fix cnt: %d\n", data->cfc);
    printf("fix_quality: %d\n", data->fix_quality);
    printf("dgps_age: %f\n", minmea_tofloat(&data->dgps_age));
    printf("faa_mode: %d\n", data->faa_mode);
    printf("height_units: %c\n", data->height_units);
    printf("altitude_units: %c\n", data->altitude_units);
    printf("####################################\n");
}

void print_all_nmea_info(const char* sentence) {
    enum minmea_sentence_id sentence_id;

    sentence_id = minmea_sentence_id(sentence, 1);

    printf("\n\n\nID: %d\n", sentence_id);

    switch (sentence_id) {
        case MINMEA_SENTENCE_RMC:
            printf("MINMEA_SENTENCE_RMC\n");
            struct minmea_sentence_rmc rmc;
            if (minmea_parse_rmc(&rmc, sentence)) {
                print_rmc(&rmc);
            }
            break;
        case MINMEA_SENTENCE_GGA:
            printf("MINMEA_SENTENCE_GGA\n");
            struct minmea_sentence_gga gga;
            if (minmea_parse_gga(&gga, sentence)) {
                print_gga(&gga);
            }
            break;
        case MINMEA_SENTENCE_GSA:
            printf("MINMEA_SENTENCE_GSA\n");
            struct minmea_sentence_gsa gsa;
            if (minmea_parse_gsa(&gsa, sentence)) {
                print_gsa(&gsa);
            }
            break;
        case MINMEA_SENTENCE_GLL:
            printf("MINMEA_SENTENCE_GLL\n");
            break;
        case MINMEA_SENTENCE_GST:
            printf("MINMEA_SENTENCE_GST\n");
            break;
        case MINMEA_SENTENCE_GSV:
            printf("MINMEA_SENTENCE_GSV\n");
            struct minmea_sentence_gsv gsv;
            if (minmea_parse_gsv(&gsv, sentence)) {
                print_gsv(&gsv);
            }
            break;
        case MINMEA_SENTENCE_VTG:
            printf("MINMEA_SENTENCE_VTG\n");
            struct minmea_sentence_vtg vtg;
            if (minmea_parse_vtg(&vtg, sentence)) {
                print_vtg(&vtg);
            }
            break;
        case MINMEA_SENTENCE_ZDA:
            printf("MINMEA_SENTENCE_ZDA\n");
            break;
        case MINMEA_INVALID:
            printf("MINMEA_INVALID\n");
            break;
        case MINMEA_UNKNOWN:
            printf("MINMEA_UNKNOWN\n");
            break;
    }
}

size_t minmea_helper_encode(gps_data_t *data, uint8_t *buff, size_t len,
                            encoding_format_t format)
{
    if (format == ENCODING_JSON) {
        int cnt = snprintf((char*)buff, len,
               "{\"t\":\"20%02d-%02d-%02dT%02d:%02d:%02d.%03dZ\",\"Lat\":\"%f\", \"Lon\":%f}",
               data->date.year,
               data->date.month,
               data->date.day,
               data->time.hours,
               data->time.minutes,
               data->time.seconds,
               data->time.microseconds,
               minmea_tofloat(&data->latitude),
               minmea_tofloat(&data->longitude));
        if ( cnt > 0 && (unsigned)cnt < len) {
            return cnt;
        }
    }
    else if (format == ENCODING_CBOR) {
        CborEncoder encoder, mapEncoder;
        cbor_encoder_init(&encoder, buff, len, 0);
        if( cbor_encoder_create_map(&encoder, &mapEncoder, 10) != CborNoError ||
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
            cbor_encode_text_stringz(&mapEncoder, "sd") != CborNoError ||
            cbor_encode_int(&mapEncoder, data->sleep_dur) != CborNoError ||
            cbor_encode_text_stringz(&mapEncoder, "cfc") != CborNoError ||
            cbor_encode_int(&mapEncoder, data->cfc) != CborNoError ||
            cbor_encoder_close_container(&encoder, &mapEncoder) != CborNoError) {
            return 0;
        }

        return cbor_encoder_get_buffer_size(&encoder, buff);
    }

    return 0;
}
