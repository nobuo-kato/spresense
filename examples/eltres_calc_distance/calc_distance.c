#include <stdio.h>
#include <stdbool.h>
#include "calc_distance.h"
#include "calc_distance_port.h"
#define _USE_MATH_DEFINES // to use M_PI, this define is needed.
#include "math.h"

#if defined(CONFIG_CALC_DISTANCE_SPEED_THRESH)
#define SPEED_THRESH (CONFIG_CALC_DISTANCE_SPEED_THRESH)
#else
//#define SPEED_THRESH (5.0)
#define SPEED_THRESH (0.0)
#endif

#define FIX_TYPE_3D (3)

//#define CALC_DISTANCE_UNIT_TEST_CORE
#define CONFIG_CALC_DISTANCE_DEBUG
#ifdef CONFIG_CALC_DISTANCE_DEBUG
static uint32_t update_called_count = 0;
static uint32_t update_calced_count = 0;
static uint32_t get_called_count = 0;
#endif

static double accumurated_distance;
static void (*lock)(void) = (void(*)(void))0;
static void (*unlock)(void) = (void(*)(void))0;

static void clearSource(CalcDistanceSource * source){
    source->fix_type = INVALID_FIX_TYPE;
    source->lon_deg = INVALID_LON;
    source->lat_deg = INVALID_LAT;
    source->speed_kph = INVALID_SPEED;
}

static double Hubeny_formula(double lat_diff, double lat_ave, double lon_diff){
    // constants not depends on position.
    static const float major_axis = 6378137;
    static const float flattering = 1.0f/298.257222101f;
    static const float e_pow_2 = flattering * (2.0f-flattering);
    static const float ns_coef = major_axis * (1.0f-e_pow_2);

    // # calc ellipsoidal correction
    double sin_lat_ave = sin(lat_ave);
    double sin_lat_ave_pow2 = sin_lat_ave * sin_lat_ave;
    double ec = 1.0f - e_pow_2 * sin_lat_ave_pow2;
    double sqrt_ec = sqrt(ec);

    // # calc radius of curvature
    double meridian_rc = ns_coef/(ec*sqrt_ec);
    double prime_vertical_rc = major_axis/sqrt_ec;

    // # calc distance along North-Soutn, East-West
    double ns_dist = lat_diff * meridian_rc;
    double ew_dist_wo_cos_pow2 = lon_diff * prime_vertical_rc;
    double cos_lat_ave_pow2 = 1.0f - sin_lat_ave_pow2;

    // # calc distance of hypotenuse
    double result = sqrt(ns_dist * ns_dist + ew_dist_wo_cos_pow2 * ew_dist_wo_cos_pow2 * cos_lat_ave_pow2);
    return result;
}


static double calc_distance(double prev_lon, double curr_lon, double prev_lat, double curr_lat) {
    double lon_diff_rad = (curr_lon - prev_lon) * M_PI / 180.0;
    double lat_diff_rad = (curr_lat - prev_lat) * M_PI / 180.0;
    double lat_ave_rad = (curr_lat + prev_lat) / 2.0 * M_PI / 180.0;

    return Hubeny_formula(lat_diff_rad, lat_ave_rad, lon_diff_rad);
}

static double prev_lon = INVALID_LON;
static double prev_lat = INVALID_LAT;

static CalcDistanceResult update_distance(CalcDistanceSource * source) {
    double current_distance = -1.0;
    CalcDistanceResult result = CALC_DISTANCE_ERROR_UNDEFINED;
#ifdef CONFIG_CALC_DISTANCE_DEBUG
    update_called_count++;
#endif
    if (source->fix_type == FIX_TYPE_3D) {
        if (SPEED_THRESH < source->speed_kph) {
            if (source->lon_deg != INVALID_LON && source->lat_deg != INVALID_LAT) {
                if (prev_lon != INVALID_LON && prev_lat != INVALID_LAT) {
#ifdef CONFIG_CALC_DISTANCE_DEBUG
                    update_calced_count++;
#endif
                    current_distance = calc_distance(prev_lon, source->lon_deg, prev_lat, source->lat_deg);
                    if (lock) {
                        lock();
                    }
                    accumurated_distance += current_distance;
                    if (unlock) {
                        unlock();
                    }
                    prev_lon = source->lon_deg;
                    prev_lat = source->lat_deg;
                    clearSource(source);
                    result = CALC_DISTANCE_NO_ERROR;
                } else {
                    prev_lon = source->lon_deg;
                    prev_lat = source->lat_deg;
                    result = CALC_DISTANCE_ERROR_INVALID_PERV_LON_OR_LAT;
                }
            } else {
                result = CALC_DISTANCE_ERROR_INVALID_CURRENT_LON_OR_LAT;
            }
        } else {
            result = CALC_DISTANCE_ERROR_LOW_SPEED;
        }
    } else {
        result = CALC_DISTANCE_ERROR_NOT_3D_FIXED;
    }
#ifdef CONFIG_CALC_DISTANCE_DEBUG
    printf("  \"cd\":%.4f, \"ad\":%.4f,\r\n",current_distance, accumurated_distance);
    printf("  \"call\":%ld, \"calc\":%ld, \"get\":%ld,\r\n",update_called_count, update_calced_count, get_called_count);
    printf("  \"result\":%d,\r\n", result);
//    printf("%.4f,%.4f, %ld, %ld, %ld, %ld\n",current_distance, accumurated_distance, update_called_count, update_calced_count, get_called_count, result);
#endif
    return result;
}

void init_calc_distance(void *param){
    init_calc_distance_port(update_distance, param);
    clear_calc_distance();
}

void clear_calc_distance(void) {
    accumurated_distance = 0.0;
    prev_lon = INVALID_LON;
    prev_lat = INVALID_LAT;
}

uint32_t get_calc_distance(void) {
    uint32_t result = (uint32_t)((accumurated_distance * 10.0) + 0.5);
    return result;
}

void get_stat(uint32_t *p_update_called_count, uint32_t *p_update_calced_count, uint32_t *p_get_called_count){
#ifdef CONFIG_CALC_DISTANCE_DEBUG
    p_update_called_count = &update_called_count;
    p_update_calced_count = &update_calced_count;
    p_get_called_count = &get_called_count;
#endif
}

//////////////////////// --------------- unit test code from here ----------------
#ifdef CALC_DISTANCE_UNIT_TEST_CORE
typedef struct {
    CalcDistanceSource source;
    double prev_lon_exp;
    double prev_lat_exp;
    CalcDistanceResult exp;
} CoreTestData1Type;

static const CoreTestData1Type test_data_1[] = {
    // combination of invalid parameters
    {{INVALID_LON, INVALID_LAT, INVALID_SPEED,    INVALID_FIX_TYPE}, INVALID_LON, INVALID_LAT, CALC_DISTANCE_ERROR_NOT_3D_FIXED},
    {{INVALID_LON, INVALID_LAT, INVALID_SPEED,    FIX_TYPE_3D},      INVALID_LON, INVALID_LAT, CALC_DISTANCE_ERROR_LOW_SPEED},
    {{INVALID_LON, INVALID_LAT, SPEED_THRESH,     FIX_TYPE_3D},      INVALID_LON, INVALID_LAT, CALC_DISTANCE_ERROR_LOW_SPEED},
    {{INVALID_LON, INVALID_LAT, SPEED_THRESH+1.0, FIX_TYPE_3D},      INVALID_LON, INVALID_LAT, CALC_DISTANCE_ERROR_INVALID_CURRENT_LON_OR_LAT},
    {{INVALID_LON, 1.0,         SPEED_THRESH+1.0, FIX_TYPE_3D},      INVALID_LON, INVALID_LAT, CALC_DISTANCE_ERROR_INVALID_CURRENT_LON_OR_LAT},
    {{2.0,         INVALID_LAT, SPEED_THRESH+1.0, FIX_TYPE_3D},      INVALID_LON, INVALID_LAT, CALC_DISTANCE_ERROR_INVALID_CURRENT_LON_OR_LAT},
    // valid parameters but prevs are not valid.
    {{3.0,         4.0,         SPEED_THRESH+1.0, FIX_TYPE_3D},      INVALID_LON, INVALID_LAT, CALC_DISTANCE_ERROR_INVALID_PERV_LON_OR_LAT},
    // all parameters are valid.
    {{5.0,         6.0,         SPEED_THRESH+1.0, FIX_TYPE_3D},      3.0,         4.0,         CALC_DISTANCE_NO_ERROR},

    // combination of invalid parameters after valid parameters.
    {{INVALID_LON, INVALID_LAT, INVALID_SPEED,    INVALID_FIX_TYPE}, 5.0,         6.0,         CALC_DISTANCE_ERROR_NOT_3D_FIXED},
    {{INVALID_LON, INVALID_LAT, INVALID_SPEED,    FIX_TYPE_3D},      5.0,         6.0,         CALC_DISTANCE_ERROR_LOW_SPEED},
    {{INVALID_LON, INVALID_LAT, SPEED_THRESH,     FIX_TYPE_3D},      5.0,         6.0,         CALC_DISTANCE_ERROR_LOW_SPEED},
    {{INVALID_LON, INVALID_LAT, SPEED_THRESH+1.0, FIX_TYPE_3D},      5.0,         6.0,         CALC_DISTANCE_ERROR_INVALID_CURRENT_LON_OR_LAT},
    {{INVALID_LON, 7.0,         SPEED_THRESH+1.0, FIX_TYPE_3D},      5.0,         6.0,         CALC_DISTANCE_ERROR_INVALID_CURRENT_LON_OR_LAT},
    {{8.0,         INVALID_LAT, SPEED_THRESH+1.0, FIX_TYPE_3D},      5.0,         6.0,         CALC_DISTANCE_ERROR_INVALID_CURRENT_LON_OR_LAT},
    // after CALC_DISTANCE_NO_ERROR, CALC_DISTANCE_ERROR_INVALID_PERV_LON_OR_LAT is not returned until init_calc_distance() or clear_calc_distance().
    // all parameters are valid.
    {{9.0,         10.0,        SPEED_THRESH+1.0, FIX_TYPE_3D},      5.0,         6.0,         CALC_DISTANCE_NO_ERROR},
};

static const CoreTestData1Type test_data_2[] = {
    {{11.0, 12.0, SPEED_THRESH+1.0, FIX_TYPE_3D}, INVALID_LON, INVALID_LAT, CALC_DISTANCE_ERROR_INVALID_PERV_LON_OR_LAT},
    {{13.0, 14.0, SPEED_THRESH+1.0, FIX_TYPE_3D}, 11.0,        12.0,        CALC_DISTANCE_NO_ERROR},
};

#endif

int calc_distance_core_test_main(void) {
    printf("calc_distance_core_test_main()\r\n");
#ifdef CALC_DISTANCE_UNIT_TEST_CORE
    int pass_count = 0;
    int fail_count = 0;

    clear_calc_distance();
    for(int test_count = 0; test_count < sizeof(test_data_1)/sizeof(test_data_1[0]); test_count++) {
        CoreTestData1Type data = test_data_1[test_count];
        printf("%d, ", test_count);
        bool prev_lon_passed = (prev_lon == data.prev_lon_exp);
        printf("%f, %f, %s, ", prev_lon, data.prev_lon_exp, prev_lon_passed?"PASS":"FAIL");
        bool prev_lat_passed = (prev_lat == data.prev_lat_exp);
        printf("%f, %f, %s, ", prev_lat, data.prev_lat_exp, prev_lat_passed?"PASS":"FAIL");
        printf("%f, %f, %f, %ld, ", data.source.lon_deg, data.source.lat_deg, data.source.speed_kph, data.source.fix_type);
        CalcDistanceResult result = update_distance(&data.source);
        bool result_passed = (result == data.exp);
        printf("%d, %d, %s\n", result, data.exp, result_passed?"PASS":"FAIL");
        if (prev_lon_passed) {
            pass_count++;
        } else {
            fail_count++;
        }
        if (prev_lat_passed) {
            pass_count++;
        } else {
            fail_count++;
        }
        if (result_passed) {
            pass_count++;
        } else {
            fail_count++;
        }
        printf("get_calc_distance()=%ld\r\n",get_calc_distance());
    }

    // checking clear_calc_distance() makes clear prevs, after all valid parameters are inputted. 
    clear_calc_distance();
    for(int test_count = 0; test_count < sizeof(test_data_2)/sizeof(test_data_2[0]); test_count++) {
        CoreTestData1Type data = test_data_2[test_count];
        printf("%d, ", test_count);
        bool prev_lon_passed = (prev_lon == data.prev_lon_exp);
        printf("%f, %f, %s, ", prev_lon, data.prev_lon_exp, prev_lon_passed?"PASS":"FAIL");
        bool prev_lat_passed = (prev_lat == data.prev_lat_exp);
        printf("%f, %f, %s, ", prev_lat, data.prev_lat_exp, prev_lat_passed?"PASS":"FAIL");
        printf("%f, %f, %f, %ld, ", data.source.lon_deg, data.source.lat_deg, data.source.speed_kph, data.source.fix_type);
        CalcDistanceResult result = update_distance(&data.source);
        bool result_passed = (result == data.exp);
        printf("%d, %d, %s\n", result, data.exp, result_passed?"PASS":"FAIL");
        if (prev_lon_passed) {
            pass_count++;
        } else {
            fail_count++;
        }
        if (prev_lat_passed) {
            pass_count++;
        } else {
            fail_count++;
        }
        if (result_passed) {
            pass_count++;
        } else {
            fail_count++;
        }
        printf("get_calc_distance()=%ld\r\n",get_calc_distance());
    }
    printf("PASS:%d, FAIL:%d\n", pass_count, fail_count);
    return fail_count;
#else
    return 0;
#endif
}
