#include <stdio.h>
#include <stdbool.h>
#include "CXM150x_GNSS.h"
#include "calc_distance_port.h"

#define CALC_DISTANCE_UNIT_TEST_PORT

#ifdef CALC_DISTANCE_UNIT_TEST_PORT
#include <string.h>
#endif

// #define IGNORE_NMEA_CHECKSUM

// DDDMM.mmmm, 0 <= len('DDD') <= 3, len('MM') == 2, len('mmmm') == 4
#define MAX_DEGREES_SIZE	(3)
#define INT_MINUTES_SIZE	(2)
#define U1_MINUTES_SIZE     (4)

static CalcDistanceSource g_calc_distance_source;
static UPDATE_DISTANCE_CALLBACK_FUNC_POINTER g_update_distance_func = (UPDATE_DISTANCE_CALLBACK_FUNC_POINTER)0;
static CXM150xNMEAGSAInfo g_gsa_info;
static CXM150xNMEARMCInfo g_rmc_info;
static CXM150xNMEAVTGInfo g_vtg_info;

static int32_t get_int_len(uint8_t* digit_str){
    int32_t result = -1;
    for(int offset = 0; offset < (MAX_DEGREES_SIZE + INT_MINUTES_SIZE + 1); offset++) {
        if (digit_str[offset] == '.') {
            // found decimal point.
            result = offset;
            break;
		} else if ('0' <= digit_str[offset] && digit_str[offset] <= '9') {
            // OK continue
        } else {
            // unexpected character detected
            break;
        }
    }
    return result;
}

static double extract_degree(uint8_t *deg_str){
    int32_t int_len = get_int_len(deg_str);
    if (INT_MINUTES_SIZE <= int_len) {
        uint32_t degrees_digits = int_len - INT_MINUTES_SIZE;
        uint32_t deg_int = 0;
        if (0 < degrees_digits){
            for (int index = 0; index < degrees_digits; index++) {
                deg_int *= 10;
                deg_int += (deg_str[index] - '0');
            }
        }
        uint32_t min_int = 0;
        for (int index = degrees_digits; index < int_len; index++) {
            min_int *= 10;
            min_int += (deg_str[index] - '0');
        }
        uint32_t min_under1 = 0;
        uint32_t denom = 1;
        for (int index = 0; index < U1_MINUTES_SIZE; index++) {
            denom *= 10;
            min_under1 *= 10;
            min_under1 += (deg_str[int_len + 1 + index] - '0');
        }
        double min_f = min_int + (double)min_under1 / (double)denom;
        return ((double)deg_int + (min_f / 60.0));
    } else {
        // integer part is too short.
        return -1.0;
    }
}

static void common_nmea_callback(void *param, uint32_t evt_id) {
    CXM150x_EVENT_CALLBACK_ID id = (CXM150x_EVENT_CALLBACK_ID)evt_id;
    switch(id) {
        case CXM150x_EVENT_CALLBACK_ID_NMEAGSA_EVENT:{
            CXM150xNMEAGSAInfo *p_gsa_info = (CXM150xNMEAGSAInfo *)param;
            if ( p_gsa_info->m_cs_correct
#ifdef IGNORE_NMEA_CHECKSUM            
                || true
#endif                
            ){
                if ('0' < p_gsa_info->m_mode2[0] && p_gsa_info->m_mode2[0] < '4'){
                    g_calc_distance_source.fix_type = p_gsa_info->m_mode2[0] - '0';
//                    printf("g_calc_distance_source.fix_type is updated to %ld\r\n", g_calc_distance_source.fix_type);
                } else {
//                    printf("gsa_info.m_mode2[0] is out of range\r\n");
                }
            }
            break;
        }
        case CXM150x_EVENT_CALLBACK_ID_NMEARMC_EVENT:{
            CXM150xNMEARMCInfo *p_rmc_info = (CXM150xNMEARMCInfo *)param;
            if ( p_rmc_info->m_cs_correct
#ifdef IGNORE_NMEA_CHECKSUM            
                || true
#endif                
            ){
                double lat = extract_degree(p_rmc_info->m_lat);
                if (0 <= lat) {
                    if (p_rmc_info->m_n_s[0] == 'S') {
                        lat *= -1.0;
                    }
                } else {
                    lat = INVALID_LAT;
                }
                g_calc_distance_source.lat_deg = lat;
//                printf("g_calc_distance_source.lat_deg is updated to %f\r\n", g_calc_distance_source.lat_deg);
                double lon = extract_degree(p_rmc_info->m_lon);
                if (0 <= lon) {
                    if (p_rmc_info->m_e_w[0] == 'W') {
                        lon *= -1.0;
                    }
                } else {
                    lon = INVALID_LON;
                }
                g_calc_distance_source.lon_deg = lon;
//                printf("g_calc_distance_source.lon_deg is updated to %f\r\n", g_calc_distance_source.lon_deg);
            }
            break;
        }
        case CXM150x_EVENT_CALLBACK_ID_NMEAVTG_EVENT:{
            CXM150xNMEAVTGInfo *p_vtg_info = (CXM150xNMEAVTGInfo *)param;
            if ( p_vtg_info->m_cs_correct
#ifdef IGNORE_NMEA_CHECKSUM            
                || true
#endif                
            ){
                g_calc_distance_source.speed_kph = p_vtg_info->m_speed_kph;
//                printf("g_calc_distance_source.speed_kph is updated to %f\r\n",g_calc_distance_source.speed_kph);
                if (g_update_distance_func) {
                    CalcDistanceResult result = g_update_distance_func(&g_calc_distance_source);
                }
            }
            break;

        }
        default:
            break;
    }
}

void init_calc_distance_port(UPDATE_DISTANCE_CALLBACK_FUNC_POINTER cb, void *param) {
    g_update_distance_func = cb;
    g_calc_distance_source.fix_type = INVALID_FIX_TYPE;
    g_calc_distance_source.lon_deg = INVALID_LON;
    g_calc_distance_source.lat_deg = INVALID_LAT;
    g_calc_distance_source.speed_kph = INVALID_SPEED;

    register_CXM150x_NMEAGSA_event(&g_gsa_info, common_nmea_callback);
    register_CXM150x_NMEARMC_event(&g_rmc_info, common_nmea_callback);
    register_CXM150x_NMEAVTG_event(&g_vtg_info, common_nmea_callback);
}

//////////////////////// --------------- unit test code from here ----------------
#ifdef CALC_DISTANCE_UNIT_TEST_PORT
typedef struct {
    char *test_str;
    int32_t get_int_len_exp;
    double extract_degree_exp_min;
    double extract_degree_exp_max;
} PortTestData1Type;

typedef struct {
    char *m_lat;
    char *m_n_s;
    char *m_lon;
    char *m_e_w;
    float m_speed_kph;
    char *m_mode2;
    double m_lat_exp_min;
    double m_lat_exp_max;
    double m_lon_exp_min;
    double m_lon_exp_max;
    float m_speed_kph_exp;
    uint8_t m_mode2_exp;
} PortTestData2Type;

static const float speed_kph_exp_accept_delta = 0.01f;

static const PortTestData1Type test_data_1[] = {
    {"17959.9999",5,179.9999,180.0000},
    {"9959.9999",4,99.9999,100.0000},
    {"959.9999",3,9.9999,10.0000},
    {"59.9999",2,0.9999,1.0000},
    {"09.9999",2,0.1666,0.1667},
    {"00.0001",2,0.0000,0.0001},
    {".",0,-1.0,-1.0},
    {"123456.0000",-1,-1.0,-1.0},
    {"12345",-1,-1.0,-1.0},
    {"1234S",-1,-1.0,-1.0},
};

static const PortTestData2Type test_data_2[] = {
    {
        "8959.9999","N","17959.9999","E",1.2,"3",
        89.9999,90.0000,179.9999,180.0000,1.2f,3
    },
    {
        "00.0001","N","00.0001","W",2.3,"2",
        0.0000,0.0001,-0.0001,-0.0000,2.3f,2
    },
    {
        "00.0001","S","00.0001","E",3.4,"1",
        -0.0001,0.0000,0.0000,0.0001,3.4f,1
    },
    {
        "8959.9999","S","17959.9999","W",4.5,"3",
        -90.0000,-89.9999,-180.0000,-179.9999,4.5f,3
    },
};

static int test_count = 0;
static int pass_count = 0;
static int fail_count = 0;

static CalcDistanceResult test_cb(CalcDistanceSource *source){
    PortTestData2Type data = test_data_2[test_count];
    printf("%d,",test_count);
    bool lon_pass = (data.m_lon_exp_min <= source->lon_deg && source->lon_deg <= data.m_lon_exp_max);
    printf("%f, %f, %f, %s, ", data.m_lon_exp_min, source->lon_deg, data.m_lon_exp_max, lon_pass?"PASS":"FAIL");
    bool lat_pass = (data.m_lat_exp_min <= source->lat_deg && source->lat_deg <= data.m_lat_exp_max);
    printf("%f, %f, %f, %s, ", data.m_lat_exp_min, source->lat_deg, data.m_lat_exp_max, lat_pass?"PASS":"FAIL");
    bool spd_pass = ((data.m_speed_kph_exp - speed_kph_exp_accept_delta) <= source->speed_kph && source->speed_kph <= (data.m_speed_kph_exp + speed_kph_exp_accept_delta));
    printf("%f, %f, %f, %s, ", (data.m_speed_kph_exp - speed_kph_exp_accept_delta), source->speed_kph, (data.m_speed_kph_exp + speed_kph_exp_accept_delta), spd_pass?"PASS":"FAIL");
    bool mode2_pass = (data.m_mode2_exp == source->fix_type);
    printf("%d, %ld, %s\r\n", data.m_mode2_exp, source->fix_type, mode2_pass?"PASS":"FAIL");
    if (lon_pass) {
        pass_count++;
    } else {
        fail_count++;
    }
    if (lat_pass) {
        pass_count++;
    } else {
        fail_count++;
    }
    if (spd_pass) {
        pass_count++;
    } else {
        fail_count++;
    }
    if (mode2_pass) {
        pass_count++;
    } else {
        fail_count++;
    }
    return CALC_DISTANCE_NO_ERROR;
}
#endif

int calc_distance_port_test_main(void) {
    printf("calc_distance_port_test_main()\r\n");
#ifdef CALC_DISTANCE_UNIT_TEST_PORT
    for(int count = 0; count < sizeof(test_data_1)/sizeof(test_data_1[0]); count++) {
        PortTestData1Type data = test_data_1[count];
        int32_t int_len = get_int_len((uint8_t *)data.test_str);
        double degree = extract_degree((uint8_t *)data.test_str);
        bool int_len_passed = (int_len == data.get_int_len_exp);
        bool degree_passed = (data.extract_degree_exp_min <= degree && degree <= data.extract_degree_exp_max);
        printf("%d, \"%s\", ", count, data.test_str);
        printf("%ld, %ld, %s, ", int_len, data.get_int_len_exp, int_len_passed?"PASS":"FAIL");
        printf("%f, %f, %f, %s", data.extract_degree_exp_min, degree, data.extract_degree_exp_max, degree_passed?"PASS":"FAIL");
        printf("\n");
        if (int_len_passed) {
            pass_count++;
        } else {
            fail_count++;
        }
        if (degree_passed) {
            pass_count++;
        } else {
            fail_count++;
        }
    }
    g_update_distance_func = test_cb;
    for(test_count = 0; test_count < sizeof(test_data_2)/sizeof(test_data_2[0]); test_count++) {
        PortTestData2Type data = test_data_2[test_count];
        strncpy((char *)g_gsa_info.m_mode2, data.m_mode2, NMEA_MODE_SIZE+1);
        g_gsa_info.m_cs_correct = true;

        strncpy((char *)g_rmc_info.m_lat, data.m_lat, NMEA_LAT_SIZE + 1);
        strncpy((char *)g_rmc_info.m_n_s, data.m_n_s, NMEA_N_S_SIZE + 1);
        strncpy((char *)g_rmc_info.m_lon, data.m_lon, NMEA_LON_SIZE + 1);
        strncpy((char *)g_rmc_info.m_e_w, data.m_e_w, NMEA_E_W_SIZE + 1);
        g_rmc_info.m_cs_correct = true;

        g_vtg_info.m_speed_kph = data.m_speed_kph;
        g_vtg_info.m_cs_correct = true;

        common_nmea_callback(&g_gsa_info, CXM150x_EVENT_CALLBACK_ID_NMEAGSA_EVENT);
        common_nmea_callback(&g_rmc_info, CXM150x_EVENT_CALLBACK_ID_NMEARMC_EVENT);
        common_nmea_callback(&g_vtg_info, CXM150x_EVENT_CALLBACK_ID_NMEAVTG_EVENT);
    }
    printf("PASS:%d, FAIL:%d\n", pass_count, fail_count);
    return fail_count;
#else
    return 0;
#endif
}
