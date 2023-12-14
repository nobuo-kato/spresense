#include "stdint.h"

#define INVALID_LON (360.0)
#define INVALID_LAT (180.0)
#define INVALID_SPEED (-1.0)
#define INVALID_FIX_TYPE (0)

typedef struct {
    double lon_deg; // -180.0 <= lon_deg <= 180.0, INVALID_LON indicates error
    double lat_deg; // -90.0 <= lat_deg <= 90.0, INVALID_LAT indicates error
    float speed_kph; // 0 <= speed_kph, INVALID_SPEED indicates error
    uint32_t fix_type; // 1：Not fixed, 2：2D-Fixed, 3：3D-Fixed, 0:Invalid
} CalcDistanceSource;

typedef enum {
    CALC_DISTANCE_NO_ERROR,
    CALC_DISTANCE_ERROR_NOT_3D_FIXED,
    CALC_DISTANCE_ERROR_LOW_SPEED,
    CALC_DISTANCE_ERROR_INVALID_CURRENT_LON_OR_LAT,
    CALC_DISTANCE_ERROR_INVALID_PERV_LON_OR_LAT,
    CALC_DISTANCE_ERROR_UNDEFINED,
} CalcDistanceResult;

typedef CalcDistanceResult (* UPDATE_DISTANCE_CALLBACK_FUNC_POINTER)(CalcDistanceSource *);
//CalcDistanceResult update_distance(CalcDistanceSource * source);

void init_calc_distance_port(UPDATE_DISTANCE_CALLBACK_FUNC_POINTER, void *);
int calc_distance_port_test_main(void);
