#ifndef EST_ALTITUDE_H
#define EST_ALTITUDE_H

/* Relative barometric altitude and filtered vertical-speed estimator. */

#include "fc_types.h"

typedef struct
{
    float reference_pressure_pa;
    float filtered_pressure_pa;
    float raw_altitude_m;
    float filtered_altitude_m;
    float vertical_velocity_mps;
    float vertical_acceleration_mps2;
    float barometer_innovation_m;
    uint32_t last_valid_barometer_ms;
    uint32_t reference_sample_count;
    uint32_t rejected_sample_count;
    bool reference_ready;
    bool inertial_aiding_active;
    bool barometer_recent;
} EstAltitudeDebug_t;

extern volatile EstAltitudeDebug_t g_est_altitude_debug;

FcStatus_t Est_AltitudeInit(void);
FcStatus_t Est_AltitudeUpdate(const FcBarometerData_t *barometer,
                              const FcImuData_t *imu,
                              const FcAttitude_t *attitude,
                              float dt_s,
                              FcAltitude_t *altitude);
void Est_AltitudeReset(void);

#endif /* EST_ALTITUDE_H */
