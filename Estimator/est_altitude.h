#ifndef EST_ALTITUDE_H
#define EST_ALTITUDE_H

/* BMP388与BMI088融合的相对高度、垂直速度估计器。 */

#include "fc_types.h"

typedef enum
{
    EST_ALT_REJECT_NONE = 0,
    EST_ALT_REJECT_ACCEL_NORM,
    EST_ALT_REJECT_ACCEL_RANGE,
    EST_ALT_REJECT_BARO_STEP,
    EST_ALT_REJECT_BARO_INNOVATION,
    EST_ALT_REJECT_ALTITUDE_RANGE,
    EST_ALT_REJECT_VELOCITY_RANGE
} EstAltitudeRejectReason_t;

typedef struct
{
    float reference_pressure_pa;
    float filtered_pressure_pa;
    float raw_altitude_m;
    float filtered_altitude_m;
    float vertical_velocity_mps;
    float raw_vertical_acceleration_mps2;
    float vertical_acceleration_mps2;
    float acceleration_bias_mps2;
    float barometer_innovation_m;
    float predicted_barometer_altitude_m;
    float kalman_position_variance;
    float kalman_velocity_variance;
    float kalman_bias_variance;
    uint32_t last_valid_barometer_ms;
    uint32_t reference_sample_count;
    uint32_t rejected_sample_count;
    uint32_t prediction_count;
    uint32_t correction_count;
    uint32_t acceleration_reject_count;
    uint32_t barometer_step_reject_count;
    uint32_t barometer_innovation_reject_count;
    uint32_t consecutive_barometer_reject_count;
    uint32_t altitude_range_fault_count;
    uint32_t velocity_range_fault_count;
    EstAltitudeRejectReason_t last_reject_reason;
    bool reference_ready;
    bool inertial_aiding_active;
    bool barometer_recent;
    bool kalman_enabled;
    bool ground_reference_tracking_active;
    bool estimate_within_limits;
} EstAltitudeDebug_t;

extern volatile EstAltitudeDebug_t g_est_altitude_debug;

FcStatus_t Est_AltitudeInit(void);
FcStatus_t Est_AltitudePredict(const FcImuData_t *imu,
                               const FcAttitude_t *attitude,
                               float dt_s,
                               uint32_t timestamp_ms,
                               FcAltitude_t *altitude);
FcStatus_t Est_AltitudeUpdate(const FcBarometerData_t *barometer,
                              const FcImuData_t *imu,
                              const FcAttitude_t *attitude,
                              float dt_s,
                              bool aircraft_grounded,
                              FcAltitude_t *altitude);
void Est_AltitudeReset(void);

#endif /* EST_ALTITUDE_H */
