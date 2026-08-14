/* BMP388/BMI088 complementary relative-altitude estimator. */

#include <math.h>
#include <stddef.h>
#include "est_altitude.h"
#include "fc_config.h"

#define DEG_TO_RAD                    (0.01745329251994329577f)
#define PRESSURE_ALTITUDE_EXPONENT    (0.19029495718363465f)
#define PRESSURE_ALTITUDE_SCALE_M     (44330.0f)
#define TWO_PI                        (6.28318530717958648f)

typedef struct
{
    float reference_pressure_sum_pa;
    float reference_pressure_pa;
    float filtered_pressure_pa;
    float altitude_m;
    float vertical_velocity_mps;
    float last_barometer_altitude_m;
    float filtered_barometer_velocity_mps;
    uint32_t reference_sample_count;
    uint32_t last_valid_barometer_ms;
    uint32_t last_update_ms;
    bool initialized;
    bool reference_ready;
    bool state_ready;
    bool barometer_velocity_ready;
} AltitudeEstimatorState_t;

static AltitudeEstimatorState_t s_state;
volatile EstAltitudeDebug_t g_est_altitude_debug;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) { return minimum; }
    if (value > maximum) { return maximum; }
    return value;
}

static float low_pass_alpha(float cutoff_hz, float dt_s)
{
    float rc_s = 1.0f / (TWO_PI * cutoff_hz);
    return dt_s / (rc_s + dt_s);
}

static float pressure_to_relative_altitude(float pressure_pa)
{
    float ratio = pressure_pa / s_state.reference_pressure_pa;
    return PRESSURE_ALTITUDE_SCALE_M * (1.0f - powf(ratio, PRESSURE_ALTITUDE_EXPONENT));
}

static bool vertical_acceleration(const FcImuData_t *imu,
                                  const FcAttitude_t *attitude,
                                  float *acceleration_mps2)
{
    float roll_rad;
    float pitch_rad;
    float sin_roll;
    float cos_roll;
    float sin_pitch;
    float cos_pitch;
    float accel_norm_sq;
    float specific_force_down_g;

    if ((imu == NULL) || (attitude == NULL) || (acceleration_mps2 == NULL) ||
        !imu->valid || !imu->calibrated || !attitude->valid)
    {
        return false;
    }
    accel_norm_sq = (imu->accel_g.x * imu->accel_g.x) +
                    (imu->accel_g.y * imu->accel_g.y) +
                    (imu->accel_g.z * imu->accel_g.z);
    if ((accel_norm_sq < FC_VERTICAL_ACCEL_MIN_NORM_SQ) ||
        (accel_norm_sq > FC_VERTICAL_ACCEL_MAX_NORM_SQ))
    {
        return false;
    }

    roll_rad = attitude->roll_deg * DEG_TO_RAD;
    pitch_rad = attitude->pitch_deg * DEG_TO_RAD;
    sin_roll = sinf(roll_rad);
    cos_roll = cosf(roll_rad);
    sin_pitch = sinf(pitch_rad);
    cos_pitch = cosf(pitch_rad);

    /* Body is X-forward/Y-right/Z-down. Stationary specific force is -1 g down. */
    specific_force_down_g = (-sin_pitch * imu->accel_g.x) +
                            (sin_roll * cos_pitch * imu->accel_g.y) +
                            (cos_roll * cos_pitch * imu->accel_g.z);
    *acceleration_mps2 = clamp_float(-(specific_force_down_g + 1.0f) * FC_GRAVITY_MPS2,
                                     -FC_VERTICAL_ACCEL_LIMIT_MPS2,
                                     FC_VERTICAL_ACCEL_LIMIT_MPS2);
    return true;
}

FcStatus_t Est_AltitudeInit(void)
{
    s_state = (AltitudeEstimatorState_t){0};
    s_state.initialized = true;
    g_est_altitude_debug = (EstAltitudeDebug_t){0};
    return FC_STATUS_OK;
}

FcStatus_t Est_AltitudeUpdate(const FcBarometerData_t *barometer,
                              const FcImuData_t *imu,
                              const FcAttitude_t *attitude,
                              float dt_s,
                              FcAltitude_t *altitude)
{
    uint32_t timestamp_ms;
    bool barometer_valid;
    bool acceleration_valid;
    float acceleration_mps2 = 0.0f;
    float barometer_altitude_m = 0.0f;
    float pressure_alpha;
    float predicted_altitude_m;
    float predicted_velocity_mps;
    float innovation_m = 0.0f;
    float raw_barometer_velocity_mps;
    float velocity_alpha;

    if ((barometer == NULL) || (altitude == NULL) ||
        (dt_s <= 0.0f) || (dt_s > FC_VERTICAL_MAX_PREDICT_DT_S))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *altitude = (FcAltitude_t){0};
    timestamp_ms = barometer->timestamp_ms;
    altitude->timestamp_ms = timestamp_ms;
    if (!s_state.initialized) { return FC_STATUS_NOT_INITIALIZED; }

    barometer_valid = barometer->valid &&
                      (barometer->pressure_pa >= 30000.0f) &&
                      (barometer->pressure_pa <= 125000.0f);
    acceleration_valid = vertical_acceleration(imu, attitude, &acceleration_mps2);

    if (!s_state.reference_ready)
    {
        if (!barometer_valid) { return FC_STATUS_INVALID_DATA; }
        s_state.reference_pressure_sum_pa += barometer->pressure_pa;
        ++s_state.reference_sample_count;
        g_est_altitude_debug.reference_sample_count = s_state.reference_sample_count;
        if (s_state.reference_sample_count < FC_BARO_REFERENCE_SAMPLE_COUNT)
        {
            return FC_STATUS_BUSY;
        }
        s_state.reference_pressure_pa = s_state.reference_pressure_sum_pa /
                                        (float)s_state.reference_sample_count;
        s_state.filtered_pressure_pa = s_state.reference_pressure_pa;
        s_state.altitude_m = 0.0f;
        s_state.vertical_velocity_mps = 0.0f;
        s_state.last_valid_barometer_ms = timestamp_ms;
        s_state.last_update_ms = timestamp_ms;
        s_state.reference_ready = true;
        s_state.state_ready = true;
        s_state.barometer_velocity_ready = false;
    }

    predicted_velocity_mps = s_state.vertical_velocity_mps;
    predicted_altitude_m = s_state.altitude_m;
    if (acceleration_valid)
    {
        predicted_altitude_m += (predicted_velocity_mps * dt_s) +
                                (0.5f * acceleration_mps2 * dt_s * dt_s);
        predicted_velocity_mps += acceleration_mps2 * dt_s;
    }

    if (barometer_valid)
    {
        pressure_alpha = low_pass_alpha(FC_BARO_PRESSURE_LPF_HZ, dt_s);
        s_state.filtered_pressure_pa += pressure_alpha *
                                       (barometer->pressure_pa - s_state.filtered_pressure_pa);
        barometer_altitude_m = pressure_to_relative_altitude(s_state.filtered_pressure_pa);
        innovation_m = barometer_altitude_m - predicted_altitude_m;
        if ((fabsf(barometer_altitude_m - s_state.altitude_m) <= FC_BARO_MAX_SAMPLE_STEP_M) &&
            (fabsf(innovation_m) <= FC_BARO_INNOVATION_LIMIT_M))
        {
            if (!s_state.barometer_velocity_ready)
            {
                s_state.last_barometer_altitude_m = barometer_altitude_m;
                s_state.filtered_barometer_velocity_mps = 0.0f;
                s_state.barometer_velocity_ready = true;
            }
            raw_barometer_velocity_mps = (barometer_altitude_m -
                                           s_state.last_barometer_altitude_m) / dt_s;
            velocity_alpha = low_pass_alpha(FC_BARO_VELOCITY_LPF_HZ, dt_s);
            s_state.filtered_barometer_velocity_mps += velocity_alpha *
                (raw_barometer_velocity_mps - s_state.filtered_barometer_velocity_mps);
            predicted_altitude_m += FC_VERTICAL_BARO_POSITION_BLEND * innovation_m;
            predicted_velocity_mps += FC_VERTICAL_BARO_VELOCITY_BLEND *
                (s_state.filtered_barometer_velocity_mps - predicted_velocity_mps);
            s_state.last_barometer_altitude_m = barometer_altitude_m;
            s_state.last_valid_barometer_ms = timestamp_ms;
        }
        else
        {
            barometer_valid = false;
            ++g_est_altitude_debug.rejected_sample_count;
        }
    }

    s_state.altitude_m = predicted_altitude_m;
    s_state.vertical_velocity_mps = predicted_velocity_mps;
    s_state.last_update_ms = timestamp_ms;
    altitude->altitude_m = s_state.altitude_m;
    altitude->vertical_velocity_mps = s_state.vertical_velocity_mps;
    altitude->valid = barometer_valid ||
        ((timestamp_ms - s_state.last_valid_barometer_ms) <= FC_BARO_INERTIAL_HOLD_TIMEOUT_MS);

    g_est_altitude_debug.reference_pressure_pa = s_state.reference_pressure_pa;
    g_est_altitude_debug.filtered_pressure_pa = s_state.filtered_pressure_pa;
    g_est_altitude_debug.raw_altitude_m = barometer_altitude_m;
    g_est_altitude_debug.filtered_altitude_m = s_state.altitude_m;
    g_est_altitude_debug.vertical_velocity_mps = s_state.vertical_velocity_mps;
    g_est_altitude_debug.vertical_acceleration_mps2 = acceleration_mps2;
    g_est_altitude_debug.barometer_innovation_m = innovation_m;
    g_est_altitude_debug.last_valid_barometer_ms = s_state.last_valid_barometer_ms;
    g_est_altitude_debug.reference_ready = true;
    g_est_altitude_debug.inertial_aiding_active = acceleration_valid;
    g_est_altitude_debug.barometer_recent = altitude->valid;
    return altitude->valid ? FC_STATUS_OK : FC_STATUS_TIMEOUT;
}

void Est_AltitudeReset(void)
{
    (void)Est_AltitudeInit();
}
