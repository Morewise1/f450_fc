/* BMP388/BMI088相对高度估计：可切换互补滤波或三状态卡尔曼。 */

#include <math.h>
#include <stddef.h>
#include "est_altitude.h"
#include "fc_config.h"

#define DEG_TO_RAD                    (0.01745329251994329577f)
#define PRESSURE_ALTITUDE_EXPONENT    (0.19029495718363465f)
#define PRESSURE_ALTITUDE_SCALE_M     (44330.0f)
#define TWO_PI                        (6.28318530717958648f)
#define KF_STATE_COUNT                3U
#define KF_HEIGHT_INDEX               0U
#define KF_VELOCITY_INDEX             1U
#define KF_ACCEL_BIAS_INDEX           2U

typedef struct
{
    float reference_pressure_sum_pa;
    float reference_pressure_pa;
    float reference_acceleration_sum_mps2;
    float filtered_pressure_pa;
    float filtered_vertical_acceleration_mps2;
    float altitude_m;
    float vertical_velocity_mps;
    float last_barometer_altitude_m;
    float filtered_barometer_velocity_mps;
    float kalman_state[KF_STATE_COUNT];
    float kalman_covariance[KF_STATE_COUNT][KF_STATE_COUNT];
    uint32_t reference_sample_count;
    uint32_t reference_acceleration_count;
    uint32_t last_valid_barometer_ms;
    uint32_t last_update_ms;
    bool initialized;
    bool reference_ready;
    bool state_ready;
    bool barometer_velocity_ready;
    bool vertical_acceleration_filter_ready;
    bool altitude_range_fault_active;
    bool velocity_range_fault_active;
    bool aircraft_grounded_last_update;
} AltitudeEstimatorState_t;

static AltitudeEstimatorState_t s_state;
volatile EstAltitudeDebug_t g_est_altitude_debug;

#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_KALMAN
static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) { return minimum; }
    if (value > maximum) { return maximum; }
    return value;
}

static float square_float(float value)
{
    return value * value;
}
#endif

static float low_pass_alpha(float cutoff_hz, float dt_s)
{
    float rc_s = 1.0f / (TWO_PI * cutoff_hz);
    return dt_s / (rc_s + dt_s);
}

#if FC_ALT_ENABLE_GROUND_REFERENCE_TRACKING
static float time_constant_alpha(float time_constant_s, float dt_s)
{
    if ((time_constant_s <= 0.0f) || (dt_s <= 0.0f)) { return 1.0f; }
    return dt_s / (time_constant_s + dt_s);
}
#endif

static float pressure_to_relative_altitude(float pressure_pa)
{
    float ratio = pressure_pa / s_state.reference_pressure_pa;
    return PRESSURE_ALTITUDE_SCALE_M *
           (1.0f - powf(ratio, PRESSURE_ALTITUDE_EXPONENT));
}

static bool calculate_vertical_acceleration(const FcImuData_t *imu,
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
        ++g_est_altitude_debug.acceleration_reject_count;
        g_est_altitude_debug.last_reject_reason = EST_ALT_REJECT_ACCEL_NORM;
        return false;
    }

    roll_rad = attitude->roll_deg * DEG_TO_RAD;
    pitch_rad = attitude->pitch_deg * DEG_TO_RAD;
    sin_roll = sinf(roll_rad);
    cos_roll = cosf(roll_rad);
    sin_pitch = sinf(pitch_rad);
    cos_pitch = cosf(pitch_rad);

    /* Body is X-forward/Y-right/Z-down. Stationary specific force is -1 g. */
    specific_force_down_g = (-sin_pitch * imu->accel_g.x) +
                            (sin_roll * cos_pitch * imu->accel_g.y) +
                            (cos_roll * cos_pitch * imu->accel_g.z);
    *acceleration_mps2 = -(specific_force_down_g + 1.0f) * FC_GRAVITY_MPS2;
    g_est_altitude_debug.raw_vertical_acceleration_mps2 = *acceleration_mps2;
    if ((*acceleration_mps2 != *acceleration_mps2) ||
        (fabsf(*acceleration_mps2) > FC_ALT_MAX_TRUSTED_ACCEL_MPS2))
    {
        ++g_est_altitude_debug.acceleration_reject_count;
        g_est_altitude_debug.last_reject_reason = EST_ALT_REJECT_ACCEL_RANGE;
        return false;
    }
    return true;
}

static float filter_vertical_acceleration(float acceleration_mps2, float dt_s)
{
#if FC_ALT_ENABLE_ACCEL_LPF
    float alpha;

    if (!s_state.vertical_acceleration_filter_ready)
    {
        s_state.filtered_vertical_acceleration_mps2 = acceleration_mps2;
        s_state.vertical_acceleration_filter_ready = true;
        return acceleration_mps2;
    }
    alpha = low_pass_alpha(FC_ALT_ACCEL_LPF_HZ, dt_s);
    s_state.filtered_vertical_acceleration_mps2 += alpha *
        (acceleration_mps2 - s_state.filtered_vertical_acceleration_mps2);
    return s_state.filtered_vertical_acceleration_mps2;
#else
    (void)dt_s;
    s_state.filtered_vertical_acceleration_mps2 = acceleration_mps2;
    s_state.vertical_acceleration_filter_ready = true;
    return acceleration_mps2;
#endif
}

static void update_ground_reference(float pressure_pa, float dt_s)
{
#if FC_ALT_ENABLE_GROUND_REFERENCE_TRACKING
    float alpha = time_constant_alpha(FC_ALT_GROUND_REFERENCE_TIME_CONSTANT_S,
                                      dt_s);
    s_state.reference_pressure_pa += alpha *
        (pressure_pa - s_state.reference_pressure_pa);
#else
    (void)pressure_pa;
    (void)dt_s;
#endif
}

static bool barometer_is_recent(uint32_t timestamp_ms)
{
    return s_state.reference_ready &&
           ((timestamp_ms - s_state.last_valid_barometer_ms) <=
            FC_BARO_INERTIAL_HOLD_TIMEOUT_MS);
}

static void synchronize_output_state(void)
{
#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_KALMAN
    s_state.altitude_m = s_state.kalman_state[KF_HEIGHT_INDEX];
    s_state.vertical_velocity_mps = s_state.kalman_state[KF_VELOCITY_INDEX];
#endif
}

#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_COMPLEMENTARY
static void apply_ground_constraint(void)
{
    s_state.altitude_m = 0.0f;
    s_state.vertical_velocity_mps = 0.0f;
}
#endif

static void write_output(uint32_t timestamp_ms, FcAltitude_t *altitude)
{
    bool recent = barometer_is_recent(timestamp_ms);
    bool altitude_safe;
    bool velocity_safe;

    synchronize_output_state();
    altitude_safe = (s_state.altitude_m == s_state.altitude_m) &&
                    (s_state.altitude_m >= FC_ALT_ESTIMATE_MIN_M) &&
                    (s_state.altitude_m <= FC_ALT_ESTIMATE_MAX_M);
    velocity_safe = (s_state.vertical_velocity_mps ==
                     s_state.vertical_velocity_mps) &&
                    (fabsf(s_state.vertical_velocity_mps) <=
                     FC_ALT_MAX_ESTIMATED_VELOCITY_MPS);
    if (!altitude_safe && !s_state.altitude_range_fault_active)
    {
        ++g_est_altitude_debug.altitude_range_fault_count;
        g_est_altitude_debug.last_reject_reason =
            EST_ALT_REJECT_ALTITUDE_RANGE;
    }
    if (!velocity_safe && !s_state.velocity_range_fault_active)
    {
        ++g_est_altitude_debug.velocity_range_fault_count;
        g_est_altitude_debug.last_reject_reason =
            EST_ALT_REJECT_VELOCITY_RANGE;
    }
    s_state.altitude_range_fault_active = !altitude_safe;
    s_state.velocity_range_fault_active = !velocity_safe;

    altitude->altitude_m = s_state.altitude_m;
    altitude->vertical_velocity_mps = s_state.vertical_velocity_mps;
    altitude->timestamp_ms = timestamp_ms;
    altitude->valid = s_state.state_ready && recent &&
                      altitude_safe && velocity_safe;

    g_est_altitude_debug.reference_pressure_pa = s_state.reference_pressure_pa;
    g_est_altitude_debug.filtered_pressure_pa = s_state.filtered_pressure_pa;
    g_est_altitude_debug.filtered_altitude_m = s_state.altitude_m;
    g_est_altitude_debug.vertical_velocity_mps = s_state.vertical_velocity_mps;
    g_est_altitude_debug.last_valid_barometer_ms =
        s_state.last_valid_barometer_ms;
    g_est_altitude_debug.reference_ready = s_state.reference_ready;
    g_est_altitude_debug.barometer_recent = recent;
    g_est_altitude_debug.estimate_within_limits = altitude_safe && velocity_safe;
#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_KALMAN
    g_est_altitude_debug.acceleration_bias_mps2 =
        s_state.kalman_state[KF_ACCEL_BIAS_INDEX];
    g_est_altitude_debug.kalman_position_variance =
        s_state.kalman_covariance[KF_HEIGHT_INDEX][KF_HEIGHT_INDEX];
    g_est_altitude_debug.kalman_velocity_variance =
        s_state.kalman_covariance[KF_VELOCITY_INDEX][KF_VELOCITY_INDEX];
    g_est_altitude_debug.kalman_bias_variance =
        s_state.kalman_covariance[KF_ACCEL_BIAS_INDEX][KF_ACCEL_BIAS_INDEX];
#endif
}

#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_KALMAN
static void initialize_kalman(float initial_acceleration_bias_mps2)
{
    uint8_t row;
    uint8_t column;

    for (row = 0U; row < KF_STATE_COUNT; ++row)
    {
        s_state.kalman_state[row] = 0.0f;
        for (column = 0U; column < KF_STATE_COUNT; ++column)
        {
            s_state.kalman_covariance[row][column] = 0.0f;
        }
    }
    s_state.kalman_state[KF_ACCEL_BIAS_INDEX] =
        clamp_float(initial_acceleration_bias_mps2,
                    -FC_ALT_KF_MAX_ACCEL_BIAS_MPS2,
                    FC_ALT_KF_MAX_ACCEL_BIAS_MPS2);
    s_state.kalman_covariance[KF_HEIGHT_INDEX][KF_HEIGHT_INDEX] =
        square_float(FC_ALT_KF_INITIAL_HEIGHT_STD_M);
    s_state.kalman_covariance[KF_VELOCITY_INDEX][KF_VELOCITY_INDEX] =
        square_float(FC_ALT_KF_INITIAL_VELOCITY_STD_MPS);
    s_state.kalman_covariance[KF_ACCEL_BIAS_INDEX][KF_ACCEL_BIAS_INDEX] =
        square_float(FC_ALT_KF_INITIAL_BIAS_STD_MPS2);
}

static void kalman_predict(float acceleration_mps2,
                           bool acceleration_valid,
                           float dt_s)
{
    float transition[KF_STATE_COUNT][KF_STATE_COUNT] =
    {
        {1.0f, dt_s, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    float intermediate[KF_STATE_COUNT][KF_STATE_COUNT] = {{0.0f}};
    float predicted_covariance[KF_STATE_COUNT][KF_STATE_COUNT] = {{0.0f}};
    float dt_sq = dt_s * dt_s;
    float acceleration_noise_variance = square_float(FC_ALT_KF_ACCEL_STD_MPS2);
    float bias_noise_variance =
        square_float(FC_ALT_KF_ACCEL_BIAS_RW_MPS2_SQRT_S);
    float acceleration_gain_height = 0.5f * dt_sq;
    float acceleration_gain_velocity = dt_s;
    float effective_acceleration_mps2 = 0.0f;
    uint8_t row;
    uint8_t column;
    uint8_t index;

    if (acceleration_valid)
    {
        effective_acceleration_mps2 = acceleration_mps2 -
            s_state.kalman_state[KF_ACCEL_BIAS_INDEX];
        transition[KF_HEIGHT_INDEX][KF_ACCEL_BIAS_INDEX] = -0.5f * dt_sq;
        transition[KF_VELOCITY_INDEX][KF_ACCEL_BIAS_INDEX] = -dt_s;
    }
    else
    {
        /* No valid accelerometer projection: use constant-velocity prediction. */
        acceleration_noise_variance *= 4.0f;
    }

    s_state.kalman_state[KF_HEIGHT_INDEX] +=
        (s_state.kalman_state[KF_VELOCITY_INDEX] * dt_s) +
        (0.5f * effective_acceleration_mps2 * dt_sq);
    s_state.kalman_state[KF_VELOCITY_INDEX] +=
        effective_acceleration_mps2 * dt_s;

    for (row = 0U; row < KF_STATE_COUNT; ++row)
    {
        for (column = 0U; column < KF_STATE_COUNT; ++column)
        {
            for (index = 0U; index < KF_STATE_COUNT; ++index)
            {
                intermediate[row][column] +=
                    transition[row][index] *
                    s_state.kalman_covariance[index][column];
            }
        }
    }
    for (row = 0U; row < KF_STATE_COUNT; ++row)
    {
        for (column = 0U; column < KF_STATE_COUNT; ++column)
        {
            for (index = 0U; index < KF_STATE_COUNT; ++index)
            {
                predicted_covariance[row][column] +=
                    intermediate[row][index] * transition[column][index];
            }
        }
    }

    predicted_covariance[KF_HEIGHT_INDEX][KF_HEIGHT_INDEX] +=
        acceleration_noise_variance * acceleration_gain_height *
        acceleration_gain_height;
    predicted_covariance[KF_HEIGHT_INDEX][KF_VELOCITY_INDEX] +=
        acceleration_noise_variance * acceleration_gain_height *
        acceleration_gain_velocity;
    predicted_covariance[KF_VELOCITY_INDEX][KF_HEIGHT_INDEX] +=
        acceleration_noise_variance * acceleration_gain_height *
        acceleration_gain_velocity;
    predicted_covariance[KF_VELOCITY_INDEX][KF_VELOCITY_INDEX] +=
        acceleration_noise_variance * acceleration_gain_velocity *
        acceleration_gain_velocity;
    predicted_covariance[KF_ACCEL_BIAS_INDEX][KF_ACCEL_BIAS_INDEX] +=
        bias_noise_variance * dt_s;

    for (row = 0U; row < KF_STATE_COUNT; ++row)
    {
        for (column = 0U; column < KF_STATE_COUNT; ++column)
        {
            s_state.kalman_covariance[row][column] =
                predicted_covariance[row][column];
        }
        s_state.kalman_covariance[row][row] =
            clamp_float(s_state.kalman_covariance[row][row],
                        FC_ALT_KF_MIN_VARIANCE,
                        FC_ALT_KF_MAX_VARIANCE);
    }
}

static bool kalman_correct(float barometer_altitude_m, float *innovation_m)
{
    float prior_covariance[KF_STATE_COUNT][KF_STATE_COUNT];
    float gain[KF_STATE_COUNT];
    float measurement_variance = square_float(FC_ALT_KF_BARO_STD_M);
    float predicted_measurement_m =
        s_state.kalman_state[KF_HEIGHT_INDEX];
    float innovation_variance;
    float innovation_gate_m;
    float covariance_symmetric;
    uint8_t row;
    uint8_t column;

#if FC_ALT_ENABLE_BARO_DELAY_COMPENSATION
    /*
     * 延迟气压高度约为h(t)-delay*v(t)。使用先验速度前推创新，
     * 但保持高度观测矩阵[1,0,0]，避免无GPS模型直接反向修正速度。
     */
    predicted_measurement_m -= FC_ALT_BARO_DELAY_S *
                               s_state.kalman_state[KF_VELOCITY_INDEX];
#endif
    *innovation_m = barometer_altitude_m - predicted_measurement_m;
    g_est_altitude_debug.predicted_barometer_altitude_m =
        predicted_measurement_m;

    innovation_variance =
        s_state.kalman_covariance[KF_HEIGHT_INDEX][KF_HEIGHT_INDEX] +
        measurement_variance;
    if (innovation_variance <= FC_ALT_KF_MIN_VARIANCE)
    {
        return false;
    }
    innovation_gate_m = FC_ALT_KF_INNOVATION_GATE_SIGMA *
                        sqrtf(innovation_variance);
    innovation_gate_m = clamp_float(innovation_gate_m,
                                    FC_ALT_KF_MIN_INNOVATION_GATE_M,
                                    FC_BARO_INNOVATION_LIMIT_M);
    if (fabsf(*innovation_m) > innovation_gate_m)
    {
        return false;
    }

    for (row = 0U; row < KF_STATE_COUNT; ++row)
    {
        gain[row] = s_state.kalman_covariance[row][KF_HEIGHT_INDEX] /
                    innovation_variance;
        for (column = 0U; column < KF_STATE_COUNT; ++column)
        {
            prior_covariance[row][column] =
                s_state.kalman_covariance[row][column];
        }
    }
    for (row = 0U; row < KF_STATE_COUNT; ++row)
    {
        s_state.kalman_state[row] += gain[row] * (*innovation_m);
        for (column = 0U; column < KF_STATE_COUNT; ++column)
        {
            s_state.kalman_covariance[row][column] =
                prior_covariance[row][column] -
                (gain[row] * prior_covariance[KF_HEIGHT_INDEX][column]);
        }
    }

    /* Suppress round-off asymmetry and keep every diagonal strictly positive. */
    for (row = 0U; row < KF_STATE_COUNT; ++row)
    {
        for (column = (uint8_t)(row + 1U);
             column < KF_STATE_COUNT;
             ++column)
        {
            covariance_symmetric = 0.5f *
                (s_state.kalman_covariance[row][column] +
                 s_state.kalman_covariance[column][row]);
            s_state.kalman_covariance[row][column] = covariance_symmetric;
            s_state.kalman_covariance[column][row] = covariance_symmetric;
        }
        s_state.kalman_covariance[row][row] =
            clamp_float(s_state.kalman_covariance[row][row],
                        FC_ALT_KF_MIN_VARIANCE,
                        FC_ALT_KF_MAX_VARIANCE);
    }
    s_state.kalman_state[KF_ACCEL_BIAS_INDEX] =
        clamp_float(s_state.kalman_state[KF_ACCEL_BIAS_INDEX],
                    -FC_ALT_KF_MAX_ACCEL_BIAS_MPS2,
                    FC_ALT_KF_MAX_ACCEL_BIAS_MPS2);
    return true;
}

#else

static bool complementary_correct(float barometer_altitude_m,
                                  float acceleration_mps2,
                                  bool acceleration_valid,
                                  float dt_s,
                                  float *innovation_m)
{
    float predicted_altitude_m = s_state.altitude_m;
    float predicted_velocity_mps = s_state.vertical_velocity_mps;
    float raw_barometer_velocity_mps;
    float velocity_alpha;

    if (acceleration_valid)
    {
        predicted_altitude_m += (predicted_velocity_mps * dt_s) +
                                (0.5f * acceleration_mps2 * dt_s * dt_s);
        predicted_velocity_mps += acceleration_mps2 * dt_s;
    }
    *innovation_m = barometer_altitude_m - predicted_altitude_m;
    if (fabsf(*innovation_m) > FC_BARO_INNOVATION_LIMIT_M)
    {
        s_state.altitude_m = predicted_altitude_m;
        s_state.vertical_velocity_mps = predicted_velocity_mps;
        return false;
    }

    if (!s_state.barometer_velocity_ready)
    {
        s_state.last_barometer_altitude_m = barometer_altitude_m;
        s_state.filtered_barometer_velocity_mps = 0.0f;
        s_state.barometer_velocity_ready = true;
    }
    raw_barometer_velocity_mps =
        (barometer_altitude_m - s_state.last_barometer_altitude_m) / dt_s;
    velocity_alpha = low_pass_alpha(FC_BARO_VELOCITY_LPF_HZ, dt_s);
    s_state.filtered_barometer_velocity_mps += velocity_alpha *
        (raw_barometer_velocity_mps - s_state.filtered_barometer_velocity_mps);
    predicted_altitude_m += FC_VERTICAL_BARO_POSITION_BLEND * (*innovation_m);
    predicted_velocity_mps += FC_VERTICAL_BARO_VELOCITY_BLEND *
        (s_state.filtered_barometer_velocity_mps - predicted_velocity_mps);
    s_state.last_barometer_altitude_m = barometer_altitude_m;
    s_state.altitude_m = predicted_altitude_m;
    s_state.vertical_velocity_mps = predicted_velocity_mps;
    return true;
}

#endif

FcStatus_t Est_AltitudeInit(void)
{
    s_state = (AltitudeEstimatorState_t){0};
    s_state.initialized = true;
    g_est_altitude_debug = (EstAltitudeDebug_t){0};
#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_KALMAN
    g_est_altitude_debug.kalman_enabled = true;
#endif
    return FC_STATUS_OK;
}

FcStatus_t Est_AltitudePredict(const FcImuData_t *imu,
                               const FcAttitude_t *attitude,
                               float dt_s,
                               uint32_t timestamp_ms,
                               FcAltitude_t *altitude)
{
    float acceleration_mps2 = 0.0f;
#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_KALMAN
    float raw_acceleration_mps2 = 0.0f;
    bool acceleration_valid;
#endif

    if ((altitude == NULL) || (dt_s <= 0.0f) ||
        (dt_s > FC_VERTICAL_MAX_PREDICT_DT_S))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *altitude = (FcAltitude_t){0};
    altitude->timestamp_ms = timestamp_ms;
    if (!s_state.initialized) { return FC_STATUS_NOT_INITIALIZED; }
    if (!s_state.reference_ready) { return FC_STATUS_BUSY; }

#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_KALMAN
    acceleration_valid = calculate_vertical_acceleration(
        imu, attitude, &raw_acceleration_mps2);
    if (acceleration_valid)
    {
        acceleration_mps2 = filter_vertical_acceleration(
            raw_acceleration_mps2, dt_s);
    }
    g_est_altitude_debug.vertical_acceleration_mps2 = acceleration_mps2;
    g_est_altitude_debug.inertial_aiding_active = acceleration_valid;
    kalman_predict(acceleration_mps2, acceleration_valid, dt_s);
    ++g_est_altitude_debug.prediction_count;
#else
    (void)imu;
    (void)attitude;
    (void)acceleration_mps2;
#endif
    s_state.last_update_ms = timestamp_ms;
    write_output(timestamp_ms, altitude);
    return altitude->valid ? FC_STATUS_OK : FC_STATUS_TIMEOUT;
}

FcStatus_t Est_AltitudeUpdate(const FcBarometerData_t *barometer,
                              const FcImuData_t *imu,
                              const FcAttitude_t *attitude,
                              float dt_s,
                              bool aircraft_grounded,
                              FcAltitude_t *altitude)
{
    uint32_t timestamp_ms;
    bool barometer_valid;
    bool acceleration_valid;
    bool correction_accepted = false;
    bool correction_applied = false;
    bool takeoff_reference_freeze = false;
    float acceleration_mps2 = 0.0f;
    float raw_acceleration_mps2 = 0.0f;
    float barometer_altitude_m = 0.0f;
    float pressure_alpha;
    float innovation_m = 0.0f;
    float initial_acceleration_bias_mps2 = 0.0f;

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
    acceleration_valid = false;
    if (!s_state.reference_ready)
    {
        acceleration_valid = calculate_vertical_acceleration(
            imu, attitude, &raw_acceleration_mps2);
        acceleration_mps2 = raw_acceleration_mps2;
        g_est_altitude_debug.vertical_acceleration_mps2 = acceleration_mps2;
        g_est_altitude_debug.inertial_aiding_active = acceleration_valid;
    }
#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_COMPLEMENTARY
    else
    {
        acceleration_valid = calculate_vertical_acceleration(
            imu, attitude, &raw_acceleration_mps2);
        if (acceleration_valid)
        {
            acceleration_mps2 = filter_vertical_acceleration(
                raw_acceleration_mps2, dt_s);
        }
        g_est_altitude_debug.vertical_acceleration_mps2 = acceleration_mps2;
        g_est_altitude_debug.inertial_aiding_active = acceleration_valid;
    }
#endif

    if (!s_state.reference_ready)
    {
        if (!barometer_valid) { return FC_STATUS_INVALID_DATA; }
        s_state.reference_pressure_sum_pa += barometer->pressure_pa;
        ++s_state.reference_sample_count;
        if (acceleration_valid)
        {
            s_state.reference_acceleration_sum_mps2 += acceleration_mps2;
            ++s_state.reference_acceleration_count;
        }
        g_est_altitude_debug.reference_sample_count =
            s_state.reference_sample_count;
        if (s_state.reference_sample_count < FC_BARO_REFERENCE_SAMPLE_COUNT)
        {
            if (aircraft_grounded)
            {
                s_state.aircraft_grounded_last_update = true;
            }
            return FC_STATUS_BUSY;
        }

        s_state.reference_pressure_pa = s_state.reference_pressure_sum_pa /
                                        (float)s_state.reference_sample_count;
        if (s_state.reference_acceleration_count > 0U)
        {
            initial_acceleration_bias_mps2 =
                s_state.reference_acceleration_sum_mps2 /
                (float)s_state.reference_acceleration_count;
        }
        s_state.filtered_pressure_pa = s_state.reference_pressure_pa;
        s_state.altitude_m = 0.0f;
        s_state.vertical_velocity_mps = 0.0f;
        s_state.last_valid_barometer_ms = timestamp_ms;
        s_state.last_update_ms = timestamp_ms;
        s_state.reference_ready = true;
        s_state.state_ready = true;
        s_state.barometer_velocity_ready = false;
#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_KALMAN
        initialize_kalman(initial_acceleration_bias_mps2);
#else
        (void)initial_acceleration_bias_mps2;
#endif
    }

    g_est_altitude_debug.ground_reference_tracking_active =
        aircraft_grounded && barometer_valid &&
        (FC_ALT_ENABLE_GROUND_REFERENCE_TRACKING != 0U);
    if (g_est_altitude_debug.ground_reference_tracking_active)
    {
        update_ground_reference(barometer->pressure_pa, dt_s);
    }

    if (barometer_valid)
    {
        pressure_alpha = low_pass_alpha(FC_BARO_PRESSURE_LPF_HZ, dt_s);
        s_state.filtered_pressure_pa += pressure_alpha *
            (barometer->pressure_pa - s_state.filtered_pressure_pa);
        takeoff_reference_freeze =
            (FC_ALT_ENABLE_GROUND_REFERENCE_TRACKING != 0U) &&
            s_state.aircraft_grounded_last_update && !aircraft_grounded;
        if (takeoff_reference_freeze)
        {
            /* 在真正离地的第一个有效气压样本冻结零面，消除预热慢漂残差。 */
            s_state.reference_pressure_pa = s_state.filtered_pressure_pa;
        }
        barometer_altitude_m =
            pressure_to_relative_altitude(s_state.filtered_pressure_pa);

#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_KALMAN
        if (aircraft_grounded || takeoff_reference_freeze)
        {
            s_state.last_barometer_altitude_m = barometer_altitude_m;
            s_state.barometer_velocity_ready = true;
            correction_accepted = true;
        }
        else if (s_state.barometer_velocity_ready &&
            (fabsf(barometer_altitude_m -
                   s_state.last_barometer_altitude_m) >
             FC_BARO_MAX_SAMPLE_STEP_M))
        {
            ++g_est_altitude_debug.barometer_step_reject_count;
            g_est_altitude_debug.last_reject_reason =
                EST_ALT_REJECT_BARO_STEP;
        }
        else if (kalman_correct(barometer_altitude_m, &innovation_m))
        {
            s_state.last_barometer_altitude_m = barometer_altitude_m;
            s_state.barometer_velocity_ready = true;
            correction_accepted = true;
            correction_applied = true;
        }
        else
        {
            ++g_est_altitude_debug.barometer_innovation_reject_count;
            g_est_altitude_debug.last_reject_reason =
                EST_ALT_REJECT_BARO_INNOVATION;
        }
#else
        if (aircraft_grounded || takeoff_reference_freeze)
        {
            s_state.last_barometer_altitude_m = barometer_altitude_m;
            s_state.barometer_velocity_ready = true;
            correction_accepted = true;
        }
        else if (s_state.barometer_velocity_ready &&
            (fabsf(barometer_altitude_m -
                   s_state.last_barometer_altitude_m) >
             FC_BARO_MAX_SAMPLE_STEP_M))
        {
            ++g_est_altitude_debug.barometer_step_reject_count;
            g_est_altitude_debug.last_reject_reason =
                EST_ALT_REJECT_BARO_STEP;
        }
        else
        {
            correction_accepted = complementary_correct(barometer_altitude_m,
                                                          acceleration_mps2,
                                                          acceleration_valid,
                                                          dt_s,
                                                          &innovation_m);
            correction_applied = correction_accepted;
            if (!correction_accepted)
            {
                ++g_est_altitude_debug.barometer_innovation_reject_count;
                g_est_altitude_debug.last_reject_reason =
                    EST_ALT_REJECT_BARO_INNOVATION;
            }
        }
#endif
        if (correction_accepted)
        {
            s_state.last_valid_barometer_ms = timestamp_ms;
            if (correction_applied)
            {
                ++g_est_altitude_debug.correction_count;
            }
            g_est_altitude_debug.consecutive_barometer_reject_count = 0U;
        }
        else
        {
            ++g_est_altitude_debug.rejected_sample_count;
            ++g_est_altitude_debug.consecutive_barometer_reject_count;
        }
    }
#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_COMPLEMENTARY
    else
    {
        /* Preserve the original short inertial bridge during pressure dropouts. */
        if (acceleration_valid)
        {
            s_state.altitude_m += (s_state.vertical_velocity_mps * dt_s) +
                                  (0.5f * acceleration_mps2 * dt_s * dt_s);
            s_state.vertical_velocity_mps += acceleration_mps2 * dt_s;
        }
    }
#endif

    if (aircraft_grounded && s_state.reference_ready)
    {
#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_KALMAN
        float ground_acceleration_bias_mps2 =
            s_state.kalman_state[KF_ACCEL_BIAS_INDEX];
        initialize_kalman(ground_acceleration_bias_mps2);
#else
        apply_ground_constraint();
#endif
    }
    if (aircraft_grounded)
    {
        s_state.aircraft_grounded_last_update = true;
    }
    else if (barometer_valid)
    {
        s_state.aircraft_grounded_last_update = false;
    }
    s_state.last_update_ms = timestamp_ms;
    g_est_altitude_debug.raw_altitude_m = barometer_altitude_m;
    g_est_altitude_debug.barometer_innovation_m = innovation_m;
    write_output(timestamp_ms, altitude);
    return altitude->valid ? FC_STATUS_OK : FC_STATUS_TIMEOUT;
}

void Est_AltitudeReset(void)
{
    (void)Est_AltitudeInit();
}
