/* Mahony attitude estimator with optional, bounded magnetic yaw correction. */

#include <math.h>
#include <stddef.h>
#include "est_attitude.h"
#include "fc_config.h"

#define DEG_TO_RAD (0.01745329251994329577f)
#define RAD_TO_DEG (57.295779513082320876f)

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
    FcVector3f_t integral_feedback;
    FcVector3f_t level_accel_sum;
    float level_q0;
    float level_q1;
    float level_q2;
    float level_q3;
    uint32_t level_sample_count;
    FcMagnetometerData_t magnetometer;
    float magnetic_yaw_offset_deg;
    bool initialized;
    bool aligned;
    bool level_calibrated;
    bool magnetic_heading_initialized;
} AttitudeEstimatorState_t;

static AttitudeEstimatorState_t s_state;
volatile EstAttitudeDebug_t g_est_attitude_debug;

static void quaternion_to_euler(float q0,
                                float q1,
                                float q2,
                                float q3,
                                FcAttitude_t *attitude);

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) { return minimum; }
    if (value > maximum) { return maximum; }
    return value;
}

#if FC_ENABLE_MAG_YAW_FUSION
static float wrap_degrees(float angle_deg)
{
    while (angle_deg > 180.0f) { angle_deg -= 360.0f; }
    while (angle_deg < -180.0f) { angle_deg += 360.0f; }
    return angle_deg;
}
#endif

static void reset_state(void)
{
    s_state = (AttitudeEstimatorState_t){0};
    s_state.q0 = 1.0f;
    s_state.level_q0 = 1.0f;
    s_state.initialized = true;
    g_est_attitude_debug = (EstAttitudeDebug_t){0};
}

static bool quaternion_from_accelerometer(const FcVector3f_t *accel_g,
                                          float *q0,
                                          float *q1,
                                          float *q2,
                                          float *q3)
{
    float norm_sq = (accel_g->x * accel_g->x) +
                    (accel_g->y * accel_g->y) +
                    (accel_g->z * accel_g->z);
    float reciprocal_norm;
    float gravity_x;
    float gravity_y;
    float gravity_z;
    float roll_rad;
    float pitch_rad;
    float half_roll;
    float half_pitch;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;

    if ((norm_sq < FC_AHRS_ACCEL_MIN_NORM_SQ) ||
        (norm_sq > FC_AHRS_ACCEL_MAX_NORM_SQ))
    {
        return false;
    }

    reciprocal_norm = 1.0f / sqrtf(norm_sq);
    gravity_x = -accel_g->x * reciprocal_norm;
    gravity_y = -accel_g->y * reciprocal_norm;
    gravity_z = -accel_g->z * reciprocal_norm;
    roll_rad = atan2f(gravity_y, gravity_z);
    pitch_rad = -asinf(clamp_float(gravity_x, -1.0f, 1.0f));

    half_roll = 0.5f * roll_rad;
    half_pitch = 0.5f * pitch_rad;
    cos_roll = cosf(half_roll);
    sin_roll = sinf(half_roll);
    cos_pitch = cosf(half_pitch);
    sin_pitch = sinf(half_pitch);

    /* ZYX quaternion with yaw deliberately initialized to zero. */
    *q0 = cos_roll * cos_pitch;
    *q1 = sin_roll * cos_pitch;
    *q2 = cos_roll * sin_pitch;
    *q3 = -sin_roll * sin_pitch;
    return true;
}

static bool align_from_accelerometer(const FcVector3f_t *accel_g)
{
    if (!quaternion_from_accelerometer(accel_g,
                                       &s_state.q0,
                                       &s_state.q1,
                                       &s_state.q2,
                                       &s_state.q3))
    {
        return false;
    }

    s_state.integral_feedback = (FcVector3f_t){0};
    s_state.aligned = true;
    return true;
}

FcStatus_t Est_AttitudeInit(void)
{
    reset_state();
    return FC_STATUS_OK;
}

FcStatus_t Est_AttitudeSetMagnetometer(const FcMagnetometerData_t *magnetometer)
{
    if (magnetometer == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    if (!s_state.initialized) { return FC_STATUS_NOT_INITIALIZED; }
    if (!magnetometer->valid || magnetometer->overflow)
    {
        ++g_est_attitude_debug.magnetic_reject_count;
        return FC_STATUS_INVALID_DATA;
    }
    s_state.magnetometer = *magnetometer;
    return FC_STATUS_OK;
}

static bool apply_magnetic_yaw_feedback(uint32_t timestamp_ms,
                                        FcVector3f_t *gyro_rad_s)
{
#if !FC_ENABLE_MAG_YAW_FUSION
    (void)timestamp_ms;
    (void)gyro_rad_s;
    g_est_attitude_debug.magnetic_aiding_active = false;
    return false;
#else
    FcAttitude_t raw_attitude = {0};
    float roll_rad;
    float pitch_rad;
    float horizontal_x;
    float horizontal_y;
    float magnetic_heading_deg;
    float desired_yaw_deg;
    float yaw_error_deg;
    float correction_dps;

    if (!s_state.level_calibrated || !s_state.magnetometer.valid ||
        ((timestamp_ms - s_state.magnetometer.timestamp_ms) > FC_MAG_DATA_TIMEOUT_MS))
    {
        g_est_attitude_debug.magnetic_aiding_active = false;
        return false;
    }

    quaternion_to_euler(s_state.q0, s_state.q1, s_state.q2, s_state.q3, &raw_attitude);
    roll_rad = raw_attitude.roll_deg * DEG_TO_RAD;
    pitch_rad = raw_attitude.pitch_deg * DEG_TO_RAD;
    horizontal_x = (s_state.magnetometer.magnetic_ut.x * cosf(pitch_rad)) +
                   (s_state.magnetometer.magnetic_ut.y * sinf(roll_rad) * sinf(pitch_rad)) +
                   (s_state.magnetometer.magnetic_ut.z * cosf(roll_rad) * sinf(pitch_rad));
    horizontal_y = (s_state.magnetometer.magnetic_ut.y * cosf(roll_rad)) -
                   (s_state.magnetometer.magnetic_ut.z * sinf(roll_rad));
    if (((horizontal_x * horizontal_x) + (horizontal_y * horizontal_y)) < 1.0f)
    {
        ++g_est_attitude_debug.magnetic_reject_count;
        g_est_attitude_debug.magnetic_aiding_active = false;
        return false;
    }

    magnetic_heading_deg = atan2f(-horizontal_y, horizontal_x) * RAD_TO_DEG;
    if (!s_state.magnetic_heading_initialized)
    {
        s_state.magnetic_yaw_offset_deg = wrap_degrees(magnetic_heading_deg -
                                                       raw_attitude.yaw_deg);
        s_state.magnetic_heading_initialized = true;
    }
    desired_yaw_deg = wrap_degrees(magnetic_heading_deg - s_state.magnetic_yaw_offset_deg);
    yaw_error_deg = wrap_degrees(desired_yaw_deg - raw_attitude.yaw_deg);
    correction_dps = clamp_float(yaw_error_deg * FC_MAG_YAW_KP,
                                 -FC_MAG_YAW_MAX_CORRECTION_DPS,
                                 FC_MAG_YAW_MAX_CORRECTION_DPS);
    gyro_rad_s->z += correction_dps * DEG_TO_RAD;
    g_est_attitude_debug.magnetic_heading_deg = magnetic_heading_deg;
    g_est_attitude_debug.magnetic_yaw_error_deg = yaw_error_deg;
    g_est_attitude_debug.magnetic_heading_initialized = true;
    g_est_attitude_debug.magnetic_aiding_active = true;
    return true;
#endif
}

static bool apply_accel_feedback(const FcVector3f_t *accel_g,
                                 float dt_s,
                                 FcVector3f_t *gyro_rad_s)
{
    float norm_sq = (accel_g->x * accel_g->x) +
                    (accel_g->y * accel_g->y) +
                    (accel_g->z * accel_g->z);
    float reciprocal_norm;
    float ax;
    float ay;
    float az;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;

    if ((norm_sq < FC_AHRS_ACCEL_MIN_NORM_SQ) ||
        (norm_sq > FC_AHRS_ACCEL_MAX_NORM_SQ))
    {
        return false;
    }

    reciprocal_norm = 1.0f / sqrtf(norm_sq);

    /*
     * Body frame is X forward, Y right, Z down. A stationary accelerometer
     * measures specific force, so level flight must read approximately -1 g
     * on body Z. Negating it gives the gravity direction used by Mahony.
     */
    ax = -accel_g->x * reciprocal_norm;
    ay = -accel_g->y * reciprocal_norm;
    az = -accel_g->z * reciprocal_norm;

    vx = 2.0f * ((s_state.q1 * s_state.q3) - (s_state.q0 * s_state.q2));
    vy = 2.0f * ((s_state.q0 * s_state.q1) + (s_state.q2 * s_state.q3));
    vz = (s_state.q0 * s_state.q0) - (s_state.q1 * s_state.q1) -
         (s_state.q2 * s_state.q2) + (s_state.q3 * s_state.q3);

    ex = (ay * vz) - (az * vy);
    ey = (az * vx) - (ax * vz);
    ez = (ax * vy) - (ay * vx);

    if (FC_AHRS_MAHONY_KI > 0.0f)
    {
        s_state.integral_feedback.x += FC_AHRS_MAHONY_KI * ex * dt_s;
        s_state.integral_feedback.y += FC_AHRS_MAHONY_KI * ey * dt_s;
        s_state.integral_feedback.z += FC_AHRS_MAHONY_KI * ez * dt_s;
        s_state.integral_feedback.x = clamp_float(s_state.integral_feedback.x,
                                                  -FC_AHRS_INTEGRAL_LIMIT_RAD_S,
                                                  FC_AHRS_INTEGRAL_LIMIT_RAD_S);
        s_state.integral_feedback.y = clamp_float(s_state.integral_feedback.y,
                                                  -FC_AHRS_INTEGRAL_LIMIT_RAD_S,
                                                  FC_AHRS_INTEGRAL_LIMIT_RAD_S);
        s_state.integral_feedback.z = clamp_float(s_state.integral_feedback.z,
                                                  -FC_AHRS_INTEGRAL_LIMIT_RAD_S,
                                                  FC_AHRS_INTEGRAL_LIMIT_RAD_S);
    }
    else
    {
        s_state.integral_feedback = (FcVector3f_t){0};
    }

    gyro_rad_s->x += (FC_AHRS_MAHONY_KP * ex) + s_state.integral_feedback.x;
    gyro_rad_s->y += (FC_AHRS_MAHONY_KP * ey) + s_state.integral_feedback.y;
    gyro_rad_s->z += (FC_AHRS_MAHONY_KP * ez) + s_state.integral_feedback.z;
    return true;
}

static bool integrate_quaternion(const FcVector3f_t *gyro_rad_s, float dt_s)
{
    float half_dt = 0.5f * dt_s;
    float q0 = s_state.q0;
    float q1 = s_state.q1;
    float q2 = s_state.q2;
    float q3 = s_state.q3;
    float norm_sq;
    float reciprocal_norm;

    s_state.q0 += (-q1 * gyro_rad_s->x - q2 * gyro_rad_s->y - q3 * gyro_rad_s->z) * half_dt;
    s_state.q1 += ( q0 * gyro_rad_s->x + q2 * gyro_rad_s->z - q3 * gyro_rad_s->y) * half_dt;
    s_state.q2 += ( q0 * gyro_rad_s->y - q1 * gyro_rad_s->z + q3 * gyro_rad_s->x) * half_dt;
    s_state.q3 += ( q0 * gyro_rad_s->z + q1 * gyro_rad_s->y - q2 * gyro_rad_s->x) * half_dt;

    norm_sq = (s_state.q0 * s_state.q0) + (s_state.q1 * s_state.q1) +
              (s_state.q2 * s_state.q2) + (s_state.q3 * s_state.q3);
    if (norm_sq < 0.01f)
    {
        reset_state();
        return false;
    }

    reciprocal_norm = 1.0f / sqrtf(norm_sq);
    s_state.q0 *= reciprocal_norm;
    s_state.q1 *= reciprocal_norm;
    s_state.q2 *= reciprocal_norm;
    s_state.q3 *= reciprocal_norm;
    return true;
}

static void quaternion_to_euler(float q0,
                                float q1,
                                float q2,
                                float q3,
                                FcAttitude_t *attitude)
{
    float sin_pitch = 2.0f * ((q0 * q2) - (q3 * q1));

    attitude->roll_deg = atan2f(2.0f * ((q0 * q1) + (q2 * q3)),
                                1.0f - (2.0f * ((q1 * q1) + (q2 * q2)))) * RAD_TO_DEG;
    attitude->pitch_deg = asinf(clamp_float(sin_pitch, -1.0f, 1.0f)) * RAD_TO_DEG;
    attitude->yaw_deg = atan2f(2.0f * ((q0 * q3) + (q1 * q2)),
                               1.0f - (2.0f * ((q2 * q2) + (q3 * q3)))) * RAD_TO_DEG;
}

static bool sample_is_stationary(const FcImuData_t *imu)
{
    float accel_norm_sq = (imu->accel_g.x * imu->accel_g.x) +
                          (imu->accel_g.y * imu->accel_g.y) +
                          (imu->accel_g.z * imu->accel_g.z);

    return (fabsf(imu->gyro_dps.x) <= FC_ATTITUDE_LEVEL_CAL_MAX_GYRO_DPS) &&
           (fabsf(imu->gyro_dps.y) <= FC_ATTITUDE_LEVEL_CAL_MAX_GYRO_DPS) &&
           (fabsf(imu->gyro_dps.z) <= FC_ATTITUDE_LEVEL_CAL_MAX_GYRO_DPS) &&
           (accel_norm_sq >= FC_ATTITUDE_LEVEL_CAL_ACCEL_MIN_SQ) &&
           (accel_norm_sq <= FC_ATTITUDE_LEVEL_CAL_ACCEL_MAX_SQ);
}

static void reset_level_accumulator(void)
{
    s_state.level_accel_sum = (FcVector3f_t){0};
    s_state.level_sample_count = 0U;
    g_est_attitude_debug.level_sample_count = 0U;
}

static bool update_level_reference(const FcImuData_t *imu,
                                   const FcAttitude_t *raw_attitude)
{
    FcVector3f_t mean_accel;
    FcAttitude_t reference_attitude = {0};

    if (!sample_is_stationary(imu) ||
        (fabsf(raw_attitude->roll_deg) > FC_ATTITUDE_LEVEL_MAX_TRIM_DEG) ||
        (fabsf(raw_attitude->pitch_deg) > FC_ATTITUDE_LEVEL_MAX_TRIM_DEG))
    {
        reset_level_accumulator();
        return false;
    }

    s_state.level_accel_sum.x += imu->accel_g.x;
    s_state.level_accel_sum.y += imu->accel_g.y;
    s_state.level_accel_sum.z += imu->accel_g.z;
    ++s_state.level_sample_count;
    g_est_attitude_debug.level_sample_count = s_state.level_sample_count;

    if (s_state.level_sample_count < FC_ATTITUDE_LEVEL_CAL_SAMPLE_COUNT)
    {
        return false;
    }

    mean_accel.x = s_state.level_accel_sum.x / (float)s_state.level_sample_count;
    mean_accel.y = s_state.level_accel_sum.y / (float)s_state.level_sample_count;
    mean_accel.z = s_state.level_accel_sum.z / (float)s_state.level_sample_count;
    if (!quaternion_from_accelerometer(&mean_accel,
                                       &s_state.level_q0,
                                       &s_state.level_q1,
                                       &s_state.level_q2,
                                       &s_state.level_q3))
    {
        reset_level_accumulator();
        return false;
    }

    /* Restart the running estimate from the averaged gravity direction. */
    s_state.q0 = s_state.level_q0;
    s_state.q1 = s_state.level_q1;
    s_state.q2 = s_state.level_q2;
    s_state.q3 = s_state.level_q3;
    s_state.integral_feedback = (FcVector3f_t){0};
    s_state.level_calibrated = true;
    quaternion_to_euler(s_state.level_q0,
                        s_state.level_q1,
                        s_state.level_q2,
                        s_state.level_q3,
                        &reference_attitude);
    g_est_attitude_debug.level_roll_trim_deg = reference_attitude.roll_deg;
    g_est_attitude_debug.level_pitch_trim_deg = reference_attitude.pitch_deg;
    g_est_attitude_debug.level_calibrated = true;
    return true;
}

static void relative_attitude_to_euler(FcAttitude_t *attitude)
{
    float q0 = (s_state.level_q0 * s_state.q0) +
               (s_state.level_q1 * s_state.q1) +
               (s_state.level_q2 * s_state.q2) +
               (s_state.level_q3 * s_state.q3);
    float q1 = (s_state.level_q0 * s_state.q1) -
               (s_state.level_q1 * s_state.q0) -
               (s_state.level_q2 * s_state.q3) +
               (s_state.level_q3 * s_state.q2);
    float q2 = (s_state.level_q0 * s_state.q2) +
               (s_state.level_q1 * s_state.q3) -
               (s_state.level_q2 * s_state.q0) -
               (s_state.level_q3 * s_state.q1);
    float q3 = (s_state.level_q0 * s_state.q3) -
               (s_state.level_q1 * s_state.q2) +
               (s_state.level_q2 * s_state.q1) -
               (s_state.level_q3 * s_state.q0);

    quaternion_to_euler(q0, q1, q2, q3, attitude);
}

FcStatus_t Est_AttitudeUpdate(const FcImuData_t *imu, float dt_s, FcAttitude_t *attitude)
{
    FcVector3f_t gyro_rad_s;
    FcAttitude_t raw_attitude = {0};

    if ((imu == NULL) || (attitude == NULL) || (dt_s <= 0.0f) || (dt_s > 0.1f))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *attitude = (FcAttitude_t){0};
    attitude->timestamp_ms = imu->timestamp_ms;

    if (!s_state.initialized)
    {
        return FC_STATUS_NOT_INITIALIZED;
    }
    if (!imu->valid || !imu->calibrated)
    {
        return FC_STATUS_INVALID_DATA;
    }
    if (!s_state.aligned && !align_from_accelerometer(&imu->accel_g))
    {
        return FC_STATUS_INVALID_DATA;
    }
    g_est_attitude_debug.aligned = s_state.aligned;

    gyro_rad_s.x = imu->gyro_dps.x * DEG_TO_RAD;
    gyro_rad_s.y = imu->gyro_dps.y * DEG_TO_RAD;
    gyro_rad_s.z = imu->gyro_dps.z * DEG_TO_RAD;
    (void)apply_accel_feedback(&imu->accel_g, dt_s, &gyro_rad_s);
    (void)apply_magnetic_yaw_feedback(imu->timestamp_ms, &gyro_rad_s);

    if (!integrate_quaternion(&gyro_rad_s, dt_s))
    {
        return FC_STATUS_INVALID_DATA;
    }

    quaternion_to_euler(s_state.q0, s_state.q1, s_state.q2, s_state.q3, &raw_attitude);
    if (!s_state.level_calibrated && !update_level_reference(imu, &raw_attitude))
    {
        return FC_STATUS_BUSY;
    }

    relative_attitude_to_euler(attitude);
    attitude->valid = true;
    return FC_STATUS_OK;
}

void Est_AttitudeReset(void)
{
    reset_state();
}
