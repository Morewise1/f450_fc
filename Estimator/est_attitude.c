/* Six-axis Mahony attitude estimator. Yaw remains gyro-relative without a magnetometer. */

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
    bool initialized;
    bool aligned;
} AttitudeEstimatorState_t;

static AttitudeEstimatorState_t s_state;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) { return minimum; }
    if (value > maximum) { return maximum; }
    return value;
}

static void reset_state(void)
{
    s_state = (AttitudeEstimatorState_t){0};
    s_state.q0 = 1.0f;
    s_state.initialized = true;
}

static bool align_from_accelerometer(const FcVector3f_t *accel_g)
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
    s_state.q0 = cos_roll * cos_pitch;
    s_state.q1 = sin_roll * cos_pitch;
    s_state.q2 = cos_roll * sin_pitch;
    s_state.q3 = -sin_roll * sin_pitch;
    s_state.integral_feedback = (FcVector3f_t){0};
    s_state.aligned = true;
    return true;
}

FcStatus_t Est_AttitudeInit(void)
{
    reset_state();
    return FC_STATUS_OK;
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

static void quaternion_to_euler(FcAttitude_t *attitude)
{
    float sin_pitch = 2.0f * ((s_state.q0 * s_state.q2) -
                              (s_state.q3 * s_state.q1));

    attitude->roll_deg = atan2f(2.0f * ((s_state.q0 * s_state.q1) +
                                        (s_state.q2 * s_state.q3)),
                                1.0f - (2.0f * ((s_state.q1 * s_state.q1) +
                                                (s_state.q2 * s_state.q2)))) * RAD_TO_DEG;
    attitude->pitch_deg = asinf(clamp_float(sin_pitch, -1.0f, 1.0f)) * RAD_TO_DEG;
    attitude->yaw_deg = atan2f(2.0f * ((s_state.q0 * s_state.q3) +
                                       (s_state.q1 * s_state.q2)),
                               1.0f - (2.0f * ((s_state.q2 * s_state.q2) +
                                               (s_state.q3 * s_state.q3)))) * RAD_TO_DEG;
}

FcStatus_t Est_AttitudeUpdate(const FcImuData_t *imu, float dt_s, FcAttitude_t *attitude)
{
    FcVector3f_t gyro_rad_s;

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

    gyro_rad_s.x = imu->gyro_dps.x * DEG_TO_RAD;
    gyro_rad_s.y = imu->gyro_dps.y * DEG_TO_RAD;
    gyro_rad_s.z = imu->gyro_dps.z * DEG_TO_RAD;
    (void)apply_accel_feedback(&imu->accel_g, dt_s, &gyro_rad_s);

    if (!integrate_quaternion(&gyro_rad_s, dt_s))
    {
        return FC_STATUS_INVALID_DATA;
    }

    quaternion_to_euler(attitude);
    attitude->valid = true;
    return FC_STATUS_OK;
}

void Est_AttitudeReset(void)
{
    reset_state();
}
