/* Unbounded-drift XY inertial diagnostic; deliberately disconnected from control. */

#include <math.h>
#include <stddef.h>
#include "est_inertial_nav.h"
#include "fc_config.h"

#define DEG_TO_RAD (0.01745329251994329577f)

static bool s_initialized;
volatile EstInertialNavDebug_t g_est_inertial_nav_debug;

static bool sample_is_stationary(const FcImuData_t *imu)
{
    float norm_sq = (imu->accel_g.x * imu->accel_g.x) +
                    (imu->accel_g.y * imu->accel_g.y) +
                    (imu->accel_g.z * imu->accel_g.z);
    return (fabsf(imu->gyro_dps.x) <= FC_ATTITUDE_LEVEL_CAL_MAX_GYRO_DPS) &&
           (fabsf(imu->gyro_dps.y) <= FC_ATTITUDE_LEVEL_CAL_MAX_GYRO_DPS) &&
           (fabsf(imu->gyro_dps.z) <= FC_ATTITUDE_LEVEL_CAL_MAX_GYRO_DPS) &&
           (norm_sq >= FC_ATTITUDE_LEVEL_CAL_ACCEL_MIN_SQ) &&
           (norm_sq <= FC_ATTITUDE_LEVEL_CAL_ACCEL_MAX_SQ);
}

FcStatus_t Est_InertialNavInit(void)
{
    g_est_inertial_nav_debug = (EstInertialNavDebug_t){0};
    s_initialized = true;
    return FC_STATUS_OK;
}

FcStatus_t Est_InertialNavUpdate(const FcImuData_t *imu,
                                 const FcAttitude_t *attitude,
                                 bool aircraft_stopped,
                                 float dt_s)
{
    float roll;
    float pitch;
    float yaw;
    float sr;
    float cr;
    float sp;
    float cp;
    float sy;
    float cy;
    float north_accel_g;
    float east_accel_g;

    if ((imu == NULL) || (attitude == NULL) ||
        (dt_s <= 0.0f) || (dt_s > 0.02f))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    if (!s_initialized) { return FC_STATUS_NOT_INITIALIZED; }
    if (!imu->valid || !attitude->valid)
    {
        g_est_inertial_nav_debug.valid = false;
        return FC_STATUS_INVALID_DATA;
    }

    g_est_inertial_nav_debug.stationary = aircraft_stopped && sample_is_stationary(imu);
    if (g_est_inertial_nav_debug.stationary)
    {
        g_est_inertial_nav_debug.earth_acceleration_mps2 = (FcVector3f_t){0};
        g_est_inertial_nav_debug.estimated_velocity_mps = (FcVector3f_t){0};
        g_est_inertial_nav_debug.estimated_position_m = (FcVector3f_t){0};
        ++g_est_inertial_nav_debug.stationary_reset_count;
        g_est_inertial_nav_debug.valid = true;
        return FC_STATUS_OK;
    }

    roll = attitude->roll_deg * DEG_TO_RAD;
    pitch = attitude->pitch_deg * DEG_TO_RAD;
    yaw = attitude->yaw_deg * DEG_TO_RAD;
    sr = sinf(roll); cr = cosf(roll);
    sp = sinf(pitch); cp = cosf(pitch);
    sy = sinf(yaw); cy = cosf(yaw);

    north_accel_g = (cy * cp * imu->accel_g.x) +
                    (((cy * sp * sr) - (sy * cr)) * imu->accel_g.y) +
                    (((cy * sp * cr) + (sy * sr)) * imu->accel_g.z);
    east_accel_g = (sy * cp * imu->accel_g.x) +
                   (((sy * sp * sr) + (cy * cr)) * imu->accel_g.y) +
                   (((sy * sp * cr) - (cy * sr)) * imu->accel_g.z);
    g_est_inertial_nav_debug.earth_acceleration_mps2.x = north_accel_g * FC_GRAVITY_MPS2;
    g_est_inertial_nav_debug.earth_acceleration_mps2.y = east_accel_g * FC_GRAVITY_MPS2;
    g_est_inertial_nav_debug.estimated_position_m.x +=
        (g_est_inertial_nav_debug.estimated_velocity_mps.x * dt_s) +
        (0.5f * g_est_inertial_nav_debug.earth_acceleration_mps2.x * dt_s * dt_s);
    g_est_inertial_nav_debug.estimated_position_m.y +=
        (g_est_inertial_nav_debug.estimated_velocity_mps.y * dt_s) +
        (0.5f * g_est_inertial_nav_debug.earth_acceleration_mps2.y * dt_s * dt_s);
    g_est_inertial_nav_debug.estimated_velocity_mps.x +=
        g_est_inertial_nav_debug.earth_acceleration_mps2.x * dt_s;
    g_est_inertial_nav_debug.estimated_velocity_mps.y +=
        g_est_inertial_nav_debug.earth_acceleration_mps2.y * dt_s;
    ++g_est_inertial_nav_debug.update_count;
    g_est_inertial_nav_debug.valid = true;
    return FC_STATUS_OK;
}

void Est_InertialNavReset(void)
{
    (void)Est_InertialNavInit();
}
