/* Host tests for barometric zeroing, height response, and bounded controller. */

#include <math.h>
#include "ctl_altitude.h"
#include "est_altitude.h"
#include "fc_config.h"

static int nearly_equal(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static void predict_one_barometer_period(const FcImuData_t *imu,
                                         const FcAttitude_t *attitude,
                                         uint32_t timestamp_ms,
                                         bool aircraft_grounded,
                                         FcAltitude_t *altitude)
{
    uint32_t prediction;

    for (prediction = 0U;
         prediction < (FC_ATTITUDE_RATE_HZ / FC_ALTITUDE_RATE_HZ);
         ++prediction)
    {
        (void)Est_AltitudePredict(imu,
                                  attitude,
                                  FC_ATTITUDE_DT_S,
                                  timestamp_ms,
                                  aircraft_grounded,
                                  altitude);
    }
}

int main(void)
{
    FcBarometerData_t barometer = {0};
    FcAltitude_t altitude = {0};
    FcImuData_t imu = {0};
    FcAttitude_t attitude = {0};
    float correction_us = 0.0f;
    float normal_pressure_pa;
    uint32_t counter_before;
    uint32_t sample;

    barometer.valid = true;
    barometer.pressure_pa = 101325.0f;
    imu.valid = true;
    imu.calibrated = true;
    imu.accel_g.z = -1.0f;
    attitude.valid = true;
    if (Est_AltitudeInit() != FC_STATUS_OK) { return 1; }
    for (sample = 0U; sample < FC_BARO_REFERENCE_SAMPLE_COUNT; ++sample)
    {
        FcStatus_t status;
        barometer.timestamp_ms = sample * 20U;
        status = Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f,
                                    true, 1.0f, &altitude);
        if (((sample + 1U) < FC_BARO_REFERENCE_SAMPLE_COUNT) &&
            (status != FC_STATUS_BUSY)) { return 2; }
        if (((sample + 1U) == FC_BARO_REFERENCE_SAMPLE_COUNT) &&
            ((status != FC_STATUS_OK) || !altitude.valid)) { return 3; }
    }
    if (!nearly_equal(altitude.altitude_m, 0.0f, 0.01f) ||
        !g_est_altitude_debug.reference_ready) { return 4; }

    /* 地面气压慢漂时更新零面；估计输出仍必须保持为零。 */
    barometer.pressure_pa = 101337.0f;
    for (sample = 0U; sample < 500U; ++sample)
    {
        barometer.timestamp_ms += 20U;
        predict_one_barometer_period(&imu, &attitude,
                                     barometer.timestamp_ms, true, &altitude);
        if (Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f,
                               true, 1.0f, &altitude) != FC_STATUS_OK)
        {
            return 14;
        }
    }
    if (!nearly_equal(altitude.altitude_m, 0.0f, 0.01f) ||
        !g_est_altitude_debug.ground_reference_tracking_active) { return 15; }

    /* RUNNING后的第一个有效样本冻结真实起飞零面。 */
    barometer.timestamp_ms += 20U;
    predict_one_barometer_period(&imu, &attitude,
                                 barometer.timestamp_ms, false, &altitude);
    if (Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f,
                           false, 1.0f, &altitude) != FC_STATUS_OK) { return 16; }

#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_KALMAN
    /* ±0.1m/s²级小振动经过低通和连续死区后不应进入二次积分。 */
    imu.accel_g.z = -1.0f - (0.10f / FC_GRAVITY_MPS2);
    if (Est_AltitudePredict(&imu, &attitude, FC_ATTITUDE_DT_S,
                            barometer.timestamp_ms + 4U, false,
                            &altitude) != FC_STATUS_OK) { return 26; }
    if (fabsf(g_est_altitude_debug.effective_vertical_acceleration_mps2) >
        0.001f) { return 27; }
    imu.accel_g.z = -1.0f;

    /* 起飞桨流阶段的气压标准差缩放必须真正进入卡尔曼R。 */
    barometer.timestamp_ms += 20U;
    predict_one_barometer_period(&imu, &attitude,
                                 barometer.timestamp_ms, false, &altitude);
    if (Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f,
                           false, FC_TAKEOFF_BARO_STD_SCALE,
                           &altitude) != FC_STATUS_OK) { return 28; }
    if (g_est_altitude_debug.effective_barometer_std_m <
        (FC_ALT_KF_BARO_STD_M * FC_TAKEOFF_BARO_STD_SCALE)) { return 29; }
#endif

    /*
     * 在2秒内将气压降低约12Pa，模拟连续上升约一米。
     * 不能把气压瞬间跳变12Pa当作真实爬升，否则会把正常的速度限幅误判为故障。
     */
    for (sample = 0U; sample < 100U; ++sample)
    {
        barometer.pressure_pa = g_est_altitude_debug.reference_pressure_pa -
            12.0f * (float)(sample + 1U) / 100.0f;
        barometer.timestamp_ms += 20U;
        predict_one_barometer_period(&imu, &attitude,
                                     barometer.timestamp_ms, false, &altitude);
        if (Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f,
                               false, 1.0f, &altitude) != FC_STATUS_OK)
        {
            return 5;
        }
    }
    /* 保持目标气压2秒，检查稳态高度和速度，而不是检查爬升过程中的速度。 */
    for (sample = 0U; sample < 100U; ++sample)
    {
        barometer.timestamp_ms += 20U;
        predict_one_barometer_period(&imu, &attitude,
                                     barometer.timestamp_ms, false, &altitude);
        if (Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f,
                               false, 1.0f, &altitude) != FC_STATUS_OK)
        {
            return 5;
        }
    }
    if ((altitude.altitude_m < 0.8f) || (altitude.altitude_m > 1.2f) ||
        (fabsf(altitude.vertical_velocity_mps) > 0.4f)) { return 6; }
#if FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_ADAPTIVE_COMPLEMENTARY
    if (!g_est_altitude_debug.adaptive_complementary_enabled ||
        (g_est_altitude_debug.estimator_mode !=
         FC_ALT_ESTIMATOR_MODE_ADAPTIVE_COMPLEMENTARY)) { return 23; }
    if (g_est_altitude_debug.barometer_velocity_sample_count !=
        FC_ALT_ADAPTIVE_BARO_VELOCITY_WINDOW) { return 24; }
    if ((g_est_altitude_debug.adaptive_height_weight_n <
         FC_ALT_ADAPTIVE_HEIGHT_WEIGHT_MIN) ||
        (g_est_altitude_debug.adaptive_height_weight_n >
         FC_ALT_ADAPTIVE_HEIGHT_WEIGHT_MAX) ||
        (g_est_altitude_debug.adaptive_velocity_weight_m <
         FC_ALT_ADAPTIVE_VELOCITY_WEIGHT_MIN) ||
        (g_est_altitude_debug.adaptive_velocity_weight_m >
         FC_ALT_ADAPTIVE_VELOCITY_WEIGHT_MAX)) { return 25; }
#elif FC_ALT_ESTIMATOR_MODE == FC_ALT_ESTIMATOR_MODE_KALMAN
    if (!g_est_altitude_debug.kalman_enabled ||
        (g_est_altitude_debug.estimator_mode !=
         FC_ALT_ESTIMATOR_MODE_KALMAN)) { return 23; }
#endif

    /* 可信模长内的异常竖直加速度必须被拒绝，不能截幅后继续积分。 */
    counter_before = g_est_altitude_debug.acceleration_reject_count;
    imu.accel_g.z = -1.5f;
#if FC_ALT_ESTIMATOR_MODE != FC_ALT_ESTIMATOR_MODE_COMPLEMENTARY
    if (Est_AltitudePredict(&imu, &attitude, FC_ATTITUDE_DT_S,
                            barometer.timestamp_ms + 4U, false, &altitude) !=
        FC_STATUS_OK) { return 17; }
#else
    barometer.timestamp_ms += 20U;
    if (Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f,
                           false, 1.0f, &altitude) != FC_STATUS_OK) { return 17; }
#endif
    imu.accel_g.z = -1.0f;
    if ((g_est_altitude_debug.acceleration_reject_count <= counter_before) ||
        g_est_altitude_debug.inertial_aiding_active) { return 18; }

    /* 一个气压尖峰应被拒绝；恢复正常后应在超时前重新接受观测。 */
    normal_pressure_pa = barometer.pressure_pa;
    counter_before = g_est_altitude_debug.barometer_step_reject_count;
    barometer.pressure_pa = g_est_altitude_debug.reference_pressure_pa - 100.0f;
    barometer.timestamp_ms += 20U;
    predict_one_barometer_period(&imu, &attitude,
                                 barometer.timestamp_ms, false, &altitude);
    if (Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f,
                           false, 1.0f, &altitude) != FC_STATUS_OK) { return 19; }
    if (g_est_altitude_debug.barometer_step_reject_count <= counter_before)
    {
        return 20;
    }
    barometer.pressure_pa = normal_pressure_pa;
    for (sample = 0U; sample < 15U; ++sample)
    {
        barometer.timestamp_ms += 20U;
        predict_one_barometer_period(&imu, &attitude,
                                     barometer.timestamp_ms, false, &altitude);
        if (Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f,
                               false, 1.0f, &altitude) != FC_STATUS_OK) { return 21; }
        if (g_est_altitude_debug.consecutive_barometer_reject_count == 0U)
        {
            break;
        }
    }
    if (g_est_altitude_debug.consecutive_barometer_reject_count != 0U)
    {
        return 22;
    }

    /* A short barometer dropout is bridged by inertial prediction. */
    barometer.valid = false;
    for (sample = 0U; sample < (FC_BARO_INERTIAL_HOLD_TIMEOUT_MS / 20U); ++sample)
    {
        barometer.timestamp_ms += 20U;
        predict_one_barometer_period(&imu, &attitude,
                                     barometer.timestamp_ms, false, &altitude);
        if (Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f,
                               false, 1.0f, &altitude) !=
            FC_STATUS_OK) { return 11; }
    }
    barometer.timestamp_ms += 20U;
    predict_one_barometer_period(&imu, &attitude,
                                 barometer.timestamp_ms, false, &altitude);
    if ((Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f,
                            false, 1.0f, &altitude) !=
         FC_STATUS_TIMEOUT) || altitude.valid) { return 12; }
    barometer.valid = true;
    barometer.timestamp_ms += 20U;
    predict_one_barometer_period(&imu, &attitude,
                                 barometer.timestamp_ms, false, &altitude);
    if (Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f,
                           false, 1.0f, &altitude) !=
        FC_STATUS_OK) { return 13; }

    if (Ctl_AltitudeInit() != FC_STATUS_OK) { return 7; }
    altitude.vertical_velocity_mps = 0.0f;
    if (Ctl_AltitudeUpdate(altitude.altitude_m + 0.5f,
                           &altitude,
                           0.02f,
                           &correction_us) != FC_STATUS_OK ||
        (correction_us <= 0.0f)) { return 8; }
    if (Ctl_AltitudeUpdate(altitude.altitude_m - 5.0f,
                           &altitude,
                           0.02f,
                           &correction_us) != FC_STATUS_OK ||
        (correction_us < -FC_ALTITUDE_CORRECTION_LIMIT_US) ||
        (correction_us > FC_ALTITUDE_CORRECTION_LIMIT_US)) { return 9; }
    Ctl_AltitudeReset();
    if (!nearly_equal(g_ctl_altitude_debug.integral_us, 0.0f, 0.001f)) { return 10; }
    return 0;
}
