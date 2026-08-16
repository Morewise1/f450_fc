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
        status = Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f, &altitude);
        if (((sample + 1U) < FC_BARO_REFERENCE_SAMPLE_COUNT) &&
            (status != FC_STATUS_BUSY)) { return 2; }
        if (((sample + 1U) == FC_BARO_REFERENCE_SAMPLE_COUNT) &&
            ((status != FC_STATUS_OK) || !altitude.valid)) { return 3; }
    }
    if (!nearly_equal(altitude.altitude_m, 0.0f, 0.01f) ||
        !g_est_altitude_debug.reference_ready) { return 4; }

    /* Roughly one metre above the reference pressure. */
    barometer.pressure_pa = 101313.0f;
    for (sample = 0U; sample < 100U; ++sample)
    {
        barometer.timestamp_ms += 20U;
        predict_one_barometer_period(&imu, &attitude,
                                     barometer.timestamp_ms, &altitude);
        if (Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f, &altitude) != FC_STATUS_OK)
        {
            return 5;
        }
    }
    if ((altitude.altitude_m < 0.8f) || (altitude.altitude_m > 1.2f)) { return 6; }

    /* A short barometer dropout is bridged by inertial prediction. */
    barometer.valid = false;
    for (sample = 0U; sample < (FC_BARO_INERTIAL_HOLD_TIMEOUT_MS / 20U); ++sample)
    {
        barometer.timestamp_ms += 20U;
        predict_one_barometer_period(&imu, &attitude,
                                     barometer.timestamp_ms, &altitude);
        if (Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f, &altitude) !=
            FC_STATUS_OK) { return 11; }
    }
    barometer.timestamp_ms += 20U;
    predict_one_barometer_period(&imu, &attitude,
                                 barometer.timestamp_ms, &altitude);
    if ((Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f, &altitude) !=
         FC_STATUS_TIMEOUT) || altitude.valid) { return 12; }
    barometer.valid = true;
    barometer.timestamp_ms += 20U;
    predict_one_barometer_period(&imu, &attitude,
                                 barometer.timestamp_ms, &altitude);
    if (Est_AltitudeUpdate(&barometer, &imu, &attitude, 0.02f, &altitude) !=
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
