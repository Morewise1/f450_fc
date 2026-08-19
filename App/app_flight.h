#ifndef APP_FLIGHT_H
#define APP_FLIGHT_H

/* Flight state machine and periodic task orchestration. */

#include <stdint.h>
#include "fc_types.h"

typedef enum
{
    APP_ALT_HOLD_PHASE_INACTIVE = 0,
    APP_ALT_HOLD_PHASE_WAIT_CAPTURE,
    APP_ALT_HOLD_PHASE_ACTIVE,
    APP_ALT_HOLD_PHASE_EXIT_WAIT,
    APP_ALT_HOLD_PHASE_EXIT_BLEND
} AppAltitudeHoldPhase_t;

typedef enum
{
    APP_TAKEOFF_PHASE_GROUNDED = 0,
    APP_TAKEOFF_PHASE_PENDING,
    APP_TAKEOFF_PHASE_AIRBORNE
} AppTakeoffPhase_t;

typedef struct
{
    uint32_t task_500hz_count;
    uint32_t task_250hz_count;
    uint32_t task_100hz_count;
    uint32_t task_50hz_count;
    uint32_t task_10hz_count;
} AppFlightTaskStats_t;

/* One Keil Watch-friendly snapshot. App internals remain privately owned. */
typedef struct
{
    FcImuData_t imu;
    FcAttitude_t attitude;
    FcBarometerData_t barometer;
    FcMagnetometerData_t magnetometer;
    FcAltitude_t altitude;
    FcRcInput_t receiver;
    FcPilotCommand_t pilot;
    FcMotorOutput_t motors;
    FcSafetyStatus_t safety;
    FcControlOutput_t control;
    FcVector3f_t target_rate_dps;
    uint16_t raw_channels[FC_IBUS_CHANNEL_COUNT];
    FcFlightState_t state;
    FcFlightMode_t mode;
    AppTakeoffPhase_t takeoff_phase;
    bool airborne;
    float barometer_noise_scale;
    AppAltitudeHoldPhase_t altitude_hold_phase;
    bool altitude_throttle_captured;
    bool altitude_entry_blend_active;
    float altitude_target_m;
    float altitude_correction_us;
    uint16_t manual_throttle_command_us;
    uint16_t automatic_throttle_command_us;
    uint16_t throttle_command_us;
    bool motor_safe;
    uint32_t publish_count;
} AppFlightDebug_t;

extern volatile AppFlightDebug_t g_fc_flight_debug;

FcStatus_t App_FlightInit(void);
void App_FlightTask500Hz(void);
void App_FlightTask250Hz(void);
void App_FlightTask100Hz(void);
void App_FlightTask50Hz(void);
void App_FlightTask10Hz(void);
FcFlightState_t App_FlightGetState(void);
FcFlightMode_t App_FlightGetMode(void);
FcStatus_t App_FlightGetMotorOutput(FcMotorOutput_t *output);
FcStatus_t App_FlightGetTaskStats(AppFlightTaskStats_t *stats);
FcStatus_t App_FlightGetDebug(AppFlightDebug_t *debug);

#endif /* APP_FLIGHT_H */
