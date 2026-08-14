#ifndef FC_TYPES_H
#define FC_TYPES_H

/* Cross-module data contracts and fixed physical units. */

#include <stdbool.h>
#include <stdint.h>
#include "fc_config.h"

typedef enum
{
    FC_STATUS_OK = 0,
    FC_STATUS_ERROR,
    FC_STATUS_NOT_INITIALIZED,
    FC_STATUS_NOT_READY,
    FC_STATUS_NOT_IMPLEMENTED,
    FC_STATUS_INVALID_ARGUMENT,
    FC_STATUS_INVALID_DATA,
    FC_STATUS_TIMEOUT,
    FC_STATUS_BUSY
} FcStatus_t;

typedef enum
{
    FC_STATE_STOP = 0,
    FC_STATE_READY,
    FC_STATE_RUNNING
} FcFlightState_t;

typedef enum
{
    FC_MODE_STABILIZE = 0,
    FC_MODE_ALT_HOLD
} FcFlightMode_t;

typedef enum
{
    FC_SAFETY_FAULT_NONE             = 0U,
    FC_SAFETY_FAULT_RC_LOST          = (1UL << 0),
    FC_SAFETY_FAULT_IMU_INVALID      = (1UL << 1),
    FC_SAFETY_FAULT_BATTERY_UNKNOWN  = (1UL << 2),
    FC_SAFETY_FAULT_BATTERY_CRITICAL = (1UL << 3),
    FC_SAFETY_FAULT_EXCESSIVE_TILT   = (1UL << 4),
    FC_SAFETY_FAULT_SCHEDULER        = (1UL << 5),
    FC_SAFETY_FAULT_ARM_SWITCH       = (1UL << 6),
    FC_SAFETY_FAULT_EMERGENCY_STOP   = (1UL << 7),
    FC_SAFETY_FAULT_INITIALIZATION   = (1UL << 8)
} FcSafetyFault_t;

typedef struct { float x; float y; float z; } FcVector3f_t;

typedef struct
{
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    FcVector3f_t accel_g;
    FcVector3f_t gyro_dps;
    float temperature_c;
    uint32_t timestamp_ms;
    bool valid;
    bool calibrated;
} FcImuData_t;

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint32_t timestamp_ms;
    bool valid;
} FcAttitude_t;

typedef struct
{
    float pressure_pa;
    float temperature_c;
    uint32_t timestamp_ms;
    bool valid;
} FcBarometerData_t;

typedef struct
{
    int32_t raw[3];
    FcVector3f_t magnetic_ut;
    uint32_t timestamp_ms;
    bool overflow;
    bool valid;
} FcMagnetometerData_t;

typedef struct
{
    float distance_m;
    uint32_t timestamp_ms;
    bool valid;
} FcRangeData_t;

typedef struct
{
    float altitude_m;
    float vertical_velocity_mps;
    uint32_t timestamp_ms;
    bool valid;
} FcAltitude_t;

typedef struct
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint16_t throttle;
    bool throttle_low;
    bool arm_switch;
    bool mode_switch;
    bool safety_switch;
    bool emergency_stop;
    bool link_valid;
    bool failsafe;
    uint32_t last_frame_ms;
} FcRcInput_t;

/* Nonlinear pilot intent after deadband/expo mapping. */
typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float throttle;
    float climb_rate;
    bool motor_safe;
    bool valid;
} FcPilotCommand_t;

typedef struct
{
    uint16_t motor_us[FC_MOTOR_COUNT];
    bool valid;
} FcMotorOutput_t;

typedef struct
{
    uint16_t adc_raw;
    float voltage_v;
    float average_cell_voltage_v;
    uint32_t timestamp_ms;
    bool valid;
    bool warning;
    bool critical;
} FcBatteryStatus_t;

typedef struct
{
    uint32_t active_faults;
    bool rc_online;
    bool imu_ready;
    bool imu_calibrated;
    bool battery_ok;
    bool tilt_ok;
    bool scheduler_ok;
    bool initialization_ok;
    bool arm_conditions_met;
    bool motor_output_allowed;
} FcSafetyStatus_t;

typedef struct
{
    uint16_t throttle_us;
    float roll_deg;
    float pitch_deg;
    float yaw_rate_dps;
    float altitude_m;
    FcFlightMode_t mode;
} FcControlTarget_t;

typedef struct
{
    float roll_cmd_us;
    float pitch_cmd_us;
    float yaw_cmd_us;
    bool valid;
} FcControlOutput_t;

#endif /* FC_TYPES_H */
