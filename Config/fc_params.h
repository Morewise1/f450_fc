#ifndef FC_PARAMS_H
#define FC_PARAMS_H

/* Initial software parameters. Flight gains are bench-start values, not flight-tested. */

#define FC_IBUS_CHANNEL_ROLL             0U
#define FC_IBUS_CHANNEL_PITCH            1U
#define FC_IBUS_CHANNEL_THROTTLE         2U
#define FC_IBUS_CHANNEL_YAW              3U
#define FC_IBUS_CHANNEL_ARM              4U
#define FC_IBUS_CHANNEL_MODE             5U

#define FC_IBUS_SWITCH_THRESHOLD_RAW  1500U
#define FC_IBUS_ARM_ACTIVE_HIGH          1U
#define FC_IBUS_MODE_ACTIVE_HIGH         1U
#define FC_IBUS_RAW_MIN               1000U
#define FC_IBUS_RAW_CENTER            1500U
#define FC_IBUS_RAW_MAX               2000U
#define FC_IBUS_RAW_VALID_MIN          750U
#define FC_IBUS_RAW_VALID_MAX         2250U

#define FC_RC_ROLL_SIGN                 1
#define FC_RC_PITCH_SIGN                1
#define FC_RC_YAW_SIGN                  1

/*
 * Cascaded stabilize controller units:
 * attitude Kp: (deg/s) / deg
 * rate Kp: us / (deg/s), Ki: us / deg, Kd: us / (deg/s^2)
 * These conservative values must be tuned on the actual frame.
 */
#define FC_RATE_ROLL_KP                 0.75f
#define FC_RATE_ROLL_KI                 0.0f
#define FC_RATE_ROLL_KD                 0.0f
#define FC_RATE_PITCH_KP                0.75f
#define FC_RATE_PITCH_KI                0.0f
#define FC_RATE_PITCH_KD                0.0f
#define FC_RATE_YAW_KP                  0.50f
#define FC_RATE_YAW_KI                  0.20f
#define FC_RATE_YAW_KD                  0.0f

#define FC_ATTITUDE_ROLL_KP             4.0f
#define FC_ATTITUDE_PITCH_KP            4.0f

/* I-term limits are deliberately lower than the total mixer authority. */
#define FC_RATE_ROLL_I_LIMIT_US        100.0f
#define FC_RATE_PITCH_I_LIMIT_US       100.0f
#define FC_RATE_YAW_I_LIMIT_US          80.0f

/* Advanced PID capabilities are available, but conservative defaults apply. */
#define FC_PID_OUTPUT_OFFSET             0.0f
#define FC_PID_INPUT_DEADBAND            0.0f
#define FC_PID_INTEGRAL_SEPARATION      80.0f
#define FC_PID_VARIABLE_I_FULL_ERROR    10.0f
#define FC_PID_VARIABLE_I_ZERO_ERROR   100.0f
#define FC_PID_DERIVATIVE_LPF_HZ        20.0f
#define FC_PID_ENABLE_INTEGRAL_SEPARATION 1U
#define FC_PID_ENABLE_VARIABLE_INTEGRAL   0U
#define FC_PID_ENABLE_ANTI_WINDUP          1U
#define FC_PID_DERIVATIVE_ON_MEASUREMENT   1U
#define FC_PID_ENABLE_DERIVATIVE_LPF       1U

#if (FC_IBUS_ARM_ACTIVE_HIGH > 1U) || (FC_IBUS_MODE_ACTIVE_HIGH > 1U)
#error "i-BUS switch polarity macros must be 0 or 1"
#endif

#endif /* FC_PARAMS_H */
