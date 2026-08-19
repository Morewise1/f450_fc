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

/*
 * 定高油门语义，以下数值均为电调PWM微秒值，不是i-BUS原始通道值。
 *
 * CH6拨到定高后，以实测悬停前馈加高度PID修正作为总油门。飞手必须先
 * 把油门放入捕获区，之后捕获区内保持目标高度，区间上方/下方才分别
 * 改变目标高度。进入和退出均使用无扰混合，避免总油门突然跳变。
 */
#define FC_HOVER_FEEDFORWARD_US                    1490U
#define FC_ALT_HOLD_STICK_CAPTURE_MIN_US            1400U
#define FC_ALT_HOLD_STICK_CAPTURE_MAX_US            1600U
#define FC_ALT_HOLD_HANDOVER_DIRECT_TOLERANCE_US      50U
#define FC_ALT_HOLD_HANDOVER_BLEND_TIME_MS           400U

#define FC_RC_ROLL_SIGN                 1
#define FC_RC_PITCH_SIGN              (-1)
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
#define FC_RATE_YAW_KP                  0.80f
#define FC_RATE_YAW_KI                  0.0f
#define FC_RATE_YAW_KD                  0.0f

#define FC_ATTITUDE_ROLL_KP             4.0f
#define FC_ATTITUDE_PITCH_KP            4.0f
#define FC_ATTITUDE_ANGLE_DEADBAND_DEG   0.5f
#define FC_ATTITUDE_YAW_KP               2.5f
#define FC_ATTITUDE_YAW_DEADBAND_DEG     1.0f
#define FC_ATTITUDE_YAW_ERROR_LIMIT_DEG 30.0f
/*
 * Yaw杆离开中位时直接控制角速度；回中稳定150ms后捕获当时航向并保持。
 * 角速度阈值只用于状态切换，不会压缩正常Yaw杆量程。
 */
#define FC_YAW_HOLD_STICK_RATE_THRESHOLD_DPS  2.0f
#define FC_YAW_HOLD_CENTER_CONFIRM_MS          150U

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

#if (FC_HOVER_FEEDFORWARD_US < 1000U) || (FC_HOVER_FEEDFORWARD_US > 2000U)
#error "FC_HOVER_FEEDFORWARD_US must be a valid ESC pulse width"
#endif
#if (FC_ALT_HOLD_STICK_CAPTURE_MIN_US >= FC_ALT_HOLD_STICK_CAPTURE_MAX_US) || \
    (FC_ALT_HOLD_STICK_CAPTURE_MIN_US < 1000U) || \
    (FC_ALT_HOLD_STICK_CAPTURE_MAX_US > 2000U)
#error "ALT_HOLD throttle capture window is invalid"
#endif
#if (FC_HOVER_FEEDFORWARD_US < FC_ALT_HOLD_STICK_CAPTURE_MIN_US) || \
    (FC_HOVER_FEEDFORWARD_US > FC_ALT_HOLD_STICK_CAPTURE_MAX_US)
#error "Hover feed-forward should lie inside the ALT_HOLD capture window"
#endif
#if (FC_ALT_HOLD_HANDOVER_BLEND_TIME_MS == 0U)
#error "ALT_HOLD handover blend time must be non-zero"
#endif

#if FC_YAW_HOLD_CENTER_CONFIRM_MS == 0U
#error "Yaw heading-hold center confirmation time must be non-zero"
#endif

#endif /* FC_PARAMS_H */
