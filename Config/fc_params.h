#ifndef FC_PARAMS_H
#define FC_PARAMS_H

/* Initial software parameters. Flight gains are placeholders, not flight-tested. */

#define FC_IBUS_CHANNEL_ROLL             0U
#define FC_IBUS_CHANNEL_PITCH            1U
#define FC_IBUS_CHANNEL_THROTTLE         2U
#define FC_IBUS_CHANNEL_YAW              3U
#define FC_IBUS_CHANNEL_ARM              4U
#define FC_IBUS_CHANNEL_MODE             5U

#define FC_IBUS_SWITCH_THRESHOLD_RAW  1500U
#define FC_IBUS_RAW_MIN               1000U
#define FC_IBUS_RAW_CENTER            1500U
#define FC_IBUS_RAW_MAX               2000U
#define FC_IBUS_RAW_VALID_MIN          750U
#define FC_IBUS_RAW_VALID_MAX         2250U

#define FC_RC_ROLL_SIGN                 1
#define FC_RC_PITCH_SIGN                1
#define FC_RC_YAW_SIGN                  1

#define FC_RATE_ROLL_KP                 0.0f
#define FC_RATE_ROLL_KI                 0.0f
#define FC_RATE_ROLL_KD                 0.0f
#define FC_RATE_PITCH_KP                0.0f
#define FC_RATE_PITCH_KI                0.0f
#define FC_RATE_PITCH_KD                0.0f
#define FC_RATE_YAW_KP                  0.0f
#define FC_RATE_YAW_KI                  0.0f
#define FC_RATE_YAW_KD                  0.0f

#define FC_ATTITUDE_ROLL_KP             0.0f
#define FC_ATTITUDE_PITCH_KP            0.0f

#endif /* FC_PARAMS_H */

