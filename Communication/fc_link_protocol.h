#ifndef FC_LINK_PROTOCOL_H
#define FC_LINK_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "fc_types.h"

#define FC_LINK_SYNC_1                 0xA5U
#define FC_LINK_SYNC_2                 0x5AU
#define FC_LINK_VERSION                0x01U
#define FC_LINK_FIXED_HEADER_SIZE      12U
#define FC_LINK_CRC_SIZE                2U
#define FC_LINK_MAX_PAYLOAD           1024U
#define FC_LINK_MAX_FRAME             \
    (FC_LINK_FIXED_HEADER_SIZE + FC_LINK_MAX_PAYLOAD + FC_LINK_CRC_SIZE)

/* FAST_TELEMETRY v1.2：原42字节后追加4字节目标高度（int32，小端，单位0.01m）。 */
#define FC_LINK_FAST_TELEMETRY_PAYLOAD_SIZE             46U
#define FC_LINK_FAST_ALTITUDE_TARGET_OFFSET             42U
#define FC_LINK_FAST_VALID_ALTITUDE_TARGET_MASK    (1U << 8)

typedef enum
{
    FC_LINK_MSG_FAST_TELEMETRY = 0x01,
    FC_LINK_MSG_STATUS = 0x02,
    FC_LINK_MSG_LINK_STATUS = 0x03,
    FC_LINK_MSG_CONTROL = 0x10,
    FC_LINK_MSG_ARM_REQUEST = 0x11,
    FC_LINK_MSG_DISARM = 0x12,
    FC_LINK_MSG_EMERGENCY_STOP = 0x13,
    FC_LINK_MSG_SET_CONTROL_SOURCE = 0x14,
    FC_LINK_MSG_APP_HELLO = 0x20,
    FC_LINK_MSG_HELLO_ACK = 0x21,
    FC_LINK_MSG_APP_HEARTBEAT = 0x22,
    FC_LINK_MSG_ACK = 0x7E,
    FC_LINK_MSG_ERROR = 0x7F
} FcLinkMessageType_t;

typedef enum
{
    FC_LINK_RESULT_OK = 0,
    FC_LINK_RESULT_BAD_LENGTH = 1,
    FC_LINK_RESULT_BAD_VALUE = 2,
    FC_LINK_RESULT_DISABLED = 3,
    FC_LINK_RESULT_WRONG_STATE = 4,
    FC_LINK_RESULT_WRONG_SESSION = 5,
    FC_LINK_RESULT_NOT_READY = 6,
    FC_LINK_RESULT_UNSUPPORTED = 7
} FcLinkResult_t;

typedef struct
{
    uint8_t type;
    uint16_t payload_length;
    uint16_t sequence;
    uint32_t timestamp_ms;
    uint8_t payload[FC_LINK_MAX_PAYLOAD];
} FcLinkFrame_t;

typedef struct
{
    uint8_t bytes[FC_LINK_MAX_FRAME];
    uint16_t index;
    uint16_t expected_length;
} FcLinkParser_t;

void FcLink_ParserInit(FcLinkParser_t *parser);
FcStatus_t FcLink_ParserInput(FcLinkParser_t *parser,
                             uint8_t value,
                             FcLinkFrame_t *frame,
                             bool *complete);
uint16_t FcLink_Crc16Ccitt(const uint8_t *data, size_t length);
FcStatus_t FcLink_Encode(uint8_t type,
                        uint16_t sequence,
                        uint32_t timestamp_ms,
                        const uint8_t *payload,
                        uint16_t payload_length,
                        uint8_t *output,
                        uint16_t output_capacity,
                        uint16_t *output_length);

uint16_t FcLink_ReadU16Le(const uint8_t *data);
int16_t FcLink_ReadI16Le(const uint8_t *data);
uint32_t FcLink_ReadU32Le(const uint8_t *data);
void FcLink_WriteU16Le(uint8_t *data, uint16_t value);
void FcLink_WriteI16Le(uint8_t *data, int16_t value);
void FcLink_WriteU32Le(uint8_t *data, uint32_t value);
void FcLink_WriteI32Le(uint8_t *data, int32_t value);

#endif /* FC_LINK_PROTOCOL_H */
