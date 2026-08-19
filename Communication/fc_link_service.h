#ifndef FC_LINK_SERVICE_H
#define FC_LINK_SERVICE_H

#include <stdint.h>
#include "fc_types.h"
#include "fc_link_protocol.h"

typedef struct
{
    uint32_t rx_frames;
    uint32_t rx_errors;
    uint32_t tx_frames;
    uint32_t tx_drops;
    uint16_t last_rx_sequence;
} FcLinkStats_t;

/* 已通过同步头、长度和CRC校验的最近一帧，便于Keil Watch直接查看。 */
typedef struct
{
    uint8_t last_rx_type;
    uint8_t last_tx_type;
    uint16_t last_rx_sequence;
    uint16_t last_tx_sequence;
    uint16_t last_rx_payload_length;
    uint16_t last_tx_frame_length;
    uint32_t last_rx_timestamp_ms;
    uint32_t valid_rx_frame_count;
    uint8_t last_rx_payload[FC_LINK_MAX_PAYLOAD];
} FcLinkDebug_t;

extern volatile FcLinkDebug_t g_fc_link_debug;

FcStatus_t FcLink_Init(void);
void FcLink_Service(void);
FcStatus_t FcLink_GetStats(FcLinkStats_t *stats);

#endif /* FC_LINK_SERVICE_H */
