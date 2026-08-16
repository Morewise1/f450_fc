#ifndef FC_LINK_SERVICE_H
#define FC_LINK_SERVICE_H

#include <stdint.h>
#include "fc_types.h"

typedef struct
{
    uint32_t rx_frames;
    uint32_t rx_errors;
    uint32_t tx_frames;
    uint32_t tx_drops;
    uint16_t last_rx_sequence;
} FcLinkStats_t;

FcStatus_t FcLink_Init(void);
void FcLink_Service(void);
FcStatus_t FcLink_GetStats(FcLinkStats_t *stats);

#endif /* FC_LINK_SERVICE_H */
