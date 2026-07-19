#ifndef DRV_IBUS_H
#define DRV_IBUS_H

/* FlySky FS-iA6B i-BUS parser; UART/DMA ownership remains in the HAL Core. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "fc_types.h"

typedef struct
{
    uint32_t valid_frame_count;
    uint32_t checksum_error_count;
    uint32_t format_error_count;
    uint32_t range_error_count;
    uint32_t sync_reset_count;
    uint32_t timeout_count;
    uint32_t last_valid_frame_ms;
} DrvIbusStats_t;

/** Initialize parser state. This does not initialize or claim a UART handle. */
FcStatus_t Drv_Ibus_Init(void);

/** Reset only the partial-frame parser while preserving the last valid input. */
void Drv_Ibus_ResetParser(void);

/**
 * Process one UART RX byte. Safe for a short UART interrupt callback.
 * Returns true only when a complete checksum/range-valid frame is committed.
 */
bool Drv_Ibus_ProcessByte(uint8_t byte, uint32_t timestamp_ms);

/** Process an interrupt/DMA byte block and return the number of valid frames. */
uint32_t Drv_Ibus_ProcessBuffer(const uint8_t *data, size_t length, uint32_t timestamp_ms);

/** Mark the link failsafe when no valid frame arrived within FC_RC_TIMEOUT_MS. */
void Drv_Ibus_UpdateTimeout(uint32_t now_ms);

/** Copy the latest normalized input. Failsafe data is always neutral/disarmed. */
FcStatus_t Drv_Ibus_GetInput(FcRcInput_t *input);

/** Copy all 14 raw channels from the last valid frame. */
FcStatus_t Drv_Ibus_GetRawChannels(uint16_t channels[FC_IBUS_CHANNEL_COUNT]);

/** Copy parser/link diagnostic counters. */
FcStatus_t Drv_Ibus_GetStats(DrvIbusStats_t *stats);

/** Return true only for a recent checksum-valid frame outside failsafe. */
bool Drv_Ibus_IsOnline(void);

#endif /* DRV_IBUS_H */

