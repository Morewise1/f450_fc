#include <string.h>
#include "fc_link_protocol.h"

uint16_t FcLink_ReadU16Le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

int16_t FcLink_ReadI16Le(const uint8_t *data)
{
    return (int16_t)FcLink_ReadU16Le(data);
}

uint32_t FcLink_ReadU32Le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

void FcLink_WriteU16Le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}

void FcLink_WriteI16Le(uint8_t *data, int16_t value)
{
    FcLink_WriteU16Le(data, (uint16_t)value);
}

void FcLink_WriteU32Le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

void FcLink_WriteI32Le(uint8_t *data, int32_t value)
{
    FcLink_WriteU32Le(data, (uint32_t)value);
}

uint16_t FcLink_Crc16Ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t index;
    uint8_t bit;

    if ((data == NULL) && (length != 0U)) { return 0U; }
    for (index = 0U; index < length; ++index)
    {
        crc ^= (uint16_t)data[index] << 8U;
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = ((crc & 0x8000U) != 0U) ?
                  (uint16_t)((crc << 1U) ^ 0x1021U) :
                  (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

void FcLink_ParserInit(FcLinkParser_t *parser)
{
    if (parser != NULL)
    {
        (void)memset(parser, 0, sizeof(*parser));
    }
}

static void parser_restart(FcLinkParser_t *parser, uint8_t current)
{
    parser->index = 0U;
    parser->expected_length = 0U;
    if (current == FC_LINK_SYNC_1)
    {
        parser->bytes[0] = current;
        parser->index = 1U;
    }
}

FcStatus_t FcLink_ParserInput(FcLinkParser_t *parser,
                             uint8_t value,
                             FcLinkFrame_t *frame,
                             bool *complete)
{
    uint16_t payload_length;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if ((parser == NULL) || (frame == NULL) || (complete == NULL))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *complete = false;

    if (parser->index == 0U)
    {
        if (value == FC_LINK_SYNC_1)
        {
            parser->bytes[0] = value;
            parser->index = 1U;
        }
        return FC_STATUS_OK;
    }
    if (parser->index == 1U)
    {
        if (value == FC_LINK_SYNC_2)
        {
            parser->bytes[1] = value;
            parser->index = 2U;
        }
        else
        {
            parser_restart(parser, value);
        }
        return FC_STATUS_OK;
    }
    if (parser->index >= FC_LINK_MAX_FRAME)
    {
        parser_restart(parser, value);
        return FC_STATUS_INVALID_DATA;
    }

    parser->bytes[parser->index++] = value;
    if (parser->index == 6U)
    {
        payload_length = FcLink_ReadU16Le(&parser->bytes[4]);
        if (payload_length > FC_LINK_MAX_PAYLOAD)
        {
            parser_restart(parser, value);
            return FC_STATUS_INVALID_DATA;
        }
        parser->expected_length = (uint16_t)(FC_LINK_FIXED_HEADER_SIZE +
                                             payload_length +
                                             FC_LINK_CRC_SIZE);
    }
    if ((parser->expected_length == 0U) ||
        (parser->index < parser->expected_length))
    {
        return FC_STATUS_OK;
    }

    payload_length = FcLink_ReadU16Le(&parser->bytes[4]);
    received_crc = FcLink_ReadU16Le(
        &parser->bytes[FC_LINK_FIXED_HEADER_SIZE + payload_length]);
    calculated_crc = FcLink_Crc16Ccitt(&parser->bytes[2],
                                       (size_t)(10U + payload_length));
    if ((parser->bytes[2] != FC_LINK_VERSION) ||
        (received_crc != calculated_crc))
    {
        parser_restart(parser, value);
        return FC_STATUS_INVALID_DATA;
    }

    frame->type = parser->bytes[3];
    frame->payload_length = payload_length;
    frame->sequence = FcLink_ReadU16Le(&parser->bytes[6]);
    frame->timestamp_ms = FcLink_ReadU32Le(&parser->bytes[8]);
    if (payload_length != 0U)
    {
        (void)memcpy(frame->payload, &parser->bytes[12], payload_length);
    }
    parser->index = 0U;
    parser->expected_length = 0U;
    *complete = true;
    return FC_STATUS_OK;
}

FcStatus_t FcLink_Encode(uint8_t type,
                        uint16_t sequence,
                        uint32_t timestamp_ms,
                        const uint8_t *payload,
                        uint16_t payload_length,
                        uint8_t *output,
                        uint16_t output_capacity,
                        uint16_t *output_length)
{
    uint16_t total_length;
    uint16_t crc;

    if ((output == NULL) || (output_length == NULL) ||
        ((payload == NULL) && (payload_length != 0U)))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    if (payload_length > FC_LINK_MAX_PAYLOAD)
    {
        return FC_STATUS_INVALID_DATA;
    }
    total_length = (uint16_t)(FC_LINK_FIXED_HEADER_SIZE + payload_length +
                              FC_LINK_CRC_SIZE);
    if (output_capacity < total_length)
    {
        return FC_STATUS_BUSY;
    }

    output[0] = FC_LINK_SYNC_1;
    output[1] = FC_LINK_SYNC_2;
    output[2] = FC_LINK_VERSION;
    output[3] = type;
    FcLink_WriteU16Le(&output[4], payload_length);
    FcLink_WriteU16Le(&output[6], sequence);
    FcLink_WriteU32Le(&output[8], timestamp_ms);
    if (payload_length != 0U)
    {
        (void)memcpy(&output[12], payload, payload_length);
    }
    crc = FcLink_Crc16Ccitt(&output[2], (size_t)(10U + payload_length));
    FcLink_WriteU16Le(&output[12U + payload_length], crc);
    *output_length = total_length;
    return FC_STATUS_OK;
}
